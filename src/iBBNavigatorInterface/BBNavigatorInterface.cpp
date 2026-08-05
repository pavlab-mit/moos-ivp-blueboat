/*************************************************************
      Name: Raymond Turrisi (orig.), Jeremy Wenger (navigator-cpp port)
      Orgn: MIT, Cambridge MA
      File: iBBNavigatorInterface/BBNavigatorInterface.cpp
   Last Ed:  2026-07-24
     Brief:
        Unified Navigator Interface for the BlueBoat ASV, built on
        navigator-cpp. See BBNavigatorInterface.h for the overview.
*************************************************************/

#include <iterator>
#include <fstream>
#include "MBUtils.h"
#include "ACTable.h"
#include "BBNavigatorInterface.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <csignal>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <numeric>

using namespace std;

//---------------------------------------------------------
// Shared Navigator instance + async shutdown state.
//
// Design note (OO vs direct-call): navigator-cpp exposes a Navigator
// class rather than navigator-lib's free functions with hidden global
// state. The app owns exactly one instance. It lives at file scope
// (not as a class member) for one reason: the signal/atexit shutdown
// path must be able to neutralize the ESCs without an object pointer,
// exactly like the old free-function design - but now the global is
// explicit instead of hidden inside the library.

static Navigator g_nav;
static std::atomic<bool> g_disarm_on_exit{false};
static std::atomic<int> g_left_pin{BBNavigatorInterface::kPwmIndexCh14};
static std::atomic<int> g_right_pin{BBNavigatorInterface::kPwmIndexCh16};
static std::atomic<bool> g_shutdown_done{false};
static char g_esc_marker_path[256] = "/dev/shm/bb_esc_armed";

// Commanded pulse range. Globals (not members) because the static
// setPinPulseWidth() must work from the signal/atexit shutdown path.
// Configured via pwm_min_us / pwm_max_us in OnStartUp().
static std::atomic<double> g_pwm_min_us{BBNavigatorInterface::PWM_MIN_US};
static std::atomic<double> g_pwm_max_us{BBNavigatorInterface::PWM_MAX_US};

void rclamp(double &val, double min, double max)
{
  if (val < min)
  {
    val = min;
  }
  else if (val > max)
  {
    val = max;
  }
}

void BBNavigatorInterface::setPinPulseWidth(int pin_num, double target)
{
  const double pwm_min = g_pwm_min_us.load();
  const double pwm_max = g_pwm_max_us.load();

  // Map normalized command [-100,100] to pulse range
  double pulse_us_span = (pwm_max - pwm_min) / 2.0;
  double pulse_us = PWM_CENTER_US + (target / 100.0) * pulse_us_span;

  // Clamp to allowable range
  if (pulse_us < pwm_min) pulse_us = pwm_min;
  if (pulse_us > pwm_max) pulse_us = pwm_max;

  g_nav.pwm_set_pulse_us(pin_num, static_cast<float>(pulse_us));
}

void safePwmShutdown()
{
  // Runs from the destructor, atexit, or a signal handler - guard so
  // the neutral hold only executes once no matter how we got here.
  if (g_shutdown_done.exchange(true))
    return;

  // Hold neutral at 50 Hz for 1 second so the ESCs see a clean,
  // sustained stop command before we exit (or cut the signal).
  for (int i = 0; i < 50; ++i)
  {
    BBNavigatorInterface::setPinPulseWidth(g_left_pin.load(), 0);
    BBNavigatorInterface::setPinPulseWidth(g_right_pin.load(), 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (g_disarm_on_exit.load())
  {
    // Explicit disarm: OE high cuts all PCA9685 outputs, and clearing
    // the per-boot marker re-permits a sweep on the next launch.
    g_nav.pwm_enable(false);
    ::unlink(g_esc_marker_path);
  }
  // Either way the ESCs lose their signal when the process exits
  // (navigator-cpp's shutdown() releases the OE line), so they disarm
  // between missions and the next launch re-arms with a neutral hold.
}

void signalHandler(int signum)
{
  safePwmShutdown();
  exit(signum);
};

struct CalibrationParams {
    double offset;
    double gain;
};

const CalibrationParams BATTERY_CALIBRATIONS[] = {
    {0.3235, 37.8788},  // 1 battery
    {0.3235, 37.8788},  // 2 batteries - TODO: calibrate
    {0.3235, 37.8788},  // 3 batteries - TODO: calibrate
    {1.616, 37.8788},   // 4 batteries - calibrated
    {0.3235, 37.8788},  // 5 batteries - TODO: calibrate
    {0.3235, 37.8788},  // 6 batteries - TODO: calibrate
    {0.3235, 37.8788},  // 7 batteries - TODO: calibrate
    {0.3235, 37.8788}   // 8 batteries - TODO: calibrate
};

//---------------------------------------------------------
// Constructor()

BBNavigatorInterface::BBNavigatorInterface()
{
  signal(SIGINT, signalHandler);  // Interrupt from keyboard (Ctrl+C)
  signal(SIGTERM, signalHandler); // Termination signal
  signal(SIGABRT, signalHandler); // Abnormal termination (abort)
  signal(SIGQUIT, signalHandler); // Quit from keyboard
  signal(SIGSEGV, signalHandler); // Invalid memory access (segmentation fault)
  signal(SIGFPE, signalHandler);  // Floating point exception
  signal(SIGHUP, signalHandler);  // Hangup detected on controlling terminal or death of controlling process
  signal(SIGPIPE, signalHandler); // Write to a pipe with no one to read it
  signal(SIGALRM, signalHandler); // Timer signal from alarm()
  signal(SIGUSR1, signalHandler); // User-defined signal 1
  signal(SIGUSR2, signalHandler); // User-defined signal 2
  atexit(safePwmShutdown);

  m_min_thrust = -10; // between -100 and 100
  m_max_thrust = 10;
  m_thruster_dead_band = 5; // Default dead band of 5%

  m_thruster_range = m_max_thrust - m_min_thrust;
  m_thruster_alpha = 0.1;

  m_virtualThrusterLeft = LPF(m_thruster_alpha);
  m_virtualThrusterRight = LPF(m_thruster_alpha);

  // percent, +- 100, zero centered
  m_desired_thrust_left = 0;
  m_desired_thrust_right = 0;
  m_all_stop = false;

  // Initialize RC controller variables
  m_rc_connected   = false;
  m_rc_frame_valid = false;  // boots untrusting; flips true on first good frame
  for (int i = 0; i < 16; i++)
  {
    m_rc_channels[i] = 0.0;
  }
  m_rc_mode = false; // Default to MOOS control mode

  // RC deadman defaults: ON, 2-second timeout. Last-good-time is
  // initialized to 0 so the deadman fires immediately at startup
  // until the first RC mail arrives - vehicle starts SAFED.
  m_rc_deadman_enabled = true;
  m_rc_deadman_timeout = 2.0;
  m_last_rc_good_time = 0.0;
  m_rc_deadman_active = false;

  // Teleop defaults: inactive, 1-second freshness timeout.
  // Last-teleop-time starts at 0 so teleop can never engage before
  // the first TELEOP_ACTIVE=true mail arrives.
  m_teleop_active = false;
  m_teleop_thrust_left = 0.0;
  m_teleop_thrust_right = 0.0;
  m_last_teleop_time = 0.0;
  m_teleop_command_timeout = 1.0;
  m_teleop_engaged = false;

  // Bring the hardware up. navigator-cpp auto-detects the Navigator
  // board revision (BMP280 vs BMP390 chip id) and the Pi model - the
  // NAVOS_VERSION / RASPBERRY_PI_VERSION build-time selection is gone.
  // init() collects per-sensor warnings instead of aborting; they are
  // surfaced in the appcast via m_last_sensor_error.
  m_last_sensor_error = g_nav.init(NAV_AUTO, PI_AUTO);

  m_left_thruster_pin = PwmChannel::Ch14;
  m_right_thruster_pin = PwmChannel::Ch16;

  m_thruster_enabled = true;
  m_left_thruster_invert = 1;
  m_right_thruster_invert = 1;

  // ESC lifecycle defaults (see header). The marker default lives on
  // tmpfs so a reboot always clears it.
  m_initialize_esc = false;
  m_esc_arm_mode = "neutral";
  m_esc_marker_path = g_esc_marker_path;
  m_disarm_on_exit = false;
  m_esc_armed = false;
  m_pwm_output_enabled = false;

  // RGBW LED placeholders. All four-element vectors are sized
  // here so the rgbw_color config parser cannot land on an empty
  // vector (writing into [0..3] of an empty std::vector is UB).
  m_port_side = {0, 255, 0, 0};
  m_starboard_side = {255, 0, 0, 0};
  m_led_color_quad = {0, 0, 0, 0};
  m_active_color_quad = {0, 0, 0, 0};

  // Initialize thrust LPF outputs (the modulation thread reads
  // these atomically; default-initialize to zero so the first
  // PWM write before any Iterate() runs is centered/off).
  m_latest_set_thrust_left = 0.0;
  m_latest_set_thrust_right = 0.0;

  // First Iterate() computes dt = MOOSTime() - m_last_update;
  // initializing here keeps the first dt close to zero rather
  // than something near-MOOSTime.
  m_last_update = 0.0;

  // ADC chip
  m_current_scale = 37.8788;
  m_current_offset = 0.3235;
  m_voltage_scale = 11.132;
  m_voltage_offset = 0.0;
  m_rolling_window_seconds = 2;
  m_rolling_window_size = -1;

  m_num_batteries = 4;

  // Initialize thrust timeout parameters
  m_thrust_command_timeout = 2.0; // Default 2 seconds timeout
  m_last_thrust_command_time = MOOSTime();
  m_thrust_timeout_enabled = true;

  // RC control parameters
  m_theta_b = 90.0;   // Bank angle limit (degrees)
  m_turn_scale = 100;  // Turn sensitivity

  // AHRS initialization
  m_sample_frequency = 150.0;
  m_use_mag = false;
  m_ahrs_kp = 0.0;       // 0 => keep library defaults
  m_ahrs_ti = 0.0;
  m_ahrs_kp_quick = 0.0;
  m_ahrs_ti_quick = 0.0;
  // rad/s. Sized from the 7/27 zoe RC range tests: ~9,800 samples of
  // aggressive RC driving never exceeded 1.03 rad/s, so 3.0 gives ~3x
  // margin over observed dynamics while still gating corrupt samples.
  m_yaw_rate_clamp = 3.0;
  for (int i = 0; i < 3; i++)
  {
    m_gyro_bias[i] = 0.0;
    m_accel_bias[i] = 0.0;
    m_mag_bias[i] = 0.0;
  }
  for (int i = 0; i < 9; i++)
    m_mag_scale[i] = (i % 4 == 0) ? 1.0 : 0.0; // identity
  m_have_mag_cal = false;
  m_roll_offset = 0.0;
  m_pitch_offset = 0.0;
  m_yaw_offset = 0.0;
  m_declination_deg = -14.058;
  m_operating_heading_offset = 0.0;
  m_roll = 0.0;
  m_pitch = 0.0;
  m_yaw = 0.0;
  m_heading = 0.0;
  m_gyro_x = 0.0;
  m_gyro_y = 0.0;
  m_gyro_z = 0.0;
  m_yaw_rate = 0.0;
  m_accel_x = 0.0;
  m_accel_y = 0.0;
  m_accel_z = 0.0;
  m_qw = 1.0; m_qx = 0.0; m_qy = 0.0; m_qz = 0.0;
  m_imu_read_errors = 0;
  m_imu_glitch_count = 0;

  m_nav_temp = 0.0;
  m_nav_pressure = 0.0;
  m_rpi_temp = 0.0;
  m_leak_detected = false;

  // Publication suffix defaults. AHRS for fused orientation
  // outputs, IMU for raw gyro / level-compensated outputs.
  m_ahrs_pub_suffix = "AHRS";
  m_imu_pub_suffix = "IMU";
}

//---------------------------------------------------------
// Procedure: ahrsName() / imuName()
// Append the configured suffix (if any) to a base var name.

string BBNavigatorInterface::ahrsName(const string &base) const
{
  if (m_ahrs_pub_suffix.empty())
    return base;
  return base + "_" + m_ahrs_pub_suffix;
}

string BBNavigatorInterface::imuName(const string &base) const
{
  if (m_imu_pub_suffix.empty())
    return base;
  return base + "_" + m_imu_pub_suffix;
}

//---------------------------------------------------------
// Destructor

BBNavigatorInterface::~BBNavigatorInterface()
{
  m_running = false;
  m_ahrs_running = false;
  if (m_modulation_thread.joinable())
  {
    m_modulation_thread.join();
    dbg_print("Joined modulation thread on shutdown\n");
  }
  if (m_sensor_thread.joinable())
  {
    m_sensor_thread.join();
    dbg_print("Joined sensor thread on shutdown\n");
  }

  // LEDs off before the (possibly blocking) neutral hold.
  uint8_t rgb_array[24][4];
  memset(rgb_array, 0, sizeof(rgb_array));
  g_nav.neopixel_set_rgbw(rgb_array, 24);

  safePwmShutdown();

  dbg_print("Proper shutdown\n");
}

//---------------------------------------------------------
// ESC lifecycle

bool BBNavigatorInterface::escMarkerExists() const
{
  return ::access(m_esc_marker_path.c_str(), F_OK) == 0;
}

void BBNavigatorInterface::writeEscMarker() const
{
  FILE *f = fopen(m_esc_marker_path.c_str(), "w");
  if (f != nullptr)
  {
    fprintf(f, "armed\n");
    fclose(f);
  }
}

void BBNavigatorInterface::clearEscMarker() const
{
  ::unlink(m_esc_marker_path.c_str());
}

// Enable PWM output and arm the ESCs. The PWM signal does NOT survive
// an app restart: navigator-cpp's shutdown() releases the OE line on
// exit and pca9685_init() re-requests it disabled, so the ESCs lose
// signal and disarm between missions. The neutral hold therefore runs
// on EVERY launch - it is the documented Basic ESC 500 arming
// procedure and commands zero throttle at all times, so repeating it
// is harmless. The per-boot marker gates only the "sweep" mode: a
// throttle sweep can run at most once per boot, and can never replay
// into a live restart.
void BBNavigatorInterface::armIfNeeded()
{
  std::string err;
  err = g_nav.pwm_set_frequency(static_cast<float>(PWM_FREQ_HZ));
  if (!err.empty())
  {
    reportRunWarning("PWM frequency set failed: " + err);
    return;
  }

  // Neutral BEFORE enabling output so the first pulse the ESCs ever
  // see is 1500us, not a stale register value.
  setPinPulseWidth(m_left_thruster_pin, 0);
  setPinPulseWidth(m_right_thruster_pin, 0);
  err = g_nav.pwm_enable(true);
  if (!err.empty())
  {
    reportRunWarning("PWM enable failed: " + err);
    return;
  }
  m_pwm_output_enabled = true;

  if (!m_initialize_esc)
  {
    dbg_print("ESC initialization skipped (disabled in config)\n");
    return;
  }

  const bool first_launch = !escMarkerExists();

  if (m_esc_arm_mode == "sweep" && first_launch)
  {
    // Legacy max/min/neutral throttle sweep (both ESCs in parallel).
    // Only reachable on the first launch per boot.
    setPinPulseWidth(m_left_thruster_pin, 100);
    setPinPulseWidth(m_right_thruster_pin, 100);
    this_thread::sleep_for(chrono::milliseconds(500));
    setPinPulseWidth(m_left_thruster_pin, -100);
    setPinPulseWidth(m_right_thruster_pin, -100);
    this_thread::sleep_for(chrono::milliseconds(500));
    setPinPulseWidth(m_left_thruster_pin, 0);
    setPinPulseWidth(m_right_thruster_pin, 0);
    this_thread::sleep_for(chrono::milliseconds(250));
    reportEvent("ESC arm sequence complete (sweep, first launch this boot)");
  }
  else
  {
    if (m_esc_arm_mode == "sweep")
      reportEvent("Sweep already ran this boot (marker present); using neutral hold");
    // BlueRobotics Basic ESC 500s arm on a stable neutral signal; hold
    // 1500us for 2 seconds. No throttle excursion at any point.
    setPinPulseWidth(m_left_thruster_pin, 0);
    setPinPulseWidth(m_right_thruster_pin, 0);
    this_thread::sleep_for(chrono::milliseconds(2000));
    reportEvent("ESC arm sequence complete (neutral hold)");
  }

  m_esc_armed = true;
  if (first_launch)
    writeEscMarker();
  dbg_print("ESC arm sequence performed (%s%s)\n", m_esc_arm_mode.c_str(),
            first_launch ? ", marker written" : "");
}

void BBNavigatorInterface::requestDisarm(const std::string &reason)
{
  m_desired_thrust_left = 0;
  m_desired_thrust_right = 0;
  m_latest_set_thrust_left.store(0.0);
  m_latest_set_thrust_right.store(0.0);
  setPinPulseWidth(m_left_thruster_pin, 0);
  setPinPulseWidth(m_right_thruster_pin, 0);
  this_thread::sleep_for(chrono::milliseconds(100));
  g_nav.pwm_enable(false);
  m_pwm_output_enabled = false;
  m_esc_armed = false;
  clearEscMarker();
  reportEvent("ESCs DISARMED (" + reason + ")");
  dbg_print("ESCs disarmed: %s\n", reason.c_str());
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool BBNavigatorInterface::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

    if (key == "DESIRED_THRUST_L")
    {
      m_desired_thrust_left = -1.0 * msg.GetDouble() * m_left_thruster_invert;
      dbg_print("Desired Left Thrust: %0.2f\n", m_desired_thrust_left);

      // Update the last command time
      m_last_thrust_command_time = MOOSTime();
    }
    else if (key == "DESIRED_THRUST_R")
    {
      m_desired_thrust_right = -1.0 * msg.GetDouble() * m_right_thruster_invert;
      dbg_print("Desired Right Thrust: %0.2f\n", m_desired_thrust_right);

      // Update the last command time
      m_last_thrust_command_time = MOOSTime();
    }
    else if (key == "MISSION_COMPLETE")
    {
      bool mission_complete = (msg.GetString() == "true") ? true : false;
      m_running = false;
      if (m_modulation_thread.joinable())
      {
        m_modulation_thread.join();
        dbg_print("Joined thread on shutdown\n");
      }
      dbg_print("mission_complete: %d\n", mission_complete);
      exit(0); // atexit -> safePwmShutdown (honors disarm_on_exit)
    }
    else if (key == "NVGR_DISARM")
    {
      // Operator / backseat commanded full disarm: cut the PWM signal
      // so the ESCs disarm, and clear the per-boot marker so the next
      // launch re-arms. Re-arm within this run by publishing
      // NVGR_DISARM=false.
      if (msg.GetString() == "true")
        requestDisarm("NVGR_DISARM via MOOS");
      else if (!m_pwm_output_enabled)
        armIfNeeded();
    }
    else if (key == "ALL_STOP")
    {
      m_all_stop = (msg.GetString() == "true");
      dbg_print("ALL_STOP received: %s\n", m_all_stop ? "true" : "false");
    }
    // Handle laptop teleop messages (from iTeleop). Thrust values
    // are stored raw in the wire convention; the -1 * invert
    // transform is applied in the Iterate() teleop branch so the
    // convention matches DESIRED_THRUST_L/R and RC exactly.
    else if (key == "TELEOP_ACTIVE")
    {
      bool was_active = m_teleop_active;
      m_teleop_active = (msg.GetString() == "true");
      m_last_teleop_time = MOOSTime();
      if (m_teleop_active != was_active)
      {
        reportEvent(std::string("Teleop ") +
                    (m_teleop_active ? "ACTIVE" : "inactive") + " via MOOS");
        dbg_print("Teleop active: %s\n", m_teleop_active ? "true" : "false");
      }
    }
    else if (key == "TELEOP_THRUST_L")
    {
      m_teleop_thrust_left = msg.GetDouble();
      m_last_teleop_time = MOOSTime();
      dbg_print("Teleop Left Thrust: %0.2f\n", m_teleop_thrust_left);
    }
    else if (key == "TELEOP_THRUST_R")
    {
      m_teleop_thrust_right = msg.GetDouble();
      m_last_teleop_time = MOOSTime();
      dbg_print("Teleop Right Thrust: %0.2f\n", m_teleop_thrust_right);
    }
    // Handle RC channel messages
    else if (key == "RC_CONNECTED")
    {
      m_rc_connected = (msg.GetString() == "true");
      // Only refresh the deadman timestamp on a "good" RC report.
      // RC_CONNECTED=false leaves m_last_rc_good_time stale so the
      // deadman timer can run out and safe the vehicle.
      if (m_rc_connected)
        m_last_rc_good_time = MOOSTime();
      dbg_print("RC connected: %s\n", m_rc_connected ? "true" : "false");
    }
    else if (key == "RC_FRAME_VALID")
    {
      // Per-frame validity flag from iRCReader. Used as the
      // primary thrust gate so a single bad SBUS frame drops
      // commanded thrust to zero immediately, without waiting on
      // the m_rc_connected hysteresis. See calculateRCThrust()
      // and the Iterate() RC-mode branch.
      m_rc_frame_valid = (msg.GetString() == "true");
      dbg_print("RC frame valid: %s\n",
                m_rc_frame_valid ? "true" : "false");
    }
    else if (key.substr(0, 5) == "RC_CH" && key.length() > 5)
    {
      // Extract channel number from key (e.g., "RC_CH1" -> 1)
      int channel = std::stoi(key.substr(5)) - 1; // Zero-indexed
      if (channel >= 0 && channel < 16)
      {
        m_rc_channels[channel] = msg.GetDouble();

        // NOTE: deadman timestamp (m_last_rc_good_time) is
        // refreshed ONLY on RC_CONNECTED=true (handler above).
        // iRCReader publishes safe-default RC_CH* values when its
        // link is bad (joystick=0, switches=1, raw=mid), so
        // RC_CH* mail must NOT count as a "good" RC tick;
        // otherwise the disconnected-fallback publishes would
        // silently defeat the watchdog.

        // Mode switch (Channel 6) — latch operator-selected mode.
        // Only update on RC_CHx mail received while the link is
        // connected; iRCReader publishes safe-default CH6=1 during
        // disconnect, and trusting it would auto-flip the boat to
        // autonomy on signal loss. Preserving the last-known mode
        // across dropouts gives the desired behavior in both deadman
        // states:
        //   deadman enabled  + last-mode RC   -> stays RC; deadman
        //                                        trips and zeros
        //                                        thrust (boat safed).
        //   deadman disabled + last-mode auto -> stays auto; autonomy
        //                                        continues (over-the-
        //                                        horizon use case).
        // iRCReader publishes RC_CONNECTED before the safe-default
        // RC_CHx values within an iterate, so the gate sees the
        // updated m_rc_connected before evaluating CH6.
        if (channel == 5 && m_rc_connected) // Channel 6 (zero-indexed as 5)
        {
          m_rc_mode = (m_rc_channels[channel] == 2.0);
          dbg_print("RC mode: %s\n", m_rc_mode ? "true" : "false");
        }

        dbg_print("RC_CH%d: %0.2f\n", channel + 1, m_rc_channels[channel]);
      }
    }
    else if (key == "RC_DEADMAN_ENABLED")
    {
      m_rc_deadman_enabled = (msg.GetString() == "true");
      reportEvent(std::string("RC deadman ") +
                  (m_rc_deadman_enabled ? "ENABLED" : "DISABLED") +
                  " via MOOS");
      dbg_print("RC deadman %s via MOOS\n",
                m_rc_deadman_enabled ? "ENABLED" : "DISABLED");
    }
    else if (key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: dbg_print()
bool BBNavigatorInterface::dbg_print(const char *format, ...)
{
  if (m_debug == true)
  {
    va_list args;
    va_start(args, format);
    m_debug_stream = fopen(m_fname, "a");
    if (m_debug_stream != nullptr)
    {
      vfprintf(m_debug_stream, format, args);
      fclose(m_debug_stream);
      return true;
    }
    else
    {
      reportRunWarning("Debug mode is enabled and file could not be opened\n");
      return false;
    }
  }
  return false;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool BBNavigatorInterface::OnConnectToServer()
{
  registerVariables();
  return (true);
}

void BBNavigatorInterface::manageModulation()
{
  while (m_running)
  {
    // If thrusters are enabled, set them to the desired speed
    if (m_thruster_enabled)
    {
      setPinPulseWidth(m_left_thruster_pin, m_latest_set_thrust_left.load());
      setPinPulseWidth(m_right_thruster_pin, m_latest_set_thrust_right.load());
    }
    else
    {
      setPinPulseWidth(m_left_thruster_pin, 0);
      setPinPulseWidth(m_right_thruster_pin, 0);
    }

    // Sleep for 20 milliseconds to achieve a 50 Hz cycle rate
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

//---------------------------------------------------------
// Procedure: calculateHeadingMixer
// Calculate a mixer value based on heading error, similar to DiffThrustPID

double BBNavigatorInterface::calculateHeadingMixer(double desired_heading, double current_heading)
{
  // Compute heading error
  double theta_r = desired_heading - current_heading;

  // Normalize angle error to [-180, 180]
  theta_r = fmod(theta_r + 180.0, 360.0);
  if (theta_r < 0)
    theta_r += 360.0;
  theta_r -= 180.0;

  // Compute the mixer value as ratio of heading error to bank angle
  double m_m = 0.0;
  if (m_theta_b != 0.0)
  {
    m_m = theta_r / m_theta_b;
  }

  // Saturate m_m to [-1, 1]
  if (m_m > 1.0)
    m_m = 1.0;
  if (m_m < -1.0)
    m_m = -1.0;

  return m_m;
}

//---------------------------------------------------------
// Procedure: calculateRCThrust()
//            Calculate thrust values from RC inputs

void BBNavigatorInterface::calculateRCThrust()
{
  // Gate on per-frame validity (sharper than m_rc_connected) AND
  // RC mode. Using m_rc_frame_valid means a single bad SBUS frame
  // drops thrust to zero immediately, without waiting for the
  // hysteresis on m_rc_connected to flip. The m_rc_mode check is
  // unchanged - mode latching uses m_rc_connected via CH6 mail.
  if (m_rc_frame_valid && m_rc_mode)
  {

    double forward_thrust = m_rc_channels[2]; // Already in [-100, 100]
    double mixer = -m_rc_channels[0];         // Turning input in [-100, 100]

    // Scale mixer
    mixer *= (m_turn_scale / 100.0); // So m_turn_scale=50 means 50% influence

    // Clamp to prevent saturation beyond limits
    rclamp(mixer, -100.0, 100.0);

    // Combine linearly
    double left_thrust = forward_thrust + mixer;
    double right_thrust = forward_thrust - mixer;

    // Clamp final outputs to [-100, 100]
    rclamp(left_thrust, -100.0, 100.0);
    rclamp(right_thrust, -100.0, 100.0);

    m_desired_thrust_left = left_thrust * m_left_thruster_invert;
    m_desired_thrust_right = right_thrust * m_right_thruster_invert;

    // Update the last command time since RC control counts as a thrust command
    m_last_thrust_command_time = MOOSTime();

    dbg_print("RC Control - Forward: %0.2f, Turn: %0.2f, Mixer: %0.2f\n",
              forward_thrust, 0.0, mixer);

    dbg_print("RC Control - Left: %0.2f, Right: %0.2f\n",
              m_desired_thrust_left, m_desired_thrust_right);
  }
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool BBNavigatorInterface::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Get the change in time for updating the virtual actuators
  double dt = MOOSTime() - m_last_update;

  // Teleop engagement, evaluated after mail so an RC takeover or a
  // TELEOP_ACTIVE=false in this cycle's mail is honored immediately.
  // Freshness doubles as a vehicle-side deadman: iTeleop re-publishes
  // TELEOP_ACTIVE/TELEOP_THRUST_* every iterate, so stale mail means
  // iTeleop is hung or dead and teleop must disengage. Priority:
  // RC (m_rc_mode) > teleop > backseat autonomy.
  const bool teleop_fresh =
      (MOOSTime() - m_last_teleop_time) < m_teleop_command_timeout;
  const bool teleop_was_engaged = m_teleop_engaged;
  m_teleop_engaged = m_teleop_active && teleop_fresh && !m_rc_mode;
  if (m_teleop_engaged && !teleop_was_engaged)
    reportEvent("Teleop ENGAGED - ignoring backseat thrust");
  if (!m_teleop_engaged && teleop_was_engaged)
    reportEvent("Teleop disengaged");

  // Check for thrust command timeout if enabled. Teleop refreshes
  // m_last_thrust_command_time in its branch below, so the timeout
  // cannot fire mid-teleop; the exemption here covers the same
  // cycle boundary case as the RC exemption.
  if (m_thrust_timeout_enabled && m_thrust_command_timeout > 0)
  {
    double time_since_last_command = MOOSTime() - m_last_thrust_command_time;

    if (time_since_last_command > m_thrust_command_timeout && !m_rc_mode &&
        !m_teleop_engaged)
    {
      // Timeout occurred, set thrusts to zero
      m_desired_thrust_left = 0;
      m_desired_thrust_right = 0;

      dbg_print("Thrust timeout occurred (%.2f seconds). Setting thrusts to zero.\n",
                time_since_last_command);

      // Publish timeout notification
      Notify("NVGR_THRUST_TIMEOUT", "true");
    }
    else
    {
      Notify("NVGR_THRUST_TIMEOUT", "false");
    }
  }

  // Check for ALL_STOP condition - only affects autonomous control.
  // Teleop is exempt like RC: it is a manual rescue mode, and a
  // latched backseat ALL_STOP must not paralyze the rescue. The GUI
  // E-STOP arrives as zero teleop thrust, not via ALL_STOP.
  if (m_all_stop && !m_rc_mode && !m_teleop_engaged)
  {
    m_desired_thrust_left = 0;
    m_desired_thrust_right = 0;
    dbg_print("ALL_STOP active - setting autonomous thrusts to zero\n");
  }

  // RC thrust path. We gate on m_rc_frame_valid (per-frame, no
  // debounce) rather than m_rc_connected so that a single bad
  // SBUS frame zeros thrust immediately. m_rc_connected is the
  // debounced state and is used for mode-related logic only.
  //
  //   m_rc_mode &&  m_rc_frame_valid -> drive thrust from RC sticks
  //   m_rc_mode && !m_rc_frame_valid -> zero thrust this cycle
  //                                     (link bad or single bad frame)
  //  !m_rc_mode                       -> autonomy / MOOS path drives thrust
  //                                     (no action here)
  if (m_rc_mode && m_rc_frame_valid)
  {
    calculateRCThrust();
  }
  else if (m_rc_mode && !m_rc_frame_valid)
  {
    m_desired_thrust_left = 0;
    m_desired_thrust_right = 0;
    dbg_print("RC frame invalid - setting thrusts to zero\n");
  }
  // Teleop thrust path (only reachable when not in RC mode).
  // Same sign/invert transform as the DESIRED_THRUST_L/R mail
  // handlers so all three command sources share one convention.
  else if (m_teleop_engaged)
  {
    m_desired_thrust_left = -1.0 * m_teleop_thrust_left * m_left_thruster_invert;
    m_desired_thrust_right = -1.0 * m_teleop_thrust_right * m_right_thruster_invert;

    // Teleop counts as a thrust command (mirrors calculateRCThrust)
    m_last_thrust_command_time = MOOSTime();

    dbg_print("Teleop Control - Left: %0.2f, Right: %0.2f\n",
              m_desired_thrust_left, m_desired_thrust_right);
  }
  // Teleop claimed the vehicle but its mail went stale (iTeleop hung
  // or died with TELEOP_ACTIVE latched true): zero thrust this cycle
  // rather than falling through to stale backseat/teleop values.
  else if (m_teleop_active && !teleop_fresh)
  {
    m_desired_thrust_left = 0;
    m_desired_thrust_right = 0;
    dbg_print("Teleop mail stale - setting thrusts to zero\n");
  }

  // RC deadman watchdog (final override). When enabled, requires
  // a "good" RC tick (RC_CONNECTED=true or any RC_CH* mail) within
  // m_rc_deadman_timeout seconds. Triggers in BOTH RC and autonomous
  // mode - treats the RC link as a vehicle-side deadman. Disable via
  // rc_deadman_enabled=false (config) or RC_DEADMAN_ENABLED=false
  // (runtime) for over-the-horizon missions where RC range loss is
  // expected.
  //
  // Teleop exemption: while teleop is engaged, the GUI link is the
  // deadman (iTeleop's gui_deadman_timeout plus the teleop freshness
  // check above - two independent layers). Requiring the SBUS
  // transmitter to also be alive would make laptop teleop useless
  // exactly when it is needed: RC out of range or transmitter dead.
  bool deadman_was_active = m_rc_deadman_active;
  m_rc_deadman_active = false;
  if (m_rc_deadman_enabled && !m_teleop_engaged)
  {
    double rc_age = MOOSTime() - m_last_rc_good_time;
    if (rc_age > m_rc_deadman_timeout)
    {
      m_rc_deadman_active = true;
      m_desired_thrust_left = 0;
      m_desired_thrust_right = 0;
      if (!deadman_was_active)
        reportRunWarning("RC deadman tripped (no RC for " +
                         doubleToString(rc_age, 1) + "s)");
      dbg_print("RC deadman ACTIVE: rc_age=%.2fs > timeout=%.2fs\n",
                rc_age, m_rc_deadman_timeout);
    }
  }
  if (deadman_was_active && !m_rc_deadman_active)
    reportEvent("RC deadman cleared");
  Notify("NVGR_RC_DEADMAN_ACTIVE",
         m_rc_deadman_active ? "true" : "false");

  // Apply dead band to thrusters
  if (fabs(m_desired_thrust_left) < m_thruster_dead_band)
  {
    m_desired_thrust_left = 0;
  }

  if (fabs(m_desired_thrust_right) < m_thruster_dead_band)
  {
    m_desired_thrust_right = 0;
  }

  // Apply min/max thrust limits
  rclamp(m_desired_thrust_left, m_min_thrust, m_max_thrust);
  rclamp(m_desired_thrust_right, m_min_thrust, m_max_thrust);

  // Update thruster values with LPF. Compute first into locals
  // so the variadic dbg_print / Notify calls below see plain
  // doubles, not std::atomic<double> (which would be UB through
  // varargs).
  const double new_thrust_left =
      m_virtualThrusterLeft.update(m_desired_thrust_left, dt);
  const double new_thrust_right =
      m_virtualThrusterRight.update(m_desired_thrust_right, dt);
  m_latest_set_thrust_left.store(new_thrust_left);
  m_latest_set_thrust_right.store(new_thrust_right);

  dbg_print("%0.2f - Desired left thruster: %0.2f - set %0.2f\n",
            MOOSTime(), m_desired_thrust_left, new_thrust_left);
  dbg_print("%0.2f - Desired right thruster: %0.2f - set %0.2f\n",
            MOOSTime(), m_desired_thrust_right, new_thrust_right);

  Notify("THRUST_SET_LEFT", new_thrust_left);
  Notify("THRUST_SET_RIGHT", new_thrust_right);

  m_last_update = MOOSTime();

  // ADC chip
  // Read ADC values
  NavADCData adc;
  std::string adc_err = g_nav.read_adc_all(adc);
  if (adc_err.empty())
  {
    m_adc_1 = adc.channel[0];
    m_adc_2 = adc.channel[1];
    m_adc_3 = adc.channel[2];
    m_adc_4 = adc.channel[3];
  }
  CalibrationParams cal = BATTERY_CALIBRATIONS[m_num_batteries - 1];
  m_latest_current = (m_adc_3 - cal.offset) * cal.gain;
  m_latest_voltage = (m_adc_4 - m_voltage_offset) * m_voltage_scale;

  m_rolling_voltage_window[m_apptick_idx] = m_latest_voltage;
  m_rolling_current_window[m_apptick_idx] = m_latest_current;
  m_rolling_power_window[m_apptick_idx] = m_latest_current * m_latest_voltage;

  m_apptick_idx = (m_apptick_idx + 1) % m_rolling_window_size;

  m_rolling_current = std::accumulate(m_rolling_current_window.begin(), m_rolling_current_window.end(), 0.0) / m_rolling_window_size;
  m_rolling_voltage = std::accumulate(m_rolling_voltage_window.begin(), m_rolling_voltage_window.end(), 0.0) / m_rolling_window_size;
  m_rolling_power = std::accumulate(m_rolling_power_window.begin(), m_rolling_power_window.end(), 0.0) / m_rolling_window_size;

  // Publish raw ADC values
  Notify("NVGR_ADC_1", m_adc_1);
  Notify("NVGR_ADC_2", m_adc_2);
  Notify("NVGR_ADC_3", m_adc_3);
  Notify("NVGR_ADC_4", m_adc_4);

  // Scale and publish current and voltage
  Notify("NVGR_CURRENT", m_latest_current);
  Notify("NVGR_VOLTAGE", m_latest_voltage);
  Notify("NVGR_ROLLING_CURRENT", m_rolling_current);
  Notify("NVGR_ROLLING_VOLTAGE", m_rolling_voltage);
  Notify("NVGR_ROLLING_POWER", m_rolling_power);

  // Publish current thrust values
  Notify("NVGR_THRUST_LEFT", new_thrust_left);
  Notify("NVGR_THRUST_RIGHT", new_thrust_right);

  // Applied thrust converted back to the wire convention (the
  // DESIRED_THRUST_L/R frame of reference). The invert flags are
  // +/-1, so multiplying by -1 * invert exactly undoes the input
  // transform. Consumed by iTeleop for GUI acks and useful for
  // backseat-side logging in the command convention.
  Notify("NVGR_THRUST_LEFT_WIRE", -1.0 * new_thrust_left * m_left_thruster_invert);
  Notify("NVGR_THRUST_RIGHT_WIRE", -1.0 * new_thrust_right * m_right_thruster_invert);

  // Publish RC control status
  Notify("NVGR_RC_MODE", m_rc_mode ? "true" : "false");
  Notify("NVGR_RC_CONNECTED", m_rc_connected ? "true" : "false");

  // Publish teleop status (dashboards, pBB_Status mode fusion)
  Notify("NVGR_TELEOP_ENGAGED", m_teleop_engaged ? "true" : "false");

  // ESC arming state (dashboards / pBB_Status)
  Notify("NVGR_ESC_ARMED", m_pwm_output_enabled ? "true" : "false");

  // IPT Sensing
  // Read temperature and pressure values (one baro transaction gives both)
  NavBaroData baro;
  if (g_nav.read_baro(baro).empty())
  {
    m_nav_temp = baro.temperature_c;
    m_nav_pressure = baro.pressure_kpa;
  }

  // Leak detector (GPIO). Newly surfaced with navigator-cpp.
  bool leak = false;
  if (g_nav.read_leak(leak).empty())
  {
    if (leak && !m_leak_detected)
      reportRunWarning("LEAK DETECTED (navigator leak probe)");
    m_leak_detected = leak;
  }
  Notify("NVGR_LEAK", m_leak_detected ? "true" : "false");

  // Read Pi CPU temperature directly from sysfs. On non-Pi systems
  // the file is absent and m_rpi_temp stays 0.
  m_rpi_temp = 0.0;
  {
    std::ifstream temp_file("/sys/class/thermal/thermal_zone0/temp");
    if (temp_file.is_open()) {
      int milli_c = 0;
      temp_file >> milli_c;
      m_rpi_temp = milli_c / 1000.0;
    }
  }

  // Publish the values
  Notify("NVGTR_IT_C", m_nav_temp);
  Notify("NVGTR_IP_KPA", m_nav_pressure);
  Notify("RPI_TEMP", m_rpi_temp);

  // AHRS output publishing
  double roll, pitch, yaw, heading;
  double gyro_x, gyro_y, gyro_z, yaw_rate;
  double accel_x, accel_y, accel_z;
  {
    std::lock_guard<std::mutex> lock(m_ahrs_mutex);
    roll = m_roll;
    pitch = m_pitch;
    yaw = m_yaw;
    heading = m_heading;
    gyro_x = m_gyro_x;
    gyro_y = m_gyro_y;
    gyro_z = m_gyro_z;
    yaw_rate = m_yaw_rate;
    accel_x = m_accel_x;
    accel_y = m_accel_y;
    accel_z = m_accel_z;
  }

  Notify(ahrsName("NAV_ROLL"), roll);
  Notify(ahrsName("NAV_PITCH"), pitch);
  Notify(ahrsName("NAV_YAW"), yaw);
  Notify(ahrsName("NAV_HEADING"), heading);
  Notify(imuName("GYRO_X"), gyro_x);
  Notify(imuName("GYRO_Y"), gyro_y);
  Notify(imuName("GYRO_Z"), gyro_z);
  Notify(imuName("GYRO_Z_LVL"), yaw_rate);

  // Bundled, coherent IMU snapshot: one atomic message at a single
  // timestamp so high-rate motion reconstructs without inter-variable
  // skew.
  char imu_state[320];
  snprintf(imu_state, sizeof(imu_state),
           "t=%.3f,roll=%.4f,pitch=%.4f,yaw=%.4f,heading=%.4f,"
           "gx=%.5f,gy=%.5f,gz=%.5f,yawrate=%.5f,ax=%.5f,ay=%.5f,az=%.5f",
           MOOSTime(), roll, pitch, yaw, heading,
           gyro_x, gyro_y, gyro_z, yaw_rate, accel_x, accel_y, accel_z);
  Notify("IMU_STATE", std::string(imu_state));

  AppCastingMOOSApp::PostReport();
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool BBNavigatorInterface::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());
  m_app_name = GetAppName();

  m_apptick = GetAppFreq();
  m_commtick = GetCommsFreq();

  m_rolling_window_size = m_rolling_window_seconds * m_apptick;
  m_apptick_idx = 0;

  m_rolling_voltage_window = std::vector<double>(m_rolling_window_size, 0.0);
  m_rolling_current_window = std::vector<double>(m_rolling_window_size, 0.0);
  m_rolling_power_window = std::vector<double>(m_rolling_window_size, 0.0);

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++)
  {
    string orig = *p;
    string line = *p;
    string param = tolower(biteStringX(line, '='));
    string value = stripBlankEnds(line);

    bool handled = false;
    if (param == "left_thruster_pin")
    {
      m_left_thruster_pin = stoi(value) - 1;
      handled = true;
    }
    else if (param == "right_thruster_pin")
    {
      m_right_thruster_pin = stoi(value) - 1;
      handled = true;
    }
    else if (param == "nbats")
    {
      m_num_batteries = stoi(value);
      if (m_num_batteries < 1 || m_num_batteries > 8) {
        reportConfigWarning("nbats must be between 1 and 8, using default of 4");
        m_num_batteries = 4;
      }
      // BATTERY_CALIBRATIONS only has a real entry for 4 packs;
      // the others use placeholder values copied from the 1-pack
      // cal. Surface this so it doesn't silently mis-report current.
      if (m_num_batteries != 4) {
        reportConfigWarning("nbats=" + std::to_string(m_num_batteries) +
                            " uses placeholder current calibration; "
                            "only nbats=4 has a measured cal.");
      }
      handled = true;
    }
    else if (param == "left_thruster_invert")
    {
      m_left_thruster_invert = (tolower(value) == "true") ? -1 : 1;
      handled = true;
    }
    else if (param == "right_thruster_invert")
    {
      m_right_thruster_invert = (tolower(value) == "true") ? -1 : 1;
      handled = true;
    }
    else if (param == "max_thrust")
    {
      m_max_thrust = stod(value);
      handled = true;
    }
    else if (param == "min_thrust")
    {
      m_min_thrust = stod(value);
      handled = true;
    }
    else if (param == "thruster_dead_band")
    {
      m_thruster_dead_band = stod(value);
      handled = true;
    }
    else if (param == "thruster_enabled")
    {
      m_thruster_enabled = tolower(value) == "true" ? true : false;
      handled = true;
    }
    else if (param == "thruster_alpha")
    {
      m_thruster_alpha = stod(value);
      m_virtualThrusterLeft = LPF(m_thruster_alpha);
      m_virtualThrusterRight = LPF(m_thruster_alpha);
      handled = true;
    }
    else if (param == "thrust_command_timeout")
    {
      m_thrust_command_timeout = stod(value);
      m_thrust_timeout_enabled = (m_thrust_command_timeout > 0);
      handled = true;
    }
    else if (param == "rc_deadman_enabled")
    {
      m_rc_deadman_enabled = (tolower(value) == "true");
      handled = true;
    }
    else if (param == "rc_deadman_timeout")
    {
      m_rc_deadman_timeout = stod(value);
      if (m_rc_deadman_timeout < 0.1)
        m_rc_deadman_timeout = 0.1;
      handled = true;
    }
    else if (param == "teleop_command_timeout")
    {
      m_teleop_command_timeout = stod(value);
      if (m_teleop_command_timeout < 0.1)
        m_teleop_command_timeout = 0.1;
      handled = true;
    }
    else if (param == "theta_b")
    {
      m_theta_b = stod(value);
      handled = true;
    }
    else if (param == "turn_scale")
    {
      m_turn_scale = stod(value);
      handled = true;
    }
    else if (param == "current_scale")
    {
      m_current_scale = atof(value.c_str());
      handled = true;
    }
    else if (param == "current_offset")
    {
      m_current_offset = atof(value.c_str());
      handled = true;
    }
    else if (param == "voltage_scale")
    {
      m_voltage_scale = atof(value.c_str());
      handled = true;
    }
    else if (param == "voltage_offset")
    {
      m_voltage_offset = atof(value.c_str());
      handled = true;
    }
    else if (param == "initialize_esc")
    {
      m_initialize_esc = (tolower(value) == "true") ? true : false;
      handled = true;
    }
    else if (param == "pwm_min_us")
    {
      double v = stod(value);
      if (v >= 500.0 && v < PWM_CENTER_US)
      {
        g_pwm_min_us.store(v);
        handled = true;
      }
      else
        reportConfigWarning("pwm_min_us must be in [500, 1500)");
    }
    else if (param == "pwm_max_us")
    {
      double v = stod(value);
      if (v > PWM_CENTER_US && v <= 2500.0)
      {
        g_pwm_max_us.store(v);
        handled = true;
      }
      else
        reportConfigWarning("pwm_max_us must be in (1500, 2500]");
    }
    else if (param == "esc_arm_mode")
    {
      string v = tolower(value);
      if (v == "neutral" || v == "sweep")
      {
        m_esc_arm_mode = v;
        handled = true;
      }
      else
        reportConfigWarning("esc_arm_mode must be 'neutral' or 'sweep'");
    }
    else if (param == "esc_armed_marker")
    {
      m_esc_marker_path = value;
      handled = true;
    }
    else if (param == "disarm_on_exit")
    {
      m_disarm_on_exit = (tolower(value) == "true");
      handled = true;
    }
    else if (param == "rgbw_color")
    {
      vector<string> parts = parseString(value, ',');
      if (parts.size() == 4)
      {
        for (int i = 0; i < 4; i++)
        {
          m_active_color_quad[i] = atoi(parts[i].c_str());
        }
        handled = true;
      }
    }
    else if (param == "rolling_window_period")
    {
      m_rolling_window_seconds = stod(value);
      m_rolling_window_size = m_rolling_window_seconds * m_apptick;
      m_rolling_voltage_window = std::vector<double>(m_rolling_window_size, 0.0);
      m_rolling_current_window = std::vector<double>(m_rolling_window_size, 0.0);
      m_rolling_power_window = std::vector<double>(m_rolling_window_size, 0.0);
      handled = true;
    }
    else if (param == "debug")
    {
      m_debug = (value == tolower("true")) ? true : false;
      if (m_debug)
      {
        time_t rawtime;
        struct tm *timeinfo;
        memset(m_fname, '\0', m_fname_buff_size);
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        char fmt[m_fname_buff_size];
        memset(fmt, '\0', m_fname_buff_size);
        strftime(fmt, m_fname_buff_size, "%F_%T", timeinfo);
        snprintf(m_fname, m_fname_buff_size, "DBG_%s_%s_DATA.dbg",
                 m_app_name.c_str(), fmt);
      }
      handled = true;
    }
    // AHRS configuration parameters
    else if (param == "sample_rate")
    {
      m_sample_frequency = stod(value);
      handled = true;
    }
    else if (param == "mag_ak_cal_file")
    {
      m_ak09915_cal_file = value;
      handled = readMagCalFile(value);
    }
    else if (param == "imu_cal_file")
    {
      m_imu_cal_file = value;
      handled = readImuCalFile(value);
    }
    else if (param == "use_mag")
    {
      m_use_mag = (tolower(value) == "true");
      handled = true;
    }
    else if (param == "gain" || param == "ahrs_gain")
    {
      // Madgwick beta - no longer meaningful for the Allgeuer
      // estimator. Accepted so existing plugs don't warn-storm, but
      // flagged so it gets cleaned out of mission configs.
      reportConfigWarning("ahrs_gain (Madgwick beta) is ignored; tune "
                          "ahrs_kp / ahrs_ti instead");
      handled = true;
    }
    else if (param == "ahrs_kp")
    {
      m_ahrs_kp = stod(value);
      handled = true;
    }
    else if (param == "ahrs_ti")
    {
      m_ahrs_ti = stod(value);
      handled = true;
    }
    else if (param == "ahrs_kp_quick")
    {
      m_ahrs_kp_quick = stod(value);
      handled = true;
    }
    else if (param == "ahrs_ti_quick")
    {
      m_ahrs_ti_quick = stod(value);
      handled = true;
    }
    else if (param == "yaw_rate_clamp")
    {
      m_yaw_rate_clamp = stod(value);
      if (m_yaw_rate_clamp <= 0.0)
        m_yaw_rate_clamp = 3.0;
      handled = true;
    }
    else if (param == "roll_offset")
    {
      m_roll_offset = stod(value);
      handled = true;
    }
    else if (param == "pitch_offset")
    {
      m_pitch_offset = stod(value);
      handled = true;
    }
    else if (param == "yaw_offset")
    {
      m_yaw_offset = stod(value);
      handled = true;
    }
    else if (param == "operating_heading_offset")
    {
      m_operating_heading_offset = stod(value);
      handled = true;
    }
    else if (param == "declination_deg")
    {
      m_declination_deg = stod(value);
      handled = true;
    }
    else if (param == "ahrs_pub_suffix")
    {
      m_ahrs_pub_suffix = value;
      handled = true;
    }
    else if (param == "imu_pub_suffix")
    {
      m_imu_pub_suffix = value;
      handled = true;
    }

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  // Publish thruster pins / disarm policy / marker path to the async
  // shutdown path before any arming happens.
  g_left_pin.store(m_left_thruster_pin);
  g_right_pin.store(m_right_thruster_pin);
  g_disarm_on_exit.store(m_disarm_on_exit);
  snprintf(g_esc_marker_path, sizeof(g_esc_marker_path), "%s",
           m_esc_marker_path.c_str());

  // Surface the detected hardware once at startup.
  {
    NavVersion nv = g_nav.detected_version();
    PiVersion pv = g_nav.detected_pi();
    string hw = string("Navigator ") +
                (nv == NAV_V1 ? "V1" : nv == NAV_V2 ? "V2" : "UNKNOWN") +
                " on Raspberry Pi " + (pv == PI_5 ? "5" : "4");
    reportEvent("Detected hardware: " + hw);
    Notify("NVGR_HW_VERSION", hw);
    if (!m_last_sensor_error.empty())
      reportRunWarning("Navigator init warnings: " + m_last_sensor_error);
  }

  // Enable PWM output; arm the ESCs only on the first launch per boot.
  armIfNeeded();

  // Keep the worker threads joinable so the destructor can drive
  // a clean shutdown via m_running / m_ahrs_running flags.
  m_modulation_thread = std::thread(&BBNavigatorInterface::manageModulation, this);

  // Configure the (built-in) Allgeuer attitude estimator. Zero gain
  // params mean "keep the library defaults".
  if (m_ahrs_kp > 0.0 && m_ahrs_ti > 0.0)
    g_nav.ahrs_set_gains(m_ahrs_kp, m_ahrs_ti,
                         (m_ahrs_kp_quick > 0.0) ? m_ahrs_kp_quick : 10.0,
                         (m_ahrs_ti_quick > 0.0) ? m_ahrs_ti_quick : 1.25);
  if (!m_use_mag)
    g_nav.ahrs_set_mag_calib(0.0, 0.0, 0.0); // disables mag consideration
  g_nav.ahrs_reset(true, true);

  // Start the AHRS sensor sampling thread
  m_ahrs_running = true;
  m_sensor_thread = std::thread(&BBNavigatorInterface::sensorSamplingThread, this);
  dbg_print("AHRS sensor thread started at %.1f Hz\n", m_sample_frequency);

  dbg_print("Left thruster pin: %d\n", m_left_thruster_pin);
  dbg_print("Right thruster pin: %d\n", m_right_thruster_pin);
  dbg_print("Left thruster invert: %d\n", (int)m_left_thruster_invert);
  dbg_print("Right thruster invert: %d\n", (int)m_right_thruster_invert);
  dbg_print("Thrust command timeout: %.2f seconds\n", m_thrust_command_timeout);

  // Set the turtle to nav lights
  uint8_t rgb_array[24][4];
  memset(rgb_array, 0, sizeof(rgb_array));
  for (int i = 0; i < 12; i++)
  {
    // Left Side
    rgb_array[i][0] = m_port_side[0];
    rgb_array[i][1] = m_port_side[1];
    rgb_array[i][2] = m_port_side[2];
    rgb_array[i][3] = m_port_side[3];

    // Right Side
    rgb_array[2 * i][0] = m_starboard_side[0];
    rgb_array[2 * i][1] = m_starboard_side[1];
    rgb_array[2 * i][2] = m_starboard_side[2];
    rgb_array[2 * i][3] = m_starboard_side[3];
  }
  g_nav.neopixel_set_rgbw(rgb_array, 24);

  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void BBNavigatorInterface::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();

  Register("DESIRED_THRUST_L", 0);
  Register("DESIRED_THRUST_R", 0);
  Register("MISSION_COMPLETE", 0);
  Register("ALL_STOP", 0);

  // ESC disarm/re-arm command (operator or backseat)
  Register("NVGR_DISARM", 0);

  // Register for RC controller messages.
  //   RC_CONNECTED   - debounced link state (mode/UI logic).
  //   RC_FRAME_VALID - per-frame validity. Sharper gate for
  //                    safety-critical thrust output: drops on
  //                    a single bad SBUS frame, recovers on the
  //                    next clean one without waiting for the
  //                    debounce hysteresis to release.
  Register("RC_CONNECTED",   0);
  Register("RC_FRAME_VALID", 0);

  // Register for all RC channels
  for (int i = 1; i <= 16; i++)
  {
    Register("RC_CH" + std::to_string(i), 0);
  }

  // Runtime toggle for the RC deadman watchdog (default behavior
  // set by rc_deadman_enabled config; this lets backseat or operator
  // override at runtime, e.g. for over-the-horizon autonomy).
  Register("RC_DEADMAN_ENABLED", 0);

  // Laptop teleop (published by iTeleop)
  Register("TELEOP_ACTIVE", 0);
  Register("TELEOP_THRUST_L", 0);
  Register("TELEOP_THRUST_R", 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool BBNavigatorInterface::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "iBBNavigatorInterface (navigator-cpp)       " << endl;
  m_msgs << "============================================" << endl;

  // Snapshot atomic thrust state once for the whole report.
  const double thrust_left_snap = m_latest_set_thrust_left.load();
  const double thrust_right_snap = m_latest_set_thrust_right.load();

  {
    NavVersion nv = g_nav.detected_version();
    PiVersion pv = g_nav.detected_pi();
    m_msgs << "Hardware: Navigator "
           << (nv == NAV_V1 ? "V1" : nv == NAV_V2 ? "V2" : "UNKNOWN")
           << " / Raspberry Pi " << (pv == PI_5 ? "5" : "4") << endl;
  }
  // m_last_sensor_error is written by the sensor thread; snapshot it
  // under the AHRS mutex before streaming.
  std::string sensor_err_snap;
  {
    std::lock_guard<std::mutex> lock(m_ahrs_mutex);
    sensor_err_snap = m_last_sensor_error;
  }
  if (!sensor_err_snap.empty())
    m_msgs << "Sensor errors: " << sensor_err_snap << endl;
  m_msgs << "\n";

  ACTable actab(2);
  actab << "Thruster States | Values";
  actab.addHeaderLines();
  actab << "Current Left Thruster:" << thrust_left_snap;
  actab << "Current Right Thruster:" << thrust_right_snap;
  actab << "Thruster Dead Band:" << m_thruster_dead_band;
  actab << "Thruster Enabled:" << (m_thruster_enabled ? "true" : "false");
  actab << "Pulse Range (us):" << (doubleToString(g_pwm_min_us.load(), 0) + "-" +
                                   doubleToString(g_pwm_max_us.load(), 0));
  actab << "PWM Output Enabled:" << (m_pwm_output_enabled ? "true" : "false");
  actab << "ESC Armed This Boot:" << (m_esc_armed ? "true" : "false");
  actab << "ESC Arm Mode:" << m_esc_arm_mode;
  actab << "Disarm On Exit:" << (m_disarm_on_exit ? "true" : "false");

  // Add thrust timeout information
  actab << "Thrust Timeout Enabled:" << (m_thrust_timeout_enabled ? "true" : "false");
  actab << "Thrust Timeout (sec):" << m_thrust_command_timeout;
  double time_since_last = MOOSTime() - m_last_thrust_command_time;
  actab << "Time Since Last Command (sec):" << time_since_last;

  // Add ALL_STOP and RC control information to the report
  actab << "ALL_STOP Active:" << (m_all_stop ? "true" : "false");
  actab << "RC Frame Valid (per-frame):" << (m_rc_frame_valid ? "true" : "false");
  actab << "RC Connected (debounced):" << (m_rc_connected ? "true" : "false");
  actab << "RC Mode Active:" << (m_rc_mode ? "true" : "false");
  actab << "RC Deadman Enabled:" << (m_rc_deadman_enabled ? "true" : "false");
  actab << "RC Deadman Timeout (sec):" << m_rc_deadman_timeout;
  double rc_age_now = MOOSTime() - m_last_rc_good_time;
  actab << "RC Mail Age (sec):" << rc_age_now;
  actab << "RC Deadman Tripped:" << (m_rc_deadman_active ? "true" : "false");
  actab << "RC Channel 1 (Turning):" << m_rc_channels[0];
  actab << "RC Channel 3 (Speed):" << m_rc_channels[2];
  actab << "RC Channel 6 (Mode Switch):" << m_rc_channels[5];
  actab << "Teleop Active:" << (m_teleop_active ? "true" : "false");
  actab << "Teleop Engaged:" << (m_teleop_engaged ? "true" : "false");
  actab << "Teleop Timeout (sec):" << m_teleop_command_timeout;
  double teleop_age = MOOSTime() - m_last_teleop_time;
  actab << "Teleop Mail Age (sec):" << teleop_age;
  actab << "Teleop Thrust L (wire):" << m_teleop_thrust_left;
  actab << "Teleop Thrust R (wire):" << m_teleop_thrust_right;
  actab << "Turn Scale:" << m_turn_scale;
  actab << "Bank Angle Limit:" << m_theta_b;

  actab << "Desired Left Thrust:" << m_desired_thrust_left;
  actab << "Desired Right Thrust:" << m_desired_thrust_right;
  actab << "Filtered Left (PWM Out):" << thrust_left_snap;
  actab << "Filtered Right (PWM Out):" << thrust_right_snap;

  m_msgs << actab.getFormattedString();
  m_msgs << "\n";

  ACTable actab_adc(2);

  actab_adc << "ADC Measurements"
            << "Value";

  actab_adc.addHeaderLines();
  actab_adc << "ADC_1" << m_adc_1;
  actab_adc << "ADC_2" << m_adc_2;
  actab_adc << "ADC_3" << m_adc_3;
  actab_adc << "ADC_4" << m_adc_4;
  actab_adc << "Current" << m_latest_current;
  actab_adc << "Voltage" << m_latest_voltage;
  actab_adc << "Rolling Voltage" << m_rolling_voltage;
  actab_adc << "Rolling Current" << m_rolling_current;
  actab_adc << "Rolling Power" << m_rolling_power;
  actab_adc << "Number of Batteries" << m_num_batteries;
  actab_adc << "Leak Detected" << (m_leak_detected ? "TRUE" : "false");

  m_msgs << actab_adc.getFormattedString();
  m_msgs << "\n";

  // AHRS section
  double roll, pitch, yaw, heading;
  double gyro_x, gyro_y, gyro_z, yaw_rate;
  uint64_t imu_errors, imu_glitches;
  {
    std::lock_guard<std::mutex> lock(m_ahrs_mutex);
    roll = m_roll;
    pitch = m_pitch;
    yaw = m_yaw;
    heading = m_heading;
    gyro_x = m_gyro_x;
    gyro_y = m_gyro_y;
    gyro_z = m_gyro_z;
    yaw_rate = m_yaw_rate;
    imu_errors = m_imu_read_errors;
    imu_glitches = m_imu_glitch_count;
  }

  ACTable actab_ahrs(2);
  actab_ahrs << "AHRS State" << "Value";
  actab_ahrs.addHeaderLines();
  actab_ahrs << "Roll (deg)" << roll;
  actab_ahrs << "Pitch (deg)" << pitch;
  actab_ahrs << "Yaw (deg)" << yaw;
  actab_ahrs << "Heading (deg)" << heading;
  actab_ahrs << "Gyro X (rad/s)" << gyro_x;
  actab_ahrs << "Gyro Y (rad/s)" << gyro_y;
  actab_ahrs << "Gyro Z (rad/s)" << gyro_z;
  actab_ahrs << "Yaw Rate Lvl (rad/s)" << yaw_rate;
  actab_ahrs << "Sample Freq (Hz)" << m_sample_frequency;
  actab_ahrs << "Mag Enabled" << (m_use_mag ? "true" : "false");
  actab_ahrs << "IMU Read Errors" << (double)imu_errors;
  actab_ahrs << "IMU Glitches Rejected" << (double)imu_glitches;
  actab_ahrs << "AHRS Running" << (m_ahrs_running ? "true" : "false");

  m_msgs << actab_ahrs.getFormattedString();

  return (true);
}

//---------------------------------------------------------
// AHRS Methods
//---------------------------------------------------------

bool BBNavigatorInterface::readMagCalFile(std::string filename)
{
  std::ifstream file(filename);
  if (!file.is_open())
  {
    reportRunWarning("Could not open calibration file: " + filename);
    dbg_print("Could not open file\n");
    return false;
  }

  std::string line;
  while (std::getline(file, line))
  {
    if (line.empty() || line[0] == '#')
      continue;

    string arg = biteString(line, '=');
    arg = removeWhite(arg);

    if (arg == "b")
    {
      std::vector<string> values = parseString(line, ',');
      if (values.size() >= 3)
        for (int i = 0; i < 3; i++)
          m_mag_bias[i] = stod(values[i]);
    }
    else if (arg == "A")
    {
      std::vector<string> values = parseString(line, ',');
      if (values.size() >= 9)
        for (int i = 0; i < 9; i++)
          m_mag_scale[i] = stod(values[i]);
    }
  }

  file.close();
  m_have_mag_cal = true;
  return true;
}

// Accepts both cal-file formats:
//   gyro_bias  = x,y,z   /  accel_bias = x,y,z   (interface format)
//   bias_x = v / bias_y = v / bias_z = v          (calibration-script format)
// The second form fixes the long-standing mismatch where the gyro cal
// script's output couldn't be parsed and the bias silently stayed zero.
bool BBNavigatorInterface::readImuCalFile(std::string filename)
{
  std::ifstream file(filename);
  if (!file.is_open())
  {
    reportRunWarning("Could not open IMU calibration file: " + filename);
    return false;
  }

  std::string line;
  for (int i = 0; i < 3; i++)
  {
    m_gyro_bias[i] = 0.0;
    m_accel_bias[i] = 0.0;
  }

  while (std::getline(file, line))
  {
    if (line.empty() || line[0] == '#')
      continue;

    string arg = biteString(line, '=');
    arg = removeWhite(arg);
    string rest = stripBlankEnds(line);

    if (arg == "gyro_bias")
    {
      std::vector<string> values = parseString(rest, ',');
      if (values.size() >= 3)
        for (int i = 0; i < 3; i++)
          m_gyro_bias[i] = stod(values[i]);
    }
    else if (arg == "accel_bias")
    {
      std::vector<string> values = parseString(rest, ',');
      if (values.size() >= 3)
        for (int i = 0; i < 3; i++)
          m_accel_bias[i] = stod(values[i]);
    }
    else if (arg == "bias_x")
      m_gyro_bias[0] = stod(rest);
    else if (arg == "bias_y")
      m_gyro_bias[1] = stod(rest);
    else if (arg == "bias_z")
      m_gyro_bias[2] = stod(rest);
  }

  file.close();
  return true;
}

//---------------------------------------------------------
// Small fixed-size linear algebra (replaces Armadillo).

static void mat3_mult(const double A[9], const double B[9], double C[9])
{
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      C[3 * r + c] = A[3 * r + 0] * B[0 + c] +
                     A[3 * r + 1] * B[3 + c] +
                     A[3 * r + 2] * B[6 + c];
}

static void mat3_vec(const double A[9], const double v[3], double out[3])
{
  for (int r = 0; r < 3; r++)
    out[r] = A[3 * r] * v[0] + A[3 * r + 1] * v[1] + A[3 * r + 2] * v[2];
}

void BBNavigatorInterface::sensorSamplingThread()
{
  auto prev_time = std::chrono::high_resolution_clock::now();

  double roll_offset_rad = m_roll_offset * M_PI / 180;
  double pitch_offset_rad = m_pitch_offset * M_PI / 180;
  double yaw_offset_rad = m_yaw_offset * M_PI / 180;

  // Mounting-offset rotation R_offset = R_yaw * R_pitch * R_roll
  const double R_roll[9] = {1, 0, 0,
                            0, cos(roll_offset_rad), -sin(roll_offset_rad),
                            0, sin(roll_offset_rad), cos(roll_offset_rad)};
  const double R_pitch[9] = {cos(pitch_offset_rad), 0, sin(pitch_offset_rad),
                             0, 1, 0,
                             -sin(pitch_offset_rad), 0, cos(pitch_offset_rad)};
  const double R_yaw[9] = {cos(yaw_offset_rad), -sin(yaw_offset_rad), 0,
                           sin(yaw_offset_rad), cos(yaw_offset_rad), 0,
                           0, 0, 1};
  double R_tmp[9], R_offset[9];
  mat3_mult(R_pitch, R_roll, R_tmp);
  mat3_mult(R_yaw, R_tmp, R_offset);

  const double nominal_dt = 1.0 / m_sample_frequency;

  while (m_ahrs_running)
  {
    auto current_time = std::chrono::high_resolution_clock::now();

    // Read sensor data from Navigator. navigator-cpp returns error
    // strings instead of throwing; count failures and skip the fusion
    // step for incomplete samples so a transient bus error can never
    // inject zeros into the estimator.
    NavAxisData accel_raw, gyro_raw;
    std::string err_a = g_nav.read_accel(accel_raw);
    std::string err_g = g_nav.read_gyro(gyro_raw);

    NavAxisData mag_raw;
    bool mag_ok = false;
    if (m_use_mag)
      mag_ok = g_nav.read_mag_ak09915(mag_raw).empty();

    // Plausibility screen. Corrupted SPI transactions (observed under
    // bus contention on zoe) return SUCCESSFULLY with full-scale
    // register patterns - e.g. gyro exactly +/-250 dps (4.3657 rad/s),
    // accel exactly +/-2 g (19.61 m/s^2) or all-zero - so an error
    // check alone cannot catch them. Reject any sample whose accel
    // norm is non-physical or whose gyro sits at/beyond the +/-250 dps
    // configured full scale (a surface boat never legitimately gets
    // there); a bad sample must never reach the estimator or the
    // published gyro.
    bool sample_ok = err_a.empty() && err_g.empty();
    if (sample_ok)
    {
      const double anorm = sqrt((double)accel_raw.x * accel_raw.x +
                                (double)accel_raw.y * accel_raw.y +
                                (double)accel_raw.z * accel_raw.z);
      const double gyro_fs = 4.3;  // just under 250 dps full scale, rad/s
      if (anorm < 2.0 || anorm > 25.0 ||
          fabs(accel_raw.x) > 19.0 || fabs(accel_raw.y) > 19.0 ||
          fabs(accel_raw.z) > 19.0 ||
          fabs(gyro_raw.x) > gyro_fs || fabs(gyro_raw.y) > gyro_fs ||
          fabs(gyro_raw.z) > gyro_fs)
      {
        sample_ok = false;
        std::lock_guard<std::mutex> lock(m_ahrs_mutex);
        m_imu_glitch_count++;
      }
    }

    if (sample_ok)
    {
      // Apply calibrations (bias subtract) in the sensor frame
      double gyro_cal[3] = {gyro_raw.x - m_gyro_bias[0],
                            gyro_raw.y - m_gyro_bias[1],
                            gyro_raw.z - m_gyro_bias[2]};
      double acc_cal[3] = {accel_raw.x - m_accel_bias[0],
                           accel_raw.y - m_accel_bias[1],
                           accel_raw.z - m_accel_bias[2]};

      // Apply body frame rotation
      double gyro_bff[3], acc_bff[3];
      mat3_vec(R_offset, gyro_cal, gyro_bff);
      mat3_vec(R_offset, acc_cal, acc_bff);

      double mag_bff[3] = {0.0, 0.0, 0.0};
      if (m_use_mag && mag_ok)
      {
        double mag_c[3] = {mag_raw.x - m_mag_bias[0],
                           mag_raw.y - m_mag_bias[1],
                           mag_raw.z - m_mag_bias[2]};
        double mag_cal[3];
        mat3_vec(m_mag_scale, mag_c, mag_cal);
        mat3_vec(R_offset, mag_cal, mag_bff);
      }

      // Coerce dt: wall-clock hiccups (scheduling, NTP steps) must not
      // reach the integrator. Allgeuer recommends clamping to a band
      // around the nominal period.
      double dt = std::chrono::duration<double>(
                      current_time - prev_time).count();
      if (dt < 0.5 * nominal_dt) dt = 0.5 * nominal_dt;
      if (dt > 3.0 * nominal_dt) dt = 3.0 * nominal_dt;

      g_nav.ahrs_update(dt,
                        gyro_bff[0], gyro_bff[1], gyro_bff[2],
                        acc_bff[0], acc_bff[1], acc_bff[2],
                        mag_bff[0], mag_bff[1], mag_bff[2]);

      NavAttitudeData att;
      g_nav.ahrs_get_attitude(att);

      {
        std::lock_guard<std::mutex> lock(m_ahrs_mutex);
        m_roll = att.roll * 180.0 / M_PI;
        m_pitch = att.pitch * 180.0 / M_PI;
        m_yaw = att.yaw * 180.0 / M_PI;
        m_heading = fmod((m_yaw + m_declination_deg + m_operating_heading_offset), 360.0);
        if (m_heading < 0)
          m_heading += 360.0;

        m_qw = att.qw;
        m_qx = att.qx;
        m_qy = att.qy;
        m_qz = att.qz;

        m_gyro_x = gyro_bff[0];
        m_gyro_y = gyro_bff[1];
        m_gyro_z = gyro_bff[2];

        m_accel_x = acc_bff[0];
        m_accel_y = acc_bff[1];
        m_accel_z = acc_bff[2];

        // Level-frame yaw rate: rotate the body angular velocity into
        // the world frame with the attitude quaternion and take the z
        // component. Unlike the Euler yaw-rate projection
        // (q*sin(phi)+r*cos(phi))/cos(theta), this is bounded by
        // |omega| for ANY attitude - there is no cos(pitch)
        // singularity to guard, which was failure mode B of the
        // NAV_HEADING spike investigation. Third row of R(q):
        const double zr0 = 2.0 * (m_qx * m_qz - m_qw * m_qy);
        const double zr1 = 2.0 * (m_qy * m_qz + m_qw * m_qx);
        const double zr2 = 1.0 - 2.0 * (m_qx * m_qx + m_qy * m_qy);
        double yaw_rate = zr0 * gyro_bff[0] + zr1 * gyro_bff[1] +
                          zr2 * gyro_bff[2];

        // Physical plausibility clamp (belt and suspenders - the
        // projection above is already bounded by the gyro range).
        rclamp(yaw_rate, -m_yaw_rate_clamp, m_yaw_rate_clamp);
        m_yaw_rate = yaw_rate;
      }
    }
    else if (!err_a.empty() || !err_g.empty())
    {
      std::lock_guard<std::mutex> lock(m_ahrs_mutex);
      m_imu_read_errors++;
      m_last_sensor_error = !err_a.empty() ? err_a : err_g;
    }

    // Sleep to maintain sample rate
    auto elapsed_time = std::chrono::duration<double>(current_time - prev_time);
    prev_time = current_time;
    auto sleep_duration = std::chrono::duration<double>(1.0 / m_sample_frequency) - elapsed_time;
    if (sleep_duration > std::chrono::duration<double>(0))
      std::this_thread::sleep_for(sleep_duration);
  }
}
