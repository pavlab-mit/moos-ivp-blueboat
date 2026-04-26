/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: iBBNavigatorInterface/BBNavigatorInterface.cpp
   Last Ed:  2025-03-30
     Brief:
        Combined Navigator Interface for Blueboat ASV.
        Handles dual thruster PWM control with safety features
        and AHRS using Madgwick filter.
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
#include <numeric>

using namespace std;
using namespace arma;

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

void safePwmShutdown()
{
  // Run at 50 Hz sending zero signal for about 2 seconds
  for (int i = 0; i < 100; ++i)
  {
    BBNavigatorInterface::setPinPulseWidth(PwmChannel::Ch14, 0);
    BBNavigatorInterface::setPinPulseWidth(PwmChannel::Ch16, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // Ensure the PWM values have been applied before disabling PWM
  // set_pwm_enable(false);
}

void signalHandler(int signum)
{
  safePwmShutdown();
  exit(signum);
};

// ETHAN CHANGE #1 START
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

// ETHAN CHANGE #1 END

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
  m_rc_connected = false;
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

  set_rgb_led_strip_size(24);
  init();

  m_left_thruster_pin = PwmChannel::Ch14;
  m_right_thruster_pin = PwmChannel::Ch16;

  m_thruster_enabled = true;
  m_left_thruster_invert = 1;
  m_right_thruster_invert = 1;

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
  m_beta = 0.2;
  m_ahrs = Madgwick(m_beta, m_sample_frequency);
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
  m_qw = 1.0; m_qx = 0.0; m_qy = 0.0; m_qz = 0.0;

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
  setPinPulseWidth(m_left_thruster_pin, 0);
  setPinPulseWidth(m_right_thruster_pin, 0);

  // Ensure the PWM values have been applied
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // set_pwm_enable(false);

  m_led_color_quad = {0, 0, 0, 0};
    uint8_t rgb_array[24][4];
    for (int i = 0; i < 24; i++)
    {
      rgb_array[i][0] = m_led_color_quad[0];
      rgb_array[i][1] = m_led_color_quad[1];
      rgb_array[i][2] = m_led_color_quad[2];
      rgb_array[i][3] = m_led_color_quad[3];
    }
  set_neopixel_rgbw(rgb_array, 24);

  dbg_print("Proper shutdown\n");
}

// Initialize multiple ESCs simultaneously in parallel
void BBNavigatorInterface::initializeESCs(const std::vector<PwmChannel> &pins)
{
  if (pins.empty())
  {
    return;
  }

  // Set all to Maximum Throttle
  for (const auto &pin : pins)
  {
    setPinPulseWidth(pin, 100);
  }
  this_thread::sleep_for(chrono::milliseconds(500)); // Wait for 0.5 seconds for all pins

  // Set all to Minimum Throttle
  for (const auto &pin : pins)
  {
    setPinPulseWidth(pin, -100);
  }
  this_thread::sleep_for(chrono::milliseconds(500)); // Wait for 0.5 seconds for all pins

  // Set all to Neutral Throttle
  for (const auto &pin : pins)
  {
    setPinPulseWidth(pin, 0);
  }
  this_thread::sleep_for(chrono::milliseconds(250)); // Wait for 0.25 seconds for all pins

  dbg_print("Initialized %d ESCs in parallel\n", pins.size());
}

void BBNavigatorInterface::initializeESC(PwmChannel pin)
{
  // Set to Maximum Throttle
  setPinPulseWidth(pin, 100);                        // Assuming 100% corresponds to 2000 microseconds
  this_thread::sleep_for(chrono::milliseconds(500)); // Wait for 0.5 seconds

  // Set to Minimum Throttle
  setPinPulseWidth(pin, -100);                       // Assuming 0% corresponds to 1000 microseconds
  this_thread::sleep_for(chrono::milliseconds(500)); // Wait for 0.5 seconds

  // Optional: Set to Neutral Throttle (e.g., for ESCs that need this)
  setPinPulseWidth(pin, 0);                          // Assuming 50% corresponds to 1500 microseconds
  this_thread::sleep_for(chrono::milliseconds(250)); // Wait for 0.25 second

  // The ESC should now be initialized and ready to accept normal operational commands
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
      exit(0);
    }
    else if (key == "ALL_STOP")
    {
      m_all_stop = (msg.GetString() == "true");
      dbg_print("ALL_STOP received: %s\n", m_all_stop ? "true" : "false");
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
    else if (key.substr(0, 5) == "RC_CH" && key.length() > 5)
    {
      // Extract channel number from key (e.g., "RC_CH1" -> 1)
      int channel = std::stoi(key.substr(5)) - 1; // Zero-indexed
      if (channel >= 0 && channel < 16)
      {
        m_rc_channels[channel] = msg.GetDouble();
        // iRCReader only publishes RC_CH* when it considers RC
        // connected, so any RC_CH* mail counts as a "good" RC tick.
        m_last_rc_good_time = MOOSTime();

        // Check mode switch (Channel 6)
        if (channel == 5) // Channel 6 (zero-indexed as 5)
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
  // Only calculate RC thrust if RC is connected and in RC mode
  if (m_rc_connected && m_rc_mode)
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

  // Check for thrust command timeout if enabled
  if (m_thrust_timeout_enabled && m_thrust_command_timeout > 0)
  {
    double time_since_last_command = MOOSTime() - m_last_thrust_command_time;

    if (time_since_last_command > m_thrust_command_timeout && !m_rc_mode)
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

  // Check for ALL_STOP condition - only affects autonomous control
  if (m_all_stop && !m_rc_mode)
  {
    m_desired_thrust_left = 0;
    m_desired_thrust_right = 0;
    dbg_print("ALL_STOP active - setting autonomous thrusts to zero\n");
  }

  // If in RC mode and RC is connected, calculate thrust from RC inputs
  if (m_rc_mode && m_rc_connected)
  {
    calculateRCThrust();
  }
  // If RC mode but controller disconnected, set thrusts to zero
  else if (m_rc_mode && !m_rc_connected)
  {
    m_desired_thrust_left = 0;
    m_desired_thrust_right = 0;
    dbg_print("RC disconnected - setting thrusts to zero\n");
  }

  // RC deadman watchdog (final override). When enabled, requires
  // a "good" RC tick (RC_CONNECTED=true or any RC_CH* mail) within
  // m_rc_deadman_timeout seconds. Triggers in BOTH RC and autonomous
  // mode - treats the RC link as a vehicle-side deadman. Disable via
  // rc_deadman_enabled=false (config) or RC_DEADMAN_ENABLED=false
  // (runtime) for over-the-horizon missions where RC range loss is
  // expected.
  bool deadman_was_active = m_rc_deadman_active;
  m_rc_deadman_active = false;
  if (m_rc_deadman_enabled)
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
  ADCData adc = read_adc_all();

  m_adc_1 = adc.channel[0];
  m_adc_2 = adc.channel[1];
  m_adc_3 = adc.channel[2];
  m_adc_4 = adc.channel[3];
  // ETHAN CHANGE #3 START
  // m_latest_current = (m_adc_3 - m_current_offset) * m_current_scale;
  CalibrationParams cal = BATTERY_CALIBRATIONS[m_num_batteries - 1];
  m_latest_current = (m_adc_3 - cal.offset) * cal.gain;
  // ETHAN CHANGE #3 ENDS
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

  // Publish RC control status
  Notify("NVGR_RC_MODE", m_rc_mode ? "true" : "false");
  Notify("NVGR_RC_CONNECTED", m_rc_connected ? "true" : "false");

  // IPT Sensing
  // Read temperature and pressure values
  m_nav_temp = read_temp();
  m_nav_pressure = read_pressure();
  m_rpi_temp = rpi_utils::getCPUTemperature();

  // Publish the values
  Notify("NVGTR_IT_C", m_nav_temp);
  Notify("NVGTR_IP_KPA", m_nav_pressure);
  Notify("RPI_TEMP", m_rpi_temp);

  // AHRS output publishing
  double roll, pitch, yaw, heading;
  double gyro_x, gyro_y, gyro_z, yaw_rate;
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
  }

  Notify(ahrsName("NAV_ROLL"), roll);
  Notify(ahrsName("NAV_PITCH"), pitch);
  Notify(ahrsName("NAV_YAW"), yaw);
  Notify(ahrsName("NAV_HEADING"), heading);
  Notify(imuName("GYRO_X"), gyro_x);
  Notify(imuName("GYRO_Y"), gyro_y);
  Notify(imuName("GYRO_Z"), gyro_z);
  Notify(imuName("GYRO_Z_LVL"), yaw_rate);

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
    string value = line;

    bool handled = false;
    if (param == "left_thruster_pin")
    {
      m_left_thruster_pin = PwmChannel(stoi(value) - 1);
      handled = true;
    }
    else if (param == "right_thruster_pin")
    {
      m_right_thruster_pin = PwmChannel(stoi(value) - 1);
      handled = true;
    }
    //ETHAN CHANGE #2 STARTS
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
    //ETHAN CHANGE #2 ENDS
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
        memset(m_fname, m_fname_buff_size, '\0');
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        char fmt[m_fname_buff_size];
        memset(fmt, m_fname_buff_size, '\0');
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
      handled = readMagCalFile(value, m_mag_ak_bias, m_mag_ak_scaling_matrix);
    }
    else if (param == "imu_cal_file")
    {
      m_imu_cal_file = value;
      handled = readImuCalFile(value);
    }
    else if (param == "gain" || param == "ahrs_gain")
    {
      m_beta = stod(value);
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

  // Use the new bulk initialization method if available

  if (m_initialize_esc)
  {
    set_pwm_freq_hz(static_cast<float>(PWM_FREQ_HZ));
    set_pwm_enable(true);
    initializeESC(m_left_thruster_pin);
    initializeESC(m_right_thruster_pin);
    dbg_print("ESC initialization performed\n");
  }
  else
  {
    dbg_print("ESC initialization skipped (disabled in config)\n");
  }

  // Keep the worker threads joinable so the destructor can drive
  // a clean shutdown via m_running / m_ahrs_running flags.
  m_modulation_thread = std::thread(&BBNavigatorInterface::manageModulation, this);

  // Reinitialize AHRS with configured parameters
  m_ahrs = Madgwick(m_beta, m_sample_frequency);

  // Start the AHRS sensor sampling thread
  m_ahrs_running = true;
  m_sensor_thread = std::thread(&BBNavigatorInterface::sensorSamplingThread, this);
  dbg_print("AHRS sensor thread started at %.1f Hz\n", m_sample_frequency);

  dbg_print("Left thruster pin: %d\n", m_left_thruster_pin);
  dbg_print("Right thruster pin: %d\n", m_right_thruster_pin);
  dbg_print("Left thruster invert: %d\n", m_left_thruster_invert);
  dbg_print("Right thruster invert: %d\n", m_right_thruster_invert);
  dbg_print("Thrust command timeout: %.2f seconds\n", m_thrust_command_timeout);

  // Set the turttle to nav lights
  uint8_t rgb_array[24][4];
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
  set_neopixel_rgbw(rgb_array, 24);

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

  // Register for RC controller messages
  Register("RC_CONNECTED", 0);

  // Register for all RC channels
  for (int i = 1; i <= 16; i++)
  {
    Register("RC_CH" + std::to_string(i), 0);
  }

  // Runtime toggle for the RC deadman watchdog (default behavior
  // set by rc_deadman_enabled config; this lets backseat or operator
  // override at runtime, e.g. for over-the-horizon autonomy).
  Register("RC_DEADMAN_ENABLED", 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool BBNavigatorInterface::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  // Snapshot atomic thrust state once for the whole report.
  const double thrust_left_snap = m_latest_set_thrust_left.load();
  const double thrust_right_snap = m_latest_set_thrust_right.load();

  ACTable actab(2);
  actab << "Thruster States | Values";
  actab.addHeaderLines();
  actab << "Current Left Thruster:" << thrust_left_snap;
  actab << "Current Right Thruster:" << thrust_right_snap;
  actab << "Thruster Dead Band:" << m_thruster_dead_band;
  actab << "Thruster Enabled:" << (m_thruster_enabled ? "true" : "false");
  actab << "ESC Enabled:" << (m_initialize_esc ? "true" : "false");

  // Add thrust timeout information
  actab << "Thrust Timeout Enabled:" << (m_thrust_timeout_enabled ? "true" : "false");
  actab << "Thrust Timeout (sec):" << m_thrust_command_timeout;
  double time_since_last = MOOSTime() - m_last_thrust_command_time;
  actab << "Time Since Last Command (sec):" << time_since_last;

  // Add ALL_STOP and RC control information to the report
  actab << "ALL_STOP Active:" << (m_all_stop ? "true" : "false");
  actab << "RC Connected:" << (m_rc_connected ? "true" : "false");
  actab << "RC Mode Active:" << (m_rc_mode ? "true" : "false");
  actab << "RC Deadman Enabled:" << (m_rc_deadman_enabled ? "true" : "false");
  actab << "RC Deadman Timeout (sec):" << m_rc_deadman_timeout;
  double rc_age_now = MOOSTime() - m_last_rc_good_time;
  actab << "RC Mail Age (sec):" << rc_age_now;
  actab << "RC Deadman Tripped:" << (m_rc_deadman_active ? "true" : "false");
  actab << "RC Channel 1 (Turning):" << m_rc_channels[0];
  actab << "RC Channel 3 (Speed):" << m_rc_channels[2];
  actab << "RC Channel 6 (Mode Switch):" << m_rc_channels[5];
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

  m_msgs << actab_adc.getFormattedString();
  m_msgs << "\n";

  // AHRS section
  double roll, pitch, yaw, heading;
  double gyro_x, gyro_y, gyro_z, yaw_rate;
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
  actab_ahrs << "Yaw Rate (rad/s)" << yaw_rate;
  actab_ahrs << "Sample Freq (Hz)" << m_sample_frequency;
  actab_ahrs << "AHRS Running" << (m_ahrs_running ? "true" : "false");

  m_msgs << actab_ahrs.getFormattedString();

  return (true);
}

//---------------------------------------------------------
// AHRS Methods
//---------------------------------------------------------

bool BBNavigatorInterface::readMagCalFile(std::string filename, arma::vec &mag_bias, arma::mat &mag_transform)
{
  std::ifstream file(filename);
  if (!file.is_open())
  {
    reportRunWarning("Could not open calibration file: " + filename);
    dbg_print("Could not open file\n");
    return false;
  }

  std::string line;
  mag_bias.resize(3);
  mag_transform.resize(3, 3);

  while (std::getline(file, line))
  {
    if (line.empty() || line[0] == '#')
      continue;

    string arg = biteString(line, '=');
    arg = removeWhite(arg);

    if (arg == "b")
    {
      std::vector<string> values = parseString(line, ',');
      for (int i = 0; i < 3; i++)
        mag_bias[i] = stod(values[i]);
    }
    else if (arg == "A")
    {
      std::vector<string> values = parseString(line, ',');
      for (int i = 0; i < 9; i++)
        mag_transform[i] = stod(values[i]);
    }
  }

  file.close();
  return true;
}

bool BBNavigatorInterface::readImuCalFile(std::string filename)
{
  std::ifstream file(filename);
  if (!file.is_open())
  {
    reportRunWarning("Could not open IMU calibration file: " + filename);
    return false;
  }

  std::string line;
  m_gyro_bias.resize(3);
  m_accel_bias.resize(3);
  m_gyro_bias.zeros();
  m_accel_bias.zeros();

  while (std::getline(file, line))
  {
    if (line.empty() || line[0] == '#')
      continue;

    string arg = biteString(line, '=');
    arg = removeWhite(arg);

    if (arg == "gyro_bias")
    {
      std::vector<string> values = parseString(line, ',');
      if (values.size() >= 3)
        for (int i = 0; i < 3; i++)
          m_gyro_bias[i] = stod(values[i]);
    }
    else if (arg == "accel_bias")
    {
      std::vector<string> values = parseString(line, ',');
      if (values.size() >= 3)
        for (int i = 0; i < 3; i++)
          m_accel_bias[i] = stod(values[i]);
    }
  }

  file.close();
  return true;
}

arma::vec BBNavigatorInterface::adjust_gyro_cal(const arma::vec &gyro_raw)
{
  if (m_gyro_bias.n_elem == 3)
    return gyro_raw - m_gyro_bias;
  return gyro_raw;
}

arma::vec BBNavigatorInterface::adjust_acc_cal(const arma::vec &acc_raw)
{
  if (m_accel_bias.n_elem == 3)
    return acc_raw - m_accel_bias;
  return acc_raw;
}

double BBNavigatorInterface::calculateYawRate(double roll_phi, double pitch_theta, double p, double q, double r)
{
  roll_phi = roll_phi * M_PI / 180.0;
  pitch_theta = pitch_theta * M_PI / 180.0;

  const double sphi = sin(roll_phi);
  const double cphi = cos(roll_phi);
  const double cth = cos(pitch_theta);

  const double eps = 1e-9;
  if (fabs(cth) < eps)
    return (q * sphi + r * cphi) * (cth >= 0 ? 1.0 / eps : -1.0 / eps);
  return (q * sphi + r * cphi) / cth;
}

void BBNavigatorInterface::sensorSamplingThread()
{
  auto prev_time = std::chrono::high_resolution_clock::now();

  double roll_offset_rad = m_roll_offset * M_PI / 180;
  double pitch_offset_rad = m_pitch_offset * M_PI / 180;
  double yaw_offset_rad = m_yaw_offset * M_PI / 180;

  // Rotation matrices for offset
  arma::mat R_roll = {{1, 0, 0},
                      {0, cos(roll_offset_rad), -sin(roll_offset_rad)},
                      {0, sin(roll_offset_rad), cos(roll_offset_rad)}};

  arma::mat R_pitch = {{cos(pitch_offset_rad), 0, sin(pitch_offset_rad)},
                       {0, 1, 0},
                       {-sin(pitch_offset_rad), 0, cos(pitch_offset_rad)}};

  arma::mat R_yaw = {{cos(yaw_offset_rad), -sin(yaw_offset_rad), 0},
                     {sin(yaw_offset_rad), cos(yaw_offset_rad), 0},
                     {0, 0, 1}};

  arma::mat R_offset = R_yaw * R_pitch * R_roll;

  while (m_ahrs_running)
  {
    auto current_time = std::chrono::high_resolution_clock::now();

    try
    {
      // Read sensor data from Navigator
      AxisData mag_1 = read_mag();
      AxisData imu = read_accel();
      AxisData gyro = read_gyro();

      arma::vec gyro_data_raw = {gyro.x, gyro.y, gyro.z};
      arma::vec acc_data_raw = {imu.x, imu.y, imu.z};
      arma::vec mag_1_data_raw = {mag_1.x, mag_1.y, mag_1.z};

      // Apply calibrations
      arma::vec gyro_data_cal = adjust_gyro_cal(gyro_data_raw);
      arma::vec acc_data_cal = adjust_acc_cal(acc_data_raw);
      arma::vec mag_1_data_cal = m_mag_ak_scaling_matrix * (mag_1_data_raw - m_mag_ak_bias);

      // Apply body frame rotation
      arma::vec gyro_bff = R_offset * gyro_data_cal;
      arma::vec acc_bff = R_offset * acc_data_cal;
      arma::vec mag_1_bff = R_offset * mag_1_data_cal;

      // Update AHRS
      {
        std::lock_guard<std::mutex> lock(m_ahrs_mutex);
        m_ahrs.update(gyro_bff[0], gyro_bff[1], gyro_bff[2],
                      acc_bff[0], acc_bff[1], acc_bff[2],
                      mag_1_bff[0], mag_1_bff[1], mag_1_bff[2]);

        m_roll = m_ahrs.getRoll();
        m_pitch = m_ahrs.getPitch();
        m_yaw = m_ahrs.getYaw();
        m_heading = fmod((m_yaw + m_declination_deg + m_operating_heading_offset), 360.0);
        if (m_heading < 0)
          m_heading += 360.0;

        m_qw = m_ahrs.getQ0();
        m_qx = m_ahrs.getQ1();
        m_qy = m_ahrs.getQ2();
        m_qz = m_ahrs.getQ3();

        m_gyro_x = gyro_bff[0];
        m_gyro_y = gyro_bff[1];
        m_gyro_z = gyro_bff[2];

        const double p = gyro_bff[0];
        const double q = gyro_bff[1];
        const double r = gyro_bff[2];
        const double roll_rad = m_roll * M_PI / 180.0;
        const double pitch_rad = m_pitch * M_PI / 180.0;
        const double sphi = sin(roll_rad);
        const double cphi = cos(roll_rad);
        const double cth = cos(pitch_rad);
        const double eps = 1e-6;

        if (fabs(cth) < eps)
          m_yaw_rate = (q * sphi + r * cphi) * (cth >= 0 ? 1.0 / eps : -1.0 / eps);
        else
          m_yaw_rate = (q * sphi + r * cphi) / cth;
      }
    }
    catch (const std::exception &e)
    {
      reportRunWarning("Sensor sampling error: " + std::string(e.what()));
    }

    // Sleep to maintain sample rate
    auto elapsed_time = std::chrono::duration<double>(current_time - prev_time);
    prev_time = current_time;
    auto sleep_duration = std::chrono::duration<double>(1.0 / m_sample_frequency) - elapsed_time;
    if (sleep_duration > std::chrono::duration<double>(0))
      std::this_thread::sleep_for(sleep_duration);
  }
}
