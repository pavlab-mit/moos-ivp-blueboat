/*************************************************************
      Name: Raymond Turrisi (orig.), Jeremy Wenger (navigator-cpp port)
      Orgn: MIT, Cambridge MA
      File: iBBNavigatorInterface/BBNavigatorInterface.h
   Last Ed:  2026-07-24
     Brief:
        Unified Navigator Interface for the BlueBoat ASV, built on
        navigator-cpp (pavlab-mit). Replaces the navigator-lib
        (Blue Robotics, Rust) based iBBNavigatorInterface_v1/_v2
        pair with a single app: navigator-cpp auto-detects the
        Navigator board revision (V1/BMP280 vs V2/BMP390) and the
        Raspberry Pi model (4 vs 5) at runtime.

        Owns:
          - Dual-thruster PWM (left/right ESCs) with per-launch
            neutral-hold arming, command watchdogs, and safe
            shutdown / signal-cut paths
          - AHRS via the vendored Allgeuer/NimbRo passive
            complementary filter (replaces Madgwick)
          - Raw gyro / level-frame yaw-rate publication
            (singularity-free world-frame projection)
          - ADC power monitoring (battery V / I)
          - Internal pressure / temperature (baro)
          - Leak detector
          - Navigator-board NeoPixel LEDs
          - RC / teleop / autonomy thrust arbitration
*************************************************************/

#ifndef BBNavigatorInterface_HEADER
#define BBNavigatorInterface_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <string>
#include <cstdint>
#include <cstdarg> //va_list, va_start, va_end
#include "nav_bindings.h"
#include <thread>
#include <atomic>
#include <mutex>

class LPF
{
private:
  double alpha;
  double previousOutput;

public:
  LPF()
  {
    alpha = 0.1;
    /*
      alpha = dt/(tau + dt)
      if we want a fin to reach 99% of the 1 seconds, consider five time constants, where tau = 0.2; if we sample at 50 hz, then dt = 0.02, and alpha = ~0.67
    */
    previousOutput = 0;
  }

  LPF(double alpha)
  {
    this->alpha = alpha;
    previousOutput = 0;
  }

  double update(double desired_state, double dt)
  {
    double output = alpha * desired_state + (1 - alpha) * previousOutput;
    previousOutput = output;
    return output;
  }

  void reset(double setState = 0.0)
  {
    previousOutput = setState;
  }
};

class BBNavigatorInterface : public AppCastingMOOSApp
{
public:
  BBNavigatorInterface();
  ~BBNavigatorInterface();

  // PWM frequency and default pulse range (100 Hz). The min/max are
  // runtime-configurable via pwm_min_us / pwm_max_us: the defaults
  // preserve the legacy 800-2200 mapping, but the BlueBoat's Basic
  // ESC 500 documents an 1100-1900 us input range - see the mission
  // plug for the tradeoff (changing the range changes the thrust
  // curve: with 800-2200, commands beyond ~57% already saturate the
  // ESC's documented range).
  static constexpr double PWM_FREQ_HZ = 100.0;      // 100 Hz for smoother response
  static constexpr double PWM_MIN_US = 800.0;       // Default min pulse width (microseconds)
  static constexpr double PWM_MAX_US = 2200.0;      // Default max pulse width (microseconds)
  static constexpr double PWM_CENTER_US = 1500.0;   // Neutral pulse width (microseconds)

  // navigator-cpp PWM channels are 0-based indices (legacy ChN -> index N-1).
  static constexpr int kPwmIndexCh14 = 13;
  static constexpr int kPwmIndexCh16 = 15;

  // Convert normalized command [-100,100] to a pulse width and drive
  // the PCA9685 through the shared Navigator instance. Static so the
  // signal-handler shutdown path can use it without an object.
  static void setPinPulseWidth(int pin_num, double target);

protected: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();
  bool buildReport();

protected:
  void registerVariables();
  bool dbg_print(const char *format, ...);

  // Compose published variable name with the appropriate suffix.
  std::string ahrsName(const std::string &base) const;
  std::string imuName(const std::string &base) const;

  void manageModulation();

  // RC thrust calculation
  void calculateRCThrust();

  // ESC lifecycle. armIfNeeded() enables PWM and runs the neutral-
  // hold arm sequence (every launch). requestDisarm() cuts the PWM
  // signal (OE high): the ESCs stop on signal loss; re-arming costs
  // a fresh armIfNeeded() neutral hold.
  void armIfNeeded();
  void requestDisarm(const std::string &reason);

  // Helper function to compute heading error mixer similar to DiffThrustPID
  double calculateHeadingMixer(double desired_heading, double current_heading);

  std::thread m_modulation_thread;
  std::thread m_sensor_thread;

  // AHRS methods
  void sensorSamplingThread();
  bool readMagCalFile(std::string filepath);
  bool readImuCalFile(std::string filepath);

private: // Configuration variables
  bool m_debug;
  FILE *m_debug_stream;
  static const uint16_t m_fname_buff_size = 256;
  std::string m_app_name;
  char m_fname[m_fname_buff_size];

  double m_min_thrust;
  double m_max_thrust;
  double m_thruster_dead_band;

  bool m_thruster_enabled;

  // RC controller variables.
  //   m_rc_connected   - debounced link state (subscribed from
  //                      RC_CONNECTED). Use for state-machine /
  //                      mode logic.
  //   m_rc_frame_valid - per-frame validity (subscribed from
  //                      RC_FRAME_VALID). Use to gate per-cycle
  //                      thrust output - sharper than m_rc_connected
  //                      and drops on a single bad SBUS frame.
  bool m_rc_connected;       // Indicates if RC controller is connected
  bool m_rc_frame_valid;     // Latest RC frame trustworthy?
  double m_rc_channels[16];  // Store RC channel values
  bool m_rc_mode;            // Mode switch: false = MOOS control, true = RC control

  // RC KILL (CH5) - neutral-lock final override
  // (docs/rc_controllers.md 7.1). While true, NO source may command
  // thrust - RC, teleop, autonomy - enforced as the last step of
  // thrust arbitration every iterate. Latched from RC_CH5 mail only
  // while m_rc_connected (the disconnected-fallback CH5=1 must never
  // silently un-kill a killed boat). ESCs stay armed at neutral, so
  // un-kill recovery is instant - no re-arm. The hardware signal-cut
  // (NVGR_DISARM) remains the independent emergency path.
  bool m_rc_kill;

  // CH8 DEADMAN_EN switch edge detector (docs/rc_controllers.md
  // 7.3). Edge-triggered so the handset switch and the backseat
  // RC_DEADMAN_ENABLED writer don't fight: only a switch state
  // CHANGE writes m_rc_deadman_enabled (last writer wins). 0 =
  // baseline not yet observed (first sample records, doesn't act).
  int m_rc_ch8_last_state;

  // CH11 THRUST_LIMIT (docs/rc_controllers.md 7.2): operator
  // authority cap from the S1 pot, percent 25-100, applied to the
  // MANUAL sources only (RC + teleop) - autonomy is never limited.
  // Feature-gated by rc_thrust_limit_enable (default off).
  bool   m_rc_thrust_limit_enable;
  double m_rc_thrust_limit;

  // RC stick sign convention (rc_stick_convention = v1|v2).
  //   false = "v1" (default): legacy RadioLink/SBUS wire - sticks
  //     arrive negated, so the mixer negates CH1 and the output
  //     skips the -1 transform. Kept as default so the iRCReader
  //     fallback remains a pure config swap.
  //   true = "v2": RC Contract v2 wire (+ = ahead on CH3,
  //     + = starboard on CH1; CRSF/TX16S handsets). Mixer uses +CH1
  //     and the output applies the SAME -1 * value * invert
  //     transform as the autonomy and teleop paths - all three
  //     command sources genuinely share one convention.
  // Getting this wrong is benign-looking but dangerous: v2 wire
  // through v1 math gives correct turns with REVERSED throttle
  // (the CH1 negation and the missing -1 cancel in yaw only).
  bool m_rc_contract_v2;

  // RC deadman watchdog. When enabled, the vehicle is safed
  // (thrust zeroed) if the RC link has been bad for longer than
  // m_rc_deadman_timeout seconds. Default-on; can be disabled at
  // config time (rc_deadman_enabled = false) or at runtime via the
  // RC_DEADMAN_ENABLED MOOS message - intended for over-the-horizon
  // missions where operating outside RC range is acceptable.
  bool m_rc_deadman_enabled;
  double m_rc_deadman_timeout;
  double m_last_rc_good_time;  // last RC_CONNECTED=true or RC_CH* mail
  bool m_rc_deadman_active;

  // Laptop teleop (via iTeleop). Priority: RC > teleop > autonomy.
  //   m_teleop_active  - latest TELEOP_ACTIVE mail (iTeleop
  //                      re-publishes it every iterate while a GUI
  //                      session is live).
  //   m_teleop_engaged - derived each Iterate(): active AND mail
  //                      fresh within m_teleop_command_timeout AND
  //                      not in RC mode. Stale teleop mail (hung or
  //                      dead iTeleop) zeros thrust - a second
  //                      deadman layer behind iTeleop's own GUI
  //                      deadman.
  bool m_teleop_active;
  double m_teleop_thrust_left;   // raw wire values from mail
  double m_teleop_thrust_right;  // (inversion applied at use)
  double m_last_teleop_time;
  double m_teleop_command_timeout;
  bool m_teleop_engaged;

private: // State variables
  // Pulse width range, microseconds
  std::atomic<bool> m_running{true};

  // Since the thrusters are open loop, maintain a virtual system and update
  // the values with a lpf to prevent hard jerks, where the goal is to
  // avoid spikes in current and damaging the hardware
  double m_thruster_alpha;

  LPF m_virtualThrusterLeft;
  LPF m_virtualThrusterRight;

  double m_last_update;

  int m_left_thruster_pin;
  int m_right_thruster_pin;

  // desired states from mail
  double m_desired_thrust_left;
  double m_desired_thrust_right;
  // Written by Iterate() (main thread), read by manageModulation
  // (PWM thread). Atomic so cross-thread loads/stores are
  // tear-free under the C++ memory model.
  std::atomic<double> m_latest_set_thrust_left;
  std::atomic<double> m_latest_set_thrust_right;
  bool m_all_stop;

  // Configured span for the ranges
  double m_thruster_range;

  double m_left_thruster_invert;
  double m_right_thruster_invert;

  // ESC arming lifecycle. NOTE (bench-verified 2026-08-14): pin
  // state PERSISTS after app exit - releasing the OE GPIO line does
  // not reset it - so between missions the ESCs sit ARMED at the
  // last-written neutral. The fleet safe state is neutral-on-the-
  // wire (props stopped, signal present), NOT absence of signal.
  // The neutral-hold arm procedure runs on every launch
  // (pca9685_init parks the channels first, so init is state-
  // independent). KNOWN GAP: on an unhandled death (SIGKILL/OOM)
  // the chip free-runs the last commanded thrust; the optional
  // bb_esc_failsafe watcher (scripts/) is NOT deployed, by fleet
  // decision 2026-08-14.
  //   m_initialize_esc   - run the arm procedure at startup
  //                        (legacy param name kept).
  //   m_disarm_on_exit   - if true, safe shutdown also raises OE
  //                        (signal-cut; ESCs beep until next arm).
  bool m_initialize_esc;
  bool m_disarm_on_exit;
  bool m_esc_armed;          // arming path completed this boot
  bool m_pwm_output_enabled; // OE state as last commanded by this app

  // ADC Measurements (vbat, current, adc 1 and adc 2 - needs to be ran in same app as pwm chip)
  int m_num_batteries;
  double m_voltage_offset;
  double m_current_offset;
  double m_voltage_scale;
  double m_current_scale;
  double m_adc_1;
  double m_adc_2;
  double m_adc_3;
  double m_adc_4;
  double m_latest_voltage;
  double m_latest_current;

  double m_rolling_window_seconds;
  uint64_t m_rolling_window_size;
  double m_apptick;
  double m_commtick;
  uint64_t m_apptick_idx;

  std::vector<double> m_rolling_voltage_window;
  std::vector<double> m_rolling_current_window;
  std::vector<double> m_rolling_power_window;

  double m_rolling_voltage;
  double m_rolling_current;
  double m_rolling_power;

  // IPT Sensors on vehicle - needs to be ran in same process as pwm chip to avoid pwm app crash
  double m_nav_temp;
  double m_nav_pressure;
  double m_rpi_temp;
  bool m_leak_detected;

  std::vector<uint16_t> m_port_side;
  std::vector<uint16_t> m_starboard_side;
  std::vector<uint16_t> m_led_color_quad;
  std::vector<uint16_t> m_active_color_quad;

  std::string m_low_pressure_flag;
  std::string m_high_pressure_flag;
  double m_low_pressure_value;
  double m_high_pressure_value;

  // Thrust timeout parameters
  double m_thrust_command_timeout;  // Timeout in seconds, -1 to disable
  double m_last_thrust_command_time; // Time of last thrust command
  bool m_thrust_timeout_enabled;    // Whether timeout is enabled

  // RC control parameters for mixer-based differential thrust
  double m_theta_b;  // Bank angle limit (degrees) for RC control
  double m_turn_scale; // Turn sensitivity for RC control

  // AHRS configuration.
  // The Allgeuer estimator lives inside the Navigator instance; the
  // app feeds it calibrated, mounting-rotated body-frame samples.
  double m_gyro_bias[3];
  double m_accel_bias[3];
  double m_mag_bias[3];
  double m_mag_scale[9];     // row-major 3x3 soft-iron matrix
  bool m_have_mag_cal;
  std::string m_ak09915_cal_file;
  std::string m_imu_cal_file;
  double m_roll_offset;
  double m_pitch_offset;
  double m_yaw_offset;
  double m_declination_deg;
  double m_operating_heading_offset;
  double m_sample_frequency;
  bool m_use_mag;            // feed AK09915 into the estimator
  // Estimator PI gains (Allgeuer: Ki = Kp/Ti). Zeros mean "library
  // defaults" (2.20 / 2.65, quick-learn 10.0 / 1.25).
  double m_ahrs_kp, m_ahrs_ti, m_ahrs_kp_quick, m_ahrs_ti_quick;
  // Level-frame yaw-rate plausibility clamp (rad/s). The published
  // GYRO_Z_LVL is the world-frame z component of the body angular
  // velocity (bounded by |omega|), but clamp anyway so a corrupted
  // sample can never reach the EKF.
  double m_yaw_rate_clamp;

  // AHRS state
  std::atomic<bool> m_ahrs_running{false};
  std::mutex m_ahrs_mutex;
  double m_roll;
  double m_pitch;
  double m_yaw;
  double m_heading;
  double m_gyro_x;
  double m_gyro_y;
  double m_gyro_z;
  double m_yaw_rate;
  double m_accel_x;
  double m_accel_y;
  double m_accel_z;
  double m_qw, m_qx, m_qy, m_qz;
  uint64_t m_imu_read_errors;   // reads that returned an error string
  uint64_t m_imu_glitch_count;  // reads that "succeeded" but failed the
                                // plausibility screen (full-scale/zero
                                // corrupted samples, e.g. SPI contention)
  std::string m_last_sensor_error;

  // Publication suffixes for the two source categories.
  //   ahrs suffix - fused orientation outputs
  //                 (NAV_ROLL, NAV_PITCH, NAV_YAW, NAV_HEADING)
  //   imu  suffix - raw gyro / level-compensated outputs
  //                 (GYRO_X, GYRO_Y, GYRO_Z, GYRO_Z_LVL)
  // Each is appended as "_<suffix>" to the base name. Empty
  // suffix publishes the base name bare.
  std::string m_ahrs_pub_suffix;
  std::string m_imu_pub_suffix;
};

#endif
