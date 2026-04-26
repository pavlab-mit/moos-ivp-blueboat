/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: iBBNavigatorInterface/BBNavigatorInterface.h
   Last Ed:  2025-03-30
     Brief:
        Combined Navigator Interface for Blueboat ASV.
        Handles dual thruster PWM control with safety features
        and AHRS (Attitude Heading Reference System) using
        Madgwick filter for IMU/magnetometer fusion.
*************************************************************/

#ifndef BBNavigatorInterface_HEADER
#define BBNavigatorInterface_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <string>
#include <cstdarg> //va_list, va_start, va_end
#include "bindings.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <armadillo>
#include "MadgwickAHRS.h"
#include "rpi_utils.h"

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

  // PWM frequency and pulse range constants (full range, 100 Hz)
  static constexpr double PWM_FREQ_HZ = 100.0;      // 100 Hz for smoother response
  static constexpr double PWM_MIN_US = 800.0;       // Minimum pulse width (microseconds)
  static constexpr double PWM_MAX_US = 2200.0;      // Maximum pulse width (microseconds)
  static constexpr double PWM_CENTER_US = 1500.0;   // Neutral pulse width (microseconds)

  // Convert normalized command [-100,100] to PCA9685 counts
  // Works with any PWM frequency and pulse range
  static void setPinPulseWidth(PwmChannel pin_num, double target)
  {
    // Map normalized command [-100,100] to pulse range
    double pulse_us_span = (PWM_MAX_US - PWM_MIN_US) / 2.0;
    double pulse_us = PWM_CENTER_US + (target / 100.0) * pulse_us_span;

    // Clamp to allowable range
    if (pulse_us < PWM_MIN_US) pulse_us = PWM_MIN_US;
    if (pulse_us > PWM_MAX_US) pulse_us = PWM_MAX_US;

    // Convert microseconds to PCA9685 counts: counts = pulse_us * freq_hz * 4096 / 1e6
    double counts = pulse_us * PWM_FREQ_HZ * 4096.0 / 1e6;

    if (counts < 0.0) counts = 0.0;
    if (counts > 4095.0) counts = 4095.0;

    set_pwm_channel_value(pin_num, static_cast<uint16_t>(counts));
  }

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
  
  // New method for RC thrust calculation
  void calculateRCThrust();

  // Function to initialize multiple ESCs simultaneously
  void initializeESCs(const std::vector<PwmChannel>& pins);

  // Helper function to compute heading error mixer similar to DiffThrustPID
  double calculateHeadingMixer(double desired_heading, double current_heading);


  void initializeESC(PwmChannel pin);

  std::thread m_modulation_thread;
  std::thread m_sensor_thread;

  // AHRS methods
  void sensorSamplingThread();
  bool readMagCalFile(std::string filepath, arma::vec &bias, arma::mat &scaling_matrix);
  bool readImuCalFile(std::string filepath);
  arma::vec adjust_gyro_cal(const arma::vec &gyro_raw);
  arma::vec adjust_acc_cal(const arma::vec &acc_raw);
  double calculateYawRate(double roll_phi, double pitch_theta, double p, double q, double r);

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

  // RC controller variables
  bool m_rc_connected;       // Indicates if RC controller is connected
  double m_rc_channels[16];  // Store RC channel values
  bool m_rc_mode;            // Mode switch: false = MOOS control, true = RC control

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

  PwmChannel m_left_thruster_pin;
  PwmChannel m_right_thruster_pin;

  // desired states from mail
  double m_desired_thrust_left;
  double m_desired_thrust_right;
  // Written by Iterate() (main thread), read by manageModulation
  // (PWM thread). Atomic so cross-thread loads/stores are
  // tear-free under the C++ memory model.
  std::atomic<double> m_latest_set_thrust_left;
  std::atomic<double> m_latest_set_thrust_right;
  bool m_all_stop;

  double m_latest_set_thrust_left_pw;
  double m_latest_set_thrust_right_pw;

  // Configured span for the ranges
  double m_thruster_range;

  double m_left_thruster_invert;
  double m_right_thruster_invert;

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

  bool m_initialize_esc;  // Whether to perform ESC initialization on startup

  // AHRS configuration
  arma::vec m_mag_ak_bias;
  arma::mat m_mag_ak_scaling_matrix;
  arma::vec m_gyro_bias;
  arma::vec m_accel_bias;
  std::string m_ak09915_cal_file;
  std::string m_imu_cal_file;
  double m_roll_offset;
  double m_pitch_offset;
  double m_yaw_offset;
  double m_declination_deg;
  double m_operating_heading_offset;
  double m_sample_frequency;
  double m_beta;

  // AHRS state
  std::atomic<bool> m_ahrs_running{false};
  Madgwick m_ahrs;
  std::mutex m_ahrs_mutex;
  double m_roll;
  double m_pitch;
  double m_yaw;
  double m_heading;
  double m_gyro_x;
  double m_gyro_y;
  double m_gyro_z;
  double m_yaw_rate;
  double m_qw, m_qx, m_qy, m_qz;

  // Publication suffixes for the two source categories.
  //   ahrs suffix - Madgwick-fused orientation outputs
  //                 (NAV_ROLL, NAV_PITCH, NAV_YAW, NAV_HEADING)
  //   imu  suffix - raw gyro / level-compensated outputs
  //                 (GYRO_X, GYRO_Y, GYRO_Z, GYRO_Z_LVL)
  // Each is appended as "_<suffix>" to the base name. Empty
  // suffix publishes the base name bare.
  std::string m_ahrs_pub_suffix;
  std::string m_imu_pub_suffix;
};

#endif