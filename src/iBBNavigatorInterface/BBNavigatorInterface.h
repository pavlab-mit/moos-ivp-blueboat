/*************************************************************
      Name: Raymond Turrisi (orig.), Jeremy Wenger
      Orgn: MIT, Cambridge MA
      File: iBBNavigatorInterface/BBNavigatorInterface.h
   Last Ed: 2026-08-27 (command-pipeline rewrite)
     Brief:
        Navigator board interface for the BlueBoat ASV.

        Two jobs, deliberately kept apart in the source:

          NavActuator.cpp  BB_MIXED_CMD -> ESC pulses. Final
                           safety, ESC mapping, the PWM writer
                           thread and its watchdog, ESC lifecycle.

          NavSensors.cpp   AHRS, raw gyro / level-frame yaw rate,
                           ADC power, baro, leak, NeoPixels.

        They share only the navigator-cpp handle. Keeping them in
        one app is not tidiness -- both own the PCA9685 and the
        I2C bus, and two processes fighting over those is a
        hazard, not a cleanup.

        WHAT THIS APP NO LONGER DOES, and must not learn again:

          - arbitrate. It consumes ONE command variable and cannot
            see RC, teleop or autonomy. Who drives is
            pBBCommandArbiter's decision (invariant 1, 3).
          - mix. Left/right allocation is pThrustMix's. Effort
            arrives physical and forward-positive.
          - filter. The old per-side LPF smoothed the DIFFERENCE
            between the sides, which is the yaw command -- it
            blunted every turn. Common-throttle slew now happens
            once, in the mixer, and yaw is never slewed.
          - apply a post-mix deadband. It deleted small
            corrections outright, removing yaw authority in
            exactly the regime where trim matters.

        WHAT IT KEEPS, because it is the last thing before water:

          - two hardware sidebands that bypass the command path,
            RC kill and the opt-in RC deadman, so they still work
            when the arbiter or mixer has failed;
          - a PWM writer fed one ATOMIC actuator frame with a TTL,
            so a stalled Iterate() cannot leave the props running
            on an old command;
          - the ESC arm/disarm lifecycle and the shutdown paths.
*************************************************************/

#ifndef BBNavigatorInterface_HEADER
#define BBNavigatorInterface_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include "actuator_stage.h"
#include "arm_sequencer.h"

#include "nav_bindings.h"

#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class BBNavigatorInterface : public AppCastingMOOSApp
{
public:
  BBNavigatorInterface();
  ~BBNavigatorInterface();

  // navigator-cpp PWM channels are 0-based (legacy ChN -> index N-1).
  static constexpr int kPwmIndexCh14 = 13;
  static constexpr int kPwmIndexCh16 = 15;

  // ArduRover BlueBoat120 output rate. Was 100; decision 2 of the
  // control refactor moves it to 50 to match.
  static constexpr double PWM_FREQ_HZ = 50.0;

  // Poll period for the slow telemetry (ADC / baro / leak): I2C
  // transactions on the MOOS thread, decimated below AppTick.
  // The rolling power window is sized from this, so they cannot
  // drift apart.
  static constexpr double SLOW_POLL_PERIOD_SEC = 0.2;

protected: // MOOSApp
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();
  bool buildReport();

protected:
  void registerVariables();
  bool dbg_print(const char *format, ...);

  std::string ahrsName(const std::string &base) const;
  std::string imuName(const std::string &base) const;

  // ---- NavActuator.cpp ----------------------------------
  //
  // Arm/disarm is a STATE MACHINE owned by the PWM thread, not a
  // blocking call. The 2 s neutral hold has to happen while
  // something is writing neutral to the pins every cycle, and the
  // only thread that may write those pins is this one. Running it
  // from OnNewMail (as the pre-rewrite app did) stalled MOOS for
  // two seconds AND let the writer race the hold for the pins.
  //
  // The cycle is operator-facing: RC disarm/arm requests arrive as
  // NVGR_DISARM and must work mid-mission. This path was broken
  // before 2026-08-14 (the PCA9685 RESTART trap) and bench-passed
  // after; the orderly all-off below is what keeps it passing.
  void   pwmWriterThread();
  bool   beginArmSequence();   // false: hardware refused; caller must back off
  void   performDisarm(const std::string &reason);
  void   commitFrame(const bb::ActuatorFrame &f);
  bool   latestFrame(bb::ActuatorFrame &out) const;
  void   writePulsePair(double left_us, double right_us);
  void   writeNeutralPair();

  // AppCasting is NOT thread-safe, and the PWM thread has things
  // to say (arm started, disarm performed, hardware refused). It
  // queues them here; Iterate() drains the queue into
  // reportEvent/reportRunWarning on the MOOS thread. Calling
  // those directly from the PWM thread raced buildReport() -- the
  // same class of bug as the iRCInterface appcast deadlock.
  void   queueNote(bool warning, const std::string &text);
  void   drainNotes();

  bb::NavigatorSafetyState snapshotSafety() const;

  // ---- NavSensors.cpp -----------------------------------
  void sensorSamplingThread();
  bool readMagCalFile(std::string filepath);
  bool readImuCalFile(std::string filepath);
  void publishSensorTelemetry();
  void setNavLights();

private: // ---- configuration -----------------------------
  bool         m_debug;
  FILE        *m_debug_stream;
  static const uint16_t m_fname_buff_size = 256;
  std::string  m_app_name;
  char         m_fname[m_fname_buff_size];

  // Command path
  bb::ActuatorConfig  m_act_cfg;
  bb::ActuatorStage  *m_stage;      // built in OnStartUp, after config
  bb::MixedMailbox    m_mixed;
  std::string         m_mixed_var;

  int  m_left_thruster_pin;
  int  m_right_thruster_pin;
  bool m_thruster_enabled;

  // ESC lifecycle. Pin state PERSISTS after this app exits, so
  // the fleet safe state is neutral-on-the-wire (props stopped,
  // signal present), NOT absence of signal.
  bool m_initialize_esc;
  bool m_disarm_on_exit;
  std::atomic<bool> m_esc_armed;
  std::atomic<bool> m_pwm_output_enabled;

  // Requested by mail (NVGR_DISARM) and at startup; the PWM
  // thread owns the transition and the resulting state.
  std::atomic<bool>  m_arm_requested;
  bb::ArmSequencer  *m_arm;             // PWM thread only; built in OnStartUp
  bb::ArmSequencerConfig m_arm_cfg;
  std::atomic<int>   m_arm_state_pub;   // ArmState, for the appcast
  std::atomic<uint64_t> m_arm_cycles_pub; // mirror; m_arm is PWM-thread-only
  double m_arm_retry_after;             // PWM thread only: backoff after a
                                        // failed beginArmSequence()

  // PWM thread -> MOOS thread notes (see queueNote above).
  mutable std::mutex m_note_mutex;
  std::vector<std::pair<bool, std::string> > m_pending_notes;

  // ---- safety sidebands ---------------------------------
  // Plain booleans on purpose: they must work when the command
  // contract does not.
  std::atomic<bool> m_rc_kill_asserted;
  std::atomic<bool> m_rc_link_alive;
  std::atomic<double> m_last_rc_good_time;
  std::atomic<bool> m_shutdown_requested;

  // ---- actuator frame exchange --------------------------
  // The PWM thread must never read a half-updated left/right
  // pair. One mutex, a copy in and a copy out, critical section
  // of a few nanoseconds.
  mutable std::mutex   m_frame_mutex;
  bb::ActuatorFrame    m_frame;
  bool                 m_have_frame;
  std::atomic<uint64_t> m_pwm_watchdog_count;
  std::atomic<uint64_t> m_pwm_writes;

  std::atomic<bool> m_running;
  std::thread m_pwm_thread;
  std::thread m_sensor_thread;

  // ---- power / environment ------------------------------
  // (current_scale/current_offset were retired: current always
  // came from the per-battery-count table, so the two config
  // knobs silently did nothing.)
  int    m_num_batteries;
  double m_voltage_offset;
  double m_voltage_scale;
  double m_adc_1, m_adc_2, m_adc_3, m_adc_4;
  double m_latest_voltage, m_latest_current;

  // The ADC/baro/leak reads are I2C transactions; at AppTick=50
  // doing them every Iterate would put 150 bus hits/s on the MOOS
  // thread while the PWM and sensor threads share the same bus.
  // Decimated to the rates the data changes at.
  double m_last_slow_poll;   // ADC + baro + leak, 5 Hz
  double m_last_rpi_poll;    // Pi CPU temp via sysfs, 1 Hz

  double   m_rolling_window_seconds;
  uint64_t m_rolling_window_size;
  uint64_t m_apptick_idx;
  std::vector<double> m_rolling_voltage_window;
  std::vector<double> m_rolling_current_window;
  std::vector<double> m_rolling_power_window;
  double m_rolling_voltage, m_rolling_current, m_rolling_power;

  double m_nav_temp, m_nav_pressure, m_rpi_temp;
  bool   m_leak_detected;

  std::vector<uint16_t> m_port_side;
  std::vector<uint16_t> m_starboard_side;
  std::vector<uint16_t> m_led_color_quad;
  std::vector<uint16_t> m_active_color_quad;

  std::string m_low_pressure_flag, m_high_pressure_flag;
  double m_low_pressure_value, m_high_pressure_value;

  // ---- AHRS ---------------------------------------------
  double m_gyro_bias[3];
  double m_accel_bias[3];
  double m_mag_bias[3];
  double m_mag_scale[9];
  bool   m_have_mag_cal;
  std::string m_ak09915_cal_file;
  std::string m_imu_cal_file;
  double m_roll_offset, m_pitch_offset, m_yaw_offset;
  double m_declination_deg, m_operating_heading_offset;
  double m_sample_frequency;
  bool   m_use_mag;
  double m_ahrs_kp, m_ahrs_ti, m_ahrs_kp_quick, m_ahrs_ti_quick;
  double m_yaw_rate_clamp;

  std::atomic<bool> m_ahrs_running;
  mutable std::mutex m_ahrs_mutex;
  double m_roll, m_pitch, m_yaw, m_heading;
  double m_gyro_x, m_gyro_y, m_gyro_z, m_yaw_rate;
  double m_accel_x, m_accel_y, m_accel_z;
  double m_qw, m_qx, m_qy, m_qz;
  uint64_t m_imu_read_errors;
  uint64_t m_imu_glitch_count;
  std::string m_last_sensor_error;

  // Measured sensor-thread rate. The old loop slept on the
  // PREVIOUS period rather than this iteration's work and so ran
  // roughly (work+nominal)/2; publishing the real rate makes that
  // class of bug visible instead of silent.
  std::atomic<double> m_sensor_rate_hz;

  std::string m_ahrs_pub_suffix;
  std::string m_imu_pub_suffix;

  // ---- diagnostics --------------------------------------
  uint64_t m_iterations;
  uint64_t m_stop_cycles;
  bb::StopReason m_last_stop_reason;
  std::string m_nav_init_error;   // init() warnings; empty = clean
};

#endif
