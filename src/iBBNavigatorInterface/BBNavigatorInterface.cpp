/*************************************************************
 *  iBBNavigatorInterface -- MOOS lifecycle.
 *
 *  Mail in, one coherent actuator frame out, sensors published.
 *  The actuation hardware is in NavActuator.cpp; the sensors are
 *  in NavSensors.cpp; the rules are in lib_bb_command.
 *
 *  Author: Raymond Turrisi (orig.), Jeremy Wenger
 *************************************************************/

#include "BBNavigatorInterface.h"
#include "BBNavigatorInterface_Info.h"

#include "wire_format.h"

#include "ACTable.h"
#include "MBUtils.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include "ColorParse.h"

using namespace std;

// One Navigator per process, shared by both halves of the app.
Navigator g_nav;

void safePwmShutdown();          // NavActuator.cpp
void g_disarm_on_exit_set(bool v);// NavActuator.cpp
void signalHandler(int signum);  // NavActuator.cpp

//---------------------------------------------------------

BBNavigatorInterface::BBNavigatorInterface()
  : m_debug(false),
    m_debug_stream(nullptr),
    m_stage(nullptr),
    m_mixed_var("BB_MIXED_CMD"),
    m_left_thruster_pin(kPwmIndexCh14),
    m_right_thruster_pin(kPwmIndexCh16),
    m_thruster_enabled(true),
    m_initialize_esc(true),
    m_disarm_on_exit(false),
    m_esc_armed(false),
    m_pwm_output_enabled(false),
    m_arm_requested(false),
    m_arm(nullptr),
    m_arm_state_pub(0),
    m_arm_cycles_pub(0),
    m_arm_retry_after(0.0),
    m_rc_kill_asserted(false),
    m_rc_link_alive(false),
    m_last_rc_good_time(0.0),
    m_shutdown_requested(false),
    m_have_frame(false),
    m_pwm_watchdog_count(0),
    m_pwm_writes(0),
    m_running(true),
    m_num_batteries(4),
    m_voltage_offset(0),
    m_voltage_scale(11.132),
    m_adc_1(0), m_adc_2(0), m_adc_3(0), m_adc_4(0),
    m_latest_voltage(0), m_latest_current(0),
    m_last_slow_poll(0), m_last_rpi_poll(0),
    m_rolling_window_seconds(2.0), m_rolling_window_size(0), m_apptick_idx(0),
    m_rolling_voltage(0), m_rolling_current(0), m_rolling_power(0),
    m_nav_temp(0), m_nav_pressure(0), m_rpi_temp(0), m_leak_detected(false),
    m_low_pressure_value(0), m_high_pressure_value(0),
    m_have_mag_cal(false),
    m_roll_offset(0), m_pitch_offset(0), m_yaw_offset(0),
    m_declination_deg(0), m_operating_heading_offset(0),
    m_sample_frequency(150.0),
    m_use_mag(false),
    m_ahrs_kp(0), m_ahrs_ti(0), m_ahrs_kp_quick(0), m_ahrs_ti_quick(0),
    m_yaw_rate_clamp(3.0),
    m_ahrs_running(false),
    m_roll(0), m_pitch(0), m_yaw(0), m_heading(0),
    m_gyro_x(0), m_gyro_y(0), m_gyro_z(0), m_yaw_rate(0),
    m_accel_x(0), m_accel_y(0), m_accel_z(0),
    m_qw(1), m_qx(0), m_qy(0), m_qz(0),
    m_imu_read_errors(0), m_imu_glitch_count(0),
    m_sensor_rate_hz(0.0),
    m_iterations(0), m_stop_cycles(0),
    m_last_stop_reason(bb::StopReason::STARTUP)
{
  for (int i = 0; i < 3; i++) { m_gyro_bias[i] = 0; m_accel_bias[i] = 0; m_mag_bias[i] = 0; }
  for (int i = 0; i < 9; i++) m_mag_scale[i] = (i % 4 == 0) ? 1.0 : 0.0;

  m_port_side        = {0, 0, 0, 0};
  m_starboard_side   = {0, 0, 0, 0};
  m_led_color_quad   = {0, 0, 0, 0};
  m_active_color_quad= {0, 0, 0, 0};

  // NOTE: hardware init has deliberately MOVED to OnStartUp.
  // The previous version called g_nav.init() from this
  // constructor -- doing bus I/O before any configuration had
  // been read, and storing the error string without ever testing
  // it. A board that failed to come up produced a running app
  // that published sensor values and accepted thrust commands.
}

BBNavigatorInterface::~BBNavigatorInterface()
{
  m_shutdown_requested.store(true);
  m_running.store(false);
  m_ahrs_running.store(false);

  if (m_pwm_thread.joinable())    m_pwm_thread.join();
  if (m_sensor_thread.joinable()) m_sensor_thread.join();

  safePwmShutdown();
  delete m_stage;
  delete m_arm;
  if (m_debug_stream) fclose(m_debug_stream);
}

//---------------------------------------------------------
// Mail: validate and cache. Never drive from here -- a callback
// that touches actuator state makes the output depend on mail
// arrival order rather than on a coherent cycle.

bool BBNavigatorInterface::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    const string key = msg.GetKey();

    if (key == m_mixed_var) {
      const bb::AcceptResult r = m_mixed.accept(msg.GetString(), MOOSTime());
      if (r == bb::AcceptResult::REJECTED) {
        reportRunWarning("rejected " + m_mixed_var + ": " +
                         m_mixed.last_reject_reason());
        Notify("NVGR_MIX_INPUT_REJECT_REASON", m_mixed.last_reject_reason());
      }
    }
    // --- hardware sidebands -----------------------------
    // Plain booleans, straight from iRCInterface. They bypass the
    // command path so they still work when it has failed.
    //
    // Parsed with the SAME relaxed-bool the arbiter uses
    // (bb::parse_bool_token). Before the audit this app accepted
    // only the string "true" while the arbiter also accepted "1"
    // -- i.e. the ENFORCING consumer of the kill sideband was the
    // stricter parser, and a poked "1" asserted kill in the trace
    // but not at the hardware.
    else if (key == "RC_KILL_ASSERTED") {
      const bool k = msg.IsDouble()
                         ? (msg.GetDouble() != 0)
                         : bb::parse_bool_token(msg.GetString(),
                                                m_rc_kill_asserted.load());
      if (k != m_rc_kill_asserted.load())
        reportEvent(k ? "RC KILL asserted" : "RC KILL released");
      m_rc_kill_asserted.store(k);
    }
    else if (key == "RC_LINK_ALIVE") {
      const bool alive = msg.IsDouble()
                             ? (msg.GetDouble() != 0)
                             : bb::parse_bool_token(msg.GetString(),
                                                    m_rc_link_alive.load());
      m_rc_link_alive.store(alive);
      if (alive) m_last_rc_good_time.store(MOOSTime());
    }
    else if (key == "NVGR_DISARM") {
      // Runtime signal-cut and re-arm, both mid-mission. This
      // serves the RC handset's disarm/arm request, so it has to
      // work while the boat is running -- it is not a launch-time
      // convenience.
      //
      // Mail only records the REQUEST. The PWM thread owns the
      // transition, because arming means writing neutral to the
      // pins for two seconds and that thread is the only one
      // allowed to write them. Doing it here stalled MOOS for the
      // duration and let the writer race the hold.
      //
      // Distinct from RC KILL: kill is a neutral-lock with the
      // ESCs left ARMED, so recovery is instant. This is the
      // signal cut -- the ESCs lose their input and re-arming
      // costs the hold.
      const bool disarm = msg.IsDouble()
                              ? (msg.GetDouble() != 0)
                              : bb::parse_bool_token(msg.GetString(),
                                                     !m_arm_requested.load());
      const bool was    = m_arm_requested.load();
      m_arm_requested.store(!disarm);
      if (was == disarm)   // i.e. the request actually changed
        reportEvent(disarm ? "NVGR_DISARM: signal cut requested"
                           : "NVGR_DISARM=false: re-arm requested");
    }
    else if (key == "MISSION_COMPLETE") {
      if (msg.IsDouble() ? (msg.GetDouble() != 0)
                         : bb::parse_bool_token(msg.GetString(), false)) {
        reportEvent("MISSION_COMPLETE: shutting down");
        m_shutdown_requested.store(true);
      }
    }
    else if (key != "APPCAST_REQ") {
      reportRunWarning("Unhandled mail: " + key);
    }
  }
  return true;
}

bool BBNavigatorInterface::OnConnectToServer()
{
  registerVariables();
  return true;
}

//---------------------------------------------------------
// One coherent actuator frame per cycle, always -- including
// stop cycles. A gap in the trace is indistinguishable from a
// dead app, and the PWM thread's TTL depends on a steady commit.

// PWM thread -> MOOS thread notes. AppCasting state is not
// thread-safe, so the PWM thread queues and Iterate() drains.

void BBNavigatorInterface::queueNote(bool warning, const std::string &text)
{
  std::lock_guard<std::mutex> lock(m_note_mutex);
  m_pending_notes.push_back(std::make_pair(warning, text));
}

void BBNavigatorInterface::drainNotes()
{
  std::vector<std::pair<bool, std::string> > notes;
  {
    std::lock_guard<std::mutex> lock(m_note_mutex);
    notes.swap(m_pending_notes);
  }
  for (size_t i = 0; i < notes.size(); ++i) {
    if (notes[i].first) reportRunWarning(notes[i].second);
    else                reportEvent(notes[i].second);
  }
}

bool BBNavigatorInterface::Iterate()
{
  AppCastingMOOSApp::Iterate();
  ++m_iterations;
  drainNotes();

  const double now = MOOSTime();
  const bb::NavigatorSafetyState safety = snapshotSafety();

  const bb::ActuatorFrame f =
      m_stage->update(now, m_mixed, safety, m_last_rc_good_time.load());

  commitFrame(f);

  if (f.neutral) ++m_stop_cycles;
  if (f.stop_reason != m_last_stop_reason) {
    reportEvent(string("actuator stop reason -> ") + bb::to_string(f.stop_reason));
    m_last_stop_reason = f.stop_reason;
  }

  Notify("NVGR_ACTUATOR_TRACE", bb::serialize_actuator_trace(f));

  // Plot-friendly mirrors. TELEMETRY ONLY -- these are outputs,
  // never recycled as command inputs (invariant 11).
  Notify("NVGR_APPLIED_LEFT",   f.left_effort);
  Notify("NVGR_APPLIED_RIGHT",  f.right_effort);
  Notify("NVGR_PWM_LEFT_US",    f.left_pwm_us);
  Notify("NVGR_PWM_RIGHT_US",   f.right_pwm_us);
  Notify("NVGR_MIX_INPUT_AGE",  f.mixed_age);
  Notify("NVGR_STOP_REASON",    bb::to_string(f.stop_reason));
  Notify("NVGR_RC_KILL",        safety.rc_kill_asserted ? "true" : "false");
  Notify("NVGR_PWM_ARMED",      safety.pwm_armed ? "true" : "false");
  Notify("NVGR_PWM_WATCHDOG_COUNT", (double)m_pwm_watchdog_count.load());
  Notify("NVGR_ARM_STATE", bb::to_string((bb::ArmState)m_arm_state_pub.load()));

  publishSensorTelemetry();

  AppCastingMOOSApp::PostReport();
  return true;
}

//---------------------------------------------------------

void BBNavigatorInterface::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();

  // ONE command input. If this list grows a second command
  // source, re-read the header.
  Register(m_mixed_var, 0);

  Register("RC_KILL_ASSERTED", 0);
  Register("RC_LINK_ALIVE", 0);
  Register("NVGR_DISARM", 0);
  Register("MISSION_COMPLETE", 0);
}

//---------------------------------------------------------

bool BBNavigatorInterface::dbg_print(const char *format, ...)
{
  if (!m_debug) return false;
  va_list args;
  va_start(args, format);
  m_debug_stream = fopen(m_fname, "a");
  if (m_debug_stream) {
    vfprintf(m_debug_stream, format, args);
    fclose(m_debug_stream);
    m_debug_stream = nullptr;
    va_end(args);
    return true;
  }
  va_end(args);
  return false;
}

string BBNavigatorInterface::ahrsName(const string &base) const
{
  return m_ahrs_pub_suffix.empty() ? base : (base + "_" + m_ahrs_pub_suffix);
}

string BBNavigatorInterface::imuName(const string &base) const
{
  return m_imu_pub_suffix.empty() ? base : (base + "_" + m_imu_pub_suffix);
}

//---------------------------------------------------------
// OnStartUp
//
// Order matters: read config, THEN bring up the hardware, THEN
// build the stage, THEN arm, THEN start the threads. The old
// version initialised the board in its constructor, before any of
// this, and never checked whether it worked.

bool BBNavigatorInterface::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  m_app_name = GetAppName();
  snprintf(m_fname, sizeof(m_fname), "%s.dbg", m_app_name.c_str());

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(m_app_name, sParams))
    reportConfigWarning("No config block found for " + m_app_name);

  auto quad = [&](const string &v, vector<uint16_t> &out) {
    vector<string> sv = parseString(v, ',');
    if (sv.size() != 4) return false;
    out.clear();
    for (size_t i = 0; i < 4; i++) out.push_back((uint16_t)atoi(sv[i].c_str()));
    return true;
  };

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;
    bool handled = false;

    // --- command path ---
    if      (param == "mixed_cmd_var")            { m_mixed_var = value; handled = true; }
    else if (param == "mixed_cmd_timeout_sec")    handled = setDoubleOnString(m_act_cfg.mixed_cmd_timeout_sec, value);
    else if (param == "actuator_frame_ttl_sec")   handled = setDoubleOnString(m_act_cfg.actuator_frame_ttl_sec, value);

    // --- ESC endpoints: the ONLY place electrical inversion lives ---
    else if (param == "left_esc_min_us")   handled = setDoubleOnString(m_act_cfg.left.min_us, value);
    else if (param == "left_esc_trim_us")  handled = setDoubleOnString(m_act_cfg.left.trim_us, value);
    else if (param == "left_esc_max_us")   handled = setDoubleOnString(m_act_cfg.left.max_us, value);
    else if (param == "left_esc_reversed") handled = setBooleanOnString(m_act_cfg.left.reversed, value);
    else if (param == "right_esc_min_us")   handled = setDoubleOnString(m_act_cfg.right.min_us, value);
    else if (param == "right_esc_trim_us")  handled = setDoubleOnString(m_act_cfg.right.trim_us, value);
    else if (param == "right_esc_max_us")   handled = setDoubleOnString(m_act_cfg.right.max_us, value);
    else if (param == "right_esc_reversed") handled = setBooleanOnString(m_act_cfg.right.reversed, value);

    // --- opt-in RC deadman (plan decision (d)) ---
    else if (param == "rc_deadman_enabled")     handled = setBooleanOnString(m_act_cfg.rc_deadman_enabled, value);
    else if (param == "rc_deadman_timeout_sec") handled = setDoubleOnString(m_act_cfg.rc_deadman_timeout_sec, value);

    // --- ESC lifecycle / pins ---
    else if (param == "left_thruster_pin")  { m_left_thruster_pin  = atoi(value.c_str()) - 1; handled = true; }
    else if (param == "right_thruster_pin") { m_right_thruster_pin = atoi(value.c_str()) - 1; handled = true; }
    else if (param == "thruster_enabled")   handled = setBooleanOnString(m_thruster_enabled, value);
    else if (param == "initialize_esc")     handled = setBooleanOnString(m_initialize_esc, value);
    else if (param == "disarm_on_exit")     handled = setBooleanOnString(m_disarm_on_exit, value);

    // --- power ---
    else if (param == "nbats")                 { m_num_batteries = atoi(value.c_str()); handled = true; }
    else if (param == "voltage_scale")         handled = setDoubleOnString(m_voltage_scale, value);
    else if (param == "voltage_offset")        handled = setDoubleOnString(m_voltage_offset, value);
    else if (param == "rolling_window_period") handled = setDoubleOnString(m_rolling_window_seconds, value);

    // --- AHRS ---
    else if (param == "sample_rate")     handled = setDoubleOnString(m_sample_frequency, value);
    else if (param == "use_mag")         handled = setBooleanOnString(m_use_mag, value);
    else if (param == "ahrs_kp")         handled = setDoubleOnString(m_ahrs_kp, value);
    else if (param == "ahrs_ti")         handled = setDoubleOnString(m_ahrs_ti, value);
    else if (param == "ahrs_kp_quick")   handled = setDoubleOnString(m_ahrs_kp_quick, value);
    else if (param == "ahrs_ti_quick")   handled = setDoubleOnString(m_ahrs_ti_quick, value);
    else if (param == "mag_ak_cal_file") { m_ak09915_cal_file = value; handled = true; }
    else if (param == "imu_cal_file")    { m_imu_cal_file = value; handled = true; }
    else if (param == "roll_offset")     handled = setDoubleOnString(m_roll_offset, value);
    else if (param == "pitch_offset")    handled = setDoubleOnString(m_pitch_offset, value);
    else if (param == "yaw_offset")      handled = setDoubleOnString(m_yaw_offset, value);
    else if (param == "declination_deg") handled = setDoubleOnString(m_declination_deg, value);
    else if (param == "operating_heading_offset") handled = setDoubleOnString(m_operating_heading_offset, value);
    else if (param == "yaw_rate_clamp")  handled = setDoubleOnString(m_yaw_rate_clamp, value);
    else if (param == "ahrs_pub_suffix") { m_ahrs_pub_suffix = value; handled = true; }
    else if (param == "imu_pub_suffix")  { m_imu_pub_suffix = value; handled = true; }

    // --- environment / lights ---
    else if (param == "low_pressure_flag")   { m_low_pressure_flag = value; handled = true; }
    else if (param == "high_pressure_flag")  { m_high_pressure_flag = value; handled = true; }
    else if (param == "low_pressure_value")  handled = setDoubleOnString(m_low_pressure_value, value);
    else if (param == "high_pressure_value") handled = setDoubleOnString(m_high_pressure_value, value);
    else if (param == "port_side")           handled = quad(value, m_port_side);
    else if (param == "starboard_side")      handled = quad(value, m_starboard_side);
    else if (param == "led_color_quad")      handled = quad(value, m_led_color_quad);
    else if (param == "active_color_quad")   handled = quad(value, m_active_color_quad);

    else if (param == "debug") handled = setBooleanOnString(m_debug, value);

    // --- retired parameters -----------------------------
    // Named explicitly so an un-migrated plug produces a clear
    // message instead of a generic "unhandled config" that an
    // operator skims past. Each of these moved somewhere with a
    // better claim to it.
    else if (param == "max_thrust" || param == "min_thrust" ||
             param == "thruster_dead_band" || param == "thruster_alpha" ||
             param == "left_thruster_invert" || param == "right_thruster_invert" ||
             param == "pwm_min_us" || param == "pwm_max_us" ||
             param == "thrust_command_timeout" || param == "rc_deadman_timeout" ||
             param == "rc_thrust_limit_enable" || param == "teleop_command_timeout" ||
             param == "rc_stick_convention" || param == "theta_b" ||
             param == "turn_scale" ||
             // Retired in the pre-hardware audit: current always came
             // from the per-battery-count calibration table, so these
             // two knobs silently did nothing.
             param == "current_scale" || param == "current_offset") {
      reportConfigWarning("RETIRED parameter '" + param +
                          "' is ignored -- see docs/control_refactor_plan.md 16.5");
      handled = true;
    }

    if (!handled) reportUnhandledConfigWarning(orig);
  }

  // --- hardware, and whether it actually came up ----------
  //
  // init() returns a WARNINGS string and reports partial init as
  // usable (navigator-cpp nav.cpp: "non-empty = partial init
  // (still usable)"). The predicate that decides whether the
  // props may turn is is_pwm_ready() -- initialized AND the
  // PCA9685 came up -- exactly as plan section 3.1 intended.
  //
  // Treating ANY sub-device warning as a grounding fault was the
  // first dock test's actual failure: a NACKing magnetometer on a
  // use_mag=false boat held HARDWARE_FAULT with pwm_ready yes.
  // A dead sensor degrades sensing (and autonomy self-inhibits
  // upstream without nav data); it must not ground manual drive.
  m_nav_init_error = g_nav.init(NAV_AUTO, PI_AUTO);
  if (!m_nav_init_error.empty())
    reportRunWarning("Navigator PARTIAL init: " + m_nav_init_error);
  if (!g_nav.is_pwm_ready())
    reportRunWarning("PCA9685 not ready: actuation stays disabled "
                     "(HARDWARE_FAULT) until it is");

  // --- configuration validation --------------------------
  const string cfg_err = m_act_cfg.validate();
  if (!cfg_err.empty()) {
    reportConfigWarning("FATAL: " + cfg_err);
    cout << termColor("red") << m_app_name << ": invalid configuration: "
         << cfg_err << endl << termColor();
    return false;
  }
  m_stage = new bb::ActuatorStage(m_act_cfg);

  // initialize_esc = false skips the neutral hold but still
  // transits ARMING, so there is exactly one path to ARMED.
  m_arm_cfg.skip_hold = !m_initialize_esc;
  const string arm_err = m_arm_cfg.validate();
  if (!arm_err.empty()) { reportConfigWarning("FATAL: " + arm_err); return false; }
  m_arm = new bb::ArmSequencer(m_arm_cfg);

  g_disarm_on_exit_set(m_disarm_on_exit);
  atexit(safePwmShutdown);
  signal(SIGINT,  signalHandler);
  signal(SIGTERM, signalHandler);

  // Ask for arm and let the PWM thread do it. Startup no longer
  // blocks for two seconds, and there is exactly one arm path
  // whether the request comes from launch or from NVGR_DISARM.
  m_arm_requested.store(true);

  m_running.store(true);
  m_pwm_thread = std::thread(&BBNavigatorInterface::pwmWriterThread, this);

  if (!m_ak09915_cal_file.empty()) readMagCalFile(m_ak09915_cal_file);
  if (!m_imu_cal_file.empty())     readImuCalFile(m_imu_cal_file);

  if (m_ahrs_kp > 0 || m_ahrs_ti > 0)
    g_nav.ahrs_set_gains(m_ahrs_kp, m_ahrs_ti, m_ahrs_kp_quick, m_ahrs_ti_quick);
  if (m_have_mag_cal)
    g_nav.ahrs_set_mag_calib(m_mag_bias[0], m_mag_bias[1], m_mag_bias[2]);
  g_nav.ahrs_reset();

  // Samples arrive at the slow-poll rate, not at AppTick -- the
  // ADC read is decimated in publishSensorTelemetry().
  m_rolling_window_size =
      (uint64_t)(m_rolling_window_seconds / SLOW_POLL_PERIOD_SEC);
  if (m_rolling_window_size < 1) m_rolling_window_size = 1;

  m_ahrs_running.store(true);
  m_sensor_thread = std::thread(&BBNavigatorInterface::sensorSamplingThread, this);

  setNavLights();
  registerVariables();
  return true;
}

//---------------------------------------------------------

bool BBNavigatorInterface::buildReport()
{
  if (!m_stage) { m_msgs << "NOT CONFIGURED" << endl; return true; }

  NavVersion nv = g_nav.detected_version();
  m_msgs << "Board:   " << (nv == NAV_V1 ? "V1" : nv == NAV_V2 ? "V2" : "UNKNOWN")
         << "   init " << (m_nav_init_error.empty() ? "OK"
                                                    : ("PARTIAL: " + m_nav_init_error))
         << "   pwm_ready " << (g_nav.is_pwm_ready() ? "yes" : "NO") << endl;
  m_msgs << "ESC:     " << bb::to_string((bb::ArmState)m_arm_state_pub.load())
         << "   output " << (m_pwm_output_enabled.load() ? "enabled" : "CUT")
         << "   " << doubleToString(PWM_FREQ_HZ, 0) << " Hz" << endl;
  m_msgs << "         left  " << doubleToString(m_act_cfg.left.min_us, 0) << "/"
         << doubleToString(m_act_cfg.left.trim_us, 0) << "/"
         << doubleToString(m_act_cfg.left.max_us, 0)
         << (m_act_cfg.left.reversed ? "  reversed" : "") << endl;
  m_msgs << "         right " << doubleToString(m_act_cfg.right.min_us, 0) << "/"
         << doubleToString(m_act_cfg.right.trim_us, 0) << "/"
         << doubleToString(m_act_cfg.right.max_us, 0)
         << (m_act_cfg.right.reversed ? "  reversed" : "") << endl;
  m_msgs << endl;

  bb::ActuatorFrame f;
  if (latestFrame(f)) {
    m_msgs << "APPLIED: " << (f.neutral ? "NEUTRAL" : "driving")
           << "   stop=" << bb::to_string(f.stop_reason)
           << (f.local_stop ? "  (local)" : "") << endl;
    m_msgs << "  effort  L " << doubleToString(f.left_effort, 2)
           << "   R " << doubleToString(f.right_effort, 2) << endl;
    m_msgs << "  pulse   L " << doubleToString(f.left_pwm_us, 1)
           << "   R " << doubleToString(f.right_pwm_us, 1) << " us" << endl;
    m_msgs << "  lineage src " << (f.source_producer.empty() ? "-" : f.source_producer)
           << " " << (f.source_epoch.empty() ? "-" : f.source_epoch) << "/" << f.source_seq
           << "  arb " << (f.decision_epoch.empty() ? "-" : f.decision_epoch) << "/" << f.decision_seq
           << "  mix " << (f.mix_epoch.empty() ? "-" : f.mix_epoch) << "/" << f.mix_seq << endl;
    m_msgs << "  mix age " << doubleToString(f.mixed_age, 3) << " s" << endl;
  } else {
    m_msgs << "APPLIED: no frame committed yet" << endl;
  }
  m_msgs << endl;

  m_msgs << "SAFETY:  RC kill " << (m_rc_kill_asserted.load() ? "ASSERTED" : "clear")
         << "   RC link " << (m_rc_link_alive.load() ? "alive" : "LOST")
         << "   deadman " << (m_act_cfg.rc_deadman_enabled ? "ENABLED" : "off") << endl;
  // m_arm is PWM-thread-only; read the atomic mirror instead.
  m_msgs << "ESC:     arm cycles " << m_arm_cycles_pub.load()
         << "   request " << (m_arm_requested.load() ? "armed" : "disarmed") << endl;
  m_msgs << "PWM:     writes " << m_pwm_writes.load()
         << "   watchdog neutralisations " << m_pwm_watchdog_count.load() << endl;
  m_msgs << "Input:   accepted " << m_mixed.accepted_count()
         << "  dup " << m_mixed.duplicate_count()
         << "  out-of-order " << m_mixed.out_of_order_count()
         << "  rejected " << m_mixed.reject_count() << endl;
  m_msgs << "Cycles:  " << m_iterations << "   neutral " << m_stop_cycles << endl;
  m_msgs << endl;

  ACTable ahrs(2);
  ahrs << "AHRS | Value";
  ahrs.addHeaderLines();
  {
    std::lock_guard<std::mutex> lock(m_ahrs_mutex);
    ahrs << "heading"    << doubleToString(m_heading, 2);
    ahrs << "roll/pitch" << (doubleToString(m_roll, 2) + " / " + doubleToString(m_pitch, 2));
    ahrs << "yaw rate"   << doubleToString(m_yaw_rate, 4);
    ahrs << "mag fusion" << (m_use_mag ? "on" : "off");
    ahrs << "read errors"<< uintToString(m_imu_read_errors);
    ahrs << "glitches"   << uintToString(m_imu_glitch_count);
  }
  // Achieved, not configured. The old loop ran well above its
  // configured rate and nothing showed it.
  ahrs << "rate (meas)" << (doubleToString(m_sensor_rate_hz.load(), 1) + " Hz of " +
                            doubleToString(m_sample_frequency, 0));
  m_msgs << ahrs.getFormattedString() << endl << endl;

  m_msgs << "POWER:   " << doubleToString(m_rolling_voltage, 2) << " V   "
         << doubleToString(m_rolling_current, 2) << " A   "
         << doubleToString(m_rolling_power, 1) << " W" << endl;
  m_msgs << "ENV:     " << doubleToString(m_nav_temp, 1) << " C   "
         << doubleToString(m_nav_pressure, 1) << " kPa   leak "
         << (m_leak_detected ? "DETECTED" : "no") << endl;
  return true;
}
