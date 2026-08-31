/************************************************************/
/*    NAME: Karan Mahesh                                    */
/*    ORGN: MIT / Project Greece                            */
/*    FILE: BBPID.cpp                                       */
/*    DATE: 2026/06/19                                      */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "BBPID.h"

using namespace std;

//---------------------------------------------------------
// Constructor

BBPID::BBPID()
{
  // Default I/O variable names
  m_desired_speed_var   = "DESIRED_SPEED";
  m_desired_heading_var = "DESIRED_HEADING";
  m_nav_speed_var       = "NAV_SPEED";
  m_nav_heading_var     = "GPS_HEADING_DGNSS";
  m_nav_yawrate_var     = "GYRO_Z_LVL_IMU";
  m_thrust_var          = "DESIRED_THRUST";
  m_rudder_var          = "DESIRED_RUDDER";
  m_autonomy_cmd_var    = "AUTONOMY_CMD";
  m_autonomy_seq        = 0;
  m_publish_autonomy_cmd = true;
  m_config_parsed       = false;

  m_desired_speed   = 0.0;
  m_desired_heading = 0.0;
  m_nav_speed       = 0.0;
  m_nav_heading     = 0.0;
  m_nav_yawrate     = 0.0;

  m_have_des_speed   = false;
  m_have_des_heading = false;
  m_have_nav_speed   = false;
  m_have_nav_heading = false;
  m_have_nav_yawrate = false;
  m_yawrate_derive   = false;

  m_active            = false;
  m_last_thrust       = 0.0;
  m_last_rudder       = 0.0;
  m_iterations_run    = 0;
  m_tstamp_last_cmd   = 0.0;
  m_cmd_stale_thresh  = 1.5;  // s
  m_last_params_pub   = 0.0;

  m_max_thrust            = 100.0;
  m_max_rudder            = 100.0;
  m_max_yawrate           = 25.0;  // deg/s
  m_speed_integral_limit  = 50.0;
  m_yawrate_integral_limit= 50.0;

  // Actuation-aware integration: OFF by default so a parity replay
  // against the pre-bb::Pid engine is clean. The mission plug enables.
  m_integrate_gate    = false;
  m_gate_stale_thresh = 2.0;   // s
  m_authority         = "";
  m_stop_reason       = "";
  m_mix_saturation    = 0.0;
  m_tstamp_authority   = 0.0;
  m_tstamp_stop_reason = 0.0;
  m_tstamp_mix_sat     = 0.0;
  m_gate_open          = true;
  m_gate_inputs_stale  = false;
}

//---------------------------------------------------------
// OnNewMail

bool BBPID::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key   = msg.GetKey();
    double dval  = msg.GetDouble();

    if(key == m_desired_speed_var) {
      m_desired_speed = dval; m_have_des_speed = true;
      m_tstamp_last_cmd = MOOSTime();
    }
    else if(key == m_desired_heading_var) {
      m_desired_heading = dval; m_have_des_heading = true;
      m_tstamp_last_cmd = MOOSTime();
    }
    else if(key == m_nav_speed_var) {
      m_nav_speed = dval; m_have_nav_speed = true;
    }
    else if(key == m_nav_heading_var) {
      m_nav_heading = dval; m_have_nav_heading = true;
    }
    else if(key == m_nav_yawrate_var) {
      m_nav_yawrate = dval; m_have_nav_yawrate = true;
    }
    // Applied-output telemetry for the integration gate (invariant 11:
    // built for exactly this). Stored with arrival times; staleness is
    // judged in updateIntegrationGate(), not here.
    else if(key == "BB_CMD_AUTHORITY") {
      m_authority = msg.GetAsString(); m_tstamp_authority = MOOSTime();
    }
    else if(key == "NVGR_STOP_REASON") {
      m_stop_reason = msg.GetAsString(); m_tstamp_stop_reason = MOOSTime();
    }
    else if(key == "BB_MIX_SATURATION") {
      m_mix_saturation = dval; m_tstamp_mix_sat = MOOSTime();
    }
    // Runtime gain-schedule control (used by the pBBPID tuner GUI)
    else if(key == "BBPID_SCHEDULE_ENABLE") {
      bool on = msg.IsString() ? (tolower(msg.GetString()) == "true")
                               : (dval != 0.0);
      m_engine.enableGainSchedule(on);
      reportEvent("Gain schedule " + string(on ? "ENABLED" : "DISABLED"));
    }
    else if(key == "BBPID_SCHEDULE_POINT") {
      if(parseSchedulePoint(msg.GetString()))
        reportEvent("Schedule point: " + msg.GetString());
    }
    else if(key == "BBPID_SCHEDULE_CLEAR") {
      m_engine.clearSchedule();
      reportEvent("Gain schedule cleared");
    }
    else if(key == "BBPID_FF_ENABLE") {
      bool on = msg.IsString() ? (tolower(msg.GetString()) == "true")
                               : (dval != 0.0);
      m_engine.setFeedforwardEnable(on);
      reportEvent("Feedforward " + string(on ? "ENABLED" : "DISABLED"));
    }
    // Generic runtime parameter channel: "key=value" (reuses the config
    // parser, so any plug param can be retuned live -- gains, limits,
    // filters, FF coeffs). Used by uBBPIDTuner.
    else if(key == "BBPID_SET") {
      string line  = msg.GetString();
      string param = tolower(biteStringX(line, '='));
      string value = line;
      if(handleConfigLine(param, value)) {
        applyEngineLimits();             // push any deferred limit changes
        reportEvent("set " + param + " = " + value);
        // Provenance for the tuning ledger: the ACTIVE param set,
        // republished on every applied change so a log always shows
        // which parameters produced which behavior.
        Notify("BBPID_PARAMS_ACTIVE", buildParamSnapshot());
      }
      else
        reportRunWarning("BBPID_SET unknown param: " + param);
    }
    else if(strBegins(key, "BBPID_") && !strBegins(key, "BBPID_PARAMS")) {
      handleLiveGainMail(key, dval);
      Notify("BBPID_PARAMS_ACTIVE", buildParamSnapshot());
    }
    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled mail: " + key);
  }
  return(true);
}

//---------------------------------------------------------
// handleLiveGainMail: dynamic weight updates over the DB.
// Posting any one gain re-applies the whole triplet for that loop,
// so partial updates are fine (unset members keep their current value).

void BBPID::handleLiveGainMail(const string& key, double dval)
{
  // Read-modify-write: pull the loop's current triplet from the engine,
  // overwrite only the named component, push it back. A partial update
  // (e.g. only BBPID_YAWRATE_KP) thus leaves the other gains untouched.
  int idx = -1;
  if(strBegins(key, "BBPID_SPEED_"))        idx = 0;
  else if(strBegins(key, "BBPID_HEADING_")) idx = 1;
  else if(strBegins(key, "BBPID_YAWRATE_")) idx = 2;
  if(idx < 0) { reportRunWarning("Unknown gain var: " + key); return; }

  double kp = (idx==0)? m_engine.speedKp() : (idx==1)? m_engine.headingKp() : m_engine.yawrateKp();
  double ki = (idx==0)? m_engine.speedKi() : (idx==1)? m_engine.headingKi() : m_engine.yawrateKi();
  double kd = (idx==0)? m_engine.speedKd() : (idx==1)? m_engine.headingKd() : m_engine.yawrateKd();

  if(strEnds(key, "_KP")) kp = dval;
  else if(strEnds(key, "_KI")) ki = dval;
  else if(strEnds(key, "_KD")) kd = dval;
  else { reportRunWarning("Unknown gain component: " + key); return; }

  if(idx == 0) m_engine.setSpeedGains(kp, ki, kd);
  if(idx == 1) m_engine.setHeadingGains(kp, ki, kd);
  if(idx == 2) m_engine.setYawRateGains(kp, ki, kd);

  reportEvent("Live gain update: " + key + " = " + doubleToStringX(dval,4));
}

//---------------------------------------------------------
// OnConnectToServer

bool BBPID::OnConnectToServer()
{
  // The connect callback can fire before OnStartUp has parsed the
  // config. Registering then subscribes the DEFAULT variable names,
  // and any *_var override (e.g. nav_speed_var = NAV_SURGE) leaves
  // the default subscription orphaned -- its mail spams the
  // unhandled-mail warning every arrival. Register only once the
  // names are final; OnStartUp does the first registration itself,
  // and this path covers genuine reconnects after that.
  if(m_config_parsed)
    registerVariables();
  return(true);
}

//---------------------------------------------------------
// Publish AUTONOMY_CMD.
//
// ON valid: this flag is about whether THIS CONTROLLER's output
// can be trusted right now, not about whether the boat should
// move. Three states map to it:
//
//   holding off  no complete nav/cmd picture yet  -> valid=0
//   stale        helm stopped commanding          -> valid=0
//   active       computing                        -> valid=1
//
// The two invalid cases already command zero, so the boat stops
// either way. The difference is what the LOG says and what the
// mixer does. valid=1 with zeros makes the arbiter SELECT
// autonomy and pass a controlled zero through the slew limiter;
// valid=0 makes it hard-stop with AUTONOMY_INVALID, which names
// the actual fault and resets the limiter so resuming ramps from
// rest rather than from wherever it happened to be.
//
// ON yaw SIGN: the contract requires yaw > 0 = starboard
// (invariant, docs/ibb_navigator_command_pipeline.md section 3).
// `rudder` inherits whatever `rudder_polarity` is configured to,
// so a mis-set polarity now publishes a CONTRACT VIOLATION
// rather than merely an internally inconsistent sign. That is
// precisely why rudder_polarity should go away once the whole
// pipeline shares one convention -- see BBPID_Info and
// docs/control_refactor_plan.md section 15.11. Until then it is
// the one place a wrong config turns the boat the wrong way
// under autonomy while RC still behaves.
void BBPID::publishAutonomyCmd(double surge, double yaw, bool valid)
{
  if(!m_publish_autonomy_cmd)
    return;

  bb::SemanticCommand cmd;
  cmd.version     = bb::kCommandContractVersion;
  cmd.producer    = "pBBPID";
  cmd.epoch       = m_autonomy_epoch;
  cmd.seq         = ++m_autonomy_seq;
  cmd.source_time = MOOSTime();
  cmd.valid       = valid;

  // Clamp rather than reject: the contract's range is [-100,100]
  // and a controller that briefly exceeds it should be limited,
  // not silenced.
  if(surge >  100.0) surge =  100.0;
  if(surge < -100.0) surge = -100.0;
  if(yaw   >  100.0) yaw   =  100.0;
  if(yaw   < -100.0) yaw   = -100.0;

  cmd.surge = valid ? surge : 0.0;
  cmd.yaw   = valid ? yaw   : 0.0;

  // Autonomy carries no authority cap; a mission is either
  // permitted to run or it is not. The arbiter ignores this field
  // for autonomy in any case.
  cmd.authority_limit = 100.0;

  cmd.extra["controller"]   = "pBBPID";
  cmd.extra["control_mode"] = "HEADING_SPEED";

  Notify(m_autonomy_cmd_var, bb::serialize_command(cmd));
}

//---------------------------------------------------------
// updateIntegrationGate: decide, once per tick, whether the loops may
// integrate -- and against which downstream rails.
//
// The controller cannot tell "my command is reaching the water" from
// "the Navigator is holding neutral under a kill" on its own; that is
// the latent kill-window windup. BB_CMD_AUTHORITY and NVGR_STOP_REASON
// carry the answer back across the seat boundary, so: integrate only
// while authority == AUTONOMY && stop == NONE.
//
// FAIL FROZEN: if those inputs go stale (bridge down, frontseat dark)
// integration FREEZES rather than resuming -- P and D stay live, so
// control degrades gracefully instead of winding blind. A run warning
// names the state, because a frozen integral with the bridge quietly
// broken must not read as "tuned Ki does nothing".
//
// Mixer saturation (q > 1) is the second rail: the loop demanding more
// than the mixer can allocate must not integrate the excess. Direction
// is our own last command's sign in the contract frame.

void BBPID::updateIntegrationGate()
{
  if(!m_integrate_gate) {
    m_gate_open = true;
    m_engine.setIntegrateEnable(true);
    m_engine.setExternalSaturation(0, 0);
    return;
  }

  double now = MOOSTime();
  bool fresh = ((now - m_tstamp_authority)   < m_gate_stale_thresh) &&
               ((now - m_tstamp_stop_reason) < m_gate_stale_thresh);

  if(!fresh && !m_gate_inputs_stale) {
    reportRunWarning("integrate gate inputs stale: integration frozen");
    m_gate_inputs_stale = true;
  }
  else if(fresh && m_gate_inputs_stale) {
    retractRunWarning("integrate gate inputs stale: integration frozen");
    m_gate_inputs_stale = false;
  }

  m_gate_open = fresh && (m_authority == "AUTONOMY") &&
                (m_stop_reason == "NONE");
  m_engine.setIntegrateEnable(m_gate_open);

  int surge_sat = 0, yaw_sat = 0;
  if(((now - m_tstamp_mix_sat) < m_gate_stale_thresh) &&
     (m_mix_saturation > 1.0)) {
    surge_sat = (m_last_thrust > 0.0) ? 1 : (m_last_thrust < 0.0) ? -1 : 0;
    yaw_sat   = (m_last_rudder > 0.0) ? 1 : (m_last_rudder < 0.0) ? -1 : 0;
  }
  m_engine.setExternalSaturation(surge_sat, yaw_sat);
}

//---------------------------------------------------------
// Iterate: compute one control tick

bool BBPID::Iterate()
{
  AppCastingMOOSApp::Iterate();
  m_iterations_run++;

  // Heartbeat the plug-default param snapshot (~every 5s) so a tuner that
  // connects at any time can initialize its controls and "reset to defaults".
  if((MOOSTime() - m_last_params_pub) > 5.0) {
    Notify("BBPID_PARAMS_DEFAULT", m_params_default);
    m_last_params_pub = MOOSTime();
  }

  // In derive_heading mode the yaw rate comes from NAV_HEADING, so don't
  // wait on a separate (never-published) yaw-rate variable.
  bool have_yawrate = m_have_nav_yawrate || m_yawrate_derive;
  bool have_nav = m_have_nav_speed && m_have_nav_heading && have_yawrate;
  bool have_cmd = m_have_des_speed && m_have_des_heading;

  // Hold-off until we have a complete picture. Publish an
  // INVALID contract rather than going silent: silence reads as
  // staleness to the arbiter, which is a different fault from
  // "alive but not ready", and only one of those is true here.
  if(!have_nav || !have_cmd) {
    m_active = false;
    publishAutonomyCmd(0.0, 0.0, false);
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  // Safety: if the helm stopped commanding, coast to zero -- but keep the
  // measured-yaw-rate scope alive (desired = 0 while idle) so the tuner
  // doesn't go blank when the boat isn't being driven.
  bool stale = (MOOSTime() - m_tstamp_last_cmd) > m_cmd_stale_thresh;
  Notify("BBPID_CMD_STALE", std::to_string(stale));
  if(stale) {
    m_active = false;
    m_last_thrust = 0.0;
    m_last_rudder = 0.0;
    Notify(m_thrust_var, 0.0);
    Notify(m_rudder_var, 0.0);
    publishAutonomyCmd(0.0, 0.0, false);
    m_engine.computeMeasYawRate(MOOSTime(), m_nav_heading, m_nav_yawrate);
    Notify("BBPID_MEAS_YAWRATE",    m_engine.getMeasYawRate());
    Notify("BBPID_DESIRED_YAWRATE", 0.0);
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  // Resuming command after a hold-off or stale gap: reset the PID
  // loops. Two defects otherwise, both seen on the first
  // handling_block run (2026-08-27, RC repositioning gaps of 43 s
  // and 14.6 s):
  //   1. ScalarPID::Run integrates Ki*e*dt with dt = time since its
  //      LAST call -- the first tick after a gap integrates across
  //      the entire gap in one step, slamming the integral to its
  //      limit (surge resumed at -13.7 with 1.5 m/s commanded).
  //   2. Integral state from before the gap survives into a
  //      situation it says nothing about.
  // resetIntegrators() rebuilds the loops AND zeroes the iteration
  // count, so the post-gap tick is a clean first tick. This is a
  // lifecycle fix, not a retune: gains untouched.
  if(!m_active) {
    m_engine.resetIntegrators();
    reportEvent("command resumed: PID integrators reset");
  }
  m_active = true;

  updateIntegrationGate();

  double thrust = 0.0, rudder = 0.0;
  m_engine.update(MOOSTime(),
                  m_desired_speed,   m_nav_speed,
                  m_desired_heading, m_nav_heading,
                  m_nav_yawrate,
                  thrust, rudder);

  m_last_thrust = thrust;
  m_last_rudder = rudder;

  Notify(m_thrust_var, thrust);
  Notify(m_rudder_var, rudder);

  // surge = thrust, yaw = rudder. They are the same quantities;
  // the contract simply carries them together with the identity
  // and sequence the arbiter's lease needs.
  publishAutonomyCmd(thrust, rudder, true);

  // Debug postings for tuning / plotting
  Notify("BBPID_SPEED_ERROR",    m_engine.getSpeedError());
  Notify("BBPID_HEADING_ERROR",  m_engine.getHeadingError());
  Notify("BBPID_DESIRED_YAWRATE", m_engine.getDesiredYawRate());
  Notify("BBPID_MEAS_YAWRATE",   m_engine.getMeasYawRate());
  Notify("BBPID_YAWRATE_ERROR",  m_engine.getYawRateError());

  // Integral terms (output units): windup health for the scorer --
  // an integral parked at its limit is the signature the 27 Aug
  // fixes exist to prevent, and the steady yawrate I term is direct
  // evidence of yaw-FF bias (what dr/d0 fail to provide, I supplies).
  Notify("BBPID_SPEED_ITERM",   m_engine.speedITerm());
  Notify("BBPID_HEADING_ITERM", m_engine.headingITerm());
  Notify("BBPID_YAWRATE_ITERM", m_engine.yawrateITerm());

  // Integration gate state, on change only (string, greppable).
  string gate = !m_integrate_gate ? "OFF"
              : (m_gate_open ? "OPEN" : "FROZEN");
  if(gate != m_gate_last_pub) {
    Notify("BBPID_GATE", gate);
    m_gate_last_pub = gate;
  }
  if(m_engine.getYawPriorityGain() > 0.0) {
    Notify("BBPID_SPEED_DERATE", m_engine.getSpeedDerate());
    Notify("BBPID_GOV_SPEED",    m_engine.getGovSpeedCmd());
  }
  if(m_engine.feedforwardEnabled()) {
    Notify("BBPID_FF_THRUST", m_engine.getFFThrust());
    Notify("BBPID_FF_RUDDER", m_engine.getFFRudder());
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// OnStartUp: read configuration block

bool BBPID::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  // A fresh epoch per launch. The arbiter rebases its sequence
  // comparison on an epoch change, so a restarted back seat is
  // not mistaken for one whose sequences regressed -- and it must
  // then wait for a valid command in the NEW epoch before
  // autonomy is eligible again.
  m_autonomy_epoch = bb::make_epoch("bbp");

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;
    if(!handleConfigLine(param, value))
      reportUnhandledConfigWarning(*p);
  }

  applyEngineLimits();
  m_config_parsed = true;   // variable names are final; see OnConnectToServer
  registerVariables();
  m_params_default = buildParamSnapshot();   // plug values, for tuner reset
  Notify("BBPID_PARAMS_ACTIVE", m_params_default);  // startup provenance
  return(true);
}

//---------------------------------------------------------
// buildParamSnapshot: ';'-separated key=value of all tunable params, in
// the same config-line syntax the tuner sends back via BBPID_SET.

string BBPID::buildParamSnapshot()
{
  string s;
  s += "speed_pid=" + doubleToStringX(m_engine.speedKp(),4) + ","
                    + doubleToStringX(m_engine.speedKi(),4) + ","
                    + doubleToStringX(m_engine.speedKd(),4);
  s += ";heading_pid=" + doubleToStringX(m_engine.headingKp(),4) + ","
                       + doubleToStringX(m_engine.headingKi(),4) + ","
                       + doubleToStringX(m_engine.headingKd(),4);
  s += ";yawrate_pid=" + doubleToStringX(m_engine.yawrateKp(),4) + ","
                       + doubleToStringX(m_engine.yawrateKi(),4) + ","
                       + doubleToStringX(m_engine.yawrateKd(),4);
  s += ";max_thrust="  + doubleToStringX(m_max_thrust,4);
  s += ";max_rudder="  + doubleToStringX(m_max_rudder,4);
  s += ";max_yawrate=" + doubleToStringX(m_max_yawrate,4);
  s += ";speed_integral_limit="   + doubleToStringX(m_speed_integral_limit,4);
  s += ";yawrate_integral_limit=" + doubleToStringX(m_yawrate_integral_limit,4);
  s += ";des_yawrate_filter=" + doubleToStringX(m_engine.getDesYawRateFilter(),4);
  s += ";yawrate_filter="     + doubleToStringX(m_engine.getYawRateFilter(),4);
  s += ";ff_hold_time="   + doubleToStringX(m_engine.getFFHoldTime(),4);
  s += ";ff_step_limit="  + doubleToStringX(m_engine.getFFStepLimitDeg(),4);
  s += ";max_dt="         + doubleToStringX(m_engine.getMaxDt(),4);
  s += ";antiwindup="     + string(m_engine.antiWindupEnabled() ? "true" : "false");
  s += ";integrate_gate=" + string(m_integrate_gate ? "true" : "false");
  s += ";gate_stale_thresh=" + doubleToStringX(m_gate_stale_thresh,4);
  s += ";ff_speed=" + doubleToStringX(m_engine.ffC0(),4) + ","
                    + doubleToStringX(m_engine.ffCv(),4) + ","
                    + doubleToStringX(m_engine.ffCrr(),5) + ","
                    + doubleToStringX(m_engine.ffCvv(),4);
  s += ";ff_yaw="   + doubleToStringX(m_engine.ffD0(),4) + ","
                    + doubleToStringX(m_engine.ffDr(),4) + ","
                    + doubleToStringX(m_engine.ffDvr(),4);
  s += ";ff_rudder_scale=" + doubleToStringX(m_engine.ffRudderScale(),4);
  s += ";yaw_priority_gain=" + doubleToStringX(m_engine.getYawPriorityGain(),4);
  s += ";yaw_priority_knee=" + doubleToStringX(m_engine.getYawPriorityKnee(),4);
  s += ";min_speed_frac="    + doubleToStringX(m_engine.getMinSpeedFrac(),4);
  s += ";derate_filter="     + doubleToStringX(m_engine.getDerateFilter(),4);
  s += ";ff_enable=" + string(m_engine.feedforwardEnabled() ? "true" : "false");
  s += ";ff_speed_enable=" + string(m_engine.ffSpeedEnabled() ? "true" : "false");
  s += ";ff_yaw_enable="   + string(m_engine.ffYawEnabled()   ? "true" : "false");
  return(s);
}

//---------------------------------------------------------
// applyEngineLimits: push cached limits into the engine once,
// so integral-limit and output-limit are always set as a pair.

void BBPID::applyEngineLimits()
{
  m_engine.setMaxThrust(m_max_thrust);
  m_engine.setMaxRudder(m_max_rudder);
  m_engine.setMaxYawRate(m_max_yawrate);
  m_engine.setSpeedLimits(m_speed_integral_limit, m_max_thrust);
  m_engine.setHeadingLimits(m_max_yawrate, m_max_yawrate);
  m_engine.setYawRateLimits(m_yawrate_integral_limit, m_max_rudder);
}

//---------------------------------------------------------
// handleConfigLine

bool BBPID::handleConfigLine(const string& param, const string& value)
{
  double dval = atof(value.c_str());

  // ---- gains: "speed_pid = kp, ki, kd" style ----
  if(param == "speed_pid") {
    string v = value;
    double kp = atof(biteStringX(v, ',').c_str());
    double ki = atof(biteStringX(v, ',').c_str());
    double kd = atof(biteStringX(v, ',').c_str());
    m_engine.setSpeedGains(kp, ki, kd);
    return(true);
  }
  else if(param == "heading_pid") {
    string v = value;
    double kp = atof(biteStringX(v, ',').c_str());
    double ki = atof(biteStringX(v, ',').c_str());
    double kd = atof(biteStringX(v, ',').c_str());
    m_engine.setHeadingGains(kp, ki, kd);
    return(true);
  }
  else if(param == "yawrate_pid") {
    string v = value;
    double kp = atof(biteStringX(v, ',').c_str());
    double ki = atof(biteStringX(v, ',').c_str());
    double kd = atof(biteStringX(v, ',').c_str());
    m_engine.setYawRateGains(kp, ki, kd);
    return(true);
  }

  // ---- limits (cached; applied together in applyEngineLimits) ----
  else if(param == "max_thrust")             { m_max_thrust = dval;             return(true); }
  else if(param == "max_rudder")             { m_max_rudder = dval;             return(true); }
  else if(param == "max_yawrate")            { m_max_yawrate = dval;            return(true); }
  else if(param == "speed_integral_limit")   { m_speed_integral_limit = dval;   return(true); }
  else if(param == "yawrate_integral_limit") { m_yawrate_integral_limit = dval; return(true); }

  // ---- feedforward (identified model) ----
  else if(param == "ff_enable") {
    m_engine.setFeedforwardEnable(tolower(value) == "true");
    return(true);
  }
  else if(param == "ff_speed_enable") {    // gate the common (speed) FF term
    m_engine.setFeedforwardSpeedEnable(tolower(value) == "true");
    return(true);
  }
  else if(param == "ff_yaw_enable") {      // gate the differential (yaw) FF term
    m_engine.setFeedforwardYawEnable(tolower(value) == "true");
    return(true);
  }
  else if(param == "ff_speed") {           // c0, cv, crr [, cvv]
    string v = value;
    double c0  = atof(biteStringX(v, ',').c_str());
    double cv  = atof(biteStringX(v, ',').c_str());
    double crr = atof(biteStringX(v, ',').c_str());
    // Optional 4th value: quadratic drag term cvv (thrust per
    // (m/s)^2, applied as cvv*v*|v*|). Absent = 0 = legacy linear.
    string cvv_s = biteStringX(v, ',');
    double cvv = cvv_s.empty() ? 0.0 : atof(cvv_s.c_str());
    m_engine.setFeedforwardSpeed(c0, cv, crr, cvv);
    return(true);
  }
  else if(param == "ff_yaw") {             // d0, dr, dvr
    string v = value;
    double d0  = atof(biteStringX(v, ',').c_str());
    double dr  = atof(biteStringX(v, ',').c_str());
    double dvr = atof(biteStringX(v, ',').c_str());
    m_engine.setFeedforwardYaw(d0, dr, dvr);
    return(true);
  }
  else if(param == "ff_rudder_scale") {
    m_engine.setFeedforwardRudderScale(dval);
    return(true);
  }
  else if(param == "des_yawrate_filter") {   // LPF time const [s] on desired yaw rate
    m_engine.setDesYawRateFilter(dval);
    return(true);
  }
  else if(param == "reset_integrators") {    // runtime command (value ignored)
    m_engine.resetIntegrators();
    return(true);
  }

  // ---- reference-FF lifecycle (27 Aug step-and-hold fixes) ----
  else if(param == "ff_hold_time") {         // s; 0 = legacy hold-forever
    m_engine.setFFHoldTime(dval);
    return(true);
  }
  else if(param == "ff_step_limit") {        // deg; steps beyond publish no FF
    m_engine.setFFStepLimitDeg(dval);
    return(true);
  }

  // ---- integration lifecycle ----
  else if(param == "max_dt") {               // s; dt credit cap; 0 = off
    m_engine.setMaxDt(dval);
    return(true);
  }
  else if(param == "antiwindup") {           // tracking AW at the rails
    bool on = false;
    if(!setBooleanOnString(on, value))
      return(false);
    m_engine.setAntiWindup(on);
    return(true);
  }
  else if(param == "integrate_gate") {       // authority-aware integration
    return(setBooleanOnString(m_integrate_gate, value));
  }
  else if(param == "gate_stale_thresh") {    // s; stale gate inputs -> freeze
    if(dval <= 0.0) {
      reportConfigWarning("gate_stale_thresh must be > 0");
      return(false);
    }
    m_gate_stale_thresh = dval;
    return(true);
  }

  // ---- yaw-priority speed governor ----
  // Trades speed for turn performance: when the commanded yaw rate uses more
  // than <knee> of the rate budget, scale the speed setpoint down toward
  // <min_speed_frac>. gain=0 disables (setpoint passes through untouched).
  else if(param == "yaw_priority_gain") {
    m_engine.setYawPriorityGain(dval);
    return(true);
  }
  else if(param == "yaw_priority_knee") {
    if((dval < 0.0) || (dval >= 1.0)) {
      reportConfigWarning("yaw_priority_knee must be in [0,1)");
      return(false);
    }
    m_engine.setYawPriorityKnee(dval);
    return(true);
  }
  else if(param == "min_speed_frac") {
    if((dval <= 0.0) || (dval > 1.0)) {
      reportConfigWarning("min_speed_frac must be in (0,1]");
      return(false);
    }
    m_engine.setMinSpeedFrac(dval);
    return(true);
  }
  else if(param == "derate_filter") {        // LPF time const [s] on the derate
    m_engine.setDerateFilter(dval);
    return(true);
  }

  // ---- gain scheduling ----
  else if(param == "enable_gain_schedule") {
    m_engine.enableGainSchedule(tolower(value) == "true");
    return(true);
  }
  else if(param == "schedule_point") {
    return(parseSchedulePoint(value));
  }

  // ---- yaw-rate feedback source ----
  else if(param == "yawrate_source") {
    string v = tolower(value);
    if(v == "derive_heading")   { m_engine.setYawRateDerive(true);  m_yawrate_derive = true;  }
    else if(v == "external")    { m_engine.setYawRateDerive(false); m_yawrate_derive = false; }
    else { reportConfigWarning("yawrate_source must be external|derive_heading"); return(false); }
    return(true);
  }
  else if(param == "yawrate_filter")  { m_engine.setYawRateFilter(dval);   return(true); }

  // ---- conventions ----
  else if(param == "yawrate_scale")   { m_engine.setYawRateScale(dval);   return(true); }
  else if(param == "rudder_polarity") { m_engine.setRudderPolarity(dval); return(true); }
  else if(param == "allow_reverse")   { m_engine.setAllowReverse(tolower(value)=="true"); return(true); }
  else if(param == "command_stale_thresh") { m_cmd_stale_thresh = dval;   return(true); }

  // ---- variable name overrides ----
  else if(param == "desired_speed_var")   { m_desired_speed_var = value;   return(true); }
  else if(param == "desired_heading_var") { m_desired_heading_var = value; return(true); }
  else if(param == "nav_speed_var")       { m_nav_speed_var = value;       return(true); }
  else if(param == "nav_heading_var")     { m_nav_heading_var = value;     return(true); }
  else if(param == "nav_yawrate_var")     { m_nav_yawrate_var = value;     return(true); }
  else if(param == "thrust_var")          { m_thrust_var = value;          return(true); }
  else if(param == "autonomy_cmd_var")    { m_autonomy_cmd_var = value;    return(true); }
  else if(param == "publish_autonomy_cmd"){ return(setBooleanOnString(m_publish_autonomy_cmd, value)); }
  else if(param == "rudder_var")          { m_rudder_var = value;          return(true); }

  return(false);
}

//---------------------------------------------------------
// parseSchedulePoint: one breakpoint of the speed->gain table.
// Syntax:  schedule_point = speed=1.5, kp=2.0, ki=0.1, kd=0, max_yawrate=20
//   - speed is required.
//   - kp/ki/kd default to 0; max_yawrate defaults to the global max_yawrate.
// Points may be listed in any order; the engine keeps them sorted.

bool BBPID::parseSchedulePoint(const string& value)
{
  bool   have_speed = false;
  double speed = 0.0;
  double kp = 0.0, ki = 0.0, kd = 0.0;
  double mxy = m_engine.getMaxYawRate();

  vector<string> fields = parseString(value, ',');
  for(unsigned int i=0; i<fields.size(); i++) {
    string fld = stripBlankEnds(fields[i]);
    string k   = tolower(biteStringX(fld, '='));
    double v   = atof(fld.c_str());
    if(k == "speed")            { speed = v; have_speed = true; }
    else if(k == "kp")          { kp = v;  }
    else if(k == "ki")          { ki = v;  }
    else if(k == "kd")          { kd = v;  }
    else if(k == "max_yawrate") { mxy = v; }
    else {
      reportConfigWarning("schedule_point: unknown field '" + k + "'");
      return(false);
    }
  }

  if(!have_speed) {
    reportConfigWarning("schedule_point missing required 'speed': " + value);
    return(false);
  }

  m_engine.addSchedulePoint(speed, kp, ki, kd, mxy);
  return(true);
}

//---------------------------------------------------------
// registerVariables

void BBPID::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();

  Register(m_desired_speed_var, 0);
  Register(m_desired_heading_var, 0);
  Register(m_nav_speed_var, 0);
  Register(m_nav_heading_var, 0);
  
  if (m_yawrate_derive) {
    // If yaw rate is derived from heading, we don't need to register the nav yaw rate variable.
    reportEvent("Yaw rate is derived from heading; not registering " + m_nav_yawrate_var);
  } else {
    Register(m_nav_yawrate_var, 0);
  }

  // Live gain retuning
  Register("BBPID_SPEED_KP", 0);   Register("BBPID_SPEED_KI", 0);   Register("BBPID_SPEED_KD", 0);
  Register("BBPID_HEADING_KP", 0); Register("BBPID_HEADING_KI", 0); Register("BBPID_HEADING_KD", 0);
  Register("BBPID_YAWRATE_KP", 0); Register("BBPID_YAWRATE_KI", 0); Register("BBPID_YAWRATE_KD", 0);

  // Runtime gain-schedule control
  Register("BBPID_SCHEDULE_ENABLE", 0);
  Register("BBPID_SCHEDULE_POINT", 0);
  Register("BBPID_SCHEDULE_CLEAR", 0);

  // Runtime feedforward toggle
  Register("BBPID_FF_ENABLE", 0);

  // Generic runtime parameter channel (key=value), used by uBBPIDTuner
  Register("BBPID_SET", 0);

  // Applied-output telemetry for the integration gate. Registered
  // unconditionally (integrate_gate can be flipped live via BBPID_SET,
  // and the values are useful in the appcast regardless).
  Register("BB_CMD_AUTHORITY", 0);
  Register("NVGR_STOP_REASON", 0);
  Register("BB_MIX_SATURATION", 0);
}

//---------------------------------------------------------
// buildReport

bool BBPID::buildReport()
{
  m_msgs << "BlueBoat Cascaded PID (speed + heading->yawrate->rudder)" << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Active: " << (m_active ? "true" : "false");
  m_msgs << "   Iterations: " << m_iterations_run << endl;

  if(m_engine.gainScheduleEnabled()) {
    m_msgs << "Gain schedule: ON  (" << m_engine.scheduleSize() << " pts)";
    m_msgs << "  @ speed=" << doubleToString(m_engine.getSchedSpeed(),2);
    m_msgs << " -> yaw Kp=" << doubleToString(m_engine.yawrateKp(),3);
    m_msgs << ", Ki=" << doubleToString(m_engine.yawrateKi(),3);
    m_msgs << ", Kd=" << doubleToString(m_engine.yawrateKd(),3);
    m_msgs << ", max_yawrate=" << doubleToString(m_engine.getMaxYawRate(),1) << endl;
  } else {
    m_msgs << "Gain schedule: OFF (flat gains)" << endl;
  }
  if(m_engine.feedforwardEnabled())
    m_msgs << "Feedforward:   ON   ff_thrust=" << doubleToString(m_engine.getFFThrust(),2)
           << "  ff_rudder=" << doubleToString(m_engine.getFFRudder(),2) << endl;
  else
    m_msgs << "Feedforward:   OFF" << endl;

  if(m_integrate_gate) {
    m_msgs << "Integrate gate: " << (m_gate_open ? "OPEN" : "FROZEN");
    m_msgs << "  (authority=" << (m_authority.empty() ? "?" : m_authority);
    m_msgs << ", stop=" << (m_stop_reason.empty() ? "?" : m_stop_reason);
    m_msgs << ", mix_sat=" << doubleToString(m_mix_saturation,2);
    if(m_gate_inputs_stale)
      m_msgs << ", INPUTS STALE";
    m_msgs << ")" << endl;
    m_msgs << "Integrals: speed=" << doubleToString(m_engine.speedITerm(),2)
           << "  heading=" << doubleToString(m_engine.headingITerm(),3)
           << "  yawrate=" << doubleToString(m_engine.yawrateITerm(),2) << endl;
  }
  else
    m_msgs << "Integrate gate: OFF (legacy always-integrate)" << endl;
  m_msgs << endl;

  ACTable atab(4);
  atab << "Loop | Desired | Measured | Error";
  atab.addHeaderLines();
  atab << "Speed"
       << doubleToString(m_desired_speed,2)
       << doubleToString(m_nav_speed,2)
       << doubleToString(m_engine.getSpeedError(),3);
  atab << "Heading"
       << doubleToString(m_desired_heading,1)
       << doubleToString(m_nav_heading,1)
       << doubleToString(m_engine.getHeadingError(),2);
  atab << "YawRate"
       << doubleToString(m_engine.getDesiredYawRate(),2)
       << doubleToString(m_engine.getMeasYawRate(),2)
       << doubleToString(m_engine.getYawRateError(),2);
  m_msgs << atab.getFormattedString() << endl << endl;

  ACTable otab(2);
  otab << "Output | Value";
  otab.addHeaderLines();
  otab << m_thrust_var << doubleToString(m_last_thrust,2);
  otab << m_rudder_var << doubleToString(m_last_rudder,2);
  m_msgs << otab.getFormattedString() << endl;

  return(true);
}
