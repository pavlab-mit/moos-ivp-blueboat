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
  m_nav_heading_var     = "NAV_HEADING";
  m_nav_yawrate_var     = "GYRO_Z_LVL_IMU";
  m_thrust_var          = "DESIRED_THRUST";
  m_rudder_var          = "DESIRED_RUDDER";

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

  m_active            = false;
  m_last_thrust       = 0.0;
  m_last_rudder       = 0.0;
  m_iterations_run    = 0;
  m_tstamp_last_cmd   = 0.0;
  m_cmd_stale_thresh  = 1.5;  // s

  m_max_thrust            = 100.0;
  m_max_rudder            = 100.0;
  m_max_yawrate           = 25.0;  // deg/s
  m_speed_integral_limit  = 50.0;
  m_yawrate_integral_limit= 50.0;
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
    else if(strBegins(key, "BBPID_")) {
      handleLiveGainMail(key, dval);
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
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Iterate: compute one control tick

bool BBPID::Iterate()
{
  AppCastingMOOSApp::Iterate();
  m_iterations_run++;

  bool have_nav = m_have_nav_speed && m_have_nav_heading && m_have_nav_yawrate;
  bool have_cmd = m_have_des_speed && m_have_des_heading;

  // Hold-off until we have a complete picture.
  if(!have_nav || !have_cmd) {
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  // Safety: if the helm stopped commanding, coast to zero.
  bool stale = (MOOSTime() - m_tstamp_last_cmd) > m_cmd_stale_thresh;
  if(stale) {
    m_last_thrust = 0.0;
    m_last_rudder = 0.0;
    Notify(m_thrust_var, 0.0);
    Notify(m_rudder_var, 0.0);
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  m_active = true;

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

  // Debug postings for tuning / plotting
  Notify("BBPID_SPEED_ERROR",   m_engine.getSpeedError());
  Notify("BBPID_HEADING_ERROR", m_engine.getHeadingError());
  Notify("BBPID_DESIRED_YAWRATE", m_engine.getDesiredYawRate());
  Notify("BBPID_YAWRATE_ERROR", m_engine.getYawRateError());

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// OnStartUp: read configuration block

bool BBPID::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

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
  registerVariables();
  return(true);
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
  else if(param == "rudder_var")          { m_rudder_var = value;          return(true); }

  return(false);
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
  Register(m_nav_yawrate_var, 0);

  // Live gain retuning
  Register("BBPID_SPEED_KP", 0);   Register("BBPID_SPEED_KI", 0);   Register("BBPID_SPEED_KD", 0);
  Register("BBPID_HEADING_KP", 0); Register("BBPID_HEADING_KI", 0); Register("BBPID_HEADING_KD", 0);
  Register("BBPID_YAWRATE_KP", 0); Register("BBPID_YAWRATE_KI", 0); Register("BBPID_YAWRATE_KD", 0);
}

//---------------------------------------------------------
// buildReport

bool BBPID::buildReport()
{
  m_msgs << "BlueBoat Cascaded PID (speed + heading->yawrate->rudder)" << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Active: " << (m_active ? "true" : "false");
  m_msgs << "   Iterations: " << m_iterations_run << endl << endl;

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
