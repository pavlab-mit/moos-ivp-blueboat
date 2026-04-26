/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: pBB_Health/BB_Health.cpp
   Last Ed: 2026-03-25
     Brief: Health monitoring for BlueBoat platform. Monitors
            battery voltage/current, thermal status, EKF health,
            GPS quality, and RC connection state. Publishes visual
            indicators for RC mode.
*************************************************************/

#include <iterator>
#include <cmath>
#include "MBUtils.h"
#include "ACTable.h"
#include "BB_Health.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

BB_Health::BB_Health()
{
  // Configuration defaults
  m_debug = false;
  m_debug_stream = nullptr;
  m_stale_time = 3.0;
  m_low_voltage_thresh = 22.0;
  m_high_current_thresh = 30.0;
  m_high_pi_temp_thresh = 80.0;
  m_high_internal_temp_thresh = 50.0;
  m_hdop_thresh = 2.0;
  m_rc_circle_radius = 2.5;
  m_vname = "";

  // Battery/Power state
  m_rolling_voltage = 0.0;
  m_rolling_current = 0.0;
  m_rolling_voltage_time = 0.0;
  m_rolling_current_time = 0.0;
  m_stale_voltage = true;
  m_stale_current = true;

  // Thermal state
  m_rpi_temp = 0.0;
  m_internal_temp = 0.0;
  m_rpi_temp_time = 0.0;
  m_internal_temp_time = 0.0;
  m_stale_rpi_temp = true;
  m_stale_internal_temp = true;

  // EKF state
  m_ekf_status = "UNKNOWN";
  m_ekf_status_time = 0.0;
  m_stale_ekf = true;

  // GPS state
  m_hdop = 99.0;
  m_fix_type = "NONE";
  m_num_sats = 0;
  m_gps_lock = false;
  m_gps_state_time = 0.0;
  m_stale_gps = true;

  // RC state
  m_rc_connected = false;
  m_rc_ch6 = 0.0;
  m_rc_connected_time = 0.0;
  m_rc_ch6_time = 0.0;
  m_stale_rc = true;

  // Navigation state
  m_nav_x = 0.0;
  m_nav_y = 0.0;
  m_nav_time = 0.0;

  // RC mode tracking
  m_rc_mode_active = false;
  m_rc_mode_prev = false;
}

//---------------------------------------------------------
// Destructor

BB_Health::~BB_Health()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool BB_Health::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    double mtime  = msg.GetTime();
    double dval   = msg.GetDouble();
    string sval   = msg.GetString();

    // Battery/Power from iBBThrustCtrl
    if(key == "NVGR_ROLLING_VOLTAGE") {
      m_rolling_voltage = dval;
      m_rolling_voltage_time = mtime;
    }
    else if(key == "NVGR_ROLLING_CURRENT") {
      m_rolling_current = dval;
      m_rolling_current_time = mtime;
    }
    // Thermal from iBBThrustCtrl
    else if(key == "RPI_TEMP") {
      m_rpi_temp = dval;
      m_rpi_temp_time = mtime;
    }
    else if(key == "NVGTR_IT_C") {
      m_internal_temp = dval;
      m_internal_temp_time = mtime;
    }
    // EKF status from pBB_DGPS_EKF
    else if(key == "EKF_STATUS") {
      m_ekf_status = sval;
      m_ekf_status_time = mtime;
    }
    // GPS state bundle (time-bundled fix state from iUnicore)
    else if(key == "FIX_STATE_DGNSS") {
      parseGPSState(sval);
      m_gps_state_time = mtime;
    }
    // RC from iRCReader
    else if(key == "RC_CONNECTED") {
      m_rc_connected = (sval == "true");
      m_rc_connected_time = mtime;
    }
    else if(key == "RC_CH6") {
      m_rc_ch6 = dval;
      m_rc_ch6_time = mtime;
    }
    // Navigation for visuals
    else if(key == "NAV_X") {
      m_nav_x = dval;
      m_nav_time = mtime;
    }
    else if(key == "NAV_Y") {
      m_nav_y = dval;
    }
    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: parseGPSState()
// Parse FIX_STATE_DGNSS string for quality metrics

bool BB_Health::parseGPSState(const string &sval)
{
  vector<string> svector = parseString(sval, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = biteStringX(svector[i], '=');
    string value = svector[i];

    if(param == "HDOP")
      m_hdop = atof(value.c_str());
    else if(param == "FIX_TYPE")
      m_fix_type = value;
    else if(param == "NUM_SATS")
      m_num_sats = atoi(value.c_str());
    else if(param == "GPS_LOCK")
      m_gps_lock = (value == "true");
  }
  return(true);
}

//---------------------------------------------------------
// Procedure: dbg_print()

bool BB_Health::dbg_print(const char *format, ...)
{
  if(m_debug) {
    va_list args;
    va_start(args, format);
    m_debug_stream = fopen(m_fname, "a");
    if(m_debug_stream != nullptr) {
      vfprintf(m_debug_stream, format, args);
      fclose(m_debug_stream);
      va_end(args);
      return true;
    }
    else {
      reportRunWarning("Debug mode enabled but file could not be opened");
      va_end(args);
      return false;
    }
  }
  return false;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool BB_Health::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool BB_Health::Iterate()
{
  AppCastingMOOSApp::Iterate();

  double now = MOOSTime();

  // Update staleness flags
  m_stale_voltage = ((now - m_rolling_voltage_time) >= m_stale_time);
  m_stale_current = ((now - m_rolling_current_time) >= m_stale_time);
  m_stale_rpi_temp = ((now - m_rpi_temp_time) >= m_stale_time);
  m_stale_internal_temp = ((now - m_internal_temp_time) >= m_stale_time);
  m_stale_ekf = ((now - m_ekf_status_time) >= m_stale_time);
  m_stale_gps = ((now - m_gps_state_time) >= m_stale_time);
  m_stale_rc = ((now - m_rc_ch6_time) >= m_stale_time);

  // Determine RC mode: CH6 == 2.0 means RC mode active
  m_rc_mode_prev = m_rc_mode_active;
  m_rc_mode_active = (fabs(m_rc_ch6 - 2.0) < 0.01) && m_rc_connected;

  // Check for transition into RC mode
  if(m_rc_mode_active && !m_rc_mode_prev) {
    publishRCModePulse();
  }

  // Check for transition out of RC mode
  if(!m_rc_mode_active && m_rc_mode_prev) {
    hideRCModeVisuals();
  }

  // Publish persistent RC mode circle if active
  if(m_rc_mode_active) {
    publishRCModeVisuals();
  }

  // Publish health warnings
  if(!m_stale_voltage && m_rolling_voltage < m_low_voltage_thresh)
    reportRunWarning("Low battery voltage: " + doubleToString(m_rolling_voltage, 1) + "V");

  if(!m_stale_current && m_rolling_current > m_high_current_thresh)
    reportRunWarning("High current draw: " + doubleToString(m_rolling_current, 1) + "A");

  if(!m_stale_rpi_temp && m_rpi_temp > m_high_pi_temp_thresh)
    reportRunWarning("High RPi temp: " + doubleToString(m_rpi_temp, 1) + "C");

  if(!m_stale_internal_temp && m_internal_temp > m_high_internal_temp_thresh)
    reportRunWarning("High internal temp: " + doubleToString(m_internal_temp, 1) + "C");

  if(!m_stale_ekf && (m_ekf_status == "UNHEALTHY" || m_ekf_status == "GPS_INVALID"))
    reportRunWarning("EKF status: " + m_ekf_status);

  if(!m_stale_gps && m_hdop > m_hdop_thresh)
    reportRunWarning("Poor GPS quality, HDOP: " + doubleToString(m_hdop, 2));

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: publishRCModeVisuals()
// Publish persistent yellow circle around vehicle when in RC mode

void BB_Health::publishRCModeVisuals()
{
  XYCircle circle(m_nav_x, m_nav_y, m_rc_circle_radius);
  circle.set_label(m_vname + "_rc_mode");
  circle.set_color("edge", "yellow");
  circle.set_color("fill", "blue");
  circle.set_transparency(0.9);
  circle.set_edge_size(2);

  Notify("VIEW_CIRCLE", circle.get_spec());
}

//---------------------------------------------------------
// Procedure: publishRCModePulse()
// Publish range pulse on transition to RC mode

void BB_Health::publishRCModePulse()
{
  XYRangePulse pulse(m_nav_x, m_nav_y);
  pulse.set_label(m_vname + "_rc_pulse");
  pulse.set_rad(50);
  pulse.set_duration(6);
  pulse.set_fill(0.9);
  pulse.set_color("edge", "white");
  pulse.set_color("fill", "white");

  Notify("VIEW_RANGE_PULSE", pulse.get_spec());
}

//---------------------------------------------------------
// Procedure: hideRCModeVisuals()
// Hide the RC mode circle when exiting RC mode

void BB_Health::hideRCModeVisuals()
{
  XYCircle circle(m_nav_x, m_nav_y, m_rc_circle_radius);
  circle.set_label(m_vname + "_rc_mode");
  circle.set_active(false);

  Notify("VIEW_CIRCLE", circle.get_spec());
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool BB_Health::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());
  m_app_name = GetAppName();

  // Get vehicle name from community
  if(!m_MissionReader.GetValue("Community", m_vname))
    reportConfigWarning("No Community/vehicle name found");

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;

    if(param == "stale_time") {
      handled = setPosDoubleOnString(m_stale_time, value);
    }
    else if(param == "low_voltage_thresh") {
      handled = setPosDoubleOnString(m_low_voltage_thresh, value);
    }
    else if(param == "high_current_thresh") {
      handled = setPosDoubleOnString(m_high_current_thresh, value);
    }
    else if(param == "high_pi_temp_thresh") {
      handled = setPosDoubleOnString(m_high_pi_temp_thresh, value);
    }
    else if(param == "high_internal_temp_thresh") {
      handled = setPosDoubleOnString(m_high_internal_temp_thresh, value);
    }
    else if(param == "hdop_thresh") {
      handled = setPosDoubleOnString(m_hdop_thresh, value);
    }
    else if(param == "rc_circle_radius") {
      handled = setPosDoubleOnString(m_rc_circle_radius, value);
    }
    else if(param == "debug") {
      m_debug = (tolower(value) == "true");
      if(m_debug) {
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

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void BB_Health::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();

  // Battery/Power from iBBThrustCtrl
  Register("NVGR_ROLLING_VOLTAGE", 0);
  Register("NVGR_ROLLING_CURRENT", 0);

  // Thermal from iBBThrustCtrl
  Register("RPI_TEMP", 0);
  Register("NVGTR_IT_C", 0);

  // EKF status from pBB_DGPS_EKF
  Register("EKF_STATUS", 0);

  // GPS state bundle
  Register("FIX_STATE_DGNSS", 0);

  // RC from iRCReader
  Register("RC_CONNECTED", 0);
  Register("RC_CH6", 0);

  // Navigation for visuals
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool BB_Health::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "pBB_Health - BlueBoat Health Monitor        " << endl;
  m_msgs << "============================================" << endl;
  m_msgs << endl;

  // Power Section
  m_msgs << "--- Power ---" << endl;
  ACTable pwr_tab(3);
  pwr_tab << "Metric | Value | Status";
  pwr_tab.addHeaderLines();

  string volt_status = m_stale_voltage ? "STALE" :
    (m_rolling_voltage < m_low_voltage_thresh ? "LOW" : "OK");
  pwr_tab << "Voltage" << doubleToString(m_rolling_voltage, 1) + " V" << volt_status;

  string curr_status = m_stale_current ? "STALE" :
    (m_rolling_current > m_high_current_thresh ? "HIGH" : "OK");
  pwr_tab << "Current" << doubleToString(m_rolling_current, 1) + " A" << curr_status;

  m_msgs << pwr_tab.getFormattedString() << endl << endl;

  // Thermal Section
  m_msgs << "--- Thermal ---" << endl;
  ACTable therm_tab(3);
  therm_tab << "Sensor | Temp (C) | Status";
  therm_tab.addHeaderLines();

  string rpi_status = m_stale_rpi_temp ? "STALE" :
    (m_rpi_temp > m_high_pi_temp_thresh ? "HIGH" : "OK");
  therm_tab << "RPi" << doubleToString(m_rpi_temp, 1) << rpi_status;

  string int_status = m_stale_internal_temp ? "STALE" :
    (m_internal_temp > m_high_internal_temp_thresh ? "HIGH" : "OK");
  therm_tab << "Internal" << doubleToString(m_internal_temp, 1) << int_status;

  m_msgs << therm_tab.getFormattedString() << endl << endl;

  // EKF Section
  m_msgs << "--- EKF ---" << endl;
  string ekf_display = m_stale_ekf ? "STALE" : m_ekf_status;
  m_msgs << "  Status: " << ekf_display << endl << endl;

  // GPS Section
  m_msgs << "--- GPS ---" << endl;
  ACTable gps_tab(2);
  gps_tab << "Metric | Value";
  gps_tab.addHeaderLines();
  if(m_stale_gps) {
    gps_tab << "Status" << "STALE";
  } else {
    gps_tab << "Lock" << (m_gps_lock ? "YES" : "NO");
    gps_tab << "Fix Type" << m_fix_type;
    gps_tab << "Satellites" << intToString(m_num_sats);
    string hdop_str = doubleToString(m_hdop, 2);
    if(m_hdop > m_hdop_thresh)
      hdop_str += " (POOR)";
    gps_tab << "HDOP" << hdop_str;
  }
  m_msgs << gps_tab.getFormattedString() << endl << endl;

  // RC Section
  m_msgs << "--- RC ---" << endl;
  ACTable rc_tab(2);
  rc_tab << "Metric | Value";
  rc_tab.addHeaderLines();
  rc_tab << "Connected" << (m_rc_connected ? "YES" : "NO");
  rc_tab << "CH6" << doubleToString(m_rc_ch6, 1);
  rc_tab << "RC Mode" << (m_rc_mode_active ? "ACTIVE" : "INACTIVE");
  m_msgs << rc_tab.getFormattedString() << endl;

  return(true);
}
