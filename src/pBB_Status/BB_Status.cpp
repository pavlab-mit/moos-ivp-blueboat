/*************************************************************
      Name:
      Orgn: MIT, Cambridge MA
      File: pBB_Status/BB_Status.cpp
   Last Ed: 2026-06-03
     Brief: Front-seat status consolidator. See BB_Status.h.
*************************************************************/

#include <cmath>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "MBUtils.h"
#include "ACTable.h"
#include "BB_Status.h"

using namespace std;

//---------------------------------------------------------
// Helper: read a boolean from a MOOS message regardless of
// whether it arrived as a string ("true"/"1"/"on") or a double.

static bool msgBool(CMOOSMsg &msg)
{
  if(msg.IsString()) {
    string s = tolower(msg.GetString());
    return (s == "true" || s == "1" || s == "on" || s == "active" || s == "yes");
  }
  return (msg.GetDouble() != 0.0);
}

//---------------------------------------------------------
// Constructor

BB_Status::BB_Status()
{
  // Configuration defaults
  m_status_var        = "BB_STATUS";
  m_vname             = "";
  m_stale_time        = 3.0;
  m_low_voltage_thresh = 22.0;
  m_publish_interval  = 0.5;     // 2 Hz
  m_last_publish_time = 0.0;

  // Shoreside UDP transport (disabled until tx_ip is configured)
  m_tx_ip   = "";
  m_tx_port = 9300;
  m_sockfd  = -1;
  m_tx_sent = 0;
  m_tx_errs = 0;
  memset(&m_dest, 0, sizeof(m_dest));

  // Power
  m_volt = m_curr = m_power = 0.0;
  m_volt_time = m_curr_time = m_power_time = 0.0;

  // Thermal / enclosure
  m_rpi_temp = m_int_temp = m_int_kpa = 0.0;
  m_rpi_temp_time = m_int_temp_time = m_int_kpa_time = 0.0;

  // RC
  m_rc_connected = false;
  m_rc_failsafe  = false;
  m_rc_deadman   = false;
  m_rc_ch6       = 0.0;
  m_rc_time      = 0.0;

  // Laptop teleop
  m_teleop_engaged = false;
  m_teleop_time    = 0.0;

  // Propulsion
  m_thr_l = m_thr_r = m_des_l = m_des_r = 0.0;
  m_thr_timeout = false;
  m_thr_time = m_des_time = 0.0;

  // Navigation / GPS
  m_nav_lat = m_nav_lon = m_nav_spd = m_nav_hdg = 0.0;
  m_nav_time = 0.0;
  m_fix_type = "NONE";
  m_num_sats = 0;
  m_hdop = 99.0;
  m_gps_time = 0.0;

  // Autonomy (brokered)
  m_helm_state = "";
  m_mode = "";
  m_deploy = false;
  m_all_stop = false;
  m_helm_time = m_mode_time = m_deploy_time = m_allstop_time = 0.0;

  // Staleness flags
  m_stale_power = m_stale_temp = m_stale_rc = true;
  m_stale_thr = m_stale_nav = m_stale_gps = m_stale_helm = true;

  m_posts = 0;
}

BB_Status::~BB_Status()
{
  if(m_sockfd >= 0)
    close(m_sockfd);
}

//---------------------------------------------------------
// setupSocket -- open a non-blocking UDP socket toward the
// shoreside collector. Non-blocking so a full send buffer can
// never stall Iterate(); a dropped status frame is harmless.

bool BB_Status::setupSocket()
{
  if(m_tx_ip == "")          // transport disabled: MOOS-only
    return true;

  m_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if(m_sockfd < 0) {
    reportConfigWarning("pBB_Status: could not open UDP socket");
    return false;
  }
  int flags = fcntl(m_sockfd, F_GETFL, 0);
  fcntl(m_sockfd, F_SETFL, flags | O_NONBLOCK);

  m_dest.sin_family = AF_INET;
  m_dest.sin_port   = htons((uint16_t)m_tx_port);
  if(inet_pton(AF_INET, m_tx_ip.c_str(), &m_dest.sin_addr) != 1) {
    reportConfigWarning("pBB_Status: bad tx_ip [" + m_tx_ip + "]");
    close(m_sockfd);
    m_sockfd = -1;
    return false;
  }
  return true;
}

//---------------------------------------------------------
// OnNewMail

bool BB_Status::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p = NewMail.begin(); p != NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key   = msg.GetKey();
    double mtime = msg.GetTime();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString();

    // ---- Power (native front seat) ----
    if(key == "NVGR_ROLLING_VOLTAGE")      { m_volt = dval;  m_volt_time = mtime; }
    else if(key == "NVGR_ROLLING_CURRENT") { m_curr = dval;  m_curr_time = mtime; }
    else if(key == "NVGR_ROLLING_POWER")   { m_power = dval; m_power_time = mtime; }

    // ---- Thermal / enclosure (native front seat) ----
    else if(key == "RPI_TEMP")   { m_rpi_temp = dval; m_rpi_temp_time = mtime; }
    else if(key == "NVGTR_IT_C") { m_int_temp = dval; m_int_temp_time = mtime; }
    else if(key == "NVGTR_IP_KPA"){ m_int_kpa = dval; m_int_kpa_time = mtime; }

    // ---- RC / override (native front seat) ----
    else if(key == "RC_CONNECTED")            { m_rc_connected = msgBool(msg); m_rc_time = mtime; }
    else if(key == "RC_FAILSAFE")             { m_rc_failsafe = msgBool(msg);  m_rc_time = mtime; }
    else if(key == "NVGR_RC_DEADMAN_ACTIVE")  { m_rc_deadman = msgBool(msg);   m_rc_time = mtime; }
    else if(key == "RC_CH6")                  { m_rc_ch6 = dval;               m_rc_time = mtime; }

    // ---- Laptop teleop (native front seat) ----
    else if(key == "NVGR_TELEOP_ENGAGED")     { m_teleop_engaged = msgBool(msg); m_teleop_time = mtime; }

    // ---- Propulsion ----
    else if(key == "NVGR_THRUST_LEFT")   { m_thr_l = dval; m_thr_time = mtime; }
    else if(key == "NVGR_THRUST_RIGHT")  { m_thr_r = dval; m_thr_time = mtime; }
    else if(key == "DESIRED_THRUST_L")   { m_des_l = dval; m_des_time = mtime; }
    else if(key == "DESIRED_THRUST_R")   { m_des_r = dval; m_des_time = mtime; }
    else if(key == "NVGR_THRUST_TIMEOUT"){ m_thr_timeout = msgBool(msg); }

    // ---- Navigation / GPS (native front seat) ----
    else if(key == "NAV_LAT_DGNSS")   { m_nav_lat = dval; m_nav_time = mtime; }
    else if(key == "NAV_LONG_DGNSS")  { m_nav_lon = dval; m_nav_time = mtime; }
    else if(key == "NAV_SPEED_DGNSS") { m_nav_spd = dval; m_nav_time = mtime; }
    else if(key == "GPS_HEADING_DGNSS"){ m_nav_hdg = dval; m_nav_time = mtime; }
    else if(key == "FIX_STATE_DGNSS") {
      vector<string> sv = parseString(sval, ',');
      for(unsigned int i=0; i<sv.size(); i++) {
        string param = biteStringX(sv[i], '=');
        string value = sv[i];
        if(param == "HDOP")          m_hdop = atof(value.c_str());
        else if(param == "FIX_TYPE") m_fix_type = value;
        else if(param == "NUM_SATS") m_num_sats = atoi(value.c_str());
      }
      m_gps_time = mtime;
    }

    // ---- Autonomy (brokered from backseat; may be absent) ----
    else if(key == "IVPHELM_STATE") { m_helm_state = sval; m_helm_time = mtime; }
    else if(key == "MODE")          { m_mode = sval;       m_mode_time = mtime; }
    else if(key == "DEPLOY")        { m_deploy = msgBool(msg);   m_deploy_time = mtime; }
    else if(key == "ALL_STOP")      { m_all_stop = msgBool(msg); m_allstop_time = mtime; }

    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }
  return(true);
}

//---------------------------------------------------------
// OnConnectToServer

bool BB_Status::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// deriveMode -- fuse front-seat RC truth with backseat helm state.
// RC override (front seat) wins over autonomy, because when the pilot
// is in manual the front seat ignores backseat thrust entirely.

string BB_Status::deriveMode() const
{
  if(m_stale_rc)
    return "RC_UNKNOWN";
  if(m_rc_failsafe)
    return "FAILSAFE";

  // CH6 == 2.0 (and connected) is the manual/RC-override position,
  // matching the convention used by pBB_Health.
  bool rc_manual = (fabs(m_rc_ch6 - 2.0) < 0.01) && m_rc_connected;
  if(rc_manual)
    return "MANUAL";

  // Laptop teleop (the navigator interface only engages it when
  // RC override is not active, so ordering below MANUAL is exact).
  bool teleop = m_teleop_engaged &&
                ((MOOSTime() - m_teleop_time) < m_stale_time);
  if(teleop)
    return "TELEOP";

  if(!m_stale_helm && m_all_stop)
    return "ALLSTOP";

  if(m_stale_helm || m_helm_state == "")
    return "NO_HELM";          // boat powered, no autonomy mission talking

  if(m_helm_state == "DRIVE") return "AUTO";
  if(m_helm_state == "PARK")  return "HOLD";
  return m_helm_state;          // STANDBY / MALCONFIG / etc. pass through
}

//---------------------------------------------------------
// missionRunning

bool BB_Status::missionRunning() const
{
  return (!m_stale_helm && m_deploy && m_helm_state == "DRIVE");
}

//---------------------------------------------------------
// staleList -- pipe-joined names of stale input groups (or "none")

string BB_Status::staleList() const
{
  string s;
  if(m_stale_power) s += "power|";
  if(m_stale_temp)  s += "temp|";
  if(m_stale_rc)    s += "rc|";
  if(m_stale_thr)   s += "thrust|";
  if(m_stale_nav)   s += "nav|";
  if(m_stale_gps)   s += "gps|";
  if(m_stale_helm)  s += "helm|";
  if(s.empty()) return "none";
  s.erase(s.size()-1);          // drop trailing '|'
  return s;
}

//---------------------------------------------------------
// buildStatusString -- the single consolidated BB_STATUS report.
// Comma-separated key=value (NODE_REPORT idiom), compact for the mesh.

string BB_Status::buildStatusString()
{
  vector<string> f;

  f.push_back("vname=" + m_vname);
  f.push_back("utc=" + doubleToString(MOOSTime(), 2));

  // Headline: fused mode + mission + raw helm (so disagreements stay visible)
  f.push_back("mode=" + deriveMode());
  f.push_back("mission=" + string(missionRunning() ? "true" : "false"));
  f.push_back("helm=" + string(m_helm_state == "" ? "NONE" : m_helm_state));
  f.push_back("deploy=" + string(m_deploy ? "true" : "false"));

  // Power
  string batt = m_stale_power ? "STALE" :
                (m_volt < m_low_voltage_thresh ? "LOW" : "OK");
  f.push_back("volt=" + doubleToString(m_volt, 1));
  f.push_back("curr=" + doubleToString(m_curr, 1));
  f.push_back("power=" + doubleToString(m_power, 1));
  f.push_back("batt=" + batt);

  // RC / override
  f.push_back("rc=" + string(m_rc_connected ? "conn" : "disc"));
  f.push_back("failsafe=" + string(m_rc_failsafe ? "true" : "false"));
  f.push_back("deadman=" + string(m_rc_deadman ? "true" : "false"));
  f.push_back("teleop=" + string(m_teleop_engaged ? "true" : "false"));

  // Propulsion: commanded (backseat) vs applied (front seat)
  f.push_back("des_l=" + doubleToString(m_des_l, 0));
  f.push_back("des_r=" + doubleToString(m_des_r, 0));
  f.push_back("thr_l=" + doubleToString(m_thr_l, 0));
  f.push_back("thr_r=" + doubleToString(m_thr_r, 0));
  f.push_back("allstop=" + string(m_all_stop ? "true" : "false"));

  // Thermal / enclosure
  f.push_back("rpi_t=" + doubleToString(m_rpi_temp, 1));
  f.push_back("int_t=" + doubleToString(m_int_temp, 1));
  f.push_back("int_kpa=" + doubleToString(m_int_kpa, 2));

  // GPS / nav
  f.push_back("fix=" + (m_stale_gps ? string("STALE") : m_fix_type));
  f.push_back("sats=" + intToString(m_num_sats));
  f.push_back("hdop=" + doubleToString(m_hdop, 2));
  f.push_back("lat=" + doubleToString(m_nav_lat, 7));
  f.push_back("lon=" + doubleToString(m_nav_lon, 7));
  f.push_back("spd=" + doubleToString(m_nav_spd, 2));
  f.push_back("hdg=" + doubleToString(m_nav_hdg, 1));

  // Freshness summary
  f.push_back("stale=" + staleList());

  // join
  string out;
  for(unsigned int i=0; i<f.size(); i++) {
    if(i) out += ",";
    out += f[i];
  }
  return out;
}

//---------------------------------------------------------
// Iterate

bool BB_Status::Iterate()
{
  AppCastingMOOSApp::Iterate();
  double now = MOOSTime();

  // Recompute staleness for each input group
  m_stale_power = ((now - m_volt_time) >= m_stale_time);
  m_stale_temp  = ((now - m_rpi_temp_time) >= m_stale_time) &&
                  ((now - m_int_temp_time) >= m_stale_time);
  m_stale_rc    = ((now - m_rc_time) >= m_stale_time);
  m_stale_thr   = ((now - m_thr_time) >= m_stale_time);
  m_stale_nav   = ((now - m_nav_time) >= m_stale_time);
  m_stale_gps   = ((now - m_gps_time) >= m_stale_time);
  m_stale_helm  = ((now - m_helm_time) >= m_stale_time);

  // Publish at the configured rate (independent of AppTick)
  if((now - m_last_publish_time) >= m_publish_interval) {
    string status = buildStatusString();

    // (1) Local MOOS post -- for scoping, logging, on-boat tools.
    Notify(m_status_var, status);

    // (2) Shoreside push -- fire-and-forget UDP to the collector.
    if(m_sockfd >= 0) {
      ssize_t n = sendto(m_sockfd, status.c_str(), status.size(), 0,
                         (struct sockaddr *)&m_dest, sizeof(m_dest));
      if(n < 0) m_tx_errs++;
      else      m_tx_sent++;
    }

    m_last_publish_time = now;
    m_posts++;
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// OnStartUp

bool BB_Status::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  if(!m_MissionReader.GetValue("Community", m_vname))
    reportConfigWarning("No Community/vehicle name found");

  STRING_LIST::iterator p;
  for(p = sParams.begin(); p != sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;
    bool handled = false;

    if(param == "status_var") {
      if(value != "") { m_status_var = stripBlankEnds(value); handled = true; }
    }
    else if(param == "stale_time")
      handled = setPosDoubleOnString(m_stale_time, value);
    else if(param == "low_voltage_thresh")
      handled = setPosDoubleOnString(m_low_voltage_thresh, value);
    else if(param == "publish_interval")
      handled = setPosDoubleOnString(m_publish_interval, value);
    else if(param == "tx_ip") {
      m_tx_ip = stripBlankEnds(value);
      handled = true;
    }
    else if(param == "tx_port") {
      int port = atoi(value.c_str());
      if(port > 0 && port < 65536) { m_tx_port = port; handled = true; }
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  setupSocket();
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// registerVariables

void BB_Status::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();

  // Power (native front seat)
  Register("NVGR_ROLLING_VOLTAGE", 0);
  Register("NVGR_ROLLING_CURRENT", 0);
  Register("NVGR_ROLLING_POWER", 0);

  // Thermal / enclosure (native front seat)
  Register("RPI_TEMP", 0);
  Register("NVGTR_IT_C", 0);
  Register("NVGTR_IP_KPA", 0);

  // RC / override (native front seat)
  Register("RC_CONNECTED", 0);
  Register("RC_FAILSAFE", 0);
  Register("NVGR_RC_DEADMAN_ACTIVE", 0);
  Register("RC_CH6", 0);

  // Laptop teleop (native front seat)
  Register("NVGR_TELEOP_ENGAGED", 0);

  // Propulsion (applied native; commanded brokered)
  Register("NVGR_THRUST_LEFT", 0);
  Register("NVGR_THRUST_RIGHT", 0);
  Register("DESIRED_THRUST_L", 0);
  Register("DESIRED_THRUST_R", 0);
  Register("NVGR_THRUST_TIMEOUT", 0);

  // Navigation / GPS (native front seat)
  Register("NAV_LAT_DGNSS", 0);
  Register("NAV_LONG_DGNSS", 0);
  Register("NAV_SPEED_DGNSS", 0);
  Register("GPS_HEADING_DGNSS", 0);
  Register("FIX_STATE_DGNSS", 0);

  // Autonomy (brokered from backseat -- requires widening
  // iBackSeatBroker.tx_vars to include IVPHELM_STATE, DEPLOY, MODE;
  // DESIRED_THRUST_* and ALL_STOP already cross today)
  Register("IVPHELM_STATE", 0);
  Register("MODE", 0);
  Register("DEPLOY", 0);
  Register("ALL_STOP", 0);
}

//------------------------------------------------------------
// buildReport (AppCast)

bool BB_Status::buildReport()
{
  m_msgs << "pBB_Status - Front-Seat Status Consolidator" << endl;
  m_msgs << "Publishing: " << m_status_var
         << "  (every " << doubleToString(m_publish_interval,2) << "s, "
         << "posts=" << m_posts << ")" << endl;
  if(m_tx_ip == "")
    m_msgs << "Shore push : disabled (MOOS-only; set tx_ip to enable)" << endl;
  else
    m_msgs << "Shore push : " << m_tx_ip << ":" << m_tx_port
           << "  sent=" << m_tx_sent << " errs=" << m_tx_errs << endl;
  m_msgs << "============================================" << endl << endl;

  m_msgs << "Fused mode : " << deriveMode()
         << "   mission_running=" << (missionRunning() ? "YES" : "no") << endl;
  m_msgs << "Helm (raw) : " << (m_helm_state=="" ? "NONE" : m_helm_state)
         << (m_stale_helm ? "  [STALE/absent - widen iBackSeatBroker.tx_vars]" : "")
         << endl << endl;

  ACTable tab(3);
  tab << "Group | Value | Status";
  tab.addHeaderLines();
  tab << "Voltage" << doubleToString(m_volt,1)+" V"
      << (m_stale_power ? "STALE" : (m_volt<m_low_voltage_thresh?"LOW":"OK"));
  tab << "Current" << doubleToString(m_curr,1)+" A" << (m_stale_power?"STALE":"OK");
  tab << "RC" << (m_rc_connected?"conn":"disc")+string(" ch6=")+doubleToString(m_rc_ch6,1)
      << (m_stale_rc?"STALE":(m_rc_failsafe?"FAILSAFE":"OK"));
  tab << "Thrust c/a" << "L "+doubleToString(m_des_l,0)+"/"+doubleToString(m_thr_l,0)
      << (m_stale_thr?"STALE":"OK");
  tab << "RPi temp" << doubleToString(m_rpi_temp,1)+" C" << (m_stale_temp?"STALE":"OK");
  tab << "Enclosure" << doubleToString(m_int_temp,1)+"C "+doubleToString(m_int_kpa,1)+"kPa"
      << (m_stale_temp?"STALE":"OK");
  tab << "GPS" << m_fix_type+" sats="+intToString(m_num_sats) << (m_stale_gps?"STALE":"OK");
  m_msgs << tab.getFormattedString() << endl << endl;

  m_msgs << "Stale groups: " << staleList() << endl;
  return(true);
}
