/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: pDiffThrustPID_v2/DiffThrustPID_v2.cpp
   Last Ed: 2026-06-20
     Brief: See docs/pDiffThrustPID_v2_spec.md
*************************************************************/

#include <algorithm>
#include <cmath>
#include "MBUtils.h"
#include "ACTable.h"
#include "DiffThrustPID_v2.h"

using namespace std;

static double clampd(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

static double wrap180(double a)
{
  a = fmod(a + 180.0, 360.0);
  if (a < 0) a += 360.0;
  return a - 180.0;
}

//---------------------------------------------------------
// Schedule

void Schedule::set(const string &spec)
{
  m_pts.clear();
  vector<string> segs = parseString(spec, ':');
  for (unsigned i = 0; i < segs.size(); i++) {
    vector<string> pr = parseString(stripBlankEnds(segs[i]), ',');
    if (pr.size() == 2)
      m_pts.push_back(make_pair(atof(pr[0].c_str()), atof(pr[1].c_str())));
  }
  sort(m_pts.begin(), m_pts.end());
}

double Schedule::eval(double x) const
{
  if (m_pts.empty()) return 0.0;
  if (x <= m_pts.front().first) return m_pts.front().second;
  if (x >= m_pts.back().first)  return m_pts.back().second;
  for (unsigned i = 1; i < m_pts.size(); i++) {
    if (x <= m_pts[i].first) {
      double x0 = m_pts[i-1].first, y0 = m_pts[i-1].second;
      double x1 = m_pts[i].first,   y1 = m_pts[i].second;
      return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
    }
  }
  return m_pts.back().second;
}

string Schedule::repr() const
{
  string s;
  for (unsigned i = 0; i < m_pts.size(); i++)
    s += (i ? " : " : "") + doubleToStringX(m_pts[i].first, 2) + "," +
         doubleToStringX(m_pts[i].second, 1);
  return s;
}

//---------------------------------------------------------
// Constructor

DiffThrustPID_v2::DiffThrustPID_v2()
{
  m_setpoint_heading_var = "DESIRED_HEADING";
  m_feedback_heading_var = "NAV_HEADING";
  m_setpoint_speed_var   = "DESIRED_SPEED";
  m_feedback_speed_var   = "NAV_SPEED";
  m_feedback_yaw_var     = "GYRO_Z_LVL_IMU";
  m_desired_thrust_l_var = "DESIRED_THRUST_L";
  m_desired_thrust_r_var = "DESIRED_THRUST_R";

  m_ff_surge.set("0,0 : 0.5,4.25 : 1.0,17 : 1.5,38 : 2.0,68 : 2.4,98");
  m_speed_ki    = 0.0;
  m_speed_i_max = 15.0;

  m_theta_b      = 30.0;
  m_max_yaw_rate = 30.0;
  m_yaw_c0       = 1.5;
  m_yaw_c1       = 2.4;
  m_yaw_ki       = 0.0;
  m_yaw_i_max    = 25.0;

  m_delta_cap.set("0,100 : 0.6,100 : 0.8,11 : 1.2,25 : 1.7,50 : 2.4,50");
  m_u_max = 100.0;

  m_v_des = m_v = m_psi_des = m_psi = m_r = 0.0;
  m_deploy = false;
  m_Jv = m_Jr = 0.0;
  m_prev_time = 0.0;
  m_T = m_D = m_TL = m_TR = m_m = m_r_des = 0.0;

  m_debug = false;
  m_debug_stream = 0;
  memset(m_fname, 0, m_fname_buff_size);
}

//---------------------------------------------------------
// OnNewMail

bool DiffThrustPID_v2::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  for (MOOSMSG_LIST::iterator p = NewMail.begin(); p != NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

    if (key == m_setpoint_heading_var && msg.IsDouble())
      m_psi_des = msg.GetDouble();
    else if (key == m_feedback_heading_var && msg.IsDouble())
      m_psi = msg.GetDouble();
    else if (key == m_setpoint_speed_var && msg.IsDouble())
      m_v_des = msg.GetDouble();
    else if (key == m_feedback_speed_var && msg.IsDouble())
      m_v = msg.GetDouble();
    else if (key == m_feedback_yaw_var && msg.IsDouble())
      m_r = msg.GetDouble() * 180.0 / M_PI;          // rad/s -> deg/s
    else if (key == "DEPLOY")
      m_deploy = (tolower(msg.GetAsString()) == "true");
    else if (key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }
  return true;
}

//---------------------------------------------------------
// Iterate

bool DiffThrustPID_v2::Iterate()
{
  AppCastingMOOSApp::Iterate();

  double now = MOOSTime();
  double dt = now - m_prev_time;
  if (dt <= 0.0) dt = 1.0 / (GetAppFreq() > 0 ? GetAppFreq() : 10.0);
  m_prev_time = now;

  // Step 1: surge -> total thrust
  double e_v = m_v_des - m_v;
  m_T = clampd(m_ff_surge.eval(m_v_des) + m_Jv, 0.0, m_u_max);

  // Step 2: yaw -> differential  (FF from heading only; gyro r only in integral)
  double e_psi = wrap180(m_psi_des - m_psi);
  m_m = clampd(e_psi / m_theta_b, -1.0, 1.0);
  m_r_des = m_max_yaw_rate * m_m;
  double e_r = m_r_des - m_r;
  double d_raw = (m_yaw_c0 + m_yaw_c1 * m_v) * m_r_des + m_Jr;
  double dcap = m_delta_cap.eval(m_v);
  m_D = clampd(d_raw, -dcap, dcap);

  // Step 3: fuse
  m_TL = m_T + m_D;
  m_TR = m_T - m_D;

  // Step 4: ceiling (preserve the turn, spend forward thrust)
  double over = max(max(m_TL - 100.0, m_TR - 100.0), 0.0);
  m_TL -= over; m_TR -= over;
  double under = min(min(m_TL + 100.0, m_TR + 100.0), 0.0);
  m_TL -= under; m_TR -= under;
  bool ceiling_active = (over != 0.0) || (under != 0.0);
  m_TL = clampd(m_TL, -100.0, 100.0);
  m_TR = clampd(m_TR, -100.0, 100.0);

  // Step 5: commit integrators (terms held in output units)
  if (!m_deploy) {
    m_Jv = m_Jr = 0.0;
  } else {
    if (!ceiling_active)
      m_Jv = clampd(m_Jv + m_speed_ki * e_v * dt, -m_speed_i_max, m_speed_i_max);
    if (fabs(d_raw) <= dcap)
      m_Jr = clampd(m_Jr + m_yaw_ki * e_r * dt, -m_yaw_i_max, m_yaw_i_max);
  }

  Notify(m_desired_thrust_l_var, m_TL);
  Notify(m_desired_thrust_r_var, m_TR);

  dbg_print("t=%.2f v=%.2f/%.2f psi=%.1f/%.1f m=%.2f T=%.1f D=%.1f L=%.1f R=%.1f\n",
            now, m_v_des, m_v, m_psi_des, m_psi, m_m, m_T, m_D, m_TL, m_TR);

  AppCastingMOOSApp::PostReport();
  return true;
}

//---------------------------------------------------------
// OnConnectToServer

bool DiffThrustPID_v2::OnConnectToServer()
{
  registerVariables();
  return true;
}

//---------------------------------------------------------
// OnStartUp

bool DiffThrustPID_v2::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();
  m_app_name = GetAppName();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  for (STRING_LIST::iterator p = sParams.begin(); p != sParams.end(); p++) {
    string orig = *p, line = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;
    bool handled = true;

    if      (param == "speed_ff_points")   m_ff_surge.set(value);
    else if (param == "speed_ki")          m_speed_ki = atof(value.c_str());
    else if (param == "speed_i_max")       m_speed_i_max = atof(value.c_str());
    else if (param == "theta_b")           m_theta_b = atof(value.c_str());
    else if (param == "max_yaw_rate")      m_max_yaw_rate = atof(value.c_str());
    else if (param == "yaw_ff_c0")         m_yaw_c0 = atof(value.c_str());
    else if (param == "yaw_ff_c1")         m_yaw_c1 = atof(value.c_str());
    else if (param == "yaw_ki")            m_yaw_ki = atof(value.c_str());
    else if (param == "yaw_i_max")         m_yaw_i_max = atof(value.c_str());
    else if (param == "delta_cap_points")  m_delta_cap.set(value);
    else if (param == "u_max")             m_u_max = atof(value.c_str());
    else if (param == "setpoint_heading_var") m_setpoint_heading_var = value;
    else if (param == "feedback_heading_var") m_feedback_heading_var = value;
    else if (param == "setpoint_speed_var")   m_setpoint_speed_var = value;
    else if (param == "feedback_speed_var")   m_feedback_speed_var = value;
    else if (param == "feedback_yaw_var")     m_feedback_yaw_var = value;
    else if (param == "desired_thrust_l_var") m_desired_thrust_l_var = value;
    else if (param == "desired_thrust_r_var") m_desired_thrust_r_var = value;
    else if (param == "debug") {
      m_debug = (tolower(value) == "true");
      if (m_debug) {
        time_t rawtime; time(&rawtime);
        char fmt[m_fname_buff_size]; memset(fmt, 0, m_fname_buff_size);
        strftime(fmt, m_fname_buff_size, "%F_%T", localtime(&rawtime));
        snprintf(m_fname, m_fname_buff_size, "DBG_%s_%s_DATA.dbg", m_app_name.c_str(), fmt);
      }
    }
    else handled = false;

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  registerVariables();
  return true;
}

//---------------------------------------------------------
// registerVariables

void DiffThrustPID_v2::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register(m_setpoint_heading_var, 0);
  Register(m_feedback_heading_var, 0);
  Register(m_setpoint_speed_var, 0);
  Register(m_feedback_speed_var, 0);
  Register(m_feedback_yaw_var, 0);
  Register("DEPLOY", 0);
}

//---------------------------------------------------------
// dbg_print

bool DiffThrustPID_v2::dbg_print(const char *format, ...)
{
  if (!m_debug) return false;
  va_list args;
  va_start(args, format);
  m_debug_stream = fopen(m_fname, "a");
  if (m_debug_stream) {
    vfprintf(m_debug_stream, format, args);
    fclose(m_debug_stream);
    va_end(args);
    return true;
  }
  va_end(args);
  reportRunWarning("Debug enabled but file could not be opened");
  return false;
}

//---------------------------------------------------------
// buildReport

bool DiffThrustPID_v2::buildReport()
{
  ACTable tab(2);
  tab << "Signal" << "Value";
  tab.addHeaderLines();
  tab << "DESIRED_SPEED / NAV_SPEED"     << doubleToString(m_v_des, 2) + " / " + doubleToString(m_v, 2);
  tab << "DESIRED_HEADING / NAV_HEADING" << doubleToString(m_psi_des, 1) + " / " + doubleToString(m_psi, 1);
  tab << "yaw rate r [deg/s]"            << doubleToString(m_r, 2);
  tab << "mixing m / r_des"              << doubleToString(m_m, 2) + " / " + doubleToString(m_r_des, 1);
  tab << "total T / diff D"              << doubleToString(m_T, 1) + " / " + doubleToString(m_D, 1);
  tab << "integ J_v / J_r"               << doubleToString(m_Jv, 2) + " / " + doubleToString(m_Jr, 2);
  tab << "THRUST_L / THRUST_R"           << doubleToString(m_TL, 1) + " / " + doubleToString(m_TR, 1);
  tab << "DEPLOY"                        << boolToString(m_deploy);
  m_msgs << tab.getFormattedString() << endl << endl;
  m_msgs << "FF_surge:  " << m_ff_surge.repr() << endl;
  m_msgs << "delta_cap: " << m_delta_cap.repr() << endl;
  return true;
}
