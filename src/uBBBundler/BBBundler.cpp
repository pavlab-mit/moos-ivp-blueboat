/************************************************************/
/*    NAME: BBBundler                                       */
/*    FILE: BBBundler.cpp                                   */
/************************************************************/

#include "BBBundler.h"

#include "ACTable.h"
#include "MBUtils.h"

#include <cstdlib>
#include <cstring>
#include <ctime>

using namespace std;

BBBundler::BBBundler() {}

BBBundler::~BBBundler() {}

bool BBBundler::OnNewMail(MOOSMSG_LIST &NewMail) {
  AppCastingMOOSApp::OnNewMail(NewMail);

  for (MOOSMSG_LIST::iterator p = NewMail.begin(); p != NewMail.end(); ++p) {
    CMOOSMsg &msg = *p;
    const string key = msg.GetKey();
    if (key == "APPCAST_REQ") {
      continue;
    }

    if (msg.IsDouble()) {
      const double dval = msg.GetDouble();
      m_latest_numeric[key] = dval;

      if (key == m_lat_var) {
        m_latest_lat = dval;
        m_have_lat = true;
      }
      if (key == m_lon_var) {
        m_latest_lon = dval;
        m_have_lon = true;
      }

      auto map_it = m_output_map.find(key);
      if (map_it != m_output_map.end()) {
        Notify(map_it->second, dval);
        m_forwarded_count++;
      }
    } else {
      const string sval = msg.GetString();
      m_latest_string[key] = sval;
      auto map_it = m_output_map.find(key);
      if (map_it != m_output_map.end()) {
        Notify(map_it->second, sval);
        m_forwarded_count++;
      }
    }
  }

  return true;
}

bool BBBundler::dbg_print(const char *format, ...) {
  if (!m_debug) {
    return false;
  }
  va_list args;
  va_start(args, format);
  m_debug_stream = fopen(m_fname, "a");
  if (m_debug_stream == nullptr) {
    va_end(args);
    return false;
  }
  vfprintf(m_debug_stream, format, args);
  fclose(m_debug_stream);
  va_end(args);
  return true;
}

bool BBBundler::OnConnectToServer() {
  registerVariables();
  return true;
}

bool BBBundler::Iterate() {
  AppCastingMOOSApp::Iterate();

  if (m_geodesy_ok && m_have_lat && m_have_lon) {
    double nav_y = 0.0;
    double nav_x = 0.0;
    if (m_geodesy.LatLong2LocalGrid(m_latest_lat, m_latest_lon, nav_y, nav_x)) {
      m_latest_x = nav_x;
      m_latest_y = nav_y;
      Notify(m_nav_x_out_var, nav_x);
      Notify(m_nav_y_out_var, nav_y);
      m_geodesy_publish_count++;
    }
  }

  AppCastingMOOSApp::PostReport();
  return true;
}

bool BBBundler::setupGeodesy() {
  double lat_origin = 0.0;
  double lon_origin = 0.0;
  if (!m_MissionReader.GetValue("LatOrigin", lat_origin)) {
    reportConfigWarning("uBBBundler missing LatOrigin");
    return false;
  }
  if (!m_MissionReader.GetValue("LongOrigin", lon_origin)) {
    reportConfigWarning("uBBBundler missing LongOrigin");
    return false;
  }
  if (!m_geodesy.Initialise(lat_origin, lon_origin)) {
    reportConfigWarning("uBBBundler geodesy initialize failed");
    return false;
  }
  return true;
}

bool BBBundler::OnStartUp() {
  AppCastingMOOSApp::OnStartUp();
  m_app_name = GetAppName();
  m_subscriptions.insert(m_lat_var);
  m_subscriptions.insert(m_lon_var);
  m_output_map["GPS_HEADING"] = "NAV_HEADING";
  m_output_map["NAV_SPEED"] = "NAV_SPEED";
  m_output_map["NAV_COG"] = "NAV_COG";

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    reportConfigWarning("No config block found for " + GetAppName());
  }

  for (STRING_LIST::iterator p = sParams.begin(); p != sParams.end(); ++p) {
    string orig = *p;
    string line = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;
    bool handled = false;

    if (param == "debug") {
      m_debug = (tolower(value) == "true");
      handled = true;
    } else if (param == "lat_var") {
      m_lat_var = value;
      m_subscriptions.insert(value);
      handled = true;
    } else if (param == "lon_var") {
      m_lon_var = value;
      m_subscriptions.insert(value);
      handled = true;
    } else if (param == "nav_x_out_var") {
      m_nav_x_out_var = value;
      handled = true;
    } else if (param == "nav_y_out_var") {
      m_nav_y_out_var = value;
      handled = true;
    } else if ((param == "nav_x_bs_out_var") || (param == "nav_y_bs_out_var") ||
               (param == "publish_backseat_xy")) {
      reportConfigWarning(param + " is deprecated and ignored; uBBBundler now publishes only one NAV_X/NAV_Y pair.");
      handled = true;
    } else if (param == "forward_map") {
      vector<string> pairs = parseString(value, ',');
      for (const string &pair_raw : pairs) {
        string pair = stripBlankEnds(pair_raw);
        string src = biteStringX(pair, ':');
        string dst = stripBlankEnds(pair);
        src = stripBlankEnds(src);
        if (!src.empty() && !dst.empty()) {
          m_output_map[src] = dst;
          m_subscriptions.insert(src);
        }
      }
      handled = true;
    }

    if (!handled) {
      reportUnhandledConfigWarning(orig);
    }
  }

  if (m_debug) {
    time_t rawtime;
    struct tm *timeinfo;
    memset(m_fname, '\0', m_fname_buff_size);
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    char fmt[m_fname_buff_size];
    memset(fmt, '\0', m_fname_buff_size);
    strftime(fmt, m_fname_buff_size, "%F_%T", timeinfo);
    snprintf(m_fname, m_fname_buff_size, "DBG_%s_%s_DATA.dbg", m_app_name.c_str(), fmt);
  }

  m_geodesy_ok = setupGeodesy();
  registerVariables();
  return true;
}

void BBBundler::registerVariables() {
  AppCastingMOOSApp::RegisterVariables();
  for (const auto &name : m_subscriptions) {
    Register(name, 0);
  }
}

bool BBBundler::buildReport() {
  m_msgs << "============================================" << endl;
  m_msgs << "           uBBBundler V2 Report             " << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "geodesy_ok=" << m_geodesy_ok << ", lat_var=" << m_lat_var
         << ", lon_var=" << m_lon_var << endl;
  m_msgs << "last_lat=" << m_latest_lat << ", last_lon=" << m_latest_lon << endl;
  m_msgs << "last_x=" << m_latest_x << ", last_y=" << m_latest_y << endl;

  ACTable table(2);
  table << "Metric | Value";
  table.addHeaderLines();
  table << "Forwarded outputs" << static_cast<double>(m_forwarded_count);
  table << "Geodesy publishes" << static_cast<double>(m_geodesy_publish_count);
  m_msgs << table.getFormattedString() << endl;

  return true;
}
