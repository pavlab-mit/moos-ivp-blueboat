/*************************************************************
      Name: 
      Orgn: MIT, Cambridge MA
      File: uBBBundler/BBBundler.h
   Last Ed:  2026-03-20
     Brief:
        Lorem ipsum dolor sit amet, consectetur adipiscing 
        elit, sed do eiusmod tempor incididunt ut labore et 
        dolore magna aliqua. Ut enim ad minim veniam, quis 
        nostrud exercitation ullamco laboris nisi ut aliquip 
        ex ea commodo consequat.
*************************************************************/

#ifndef BBBundler_HEADER
#define BBBundler_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"

#include <cstdarg>
#include <cstdio>
#include <map>
#include <set>
#include <string>

class BBBundler : public AppCastingMOOSApp {
 public:
  BBBundler();
  ~BBBundler();

 protected:
  bool OnNewMail(MOOSMSG_LIST &NewMail) override;
  bool Iterate() override;
  bool OnConnectToServer() override;
  bool OnStartUp() override;
  bool buildReport() override;

 private:
  void registerVariables();
  bool dbg_print(const char *format, ...);
  bool setupGeodesy();

 private:
  bool m_debug = false;
  FILE *m_debug_stream = nullptr;
  static const uint16_t m_fname_buff_size = 256;
  std::string m_app_name;
  char m_fname[m_fname_buff_size];

  std::string m_lat_var = "NAV_LAT";
  std::string m_lon_var = "NAV_LONG";
  std::string m_nav_x_out_var = "NAV_X";
  std::string m_nav_y_out_var = "NAV_Y";

  std::map<std::string, std::string> m_output_map;
  std::set<std::string> m_subscriptions;
  std::map<std::string, double> m_latest_numeric;
  std::map<std::string, std::string> m_latest_string;

  CMOOSGeodesy m_geodesy;
  bool m_geodesy_ok = false;
  bool m_have_lat = false;
  bool m_have_lon = false;
  double m_latest_lat = 0.0;
  double m_latest_lon = 0.0;
  double m_latest_x = 0.0;
  double m_latest_y = 0.0;

  unsigned long m_forwarded_count = 0;
  unsigned long m_geodesy_publish_count = 0;
};

#endif
