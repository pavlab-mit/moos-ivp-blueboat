/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: pBB_Health/BB_Health.h
   Last Ed: 2026-03-25
     Brief: Health monitoring for BlueBoat platform. Monitors
            battery voltage/current, thermal status, EKF health,
            GPS quality, and RC connection state. Publishes visual
            indicators for RC mode.
*************************************************************/

#ifndef BB_Health_HEADER
#define BB_Health_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYCircle.h"
#include "XYRangePulse.h"
#include <string>
#include <cstdarg>

class BB_Health : public AppCastingMOOSApp
{
 public:
   BB_Health();
   ~BB_Health();

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
   bool buildReport();

 protected:
   void registerVariables();
   bool dbg_print(const char *format, ...);
   bool parseGPSState(const std::string &sval);
   void publishRCModeVisuals();
   void publishRCModePulse();
   void hideRCModeVisuals();

 private: // Configuration variables
   bool m_debug;
   FILE *m_debug_stream;
   static const uint16_t m_fname_buff_size = 256;
   std::string m_app_name;
   char m_fname[m_fname_buff_size];

   // Thresholds
   double m_stale_time;
   double m_low_voltage_thresh;
   double m_high_current_thresh;
   double m_high_pi_temp_thresh;
   double m_high_internal_temp_thresh;
   double m_hdop_thresh;

   // Circle visual config
   double m_rc_circle_radius;
   std::string m_vname;

 private: // State variables - Battery/Power (from iBBThrustCtrl)
   double m_rolling_voltage;
   double m_rolling_current;
   double m_rolling_voltage_time;
   double m_rolling_current_time;
   bool m_stale_voltage;
   bool m_stale_current;

 private: // State variables - Thermal (from iBBThrustCtrl)
   double m_rpi_temp;
   double m_internal_temp;
   double m_rpi_temp_time;
   double m_internal_temp_time;
   bool m_stale_rpi_temp;
   bool m_stale_internal_temp;

 private: // State variables - EKF (from pBB_DGPS_EKF)
   std::string m_ekf_status;
   double m_ekf_status_time;
   bool m_stale_ekf;

 private: // State variables - GPS (parsed from GPS_STATE)
   double m_hdop;
   std::string m_fix_type;
   int m_num_sats;
   bool m_gps_lock;
   double m_gps_state_time;
   bool m_stale_gps;

 private: // State variables - RC (from iRCReader)
   bool m_rc_connected;
   double m_rc_ch6;
   double m_rc_connected_time;
   double m_rc_ch6_time;
   bool m_stale_rc;

 private: // State variables - Navigation (for visuals)
   double m_nav_x;
   double m_nav_y;
   double m_nav_time;

 private: // RC mode tracking
   bool m_rc_mode_active;
   bool m_rc_mode_prev;
};

#endif
