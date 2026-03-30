/*************************************************************
      Name: Jeremy Wenger
      Orgn: MIT, Cambridge MA
      File: iUnicoreGPS/UnicoreGPS.h
   Last Ed: 2026-03-04
     Brief:
        MOOS interface for Unicore UM982 dual-antenna GNSS
        receiver. Reads position, velocity, heading, and DOP
        data via serial and publishes to MOOSDB with
        configurable publish groups.
*************************************************************/

#ifndef UnicoreGPS_HEADER
#define UnicoreGPS_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "UnicoreParser.h"
#include "UnicoreDataDTO.hpp"

#include <string>
#include <cstdarg>
#include <mutex>
#include <thread>
#include <atomic>

class UnicoreGPS : public AppCastingMOOSApp
{
 public:
   UnicoreGPS();
   ~UnicoreGPS();

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

 private: // Initialization helpers
   bool GeodesySetup();
   bool initializeGPS();
   void gpsReaderThread();
   std::string buildStateString();

 private: // Configuration variables
   bool m_debug;
   FILE *m_debug_stream;
   static const uint16_t m_fname_buff_size = 256;
   std::string m_app_name;
   char m_fname[m_fname_buff_size];

   // Serial config
   std::string m_port_name;
   int m_baud_rate;

   // Navigation config
   std::string m_nav_suffix;
   bool m_enable_geodesic;
   bool m_geodesy_initialized;

   // Publish group flags
   bool m_publish_position;
   bool m_publish_velocity;
   bool m_publish_heading;
   bool m_publish_dops;
   bool m_publish_status;
   bool m_publish_time;
   bool m_publish_state;
   bool m_publish_dto;

 private: // State variables

   // Geodesy
   CMOOSGeodesy m_geodesy;

   // Parser
   UnicoreParser *m_parser;

   // Threading
   std::thread m_reader_thread;
   std::mutex m_data_mutex;
   std::atomic<bool> m_thread_running;
   bool m_new_data_available;
  bool m_data_stale;
  double m_last_gps_data_moos_time;
  double m_stale_data_timeout_sec;
  double m_stale_warn_repeat_sec;
  double m_last_stale_warn_moos_time;
  bool m_quality_degraded;
  double m_quality_warn_repeat_sec;
  double m_last_quality_warn_moos_time;

   // Position data (from BESTNAVA)
   double m_nav_x, m_nav_y;
   double m_nav_lat, m_nav_lon;
   double m_nav_alt;
   double m_h_acc, m_v_acc;
   std::string m_fix_type;
   uint8_t m_num_sats;
   bool m_gps_lock;

   // Velocity data (from BESTVELA)
   double m_nav_speed;
   double m_vel_n, m_vel_e, m_vel_d;
   double m_track_over_ground;

   // Heading data (from UNIHEADINGA)
   double m_gps_heading;
   double m_gps_heading_acc;
   double m_gps_baseline;
   double m_gps_pitch;
   std::string m_carr_soln;
   bool m_heading_valid;

   // Position DOP data (from STADOPA preferred, PSRDOPA fallback)
   float m_hdop, m_vdop, m_pdop, m_gdop;

   // Heading DOP data (from STADOPHA)
   float m_hdg_hdop, m_hdg_vdop, m_hdg_pdop, m_hdg_gdop;

   // RTK status data (from RTKSTATUSA - Unicore Ref Manual 7.5.84)
   bool m_corr_healthy;
   uint8_t m_rtk_calc_status;  // 0-5, 5=RTK solution available

   // Correction age and base station ID (from BESTNAVA)
   float m_corr_age;           // Differential correction age (seconds)
   float m_sol_age;            // Solution age (seconds)
   std::string m_base_id;      // Base station ID

   // RTCM correction injection stats
   unsigned long m_rtcm_bytes_written;
   unsigned int m_rtcm_msgs_received;

   // Time
   double m_epoch_time;

   // DTOs
   UnicorePositionDTO m_position_dto;
   UnicoreHeadingDTO m_heading_dto;
   UnicoreVelocityDTO m_velocity_dto;
   UnicoreDOPsDTO m_dops_dto;
   UnicoreStatusDTO m_status_dto;
   UnicoreRtkStatusDTO m_rtk_status_dto;

   // Message counters for buildReport
   int m_bestnava_count;
   int m_uniheadinga_count;
   int m_bestvela_count;
   int m_psrdopa_count;
   int m_stadopa_count;
   int m_stadopha_count;
   int m_rtkstatusa_count;
};

#endif
