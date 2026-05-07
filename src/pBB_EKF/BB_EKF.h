/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: pBB_EKF/BB_EKF.h
   Last Ed:  2025-11-25
     Brief:
        Extended Kalman Filter for blueboat navigation. Fuses
        GNSS position/speed/heading, AHRS magnetic heading, and
        level-frame gyro into helm-facing state. Lat/lon inputs
        are converted to the local grid via an internal geodesy
        seeded by LatOrigin/LongOrigin in the mission file.
*************************************************************/

#ifndef BB_EKF_HEADER
#define BB_EKF_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include <string>
#include <cstdarg> //va_list, va_start, va_end
#include "NavEKF.hpp"

class BB_EKF : public AppCastingMOOSApp
{
 public:
   BB_EKF();
   ~BB_EKF();

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
   double wrapAngle(double angle);
   bool setupGeodesy();
   std::string outName(const std::string &base) const;

 private: // Configuration variables

  bool m_debug;
  FILE *m_debug_stream;
  static const uint16_t m_fname_buff_size = 256;
  std::string m_app_name;
  char m_fname[m_fname_buff_size];

 private: // State variables
   
   // Measurement bundling classes
   class NavPosGNSS {
   public:
      double x;
      double y;
      bool x_new;
      bool y_new;
      double timestamp;
      
      NavPosGNSS() : x(0), y(0), x_new(false), y_new(false), timestamp(0) {}
      bool ready() const { return x_new && y_new; }
      void reset() { x_new = false; y_new = false; }
   };
   
   class NavHeadingGNSS {
   public:
      double heading;
      bool ready_flag;
      double timestamp;
      
      NavHeadingGNSS() : heading(0), ready_flag(false), timestamp(0) {}
      bool ready() const { return ready_flag; }
      void reset() { ready_flag = false; }
   };
   
   class NavSpeedGNSS {
   public:
      double speed;
      bool ready_flag;
      double timestamp;
      
      NavSpeedGNSS() : speed(0), ready_flag(false), timestamp(0) {}
      bool ready() const { return ready_flag; }
      void reset() { ready_flag = false; }
   };
   
   class MagHeading {
   public:
      double heading;
      bool ready_flag;
      double timestamp;
      
      MagHeading() : heading(0), ready_flag(false), timestamp(0) {}
      bool ready() const { return ready_flag; }
      void reset() { ready_flag = false; }
   };
   
   class GyroZ {
   public:
      double gz_lvl;
      bool ready_flag;
      double timestamp;
      
      GyroZ() : gz_lvl(0), ready_flag(false), timestamp(0) {}
      bool ready() const { return ready_flag; }
      void reset() { ready_flag = false; }
   };
   
   // Measurement objects
   NavPosGNSS m_nav_pos_gnss;
   NavHeadingGNSS m_nav_heading_gnss;
   NavSpeedGNSS m_nav_speed_gnss;
   MagHeading m_mag_heading;
   GyroZ m_gyro_z;
   
   // EKF instance
   NavEKF m_ekf;
   
   // Timing
   double m_last_update_time;
   double m_dt_nominal;      // Nominal timestep if dt <= 0 (default 0.1s)
   
   // Store last known magnetic heading for low-speed output
   double m_last_mag_heading_cart;  // In Cartesian frame
   
   // Configuration parameters
   double m_speed_threshold; // Minimum speed for bias updates (default 0.3 m/s)
   double m_reverse_heading_error_thresh; // Heading error threshold for reverse detection (default 90 deg)
   
   // Process variances
   double m_pos_process_variance;     // Position process variance (m^2)
   double m_speed_process_variance;   // Speed process variance (m/s)^2
   double m_heading_process_variance; // Heading process variance (rad^2)
   double m_bias_process_variance;    // Bias process variance (rad^2)
   
   // Measurement standard deviations
   double m_gps_pos_stddev;      // GPS position measurement stddev (m)
   double m_gps_speed_stddev;    // GPS speed measurement stddev (m/s)
   double m_mag_meas_stddev;     // Magnetic heading measurement stddev (deg)
   double m_bias_meas_stddev;    // Bias measurement stddev (deg)
   
   // Variable name mappings
   // Input variables (lat/lon - we do the geodesic conversion here)
   std::string m_input_gps_lat;       // Default: NAV_LAT_DGNSS
   std::string m_input_gps_lon;       // Default: NAV_LONG_DGNSS
   std::string m_input_gps_speed;     // Default: NAV_SPEED_DGNSS
   std::string m_input_gps_heading;   // Default: GPS_HEADING_DGNSS
   std::string m_input_mag_heading;   // Default: NAV_HEADING_AHRS
   std::string m_input_gyro_z;        // Default: GYRO_Z_LVL_IMU

   // Latest lat/lon staging (so we only convert once both have arrived)
   double m_latest_lat;
   double m_latest_lon;
   bool m_have_lat;
   bool m_have_lon;

   // Output bases (NAV_X, NAV_Y, NAV_SPEED, NAV_HEADING).
   // Suffix m_pub_suffix is appended at publish time. Empty by
   // default - bare names go to the helm.
   std::string m_pub_suffix;

   // Geodesy for lat/lon -> local grid conversion
   CMOOSGeodesy m_geodesy;
   bool m_geodesy_ok;
};

#endif 
