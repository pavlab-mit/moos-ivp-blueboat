/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: pBB_EKF/BB_EKF.h
   Last Ed:  2025-11-25
     Brief:
        Lorem ipsum dolor sit amet, consectetur adipiscing 
        elit, sed do eiusmod tempor incididunt ut labore et 
        dolore magna aliqua. Ut enim ad minim veniam, quis 
        nostrud exercitation ullamco laboris nisi ut aliquip 
        ex ea commodo consequat.
*************************************************************/

#ifndef BB_EKF_HEADER
#define BB_EKF_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
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
   // Input variables
   std::string m_input_gps_x;         // Default: NAV_X_GPS
   std::string m_input_gps_y;         // Default: NAV_Y_GPS
   std::string m_input_gps_speed;     // Default: NAV_SPEED_GPS
   std::string m_input_gps_heading;   // Default: NAV_HEADING_GPS
   std::string m_input_mag_heading;   // Default: NAV_HEADING_MAG
   std::string m_input_gyro_z;        // Default: GYRO_Z_LVL
   
   // Output variables
   std::string m_output_nav_x;        // Default: NAV_X
   std::string m_output_nav_y;        // Default: NAV_Y
   std::string m_output_nav_speed;    // Default: NAV_SPEED
   std::string m_output_nav_heading;  // Default: NAV_HEADING
};

#endif 
