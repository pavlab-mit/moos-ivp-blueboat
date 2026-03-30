/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: pBB_DGPS_EKF/BB_DGPS_EKF.h
   Last Ed: 2026-03-25
     Brief:
        Extended Kalman Filter for BlueBat navigation using
        dual-antenna GPS (DGPS) heading and level-frame gyroscope.

        State: [x, y, phi, gamma, s]
          x, y   - position in local grid (m)
          phi    - true heading from DGPS + gyro fusion (rad)
          gamma  - course over ground (rad)
          s      - speed over ground (m/s)

        Also includes a DriftEstimator for current/drift estimation.
        When thrusters are off for a configurable period, the drift
        estimator runs to capture current velocities [v_x, v_y].

        Subscribes to GPS_STATE (iUnicoreGPS bundle), GYRO_LVL_Z,
        and DESIRED_THRUST_L/R for drift detection.
        Publishes NAV_X, NAV_Y, NAV_HEADING, NAV_SPEED, NAV_COG,
        and drift estimates (DRIFT_VX, DRIFT_VY, etc.).
*************************************************************/

#ifndef BB_DGPS_EKF_HEADER
#define BB_DGPS_EKF_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "BB_DGPS_EKF_Model.hpp"
#include <string>
#include <cstdarg>

class BB_DGPS_EKF : public AppCastingMOOSApp
{
 public:
   BB_DGPS_EKF();
   ~BB_DGPS_EKF();

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
   bool parseGPSState(const std::string &sval, BB_DGPS_EKF_Model::GPSMeasurement &gps);

 private: // Configuration variables
   bool m_debug;
   FILE *m_debug_stream;
   static const uint16_t m_fname_buff_size = 256;
   std::string m_app_name;
   char m_fname[m_fname_buff_size];

   // Input variable names
   std::string m_input_gps_state;     // GPS_STATE from iUnicoreGPS
   std::string m_input_gyro_z;        // GYRO_LVL_Z from AHRS/navigator
   std::string m_input_thrust_l;      // DESIRED_THRUST_L
   std::string m_input_thrust_r;      // DESIRED_THRUST_R

   // Output variable names
   std::string m_output_nav_x;
   std::string m_output_nav_y;
   std::string m_output_nav_lat;
   std::string m_output_nav_long;
   std::string m_output_nav_heading;
   std::string m_output_nav_speed;
   std::string m_output_nav_cog;
   std::string m_output_nav_state;

   // Speed threshold for COG updates
   double m_speed_threshold;

   // Data freshness threshold (seconds)
   double m_data_freshness_threshold;

   // Drift estimation configuration
   double m_drift_decay_period;       // Seconds of low thrust before drift estimation starts
   double m_drift_thrust_threshold;   // Thrust threshold (0-100) absolute value below which is "off"

   // GPS validity / manual override configuration
   double m_gps_stale_threshold;      // Seconds before GPS is considered stale for override
   bool m_enable_manual_override;     // Whether to publish MOOS_MANUAL_OVERRIDE

   // Drift visualization configuration
   bool m_enable_drift_seglist;       // Whether to publish VIEW_SEGLIST for drift vector
   double m_drift_vector_scale;       // Scale factor for drift vector visualization (m per m/s)

 private: // State variables
   // EKF model
   BB_DGPS_EKF_Model m_ekf;

   // Drift estimator
   DriftEstimator m_drift_estimator;

   // Latest measurements
   BB_DGPS_EKF_Model::GPSMeasurement m_latest_gps;
   double m_latest_gyro_z;

   // Thrust tracking for drift detection
   double m_latest_thrust_l;
   double m_latest_thrust_r;
   double m_thrust_off_start_time;    // When both thrusts went below threshold
   bool m_thrusts_below_threshold;    // Are both thrusts currently below threshold?

   // Timestamps for freshness checking
   double m_last_gps_time;
   double m_last_gyro_time;
   double m_last_iterate_time;

   // Flags
   bool m_gps_ready;
   bool m_gyro_ready;
   bool m_gps_valid;                  // Is GPS currently valid (fresh and locked)?
   bool m_nav_published;              // Have we started publishing NAV_* states?

   // Statistics
   unsigned int m_gps_update_count;
   unsigned int m_iterate_count;

   // Geodesy for local coordinate conversion
   CMOOSGeodesy m_geodesy;
   bool m_geodesy_initialized;
};

#endif