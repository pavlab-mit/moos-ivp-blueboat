/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: pBB_DGPS_EKF/BB_DGPS_EKF_Info.cpp
   Last Ed: 2026-03-25
     Brief:
        Help and info functions for pBB_DGPS_EKF application.
*************************************************************/

#include <cstdlib>
#include <iostream>
#include "BB_DGPS_EKF_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pBB_DGPS_EKF is an Extended Kalman Filter for BlueBat         ");
  blk("  navigation using dual-antenna GPS (DGPS) heading and          ");
  blk("  level-frame gyroscope for smooth heading estimation.          ");
  blk("                                                                ");
  blk("  State vector: [x, y, phi, gamma, s]                           ");
  blk("    x, y   - position in local grid (m)                         ");
  blk("    phi    - true heading from DGPS + gyro fusion (rad)         ");
  blk("    gamma  - course over ground (rad)                           ");
  blk("    s      - speed over ground (m/s)                            ");
  blk("                                                                ");
  blk("  Also includes a DriftEstimator for current/drift estimation.  ");
  blk("  When thrusters are off for a configurable period, the drift   ");
  blk("  estimator runs to capture current velocities [v_x, v_y].      ");
  blk("                                                                ");
  blk("  Subscribes to GPS_STATE (bundled GPS data from iUnicoreGPS),  ");
  blk("  GYRO_LVL_Z (level-frame gyro), and DESIRED_THRUST_L/R for     ");
  blk("  drift detection. Uses adaptive measurement noise from GPS     ");
  blk("  accuracy metrics.                                             ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pBB_DGPS_EKF file.moos [OPTIONS]                         ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pBB_DGPS_EKF with the given process name.          ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pBB_DGPS_EKF.              ");
  blk("                                                                ");
  blk("Note: If argv[2] does not otherwise match a known option,       ");
  blk("      then it will be interpreted as a run alias. This is       ");
  blk("      to support pAntler launching conventions.                 ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pBB_DGPS_EKF Example MOOS Configuration                         ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pBB_DGPS_EKF                                    ");
  blk("{                                                               ");
  blk("  AppTick   = 20                                                ");
  blk("  CommsTick = 20                                                ");
  blk("                                                                ");
  blk("  // Input variable names                                       ");
  blk("  input_gps_state = GPS_STATE        // Bundled GPS from iUnicoreGPS");
  blk("  input_gyro_z    = GYRO_LVL_Z       // Level-frame gyro (rad/s)");
  blk("                                                                ");
  blk("  // Output variable names                                      ");
  blk("  output_nav_x       = NAV_X         // Filtered X position (m) ");
  blk("  output_nav_y       = NAV_Y         // Filtered Y position (m) ");
  blk("  output_nav_lat     = NAV_LAT       // Filtered latitude (deg) ");
  blk("  output_nav_long    = NAV_LONG      // Filtered longitude (deg)");
  blk("  output_nav_heading = NAV_HEADING   // Filtered heading (deg)  ");
  blk("  output_nav_speed   = NAV_SPEED     // Filtered speed (m/s)    ");
  blk("  output_nav_cog     = NAV_COG       // Filtered COG (deg)      ");
  blk("  output_nav_state   = NAV_STATE     // Bundled EKF state       ");
  blk("                                                                ");
  blk("  // Process noise (state drift uncertainty)                    ");
  blk("  sigma_x       = 0.1     // Position X process noise (m)       ");
  blk("  sigma_y       = 0.1     // Position Y process noise (m)       ");
  blk("  sigma_phi     = 0.01    // Heading process noise (rad)        ");
  blk("  sigma_gamma   = 0.05    // COG process noise (rad)            ");
  blk("  sigma_s       = 0.1     // Speed process noise (m/s)          ");
  blk("                                                                ");
  blk("  // Measurement noise (base values, scaled by GPS accuracy)    ");
  blk("  sigma_gps_pos = 1.0     // GPS position base noise (m)        ");
  blk("  sigma_gps_hdg = 3.0     // GPS heading base noise (deg)       ");
  blk("  sigma_gps_cog = 6.0     // GPS COG base noise (deg)           ");
  blk("  sigma_gps_spd = 0.1     // GPS speed noise (m/s)              ");
  blk("                                                                ");
  blk("  // Operational thresholds                                     ");
  blk("  speed_threshold          = 0.3   // Min speed for COG updates ");
  blk("  data_freshness_threshold = 5.0   // Max age of measurements (s)");
  blk("                                                                ");
  blk("  // GPS validity / manual override                             ");
  blk("  gps_stale_threshold     = 5.0    // GPS stale threshold (s)   ");
  blk("  enable_manual_override  = true   // Publish MOOS_MANUAL_OVERRIDE");
  blk("                                                                ");
  blk("  // Drift estimator (current estimation when thrusters off)    ");
  blk("  drift_decay_period      = 20.0   // Seconds before drift mode ");
  blk("  drift_thrust_threshold  = 1.0    // |Thrust| deadband (0-100) ");
  blk("  enable_drift_seglist    = true   // Publish drift VIEW_SEGLIST");
  blk("  drift_vector_scale      = 10.0   // m per m/s for visualization");
  blk("                                                                ");
  blk("  debug = false                                                 ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}


//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pBB_DGPS_EKF INTERFACE                                          ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  GPS_STATE        - Bundled GPS data from iUnicoreGPS:         ");
  blk("                     MOOSTime, NAV_LAT, NAV_LONG, NAV_SPEED,    ");
  blk("                     NAV_COG, GPS_HEADING, GPS_HEADING_ACC,     ");
  blk("                     GPS_HEADING_VALID, HDOP, H_ACC, GPS_LOCK,  ");
  blk("                     FIX_TYPE (local x/y computed via geodesy)  ");
  blk("                                                                ");
  blk("  GYRO_LVL_Z       - Level-frame gyroscope Z-axis (rad/s)       ");
  blk("                     NED convention: positive = CW rotation     ");
  blk("                                                                ");
  blk("  DESIRED_THRUST_L - Left thruster command (0-100) for drift    ");
  blk("  DESIRED_THRUST_R - Right thruster command (0-100) detection   ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  NAV_X            - Filtered X position (m, local grid)        ");
  blk("  NAV_Y            - Filtered Y position (m, local grid)        ");
  blk("  NAV_LAT          - Filtered latitude (deg, reverse geodesy)   ");
  blk("  NAV_LONG         - Filtered longitude (deg, reverse geodesy)  ");
  blk("  NAV_HEADING      - Filtered heading (deg, compass: 0=N, CW+)  ");
  blk("  NAV_SPEED        - Filtered speed (m/s)                       ");
  blk("  NAV_COG          - Filtered course over ground (deg, compass) ");
  blk("                     NOTE: NAV_* only published when GPS valid  ");
  blk("                                                                ");
  blk("  NAV_STATE        - Bundled EKF state (NODE_REPORT-like):      ");
  blk("                     MOOSTime, NAV_X, NAV_Y, NAV_LAT, NAV_LONG, ");
  blk("                     NAV_HEADING, NAV_SPEED, NAV_COG,           ");
  blk("                     STD_X, STD_Y, STD_HEADING, STD_SPEED,      ");
  blk("                     GPS_LOCK, FIX_TYPE, H_ACC, HDOP,           ");
  blk("                     HEADING_VALID, GPS_HEADING, GPS_HEADING_ACC");
  blk("                                                                ");
  blk("  MOOS_MANUAL_OVERRIDE - 'true' when GPS stale/invalid (helm    ");
  blk("                     safety); 'false' when GPS valid            ");
  blk("                                                                ");
  blk("  EKF_STATUS       - Filter status: WAITING_FOR_GPS, RUNNING,   ");
  blk("                     INITIALIZED, GPS_INVALID, UNHEALTHY        ");
  blk("  EKF_STD_X        - Position X standard deviation (m)          ");
  blk("  EKF_STD_Y        - Position Y standard deviation (m)          ");
  blk("  EKF_STD_HEADING  - Heading standard deviation (deg)           ");
  blk("  EKF_INTEGRATED_DIST - Distance since last GPS fix (m)         ");
  blk("  EKF_GPS_UPDATE_COUNT - Total GPS measurement updates          ");
  blk("                                                                ");
  blk("  Drift Estimator Outputs:                                      ");
  blk("  DRIFT_ACTIVE     - 1 if drift estimation active, 0 otherwise  ");
  blk("  DRIFT_VX         - Estimated drift velocity X (m/s)           ");
  blk("  DRIFT_VY         - Estimated drift velocity Y (m/s)           ");
  blk("  DRIFT_SPEED      - Estimated drift speed (m/s)                ");
  blk("  DRIFT_DIR        - Drift direction (deg, compass: 0=N, CW+)   ");
  blk("  DRIFT_STD_VX     - Drift Vx standard deviation (m/s)          ");
  blk("  DRIFT_STD_VY     - Drift Vy standard deviation (m/s)          ");
  blk("  DRIFT_SAMPLES    - Number of drift samples collected          ");
  blk("                                                                ");
  blk("  VIEW_VECTOR      - Drift vector visualization for pMarineViewer");
  blk("                     Label shows [vx,vy] in m/s                 ");
  blk("                     Scaled by drift_vector_scale parameter     ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pBB_DGPS_EKF", "gpl");
  exit(0);
}
