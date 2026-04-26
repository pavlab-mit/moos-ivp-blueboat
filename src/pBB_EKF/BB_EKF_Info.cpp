/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: pBB_EKF/BB_EKF_Info.cpp
   Last Ed:  2026-04-26
     Brief:
        Help, example config, and interface text for pBB_EKF.
*************************************************************/

#include <cstdlib>
#include <iostream>
#include "BB_EKF_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  Extended Kalman Filter for blueboat navigation. Fuses GNSS    ");
  blk("  position/speed/heading, AHRS magnetic heading, and level-frame");
  blk("  gyro into a single helm-facing state. Lat/lon inputs are      ");
  blk("  converted to local grid via internal geodesy seeded by        ");
  blk("  LatOrigin / LongOrigin in the mission file. Outputs default to");
  blk("  bare NAV_X / NAV_Y / NAV_HEADING / NAV_SPEED for the helm; set");
  blk("  pub_suffix to namespace the publications when running multiple");
  blk("  state sources side-by-side.                                   ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pBB_EKF file.moos [OPTIONS]                              ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pBB_EKF with the given process name rather than    ");
  blk("      pBB_EKF.                                                  ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pBB_EKF.                   ");
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
  blu("pBB_EKF Example MOOS Configuration                              ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pBB_EKF                                         ");
  blk("{                                                               ");
  blk("  AppTick   = 10                                                ");
  blk("  CommsTick = 10                                                ");
  blk("                                                                ");
  blk("  // Input variables (defaults shown).                          ");
  blk("  input_gps_lat     = NAV_LAT_DGNSS                             ");
  blk("  input_gps_lon     = NAV_LONG_DGNSS                            ");
  blk("  input_gps_speed   = NAV_SPEED_DGNSS                           ");
  blk("  input_gps_heading = GPS_HEADING_DGNSS                         ");
  blk("  input_mag_heading = NAV_HEADING_AHRS                          ");
  blk("  input_gyro_z      = GYRO_Z_LVL_IMU                            ");
  blk("                                                                ");
  blk("  // Output suffix - empty publishes bare NAV_X/Y/HEADING/SPEED.");
  blk("  // Set non-empty to namespace as <base>_<pub_suffix>.         ");
  blk("  pub_suffix =                                                  ");
  blk("                                                                ");
  blk("  // Operational thresholds                                     ");
  blk("  speed_threshold              = 0.5    // m/s, bias gating     ");
  blk("  reverse_heading_error_thresh = 90.0   // deg, reverse detect  ");
  blk("                                                                ");
  blk("  // Process noise                                              ");
  blk("  pos_process_variance     = 0.05                               ");
  blk("  speed_process_variance   = 0.01                               ");
  blk("  heading_process_variance = 0.001                              ");
  blk("  bias_process_variance    = 0.0001                             ");
  blk("                                                                ");
  blk("  // Measurement noise                                          ");
  blk("  gps_pos_stddev   = 1.0   // m                                 ");
  blk("  gps_speed_stddev = 0.5   // m/s                               ");
  blk("  mag_meas_stddev  = 1.0   // deg                               ");
  blk("  bias_meas_stddev = 1.0   // deg                               ");
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
  blu("pBB_EKF INTERFACE                                               ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS (defaults; rebindable via input_*):                ");
  blk("------------------------------------                            ");
  blk("  NAV_LAT_DGNSS, NAV_LONG_DGNSS  - From iUnicore (lat/lon)      ");
  blk("  NAV_SPEED_DGNSS                - GNSS ground speed (m/s)      ");
  blk("  GPS_HEADING_DGNSS              - GNSS heading (compass deg)   ");
  blk("  NAV_HEADING_AHRS               - Madgwick magnetic heading    ");
  blk("  GYRO_Z_LVL_IMU                 - Level-frame yaw rate (rad/s) ");
  blk("                                                                ");
  blk("PUBLICATIONS (defaults; suffixed by pub_suffix when set):        ");
  blk("------------------------------------                            ");
  blk("  NAV_X, NAV_Y         - Filtered local-grid position (m)       ");
  blk("  NAV_HEADING          - Filtered heading (compass deg)         ");
  blk("  NAV_SPEED            - Filtered speed (m/s)                   ");
  blk("  EKF_STATUS           - INITIALIZED / UNHEALTHY                ");
  blk("  EKF_BIAS_DEG         - Mag-vs-GPS heading bias (deg)          ");
  blk("  EKF_REVERSE_MOTION   - 1 when reverse-heading detected        ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pBB_EKF", "gpl");
  exit(0);
}
