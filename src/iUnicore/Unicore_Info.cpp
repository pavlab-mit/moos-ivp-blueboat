/*************************************************************
      Name: Jeremy Wenger
      Orgn: MIT, Cambridge MA
      File: iUnicore/Unicore_Info.cpp
   Last Ed: 2026-04-26
     Brief:
        Help and interface documentation for iUnicore.
*************************************************************/

#include <cstdlib>
#include "Unicore_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The iUnicore application reads position, velocity, and        ");
  blk("  dual-antenna heading from a Unicore UM982 GNSS receiver via   ");
  blk("  serial. All publications are appended with an underscore +    ");
  blk("  pub_suffix (default DGNSS), so consumers can disambiguate     ");
  blk("  this sensor from any other state source. No geodesic / no     ");
  blk("  NAV_X or NAV_Y publishing - that conversion lives downstream  ");
  blk("  in the EKF / bundler that owns helm state.                    ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: iUnicore file.moos [OPTIONS]                             ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch iUnicore with the given process name.              ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of iUnicore.                  ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("iUnicore Example MOOS Configuration                             ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = iUnicore                                        ");
  blk("{                                                               ");
  blk("  AppTick   = 10                                                ");
  blk("  CommsTick = 10                                                ");
  blk("                                                                ");
  blk("  // Serial port configuration                                  ");
  blk("  port      = /dev/ttyUSB0                                      ");
  blk("  baud_rate = 460800                                            ");
  blk("                                                                ");
  blk("  // Publication suffix - appended as _<suffix> to every        ");
  blk("  // published variable. Empty = no suffix (bare names).        ");
  blk("  pub_suffix = DGNSS                                            ");
  blk("                                                                ");
  blk("  // Publish groups (true/false)                                ");
  blk("  publish_position = true                                       ");
  blk("  publish_velocity = true                                       ");
  blk("  publish_heading  = true                                       ");
  blk("  publish_dops     = true                                       ");
  blk("  publish_status   = true                                       ");
  blk("  publish_time     = false                                      ");
  blk("  publish_state    = true                                       ");
  blk("  publish_dto      = false                                      ");
  blk("                                                                ");
  blk("  // Serial watchdog                                            ");
  blk("  stale_data_timeout_sec = 2.0   // No-data timeout (seconds)   ");
  blk("  stale_warn_repeat_sec  = 5.0   // Repeat warning period (sec) ");
  blk("  quality_warn_repeat_sec = 10.0 // Quality debug repeat (sec)  ");
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
  blu("iUnicore INTERFACE                                              ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  RTCM_DATA  -  Binary RTCM stream forwarded to GNSS serial.    ");
  blk("                                                                ");
  blk("PUBLICATIONS (all suffixed with _<pub_suffix>; default DGNSS):  ");
  blk("------------------------------------                            ");
  blk("  Position group (publish_position):                            ");
  blk("    NAV_LAT, NAV_LONG             - Geodetic coords (deg)       ");
  blk("                                                                ");
  blk("  Velocity group (publish_velocity):                            ");
  blk("    NAV_SPEED                     - Ground speed (m/s)          ");
  blk("    VEL_N, VEL_E, VEL_D           - Velocity components (m/s)   ");
  blk("    NAV_COG                       - Course over ground (deg)    ");
  blk("                                                                ");
  blk("  Heading group (publish_heading):                              ");
  blk("    GPS_HEADING                   - Dual-antenna heading (deg)  ");
  blk("    GPS_HEADING_ACC               - Heading accuracy (deg)      ");
  blk("    GPS_BASELINE                  - Antenna baseline (m)        ");
  blk("    GPS_PITCH                     - Pitch from baseline (deg)   ");
  blk("    GPS_CARR_SOLN                 - Carrier solution type       ");
  blk("    GPS_HEADING_VALID             - Heading validity            ");
  blk("                                                                ");
  blk("  DOP group (publish_dops):                                     ");
  blk("    HDOP, VDOP, PDOP, GDOP        - Dilution of precision       ");
  blk("    HDG_HDOP, HDG_PDOP            - Heading-only DOPs           ");
  blk("                                                                ");
  blk("  Status group (publish_status):                                ");
  blk("    GPS_HAS_LOCK, GPS_FIX_TYPE,                                 ");
  blk("    GPS_NUM_SATS, GPS_CORR_HEALTHY,                             ");
  blk("    GPS_RTK_CALC_STATUS, GPS_CORR_AGE,                          ");
  blk("    GPS_SOL_AGE, GPS_BASE_ID                                    ");
  blk("                                                                ");
  blk("  Time group (publish_time):                                    ");
  blk("    GPS_EPOCH_TIME                                              ");
  blk("                                                                ");
  blk("  State group (publish_state):                                  ");
  blk("    FIX_STATE                     - Time-bundled CSV of full   ");
  blk("                                    fix state for EKF intake.   ");
  blk("                                                                ");
  blk("  DTO group (publish_dto):                                      ");
  blk("    GNSS_POSITION, GNSS_HEADING, GNSS_VELOCITY,                 ");
  blk("    GNSS_DOPS, GNSS_STATUS, GNSS_RTK_STATUS                     ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("iUnicore", "gpl");
  exit(0);
}
