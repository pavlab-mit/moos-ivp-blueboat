/*************************************************************
      Name: Jeremy Wenger
      Orgn: MIT, Cambridge MA
      File: iUnicoreGPS/UnicoreGPS_Info.cpp
   Last Ed: 2026-03-04
     Brief:
        Help and interface documentation for iUnicoreGPS.
*************************************************************/

#include <cstdlib>
#include "UnicoreGPS_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The iUnicoreGPS application reads position, velocity, and     ");
  blk("  dual-antenna heading data from a Unicore UM982 GNSS receiver  ");
  blk("  via serial port and publishes navigation data to the MOOSDB.  ");
  blk("  Publish groups are configurable to control which variables    ");
  blk("  are published.                                                ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: iUnicoreGPS file.moos [OPTIONS]                          ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch iUnicoreGPS with the given process name.           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of iUnicoreGPS.               ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("iUnicoreGPS Example MOOS Configuration                          ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = iUnicoreGPS                                     ");
  blk("{                                                               ");
  blk("  AppTick   = 10                                                ");
  blk("  CommsTick = 10                                                ");
  blk("                                                                ");
  blk("  // Serial port configuration                                  ");
  blk("  port      = /dev/ttyUSB0                                      ");
  blk("  baud_rate = 460800                                            ");
  blk("                                                                ");
  blk("  // Navigation                                                 ");
  blk("  nav_suffix      =            // Suffix for NAV_X/Y/LAT/LONG  ");
  blk("  enable_geodesic = true       // Lat/lon -> local X/Y          ");
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
  blu("iUnicoreGPS INTERFACE                                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  None                                                          ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  Position group (publish_position):                            ");
  blk("    NAV_X{suffix}, NAV_Y{suffix}  - Local coordinates (m)      ");
  blk("    NAV_X_GPS, NAV_Y_GPS          - Raw GPS local coords (m)   ");
  blk("    NAV_LAT, NAV_LONG             - Geodetic coords (deg)      ");
  blk("                                                                ");
  blk("  Velocity group (publish_velocity):                            ");
  blk("    NAV_SPEED                     - Ground speed (m/s)          ");
  blk("    VEL_N, VEL_E, VEL_D          - Velocity components (m/s)   ");
  blk("    NAV_COG                       - Course over ground (deg)    ");
  blk("                                                                ");
  blk("  Heading group (publish_heading):                              ");
  blk("    GPS_HEADING                   - Dual-antenna heading (deg)  ");
  blk("    GPS_HEADING_ACC               - Heading accuracy (deg)      ");
  blk("    GPS_BASELINE                  - Antenna baseline (m)        ");
  blk("    GPS_PITCH                     - Pitch from baseline (deg)   ");
  blk("    GPS_CARR_SOLN                 - Carrier solution type       ");
  blk("    GPS_HEADING_VALID             - Heading validity (true/false)");
  blk("                                                                ");
  blk("  DOP group (publish_dops):                                     ");
  blk("    HDOP, VDOP, PDOP, GDOP       - Dilution of precision       ");
  blk("                                                                ");
  blk("  Status group (publish_status):                                ");
  blk("    GPS_HAS_LOCK                  - Fix status (true/false)     ");
  blk("    GPS_FIX_TYPE                  - Fix type string             ");
  blk("    GPS_NUM_SATS                  - Number of satellites used   ");
  blk("                                                                ");
  blk("  Time group (publish_time):                                    ");
  blk("    GPS_EPOCH_TIME                - GPS epoch timestamp (s)     ");
  blk("                                                                ");
  blk("  State group (publish_state):                                  ");
  blk("    GPS_STATE                     - Comprehensive state string  ");
  blk("                                                                ");
  blk("  DTO group (publish_dto):                                      ");
  blk("    GNSS_POSITION, GNSS_HEADING, GNSS_VELOCITY,                ");
  blk("    GNSS_DOPS, GNSS_STATUS        - Serialized DTO strings     ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("iUnicoreGPS", "gpl");
  exit(0);
}
