/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: pBB_Health/BB_Health_Info.cpp
   Last Ed: 2026-03-25
     Brief: Info functions for pBB_Health
*************************************************************/

#include <cstdlib>
#include <iostream>
#include "BB_Health_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pBB_Health monitors BlueBoat system health including:         ");
  blk("    - Battery voltage and current (from iBBThrustCtrl)          ");
  blk("    - Thermal status (RPi temp, internal temp)                  ");
  blk("    - EKF health status (from pBB_DGPS_EKF)                     ");
  blk("    - GPS quality (HDOP, fix type, satellites)                  ");
  blk("    - RC connection and mode state (from iRCReader)             ");
  blk("                                                                ");
  blk("  Publishes visual indicators when in RC mode (CH6 == 2.0):     ");
  blk("    - Yellow ring around vehicle (persistent)                   ");
  blk("    - White range pulse on RC mode transition                   ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pBB_Health file.moos [OPTIONS]                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pBB_Health with the given process name             ");
  blk("      rather than pBB_Health.                                   ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pBB_Health.                ");
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
  blu("pBB_Health Example MOOS Configuration                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pBB_Health                                      ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  // Staleness threshold (seconds)                              ");
  blk("  stale_time = 3.0                                              ");
  blk("                                                                ");
  blk("  // Battery thresholds                                         ");
  blk("  low_voltage_thresh = 22.0      // Volts                       ");
  blk("  high_current_thresh = 30.0     // Amps                        ");
  blk("                                                                ");
  blk("  // Thermal thresholds (Celsius)                               ");
  blk("  high_pi_temp_thresh = 80.0                                    ");
  blk("  high_internal_temp_thresh = 50.0                              ");
  blk("                                                                ");
  blk("  // GPS quality threshold                                      ");
  blk("  hdop_thresh = 2.0                                             ");
  blk("                                                                ");
  blk("  // RC mode circle radius (meters)                             ");
  blk("  rc_circle_radius = 2.5                                        ");
  blk("                                                                ");
  blk("  // Debug logging                                              ");
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
  blu("pBB_Health INTERFACE                                            ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NVGR_ROLLING_VOLTAGE  - Rolling avg battery voltage (V)       ");
  blk("  NVGR_ROLLING_CURRENT  - Rolling avg battery current (A)       ");
  blk("  RPI_TEMP              - Raspberry Pi temperature (C)          ");
  blk("  NVGTR_IT_C            - Internal enclosure temperature (C)    ");
  blk("  EKF_STATUS            - EKF health status string              ");
  blk("  FIX_STATE_DGNSS       - Time-bundled GNSS fix from iUnicore   ");
  blk("  RC_CONNECTED          - RC transmitter connection status      ");
  blk("  RC_CH6                - RC channel 6 value (2.0 = RC mode)    ");
  blk("  NAV_X                 - Vehicle X position (for visuals)      ");
  blk("  NAV_Y                 - Vehicle Y position (for visuals)      ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  VIEW_CIRCLE       - Yellow circle around vehicle in RC mode   ");
  blk("  VIEW_RANGE_PULSE  - White pulse on RC mode transition         ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pBB_Health", "gpl");
  exit(0);
}
