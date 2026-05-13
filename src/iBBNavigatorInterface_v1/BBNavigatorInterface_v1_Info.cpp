/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: iBBNavigatorInterface_v1/BBNavigatorInterface_v1_Info.cpp
   Last Ed:  2025-03-30
     Brief:
        Combined Navigator Interface for Blueboat ASV for Navigator version 0.0.6.
*************************************************************/

#include <cstdlib>
#include <iostream>
#include "BBNavigatorInterface_v1_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The iBBNavigatorInterface application provides combined       ");
  blk("  thrust control and AHRS sensing for Blueboat ASV.             ");
  blk("                                                                ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: iBBNavigatorInterface file.moos [OPTIONS]                ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch iBBNavigatorInterface with the given process name  ");
  blk("      rather than iBBNavigatorInterface.                        ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of iBBNavigatorInterface.     ");
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
  blu("iBBNavigatorInterface Example MOOS Configuration                ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = iBBNavigatorInterface                           ");
  blk("{                                                               ");
  blk("  AppTick   = 10                                                ");
  blk("  CommsTick = 10                                                ");
  blk("                                                                ");
  blk("  // Thrust control                                             ");
  blk("  left_thruster_pin = 14                                        ");
  blk("  right_thruster_pin = 16                                       ");
  blk("  max_thrust = 100                                              ");
  blk("  min_thrust = -100                                             ");
  blk("  thruster_dead_band = 5                                        ");
  blk("                                                                ");
  blk("  // AHRS config                                                ");
  blk("  sample_rate = 150                                             ");
  blk("  ahrs_gain = 0.2                                               ");
  blk("  mag_ak_cal_file = /path/to/mag_cal.txt                        ");
  blk("  declination_deg = -14.058                                     ");
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
  blu("iBBNavigatorInterface INTERFACE                                 ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  DESIRED_THRUST_L, DESIRED_THRUST_R - Thrust commands          ");
  blk("  ALL_STOP, MISSION_COMPLETE - Control signals                  ");
  blk("  RC_CONNECTED, RC_CH1-16 - RC controller input                 ");
  blk("  RC_DEADMAN_ENABLED - Runtime override for RC deadman watchdog ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  AHRS group (suffix=<ahrs_pub_suffix>, default AHRS):           ");
  blk("    NAV_ROLL, NAV_PITCH, NAV_YAW, NAV_HEADING                    ");
  blk("  IMU group  (suffix=<imu_pub_suffix>,  default IMU):            ");
  blk("    GYRO_X, GYRO_Y, GYRO_Z, GYRO_Z_LVL                           ");
  blk("  Power/health (unsuffixed):                                     ");
  blk("    NVGR_VOLTAGE, NVGR_CURRENT, NVGR_ROLLING_POWER               ");
  blk("    NVGR_THRUST_LEFT, NVGR_THRUST_RIGHT                          ");
  blk("    NVGR_THRUST_TIMEOUT - autonomous-mode command timeout        ");
  blk("    NVGR_RC_DEADMAN_ACTIVE - RC deadman state                    ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("iBBNavigatorInterface", "gpl");
  exit(0);
}

