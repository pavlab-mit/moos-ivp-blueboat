/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: pDiffThrustPID_v2/DiffThrustPID_v2_Info.cpp
   Last Ed: 2026-06-20
*************************************************************/

#include <cstdlib>
#include "DiffThrustPID_v2_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  Differential-thrust controller for the BlueBoat. Surge and    ");
  blk("  yaw feedforward+integrator regulators are summed into left/   ");
  blk("  right thrust, with a speed-scheduled differential cap.         ");
}

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pDiffThrustPID_v2 file.moos [OPTIONS]                     ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch with the given process name rather than the default");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pDiffThrustPID_v2.         ");
  blk("                                                                ");
  exit(0);
}

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pDiffThrustPID_v2 Example MOOS Configuration                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pDiffThrustPID_v2                               ");
  blk("{                                                               ");
  blk("  AppTick   = 10                                                ");
  blk("  CommsTick = 10                                                ");
  blk("                                                                ");
  blk("  speed_ff_points = 0,0 : 1.0,17 : 2.0,68 : 2.4,98              ");
  blk("  speed_ki        = 0                                           ");
  blk("  speed_i_max     = 15                                          ");
  blk("                                                                ");
  blk("  theta_b      = 30                                             ");
  blk("  max_yaw_rate = 30                                             ");
  blk("  yaw_ff_c0    = 1.5                                            ");
  blk("  yaw_ff_c1    = 2.4                                            ");
  blk("  yaw_ki       = 0                                              ");
  blk("  yaw_i_max    = 25                                             ");
  blk("                                                                ");
  blk("  delta_cap_points = 0,100 : 0.6,100 : 0.8,11 : 1.7,50 : 2.4,50 ");
  blk("  u_max            = 100                                        ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pDiffThrustPID_v2 INTERFACE                                     ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  DESIRED_HEADING, DESIRED_SPEED   (helm setpoints)             ");
  blk("  NAV_HEADING, NAV_SPEED           (measured state)             ");
  blk("  GYRO_Z_LVL_IMU                   (yaw rate, integral only)    ");
  blk("  DEPLOY                                                        ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  DESIRED_THRUST_L, DESIRED_THRUST_R                            ");
  blk("                                                                ");
  exit(0);
}

void showReleaseInfoAndExit()
{
  showReleaseInfo("pDiffThrustPID_v2", "gpl");
  exit(0);
}
