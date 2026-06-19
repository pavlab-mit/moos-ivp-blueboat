/************************************************************/
/*    NAME: Karan Mahesh                                    */
/*    ORGN: MIT / Project Greece                            */
/*    FILE: BBPID_Info.cpp                                  */
/*    DATE: 2026/06/19                                      */
/************************************************************/

#include <cstdlib>
#include <iostream>
#include "BBPID_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  Cascaded PID controller for the BlueBoat. Two control         ");
  blk("  problems, each a TRUE positional PID (output = Kp*e + Ki*Ie + ");
  blk("  Kd*de/dt), unlike pMarinePID whose incremental speed loop     ");
  blk("  behaves as a pure integrator regardless of Kp/Kd.             ");
  blk("                                                                ");
  blk("    Speed loop : DESIRED_SPEED  - NAV_SPEED   -> DESIRED_THRUST ");
  blk("    Yaw cascade: (outer) DESIRED_HEADING - NAV_HEADING          ");
  blk("                          -> desired yaw rate (deg/s, clamped)  ");
  blk("                 (inner) des_rate - meas_yawrate -> DESIRED_RUDDER");
  blk("                                                                ");
  blk("  The inner loop closes on measured yaw rate, so the speed-     ");
  blk("  dependent turn authority of the deep hull is rejected         ");
  blk("  directly. All gains are live-updatable over the MOOSDB.       ");
}

//----------------------------------------------------------------
void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pBBPID file.moos [OPTIONS]                               ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pBBPID with the given process name.                ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  exit(0);
}

//----------------------------------------------------------------
void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pBBPID Example MOOS Configuration                              ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pBBPID                                          ");
  blk("{                                                               ");
  blk("  AppTick   = 10                                                ");
  blk("  CommsTick = 10                                                ");
  blk("                                                                ");
  blk("  // Gains:  param = Kp, Ki, Kd                                 ");
  blk("  speed_pid   = 20, 5, 0       // thrust per (m/s) speed error  ");
  blk("  heading_pid = 1.0, 0, 0      // outer: hdg err -> yaw rate    ");
  blk("  yawrate_pid = 2.0, 0.1, 0    // inner: rate err -> rudder     ");
  blk("                                                                ");
  blk("  // Limits                                                     ");
  blk("  max_thrust             = 100                                  ");
  blk("  max_rudder             = 100                                  ");
  blk("  max_yawrate            = 25     // deg/s commanded turn cap    ");
  blk("  speed_integral_limit   = 50                                   ");
  blk("  yawrate_integral_limit = 50                                   ");
  blk("                                                                ");
  blk("  // Speed-scheduled yaw-rate gains (linearly interpolated).     ");
  blk("  // Shrink Kp + max_yawrate as speed rises so the deep hull     ");
  blk("  // isn't over-driven at speed. Endpoints are held (no extrap). ");
  blk("  enable_gain_schedule = true                                   ");
  blk("  schedule_point = speed=0.5, kp=3.0, ki=0.10, kd=0.0,  max_yawrate=30");
  blk("  schedule_point = speed=1.5, kp=2.0, ki=0.10, kd=0.0,  max_yawrate=20");
  blk("  schedule_point = speed=3.0, kp=1.2, ki=0.05, kd=0.2,  max_yawrate=12");
  blk("                                                                ");
  blk("  // Conventions (verify on the bench!)                         ");
  blk("  yawrate_scale    = 57.2958  // raw yawrate -> deg/s; sign flip");
  blk("  rudder_polarity  = 1        // set -1 if rudder sign inverted ");
  blk("  allow_reverse    = false                                      ");
  blk("  command_stale_thresh = 1.5  // s; zero output if helm silent  ");
  blk("                                                                ");
  blk("  // Variable name overrides (defaults shown)                   ");
  blk("  // nav_yawrate_var = GYRO_Z_LVL_IMU                           ");
  blk("}                                                               ");
  exit(0);
}

//----------------------------------------------------------------
void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pBBPID INTERFACE                                               ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("  DESIRED_SPEED, DESIRED_HEADING   (from IvP Helm)              ");
  blk("  NAV_SPEED, NAV_HEADING           (nav feedback)              ");
  blk("  GYRO_Z_LVL_IMU                   (yaw-rate feedback)         ");
  blk("  BBPID_{SPEED,HEADING,YAWRATE}_{KP,KI,KD}  (live retuning)    ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("  DESIRED_THRUST, DESIRED_RUDDER   (-> pThrustMix)             ");
  blk("  BBPID_SPEED_ERROR, BBPID_HEADING_ERROR                       ");
  blk("  BBPID_DESIRED_YAWRATE, BBPID_YAWRATE_ERROR                   ");
  exit(0);
}

//----------------------------------------------------------------
void showReleaseInfoAndExit()
{
  showReleaseInfo("pBBPID", "gpl");
  exit(0);
}
