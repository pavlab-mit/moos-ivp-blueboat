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
  blk("  des_yawrate_filter = 0.3     // LPF time const [s] on desired ");
  blk("                               // yaw rate (0=off); tames FF noise");
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
  blk("  // Feedforward (identified static thrust map; bb_dynamics_id.py).");
  blk("  //   c = c0+cv*v*+crr*r*^2 -> DESIRED_THRUST                   ");
  blk("  //   d = d0+dr*r*+dvr*v*r* -> DESIRED_RUDDER (scaled). PID corrects.");
  blk("  ff_enable        = true                                       ");
  blk("  ff_speed         = 4.428, 4.960, 0.4505    // c0, cv, crr     ");
  blk("  ff_yaw           = -3.420, -6.0653, 8.4952 // d0, dr, dvr     ");
  blk("  ff_rudder_scale  = 1.0    // diff-%% -> rudder units (calibrate)");
  blk("                                                                ");
  blk("  // Reference-FF lifecycle (27 Aug step-and-hold fixes).       ");
  blk("  // The held desired-heading derivative is right for a helm    ");
  blk("  // staircase, wrong once the command settles (standing error  ");
  blk("  // of ff/kp) and wrong for a repositioning step (a 180 deg    ");
  blk("  // step's implied-rate sign is numeric noise). 0 = legacy.    ");
  blk("  ff_hold_time     = 2.0    // s; zero held FF this long after  ");
  blk("                            // the last command change (~2x helm");
  blk("                            // cadence)                          ");
  blk("  ff_step_limit    = 150    // deg; larger steps publish no FF  ");
  blk("                                                                ");
  blk("  // Integration lifecycle (brief 3.2: actuation-aware).        ");
  blk("  max_dt           = 0.5    // s; dt credit cap per tick (0=off);");
  blk("                            // a 43 s gap earns ONE tick, not 43s");
  blk("  antiwindup       = true   // tracking anti-windup at the rails ");
  blk("                            // (own clamp, post-FF, mixer): I    ");
  blk("                            // parks at the rail's worth, first- ");
  blk("                            // tick recovery. false = ScalarPID- ");
  blk("                            // verbatim rail behavior.           ");
  blk("  integrate_gate   = true   // integrate only while             ");
  blk("                            // BB_CMD_AUTHORITY==AUTONOMY and   ");
  blk("                            // NVGR_STOP_REASON==NONE; mixer    ");
  blk("                            // saturation (BB_MIX_SATURATION>1) ");
  blk("                            // stops winding the excess. Inputs ");
  blk("                            // stale -> integration FREEZES (P/D");
  blk("                            // live) + run warning.             ");
  blk("  gate_stale_thresh = 2.0   // s; staleness bound on gate inputs");
  blk("                                                                ");
  blk("  // Conventions (verify on the bench!)                         ");
  blk("  yawrate_scale    = 57.2958  // raw yawrate -> deg/s; sign flip");
  blk("  // DEPRECATED. Since pBBPID now publishes AUTONOMY_CMD, the   ");
  blk("  // sign of rudder IS the contract's yaw, which is defined as  ");
  blk("  // + = starboard. A mis-set polarity therefore publishes a     ");
  blk("  // contract violation, turning the boat the wrong way under    ");
  blk("  // autonomy while RC still behaves correctly. Remove once the  ");
  blk("  // pipeline is fully on one convention.                        ");
  blk("  rudder_polarity  = 1        // set -1 if rudder sign inverted ");
  blk("  allow_reverse    = false                                      ");
  blk("  command_stale_thresh = 1.5  // s; zero output if helm silent  ");
  blk("                                                                ");
  blk("  // Variable name overrides (defaults shown)                   ");
  blk("  // nav_yawrate_var  = GYRO_Z_LVL_IMU                          ");
  blk("  // autonomy_cmd_var = AUTONOMY_CMD                            ");
  blk("                                                                ");
  blk("  // Set false only to run this controller with the command     ");
  blk("  // contract switched off entirely (bench work). On a boat the ");
  blk("  // arbiter then sees autonomy as never having produced a       ");
  blk("  // command, and autonomy cannot take the boat.                ");
  blk("  // publish_autonomy_cmd = true                                ");
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
  blk("  BB_CMD_AUTHORITY, NVGR_STOP_REASON, BB_MIX_SATURATION        ");
  blk("                   (applied-output telemetry driving the       ");
  blk("                    integrate gate; see integrate_gate)        ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  AUTONOMY_CMD     The command contract consumed by             ");
  blk("                   pBBCommandArbiter across the seat boundary.  ");
  blk("                   Carries surge/yaw TOGETHER with an epoch and ");
  blk("                   sequence, replacing the coherent-pair        ");
  blk("                   violation of two scalars a consumer had to   ");
  blk("                   re-pair for itself.                          ");
  blk("                     valid=1  actively computing                ");
  blk("                     valid=0  holding off, or helm gone stale.  ");
  blk("                              An INVALID frame is published     ");
  blk("                              rather than going silent: silence ");
  blk("                              reads as staleness, a different   ");
  blk("                              fault from alive-but-not-ready.   ");
  blk("                   Repeated broker re-sends of one sequence do  ");
  blk("                   NOT refresh the arbiter lease, so a hung     ");
  blk("                   controller cannot look alive.                ");
  blk("                                                                ");
  blk("  Diagnostics, retained:                                        ");
  blk("  DESIRED_THRUST, DESIRED_RUDDER   (-> pThrustMix)             ");
  blk("  BBPID_SPEED_ERROR, BBPID_HEADING_ERROR                       ");
  blk("  BBPID_DESIRED_YAWRATE, BBPID_YAWRATE_ERROR                   ");
  blk("  BBPID_{SPEED,HEADING,YAWRATE}_ITERM  (integral terms, for    ");
  blk("                   windup health + FF-bias evidence -- see     ");
  blk("                   docs/tuning_playbook.md)                    ");
  blk("  BBPID_GATE       OPEN/FROZEN/OFF, published on change        ");
  blk("  BBPID_PARAMS_ACTIVE  full active param set, on startup and   ");
  blk("                   every applied change (tuning provenance)    ");
  exit(0);
}

//----------------------------------------------------------------
void showReleaseInfoAndExit()
{
  showReleaseInfo("pBBPID", "gpl");
  exit(0);
}
