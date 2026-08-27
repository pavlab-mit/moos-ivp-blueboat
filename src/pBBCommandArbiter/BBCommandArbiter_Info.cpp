/****************************************************************/
/*   NAME: J. Wenger                                            */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: BBCommandArbiter_Info.cpp                            */
/*   DATE: 2026-08-27                                           */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "BBCommandArbiter_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The only process allowed to decide who is driving the boat.    ");
  blk("                                                                 ");
  blk("      RC_INPUT_STATE  ]                                          ");
  blk("      TELEOP_CMD      ]->  BB_SELECTED_CMD                       ");
  blk("      AUTONOMY_CMD    ]                                          ");
  blk("                                                                 ");
  blk("  Priority is RC > TELEOP > AUTONOMY, but priority is not the    ");
  blk("  interesting part. Two rules are:                               ");
  blk("                                                                 ");
  blk("  COMMAND IS NOT AUTHORITY. Publishing a valid command does not  ");
  blk("  grant the right to drive. RC must hold the guarded mode        ");
  blk("  switch; teleop must assert a claim. A stick knocked in AUTO    ");
  blk("  cannot take the boat.                                          ");
  blk("                                                                 ");
  blk("  NO SILENT FALLTHROUGH. If a manual owner goes stale or         ");
  blk("  invalid, the boat STOPS. It is never quietly handed to         ");
  blk("  autonomy. An operator whose link just died is not asking for   ");
  blk("  an autonomous mission.                                         ");
  blk("                                                                 ");
  blk("  It emits semantic surge/yaw only. It does not mix, does not    ");
  blk("  emit PWM, does not enforce RC kill, and does not implement     ");
  blk("  the RC deadman.                                                ");
}

void showHelpAndExit()
{
  blk("                                                          ");
  blu("=============================================================== ");
  blu("Usage: pBBCommandArbiter file.moos [OPTIONS]                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch with the given process name.                       ");
  mag("  --example, -e   ");  blk("  Show an example configuration.    ");
  mag("  --help, -h      ");  blk("  Show this message.                ");
  mag("  --interface, -i ");  blk("  Show publications/subscriptions.  ");
  mag("  --version, -v   ");  blk("  Show release version.             ");
  blk("                                                                ");
  exit(0);
}

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pBBCommandArbiter Example MOOS Configuration                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pBBCommandArbiter                               ");
  blk("{                                                               ");
  blk("  AppTick   = 50                                                ");
  blk("  CommsTick = 50                                                ");
  blk("                                                                ");
  blk("  // SOURCE LEASES. How long a source's last NEW sequence stays ");
  blk("  // usable. A repeated sequence does NOT refresh these: the    ");
  blk("  // broker re-sends its last field map every tick, so mail     ");
  blk("  // arriving is not evidence the controller is alive.          ");
  blk("  //                                                             ");
  blk("  // Start lenient and tighten only once the measured           ");
  blk("  // BB_ARB_LAST_VALID_*_AGE distributions justify it.          ");
  blk("  rc_timeout_sec       = 1.0                                    ");
  blk("  teleop_timeout_sec   = 1.5                                    ");
  blk("  autonomy_timeout_sec = 2.0                                    ");
  blk("                                                                ");
  blk("  // The window BEFORE the first valid RC frame ever arrives.   ");
  blk("  // true (default): a boat launched with the handset off can   ");
  blk("  // still run autonomously. false: it refuses until it has     ");
  blk("  // heard from a handset at least once. Once ANY RC frame has  ");
  blk("  // been seen the mode switch is authoritative and this is     ");
  blk("  // irrelevant.                                                ");
  blk("  allow_autonomy_before_first_rc = true                         ");
  blk("                                                                ");
  blk("  // Wire names, for bench rigs. Leave alone on a boat.         ");
  blk("  // rc_input_var     = RC_INPUT_STATE                          ");
  blk("  // teleop_cmd_var   = TELEOP_CMD                              ");
  blk("  // autonomy_cmd_var = AUTONOMY_CMD                            ");
  blk("  // selected_cmd_var = BB_SELECTED_CMD                         ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pBBCommandArbiter INTERFACE                                     ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  RC_INPUT_STATE    from iRCInterface. Carries mode=MANUAL|      ");
  blk("                    NON_MANUAL, which is how RC REQUESTS        ");
  blk("                    authority. Sticks alone do not.             ");
  blk("  TELEOP_CMD        from iTeleop. Carries claim=0|1 and         ");
  blk("                    estop=0|1. Command presence is not a claim. ");
  blk("  AUTONOMY_CMD      from pBBPID. Never requests; takes the boat ");
  blk("                    when no manual source wants it.             ");
  blk("  ALL_STOP          gates AUTONOMY ONLY. It must never be able  ");
  blk("                    to paralyse a manual rescue, so a latched   ");
  blk("                    ALL_STOP does not block RC or teleop.       ");
  blk("  RC_KILL_ASSERTED  TRACE ONLY. Recorded so the log explains a  ");
  blk("                    standstill. Enforcement is the Navigator's, ");
  blk("                    on the hardware side, where it survives a   ");
  blk("                    failure of this process.                    ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  BB_SELECTED_CMD   Primary contract, EVERY cycle including     ");
  blk("                    stop cycles. A gap is indistinguishable     ");
  blk("                    from a dead arbiter, and the mixer's lease  ");
  blk("                    depends on a steady sequence.               ");
  blk("                                                                ");
  blk("  BB_AUTHORITY_EVENT  On any authority or stop-reason change,   ");
  blk("                    published immediately, never decimated.     ");
  blk("                    Carries fail_closed=1 when a manual owner   ");
  blk("                    lost the boat. Read this first after an     ");
  blk("                    incident.                                   ");
  blk("                                                                ");
  blk("  Scalar mirrors -- TELEMETRY ONLY, never a command source:     ");
  blk("    BB_CMD_AUTHORITY        NONE|RC|TELEOP|AUTONOMY             ");
  blk("    BB_CMD_STOP_REASON      stable token                        ");
  blk("    BB_CMD_SURGE_SELECTED   post authority-cap, 0 when stopped  ");
  blk("    BB_CMD_YAW_SELECTED     post authority-cap, 0 when stopped  ");
  blk("    BB_ARB_LAST_VALID_RC_AGE                                    ");
  blk("    BB_ARB_LAST_VALID_TELEOP_AGE                                ");
  blk("    BB_ARB_LAST_VALID_AUTONOMY_AGE                              ");
  blk("      ^ these three are how the timeouts above get chosen from  ");
  blk("        evidence rather than taste.                             ");
  blk("    BB_ARB_REJECT_SOURCE / _REASON / _COUNT                     ");
  blk("                                                                ");
  blk("STOP REASONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  STARTUP            nothing usable yet                         ");
  blk("  RC_INVALID         RC holds MANUAL but its frame is invalid   ");
  blk("  RC_STALE           RC holds MANUAL and its link went quiet    ");
  blk("  TELEOP_ESTOP       operator pressed E-STOP                    ");
  blk("  TELEOP_INVALID     claim held, frame invalid                  ");
  blk("  TELEOP_STALE       claim held, iTeleop went quiet             ");
  blk("  AUTONOMY_ALL_STOP  ALL_STOP latched                           ");
  blk("  AUTONOMY_INVALID   no valid autonomy command                  ");
  blk("  AUTONOMY_STALE     back seat went quiet                       ");
  blk("  INTERNAL_FAULT     a non-finite value reached the output      ");
  blk("                                                                ");
  blk("  RC_STALE and TELEOP_STALE are the fail-closed reasons: the    ");
  blk("  boat stopped rather than fall through to autonomy.            ");
  blk("                                                                ");
  exit(0);
}

void showReleaseInfoAndExit()
{
  showReleaseInfo("pBBCommandArbiter", "gpl");
  exit(0);
}
