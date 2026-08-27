/****************************************************************/
/*   NAME: J. Wenger                                            */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: ThrustMix_Info.cpp                                   */
/*   DATE: 2026-08-27 (rewrite)                                 */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "ThrustMix_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  Front-seat thrust mixer for the BlueBoat.                      ");
  blk("                                                                 ");
  blk("      BB_SELECTED_CMD  ->  BB_MIXED_CMD                          ");
  blk("                                                                 ");
  blk("  Converts the arbiter's semantic surge/yaw into physical        ");
  blk("  left/right motor effort using the ArduRover 4.7 skid-steer     ");
  blk("  allocation (MOT_THST_ASYM / MOT_STR_THR_MIX), slew-limits      ");
  blk("  COMMON THROTTLE ONLY, and copies the upstream lineage onward   ");
  blk("  unchanged so any actuator frame resolves back to one command.  ");
  blk("                                                                 ");
  blk("  It subscribes to exactly ONE variable. It does not see RC,     ");
  blk("  teleop, autonomy or ALL_STOP, does not arbitrate, does not     ");
  blk("  emit PWM, and closes no feedback loop.                         ");
  blk("                                                                 ");
  blk("  Positive effort means physical FORWARD thrust on that side.    ");
  blk("  Electrical inversion happens only in iBBNavigatorInterface.    ");
}

//----------------------------------------------------------------
void showHelpAndExit()
{
  blk("                                                          ");
  blu("=============================================================== ");
  blu("Usage: pThrustMix file.moos [OPTIONS]                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pThrustMix with the given process name.            ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display release version of pThrustMix.                    ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pThrustMix Example MOOS Configuration                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pThrustMix                                      ");
  blk("{                                                               ");
  blk("  AppTick   = 50                                                ");
  blk("  CommsTick = 50                                                ");
  blk("                                                                ");
  blk("  // REQUIRED. No default, deliberately: a log must never be    ");
  blk("  // ambiguous about which equation produced a motor pair.      ");
  blk("  // Startup FAILS without it.                                  ");
  blk("  mixer_model = ARDUROVER_4_7_SKID                              ");
  blk("                                                                ");
  blk("  // Input lease. Start lenient; tighten only once the measured ");
  blk("  // BB_MIX_INPUT_AGE distribution justifies it.                ");
  blk("  selected_cmd_timeout_sec = 0.5                                ");
  blk("                                                                ");
  blk("  // MOT_SLEWRATE. 200 %/s = zero to full forward in 0.5 s.     ");
  blk("  // Applies to COMMON THROTTLE ONLY -- yaw is never slewed,    ");
  blk("  // which is what keeps turn onset crisp.                      ");
  blk("  slew_rate_pct_sec = 200                                       ");
  blk("                                                                ");
  blk("  // Caps how much rate credit one cycle may earn, so a         ");
  blk("  // descheduled app cannot be granted an unbounded step.       ");
  blk("  slew_max_dt_sec = 0.5                                         ");
  blk("                                                                ");
  blk("  // false (default): carry common-throttle state across a      ");
  blk("  // source handoff, so an operator taking manual control       ");
  blk("  // mid-transit does not get a drop to zero and a ramp back.   ");
  blk("  slew_reset_on_handoff = false                                 ");
  blk("                                                                ");
  blk("  // ArduRover BlueBoat120 defaults. thrust_asymmetry is        ");
  blk("  // copied, not validated -- tune only from tank or on-water   ");
  blk("  // evidence.                                                  ");
  blk("  thrust_asymmetry      = 1.6   // MOT_THST_ASYM                 ");
  blk("  steering_throttle_mix = 0.6   // MOT_STR_THR_MIX              ");
  blk("                                                                ");
  blk("  // Wire names, for bench rigs. Leave alone on a boat.         ");
  blk("  // selected_cmd_var = BB_SELECTED_CMD                         ");
  blk("  // mixed_cmd_var    = BB_MIXED_CMD                            ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pThrustMix INTERFACE                                            ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  BB_SELECTED_CMD  The arbiter's decision. One coherent string  ");
  blk("                   carrying surge/yaw, the stop flag and reason,");
  blk("                   and the full upstream lineage.               ");
  blk("                   This is the ONLY command input.              ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  BB_MIXED_CMD     Primary contract. left/right physical effort ");
  blk("                   plus every intermediate and the complete     ");
  blk("                   source -> decision -> mix lineage.           ");
  blk("                                                                ");
  blk("  Scalar mirrors -- TELEMETRY ONLY. Nothing downstream may      ");
  blk("  consume these as a command; they exist for plotting.          ");
  blk("    BB_MIX_LEFT           physical left effort  [-100,100]      ");
  blk("    BB_MIX_RIGHT          physical right effort [-100,100]      ");
  blk("    BB_MIX_SURGE_SHAPED   common throttle after slew            ");
  blk("    BB_MIX_YAW_SHAPED     yaw (equals yaw in; never slewed)     ");
  blk("    BB_MIX_INPUT_AGE      seconds since the last new decision   ");
  blk("    BB_MIX_HARD_STOP      true/false                            ");
  blk("    BB_MIX_STOP_REASON    stable token, see below               ");
  blk("    BB_MIX_SATURATION     ArduRover q; > 1 means the pair did   ");
  blk("                          not fit and priority was applied      ");
  blk("    BB_MIX_INPUT_REJECT_REASON  on a rejected input frame       ");
  blk("                                                                ");
  blk("STOP REASONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  Propagated from the arbiter when it stopped deliberately:     ");
  blk("    RC_INVALID, RC_STALE, TELEOP_ESTOP, TELEOP_INVALID,         ");
  blk("    TELEOP_STALE, AUTONOMY_ALL_STOP, AUTONOMY_INVALID,          ");
  blk("    AUTONOMY_STALE, STARTUP                                     ");
  blk("  Raised by this app about its own input:                       ");
  blk("    MIXER_INPUT_INVALID   never heard from the arbiter at all   ");
  blk("    MIXER_INPUT_STALE     arbiter stopped publishing            ");
  blk("  These three are kept distinct on purpose: no arbiter, a quiet ");
  blk("  arbiter, and a deliberately stopped arbiter are different     ");
  blk("  faults, and a log that conflates them cannot be debugged.     ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
void showReleaseInfoAndExit()
{
  showReleaseInfo("pThrustMix", "gpl");
  exit(0);
}
