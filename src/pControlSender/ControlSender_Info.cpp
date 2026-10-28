/****************************************************************/
/*   NAME: Tyler Paine                                          */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: ControlSender_Info.cpp                               */
/*   DATE: 2026-10-28                                           */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "ControlSender_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  Puts the barrier filter between pBBPID and the arbiter.        ");
  blk("                                                                 ");
  blk("    AUTONOMY_CMD_PRECBF  (pBBPID's frame)        ]               ");
  blk("    DESIRED_THRUST       (filter output)         ]-> AUTONOMY_CMD");
  blk("    DESIRED_RUDDER       (filter output)         ]               ");
  blk("   [CBF_SEQ]             (optional: seq filtered)]               ");
  blk("                                                                 ");
  blk("  AUTONOMY_CMD is pBBPID's frame re-sent with the filtered        ");
  blk("  thrust/rudder swapped in. epoch, seq, source_time, valid and   ");
  blk("  all extra fields are pBBPID's. Only surge/yaw, the producer    ");
  blk("  and a cbf=1 marker change.                                     ");
  blk("                                                                 ");
  blk("  It never creates a sequence number, so a dead pBBPID still     ");
  blk("  stops the boat through the arbiter's lease (AUTONOMY_STALE).   ");
  blk("  A dead filter stops it sooner: once the filter has been quiet  ");
  blk("  for filter_timeout_sec, valid frames go out as valid=0         ");
  blk("  (AUTONOMY_INVALID) with cbf_fault naming why. It never falls   ");
  blk("  back to pBBPID's unfiltered values.                            ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pControlSender file.moos [OPTIONS]                       ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pControlSender with the given process name         ");
  blk("      rather than pControlSender.                               ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pControlSender.            ");
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
  blu("pControlSender Example MOOS Configuration                       ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pControlSender                                  ");
  blk("{                                                               ");
  blk("  // 50 Hz, like the arbiter and mixer: each cycle of delay here ");
  blk("  // sits inside pBBPID's yaw-rate loop.                         ");
  blk("  AppTick   = 50                                                ");
  blk("  CommsTick = 50                                                ");
  blk("                                                                ");
  blk("  // Wire names (defaults shown).                               ");
  blk("  precbf_var  = AUTONOMY_CMD_PRECBF   // pBBPID autonomy_cmd_var ");
  blk("  thrust_var  = DESIRED_THRUST        // filter output           ");
  blk("  rudder_var  = DESIRED_RUDDER        // filter output           ");
  blk("  output_var  = AUTONOMY_CMD          // read by the arbiter     ");
  blk("                                                                ");
  blk("  // Optional. Set when the filter publishes the seq of the      ");
  blk("  // AUTONOMY_CMD_PRECBF frame it filtered, in the same cycle as  ");
  blk("  // thrust/rudder. Labels become exact. Unset: each filter      ");
  blk("  // output goes to the newest frame that had arrived when it    ");
  blk("  // did (exact when the filter is faster than pBBPID's 100 ms   ");
  blk("  // tick, up to one tick late otherwise).                       ");
  blk("  // filter_seq_var = CBF_SEQ                                   ");
  blk("                                                                ");
  blk("  // Written into every frame. pBBPID_CBF makes filtering        ");
  blk("  // visible in BB_SELECTED_CMD/BB_MIXED_CMD lineage. Nothing    ");
  blk("  // downstream checks it; set pBBPID for a strictly identical   ");
  blk("  // frame. Must not contain a comma.                            ");
  blk("  producer = pBBPID_CBF                                         ");
  blk("                                                                ");
  blk("  // Filter silent (or matching no frame) this long -> valid     ");
  blk("  // frames are sent as valid=0. Must exceed the filter's worst  ");
  blk("  // normal gap between outputs. 0.5 s = 5 pBBPID ticks.         ");
  blk("  filter_timeout_sec = 0.5                                      ");
  blk("                                                                ");
  blk("  // Max spread between the publisher timestamps of the halves  ");
  blk("  // of ONE filter output. Publish them in one cycle.            ");
  blk("  pair_tolerance_sec = 0.02                                     ");
  blk("                                                                ");
  blk("  // Mail stamped older than this is dropped: it is the DB      ");
  blk("  // replaying last-known values to a newly started app.         ");
  blk("  max_input_age_sec = 1.0                                       ");
  blk("                                                                ");
  blk("  // Run warning when pBBPID's frame is absent this long.        ");
  blk("  precbf_warn_sec = 3.0                                         ");
  blk("                                                                ");
  blk("  // A frame on output_var without our cbf=1 marker means a     ");
  blk("  // second writer is bypassing the filter (usually pBBPID on    ");
  blk("  // its default autonomy_cmd_var). true: post ALL_STOP=true     ");
  blk("  // (autonomy only; RC/teleop unaffected), re-posted at 1 Hz    ");
  blk("  // while it persists. false: warn only.                        ");
  blk("  all_stop_on_foreign_writer = true                             ");
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
  blu("pControlSender INTERFACE                                        ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  AUTONOMY_CMD_PRECBF  pBBPID's command frame (envelope v1).    ");
  blk("  DESIRED_THRUST       filter output, contract surge, [-100,100]");
  blk("  DESIRED_RUDDER       filter output, contract yaw, + starboard ");
  blk("  CBF_SEQ              optional; seq of the frame filtered      ");
  blk("  AUTONOMY_CMD         our own output, for the bypass guard     ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  AUTONOMY_CMD         pBBPID's frame, filtered values swapped  ");
  blk("                       in, producer replaced, cbf=1 added.      ");
  blk("                       cbf_fault=filter_silent|filter_unmatched|");
  blk("                       filter_never_heard on frames sent        ");
  blk("                       valid=0 because the filter is unhealthy. ");
  blk("  CTLSEND_STATE        WAITING_FOR_PRECBF | FILTERED |          ");
  blk("                       FORWARD_INVALID | FILTER_STALE, on change");
  blk("  CTLSEND_LATENCY      pBBPID tick -> send, s (per send)        ");
  blk("  CTLSEND_SEQ_LAG      newest pBBPID seq - sent seq (per send)  ");
  blk("  CTLSEND_FILTER_AGE   s since the last filter output           ");
  blk("  CTLSEND_SURGE_DELTA  filtered - pBBPID surge (filtered sends) ");
  blk("  CTLSEND_YAW_DELTA    filtered - pBBPID yaw (filtered sends)   ");
  blk("  CTLSEND_REJECT_REASON  on a rejected AUTONOMY_CMD_PRECBF frame");
  blk("  CTLSEND_FOREIGN_WRITER producer/epoch of a second writer      ");
  blk("  ALL_STOP             true, on a second writer, if enabled     ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pControlSender", "gpl");
  exit(0);
}
