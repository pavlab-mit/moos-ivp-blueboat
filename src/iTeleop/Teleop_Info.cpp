/*************************************************************
      Name:
      Orgn: MIT, Cambridge MA
      File: iTeleop/Teleop_Info.cpp
   Last Ed:  2026-07-15
     Brief:
        Help, example config, and interface text for iTeleop.
*************************************************************/

#include <cstdlib>
#include "Teleop_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  iTeleop is the front-seat endpoint for emergency laptop       ");
  blk("  teleoperation. It listens on UDP for a shore GUI, holds a     ");
  blk("  single-client session with a deadman timeout, and publishes   ");
  blk("  TELEOP_ACTIVE / TELEOP_THRUST_L / TELEOP_THRUST_R which the   ");
  blk("  navigator interface arbitrates against RC and autonomy        ");
  blk("  (priority: RC > teleop > autonomy).                           ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: iTeleop file.moos [OPTIONS]                              ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch iTeleop with the given process name rather         ");
  blk("      than iTeleop.                                             ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of iTeleop.                   ");
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
  blu("iTeleop Example MOOS Configuration                              ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = iTeleop                                         ");
  blk("{                                                               ");
  blk("  AppTick     = 20                                              ");
  blk("  CommsTick   = 20                                              ");
  blk("  listen_ip   = $(IP_ADDR)                                      ");
  blk("  listen_port = 9310                                            ");
  blk("  gui_deadman_timeout = 1.0   // sec without GUI frame -> release");
  blk("  max_thrust  = 100           // clamp on inbound |thrust|      ");
  blk("  ack_heartbeat_hz = 5        // idle ack rate while connected  ");
  blk("  debug       = false                                           ");
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
  blu("iTeleop INTERFACE                                               ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NVGR_THRUST_LEFT_WIRE   double  applied thrust (wire conv.)   ");
  blk("  NVGR_THRUST_RIGHT_WIRE  double  applied thrust (wire conv.)   ");
  blk("  NVGR_RC_MODE            string  physical RC override active   ");
  blk("  NVGR_RC_DEADMAN_ACTIVE  string  RC deadman tripped            ");
  blk("  NVGR_ROLLING_VOLTAGE    double  pack voltage for GUI acks     ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  TELEOP_CMD       The command contract consumed by             ");
  blk("                   pBBCommandArbiter. Published EVERY iterate,  ");
  blk("                   session or not.                              ");
  blk("                     claim=1  requests authority                ");
  blk("                     claim=0  EXPLICIT RELEASE -- different     ");
  blk("                              from silence, which the arbiter   ");
  blk("                              treats as stale and fail-closes   ");
  blk("                              on if a claim was held.           ");
  blk("                     estop=1  hard stop while claiming          ");
  blk("                     cmd_form semantic | derived_from_tank |    ");
  blk("                              none                              ");
  blk("                                                                ");
  blk("  Legacy, retained during migration:                            ");
  blk("------------------------------------                            ");
  blk("  TELEOP_ACTIVE     string  true while a GUI session owns the   ");
  blk("                            vehicle (re-published every iterate)");
  blk("  TELEOP_THRUST_L   double  teleop thrust, wire convention      ");
  blk("  TELEOP_THRUST_R   double  teleop thrust, wire convention      ");
  blk("  TELEOP_CLIENT     string  operator label + address, or none   ");
  blk("                                                                ");
  blk("UDP FRAME FORMAT (port 9310, broker_v2 framing):                ");
  blk("------------------------------------                            ");
  blk("  GUI->boat: <TOP_CMD=S:CMD|SID=S:..|SEQ=D:n|THRUST_L=D:..|     ");
  blk("              THRUST_R=D:..|ESTOP=D:0>                          ");
  blk("  boat->GUI: <TOP_ACK=S:OK|SEQ=D:n|MODE=S:TELEOP|THR_L=D:..|    ");
  blk("              THR_R=D:..|ESTOP=D:0|VOLT=D:..|RC_DEADMAN=D:0|    ");
  blk("              VNAME=S:zoe>                                      ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("iTeleop", "gpl");
  exit(0);
}
