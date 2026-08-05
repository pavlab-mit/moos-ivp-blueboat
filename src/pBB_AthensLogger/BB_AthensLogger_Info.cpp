/*************************************************************
      Name: Ray Turrisi
      Orgn: MIT, Cambridge MA
      File: pBB_AthensLogger/BB_AthensLogger_Info.cpp
   Last Ed: 2026-06-22
     Brief:
        Lorem ipsum dolor sit amet, consectetur adipiscing
        elit, sed do eiusmod tempor incididunt ut labore et
        dolore magna aliqua. Ut enim ad minim veniam, quis
        nostrud exercitation ullamco laboris nisi ut aliquip
        ex ea commodo consequat.
*************************************************************/

#include <cstdlib>
#include <iostream>
#include "BB_AthensLogger_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pBB_AthensLogger is a front-seat data logger. It is a pure    ");
  blk("  sink: it subscribes to the front-seat MOOSDB (wildcard by     ");
  blk("  default) and appends every variable update, as it arrives,    ");
  blk("  to a single per-day pipe-delimited time-series file:          ");
  blk("                                                                ");
  blk("      time|var|src|type|value                                   ");
  blk("                                                                ");
  blk("  Files are append-only (LOG_<vname>_<YYYYMMDD>.psv) and roll   ");
  blk("  at the day boundary; missions are separated after the fact    ");
  blk("  by the logged MISSION_HASH rows. It never publishes and is    ");
  blk("  not in the control path, so a crash is harmless.              ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pBB_AthensLogger file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pBB_AthensLogger with the given process name         ");
  blk("      rather than pBB_AthensLogger.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pBB_AthensLogger.        ");
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
  blu("pBB_AthensLogger Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pBB_AthensLogger                                ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  vname    = asha            // used in the filename            ");
  blk("  log_dir  = /home/pi/bb_daily_logs  // launch-stable abs path  ");
  blk("                                                                ");
  blk("  wildcard = true            // log all vars (default)          ");
  blk("  utc      = false           // day boundary: local (def)/UTC   ");
  blk("  value_digits = 8           // decimals for double values      ");
  blk("                                                                ");
  blk("  // omit = DB_*, APPCAST*, *_ITER_GAP, *_ITER_LEN, *_STATUS    ");
  blk("  // keep = BB_STATUS        // exact names that beat omit       ");
  blk("  // log  = NAV_X, NAV_Y     // used only when wildcard = false  ");
  blk("                                                                ");
  blk("  debug    = false                                              ");
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
  blu("pBB_AthensLogger INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  All variables (wildcard '*') by default, minus the configured ");
  blk("  omit patterns; or the explicit 'log' list when wildcard=false.");
  blk("  MISSION_HASH is always tracked for session markers.           ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  None. pBB_AthensLogger is a pure sink and never publishes.    ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pBB_AthensLogger", "gpl");
  exit(0);
}
