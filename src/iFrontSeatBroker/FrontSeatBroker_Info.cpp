/*************************************************************
      Name: 
      Orgn: MIT, Cambridge MA
      File: iFrontSeatBroker/FrontSeatBroker_Info.cpp
   Last Ed:  2026-03-23
     Brief:
        Help, example config, and interface text for
        iFrontSeatBroker.
*************************************************************/

#include <cstdlib>
#include "FrontSeatBroker_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  iFrontSeatBroker is a simple UDP <-> MOOS bridge.             ");
  blk("  It sends configured MOOS vars over UDP and receives typed      ");
  blk("  UDP frames to publish into MOOSDB, with optional rename map.   ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: iFrontSeatBroker file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch iFrontSeatBroker with the given process name         ");
  blk("      rather than iFrontSeatBroker.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of iFrontSeatBroker.        ");
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
  blu("iFrontSeatBroker Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = iFrontSeatBroker                              ");
  blk("{                                                               ");
  blk("  AppTick     = 20                                              ");
  blk("  CommsTick   = 20                                              ");
  blk("  listen_ip   = $(IP_ADDR)                                      ");
  blk("  listen_port = 9201                                            ");
  blk("  tx_ip       = $(BSEAT_IP)                                     ");
  blk("  tx_port     = 9200                                            ");
  blk("  tx_vars     = NAV_LAT_DGNSS,NAV_LONG_DGNSS,GPS_HEADING_DGNSS  ");
  blk("  in_map      = DESIRED_THRUST_L:DESIRED_THRUST_L               ");
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
  blu("iFrontSeatBroker INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  (configured via tx_vars)                                      ");
  blk("  Example: NAV_LAT_DGNSS, NAV_LONG_DGNSS, GPS_HEADING_DGNSS     ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  Incoming UDP frame fields become MOOS vars.                   ");
  blk("  Field names are remapped by in_map when provided.             ");
  blk("                                                                ");
  blk("UDP FRAME FORMAT:                                               ");
  blk("------------------------------------                            ");
  blk("  <VAR1=D:123.4|VAR2=S:hello%20world>                           ");
  blk("  D: double, S: percent-encoded string                          ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("iFrontSeatBroker", "gpl");
  exit(0);
}

