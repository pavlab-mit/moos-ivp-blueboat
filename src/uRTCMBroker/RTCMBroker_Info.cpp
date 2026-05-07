/*************************************************************
      Name: Jeremy Wenger
      Orgn: MIT, Cambridge MA
      File: uRTCMBroker/RTCMBroker_Info.cpp
   Last Ed: 2026-03-17
     Brief:
        NTRIP client that connects to an RTCM caster and
        distributes binary RTCM corrections via MOOS.
*************************************************************/

#include <cstdlib>
#include <iostream>
#include "RTCMBroker_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The uRTCMBroker application connects to an NTRIP caster      ");
  blk("  and streams RTCM corrections as binary MOOS messages.        ");
  blk("  Designed to run shoreside and distribute corrections to       ");
  blk("  vehicle communities running iUnicore via pShare.             ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: uRTCMBroker file.moos [OPTIONS]                         ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch uRTCMBroker with the given process name.          ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of uRTCMBroker.              ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("uRTCMBroker Example MOOS Configuration                         ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = uRTCMBroker                                     ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  // NTRIP caster connection                                    ");
  blk("  ntrip_host       = macorsrtk.massdot.state.ma.us             ");
  blk("  ntrip_port       = 10000                                     ");
  blk("  ntrip_mountpoint = RTCM3MSM_IMAX                             ");
  blk("  ntrip_user       = myuser                                    ");
  blk("  ntrip_password   = mypass                                    ");
  blk("                                                                ");
  blk("  // GGA position for VRS casters                               ");
  blk("  send_gga   = true                                            ");
  blk("  gga_lat    = 42.358431    // or uses LatOrigin from .moos    ");
  blk("  gga_lon    = -71.093124   // or uses LongOrigin from .moos   ");
  blk("  gga_alt    = 10.0                                            ");
  blk("                                                                ");
  blk("  // Timing                                                     ");
  blk("  gga_interval       = 10   // seconds between GGA sends       ");
  blk("  reconnect_interval = 5    // seconds between reconnects      ");
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
  blu("uRTCMBroker INTERFACE                                          ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  None.                                                         ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  RTCM_DATA   = (binary) Raw RTCM3 correction data             ");
  blk("  RTCM_STATUS = connected=true,bytes_rx=12345,...               ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("uRTCMBroker", "gpl");
  exit(0);
}
