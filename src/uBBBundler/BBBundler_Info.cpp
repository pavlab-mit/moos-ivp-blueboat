/*************************************************************
      Name: 
      Orgn: MIT, Cambridge MA
      File: uBBBundler/BBBundler_Info.cpp
   Last Ed:  2026-03-20
     Brief:
        Help, example config, and interface text for uBBBundler.
*************************************************************/

#include <cstdlib>
#include <iostream>
#include "BBBundler_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:");
  blk("------------------------------------");
  blk("  uBBBundler fuses/normalizes broker-fed nav values for helm.");
  blk("  It performs geodesy conversion from lat/lon to local X/Y and");
  blk("  forwards configured source variables to helm-facing names.");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: uBBBundler file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch uBBBundler with the given process name         ");
  blk("      rather than uBBBundler.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of uBBBundler.        ");
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
  blu("uBBBundler Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = uBBBundler");
  blk("{");
  blk("  AppTick   = 10");
  blk("  CommsTick = 10");
  blk("                                                                ");
  blk("  // Lat/lon inputs (defaults shown).");
  blk("  lat_var = NAV_LAT_DGNSS");
  blk("  lon_var = NAV_LONG_DGNSS");
  blk("                                                                ");
  blk("  // Geodesic-conversion output bases (suffixed by pub_suffix).");
  blk("  nav_x_out_base = NAV_X");
  blk("  nav_y_out_base = NAV_Y");
  blk("                                                                ");
  blk("  // Forwarded vars: <source>:<dest>,...  Destinations are");
  blk("  // suffixed by pub_suffix at publish time.");
  blk("  forward_map = GPS_HEADING_DGNSS:NAV_HEADING,NAV_SPEED_DGNSS:NAV_SPEED");
  blk("                                                                ");
  blk("  // Output suffix - empty publishes bare helm-facing names.");
  blk("  pub_suffix =");
  blk("}");
  blk("                                                                ");
  exit(0);
}


//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("uBBBundler INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:");
  blk("------------------------------------");
  blk("  Whatever is configured as lat_var, lon_var, plus every");
  blk("  source key appearing on the forward_map list.");
  blk("");
  blk("PUBLICATIONS (suffixed by pub_suffix when set):");
  blk("------------------------------------");
  blk("  nav_x_out_base, nav_y_out_base (default NAV_X / NAV_Y),");
  blk("  and every forward_map destination.");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("uBBBundler", "gpl");
  exit(0);
}

