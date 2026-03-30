/*************************************************************
      Name: 
      Orgn: MIT, Cambridge MA
      File: uBBBundler/BBBundler_Info.cpp
   Last Ed:  2026-03-20
     Brief:
        Lorem ipsum dolor sit amet, consectetur adipiscing 
        elit, sed do eiusmod tempor incididunt ut labore et 
        dolore magna aliqua. Ut enim ad minim veniam, quis 
        nostrud exercitation ullamco laboris nisi ut aliquip 
        ex ea commodo consequat.
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
  blk("  lat_var       = NAV_LAT");
  blk("  lon_var       = NAV_LONG");
  blk("  nav_x_out_var = NAV_X");
  blk("  nav_y_out_var = NAV_Y");
  blk("  forward_map   = GPS_HEADING:NAV_HEADING,NAV_SPEED:NAV_SPEED");
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
  blk("  lat_var, lon_var, and keys appearing on forward_map.");
  blk("");
  blk("PUBLICATIONS:");
  blk("------------------------------------");
  blk("  nav_x_out_var/nav_y_out_var,");
  blk("  and all forward_map destination variables.");
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

