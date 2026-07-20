/*************************************************************
      Name:
      Orgn: MIT, Cambridge MA
      File: pBB_Status/BB_Status_Info.cpp
   Last Ed: 2026-06-03
     Brief: Help, example config, and interface text for
            pBB_Status.
*************************************************************/

#include <cstdlib>
#include "BB_Status_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pBB_Status runs in the FRONT-SEAT MOOS community. It          ");
  blk("  subscribes to native front-seat hardware, RC, and nav         ");
  blk("  variables plus a few autonomy variables brokered from the     ");
  blk("  backseat, and publishes ONE consolidated BB_STATUS string.    ");
  blk("  Each post is written to MOOSDB and, if tx_ip is set, pushed    ");
  blk("  as a UDP datagram to the shoreside collector. It is a          ");
  blk("  read-only aggregator and commands nothing.                    ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pBB_Status file.moos [OPTIONS]                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pBB_Status with the given process name.            ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pBB_Status.                ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pBB_Status Example MOOS Configuration                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pBB_Status                                      ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  status_var         = BB_STATUS   // var to publish            ");
  blk("  publish_interval   = 0.5         // sec between posts (2 Hz)  ");
  blk("  stale_time         = 3.0         // sec before input = stale  ");
  blk("  low_voltage_thresh = 22.0        // V, drives batt=LOW        ");
  blk("  tx_ip              = 10.1.0.10   // collector IP or hostname;  ");
  blk("                                   // empty = MOOS only          ");
  blk("  tx_port            = 9300        // collector UDP port        ");
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
  blu("pBB_Status INTERFACE                                            ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS (native front seat):                              ");
  blk("------------------------------------                            ");
  blk("  NVGR_ROLLING_VOLTAGE, NVGR_ROLLING_CURRENT, NVGR_ROLLING_POWER");
  blk("  RPI_TEMP, NVGTR_IT_C, NVGTR_IP_KPA                            ");
  blk("  RC_CONNECTED, RC_FAILSAFE, NVGR_RC_DEADMAN_ACTIVE, RC_CH6     ");
  blk("  NVGR_THRUST_LEFT, NVGR_THRUST_RIGHT, NVGR_THRUST_TIMEOUT      ");
  blk("  NAV_LAT_DGNSS, NAV_LONG_DGNSS, NAV_SPEED_DGNSS,               ");
  blk("  GPS_HEADING_DGNSS, FIX_STATE_DGNSS                            ");
  blk("                                                                ");
  blk("SUBSCRIPTIONS (brokered from backseat):                         ");
  blk("------------------------------------                            ");
  blk("  DESIRED_THRUST_L, DESIRED_THRUST_R, ALL_STOP  (cross today)   ");
  blk("  IVPHELM_STATE, DEPLOY, MODE  (require widening                ");
  blk("    iBackSeatBroker.tx_vars; reported as stale until then)      ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  BB_STATUS (or status_var) -- one comma-separated key=value    ");
  blk("    string: vname,utc,mode,mission,helm,deploy,volt,curr,power, ");
  blk("    batt,rc,failsafe,deadman,des_l,des_r,thr_l,thr_r,allstop,    ");
  blk("    rpi_t,int_t,int_kpa,fix,sats,hdop,lat,lon,spd,hdg,stale      ");
  blk("                                                                ");
  blk("UDP PUSH (when tx_ip set):                                      ");
  blk("------------------------------------                            ");
  blk("  The same BB_STATUS string is sent verbatim as a UDP datagram  ");
  blk("  to tx_ip:tx_port (no envelope). Front seat -> collector is a   ");
  blk("  direct 10.1.0.0/24 unicast; the source IP identifies the boat.");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pBB_Status", "gpl");
  exit(0);
}
