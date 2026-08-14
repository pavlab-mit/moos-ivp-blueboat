/*************************************************************
      Name: Jeremy Wenger
      Orgn: MIT, Cambridge MA
      File: iRCInterface/RCInterface_Info.cpp
   Last Ed: 2026-08-11
     Brief:
        Help, example config, and interface text for
        iRCInterface.
*************************************************************/

#include <cstdlib>
#include <iostream>
#include "RCInterface_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  iRCInterface owns the CRSF RC serial link in both directions. ");
  blk("  It decodes RC channels and link statistics from an ExpressLRS ");
  blk("  receiver and publishes RC_CH1..16 plus connection state per   ");
  blk("  RC Contract v2 (docs/rc_handset_architecture.md), and sends   ");
  blk("  battery / GPS / flight-mode / distance telemetry back to the  ");
  blk("  handset on the same port. Telemetry writes are non-blocking   ");
  blk("  and can never stall the RC decode path.                       ");
  blk("                                                                ");
  blk("  Supersedes iRCReader (SBUS), which remains available as the   ");
  blk("  transitional fallback. Design: docs/rc_crsf_design.md         ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: iRCInterface file.moos [OPTIONS]                         ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch iRCInterface with the given process name           ");
  blk("      rather than iRCInterface.                                 ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of iRCInterface.              ");
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
  blu("iRCInterface Example MOOS Configuration                         ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = iRCInterface                                    ");
  blk("{                                                               ");
  blk("  AppTick   = 16                                                ");
  blk("  CommsTick = 16                                                ");
  blk("                                                                ");
  blk("  // UART wired to the CRSF receiver. 420k 8N1, no parity      ");
  blk("  // constraint: any Pi UART works (no disable-bt needed).     ");
  blk("  //   RadioMaster XR4: /dev/ttyAMA1 (Navigator SERIAL 3)      ");
  blk("  // Verify with: rc_probe -p crsf -d <dev>                    ");
  blk("  device = /dev/ttyAMA1     // default                          ");
  blk("                                                                ");
  blk("  // Telemetry back to the handset (battery, flight mode,      ");
  blk("  // GPS, distance). Writes are non-blocking; disabling only   ");
  blk("  // silences the return path, RC decode is unaffected.        ");
  blk("  telemetry = true          // default                          ");
  blk("  telem_hz  = 2             // battery + flight-mode rate      ");
  blk("  gps_hz    = 1             // GPS + distance rate, 0 = off    ");
  blk("                                                                ");
  blk("  debug  = false                                                ");
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
  blu("iRCInterface INTERFACE                                          ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS: (telemetry sources only)                         ");
  blk("------------------------------------                            ");
  blk("  NVGR_ROLLING_VOLTAGE   = battery voltage -> 0x08 frame        ");
  blk("  NVGR_ROLLING_CURRENT   = battery current -> 0x08 frame        ");
  blk("  NAV_LAT / NAV_LONG     = position        -> 0x02 GPS frame    ");
  blk("  NAV_SPEED / NAV_HEADING= speed, heading  -> 0x02 GPS frame    ");
  blk("  NAV_X / NAV_Y          = local position  -> 0x80 distance     ");
  blk("  NVGR_RC_MODE           = \"true\"/\"false\" -> FM string      ");
  blk("  NVGR_RC_DEADMAN_ACTIVE = \"true\"/\"false\" -> FM string      ");
  blk("                                                                ");
  blk("PUBLICATIONS: (channel map = RC Contract v2, frozen 2026-08-11) ");
  blk("------------------------------------                            ");
  blk("  RC_CH1..RC_CH4  = axes scaled to -100..100                    ");
  blk("                    CH1 STEER, CH2 aux, CH3 THRUST, CH4 aux     ");
  blk("  RC_CH5          = KILL         1=RUNNING 2=KILLED             ");
  blk("  RC_CH6          = MODE         1=AUTO    2=RC                 ");
  blk("  RC_CH7          = ACTION       1=HOLD 2=RESUME 3=RETURN       ");
  blk("  RC_CH8          = DEADMAN_EN   1=ENABLED 2=DISABLED           ");
  blk("  RC_CH9          = PAYLOAD_PWR  1=OFF     2=ON                 ");
  blk("  RC_CH10         = PAYLOAD_MODE 3-pos                          ");
  blk("  RC_CH11         = THRUST_LIMIT percent, floored at 25         ");
  blk("  RC_CH12         = MARK         1=idle    2=pressed            ");
  blk("  RC_MARK         = MARK press count (rising edge only, one    ");
  blk("                    increment per press - the loggable event)   ");
  blk("  RC_CH13..RC_CH16= reserved, raw 11-bit values                 ");
  blk("  RC_CH17, RC_CH18= always \"false\" (no CRSF equivalent)       ");
  blk("  RC_FRAME_VALID  = per-frame validity (safety gate)            ");
  blk("  RC_CONNECTED    = debounced link state (mode/UI)              ");
  blk("  RC_FRAME_LOST   = frames stopped arriving                     ");
  blk("  RC_FAILSAFE     = receiver declares dead uplink (LQ 0)        ");
  blk("  RC_LINK_LQ      = uplink link quality, percent                ");
  blk("  RC_LINK_RSSI    = uplink RSSI, dBm (negative)                 ");
  blk("  RC_LINK_SNR     = uplink SNR, dB                              ");
  blk("  RC_PROTOCOL     = \"crsf\" (once, for log forensics)          ");
  blk("                                                                ");
  blk("  Discrete channels use guard-band decode (state 1 < 600,       ");
  blk("  top state > 1400). A 2-pos dead-band value is a config        ");
  blk("  fault: safe state 1 + run warning, except KILL which          ");
  blk("  latches its last valid state.                                 ");
  blk("                                                                ");
  blk("  On disconnect, safe defaults are published for all channels   ");
  blk("  (CH1-4=0, CH5-10 and CH12=1, CH11=100, CH13-16=mid), AFTER    ");
  blk("  RC_CONNECTED=false within the same iterate.                   ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("iRCInterface", "gpl");
  exit(0);
}
