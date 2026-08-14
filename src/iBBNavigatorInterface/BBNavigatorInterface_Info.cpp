/*************************************************************
      Name: Raymond Turrisi (orig.), Jeremy Wenger (navigator-cpp port)
      Orgn: MIT, Cambridge MA
      File: iBBNavigatorInterface/BBNavigatorInterface_Info.cpp
   Last Ed:  2026-07-24
     Brief:
        Unified Navigator Interface for the BlueBoat ASV
        (navigator-cpp; runtime V1/V2 + Pi4/Pi5 detection).
*************************************************************/

#include <cstdlib>
#include <iostream>
#include "BBNavigatorInterface_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The iBBNavigatorInterface application provides combined       ");
  blk("  thrust control and AHRS sensing for the Blueboat ASV. Built   ");
  blk("  on navigator-cpp: the Navigator board revision (V1/V2) and    ");
  blk("  Raspberry Pi model (4/5) are auto-detected at runtime - one   ");
  blk("  binary covers the whole fleet.                                ");
  blk("                                                                ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: iBBNavigatorInterface file.moos [OPTIONS]                ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch iBBNavigatorInterface with the given process name  ");
  blk("      rather than iBBNavigatorInterface.                        ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of iBBNavigatorInterface.     ");
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
  blu("iBBNavigatorInterface Example MOOS Configuration                ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = iBBNavigatorInterface                           ");
  blk("{                                                               ");
  blk("  AppTick   = 10                                                ");
  blk("  CommsTick = 10                                                ");
  blk("                                                                ");
  blk("  // Thrust control                                             ");
  blk("  left_thruster_pin = 14                                        ");
  blk("  right_thruster_pin = 16                                       ");
  blk("  max_thrust = 100                                              ");
  blk("  min_thrust = -100                                             ");
  blk("  thruster_dead_band = 5                                        ");
  blk("  // Pulse range: default 800-2200 (legacy). Basic ESC 500      ");
  blk("  // documents 1100-1900; switching changes the thrust curve.   ");
  blk("  // pwm_min_us = 1100                                          ");
  blk("  // pwm_max_us = 1900                                          ");
  blk("                                                                ");
  blk("  // ESC lifecycle                                              ");
  blk("  initialize_esc   = true      // neutral-hold arm every launch ");
  blk("  disarm_on_exit   = false     // true: cut PWM signal on exit  ");
  blk("  rc_thrust_limit_enable = false // CH11 pot caps RC+teleop     ");
  blk("  rc_stick_convention = v1     // v1 legacy SBUS | v2 CRSF wire ");
  blk("                                                                ");
  blk("  // AHRS config (Allgeuer passive complementary filter)        ");
  blk("  sample_rate = 150                                             ");
  blk("  // ahrs_kp = 2.20   ahrs_ti = 2.65   (library defaults)       ");
  blk("  use_mag = false                                               ");
  blk("  mag_ak_cal_file = /path/to/mag_cal.txt                        ");
  blk("  imu_cal_file    = /path/to/imu_cal.txt                        ");
  blk("  declination_deg = -14.058                                     ");
  blk("  yaw_rate_clamp  = 3.0        // rad/s, GYRO_Z_LVL envelope    ");
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
  blu("iBBNavigatorInterface INTERFACE                                 ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  DESIRED_THRUST_L, DESIRED_THRUST_R - Thrust commands          ");
  blk("  ALL_STOP, MISSION_COMPLETE - Control signals                  ");
  blk("  NVGR_DISARM - true: cut PWM signal (ESCs stop on signal loss) ");
  blk("                false: re-enable + neutral-hold re-arm          ");
  blk("  RC_CONNECTED, RC_FRAME_VALID, RC_CH1-16 - RC controller input ");
  blk("    CH5 KILL (2=neutral-lock, latched while connected)          ");
  blk("    CH6 MODE (2=RC drive), CH8 DEADMAN_EN (edge-triggered       ");
  blk("    toggle of the watchdog), CH11 THRUST_LIMIT (manual cap %)   ");
  blk("  RC_DEADMAN_ENABLED - Runtime override for RC deadman watchdog ");
  blk("  TELEOP_ACTIVE, TELEOP_THRUST_L/R - Laptop teleop (iTeleop)    ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  AHRS group (suffix=<ahrs_pub_suffix>, default AHRS):           ");
  blk("    NAV_ROLL, NAV_PITCH, NAV_YAW, NAV_HEADING                    ");
  blk("  IMU group  (suffix=<imu_pub_suffix>,  default IMU):            ");
  blk("    GYRO_X, GYRO_Y, GYRO_Z, GYRO_Z_LVL                           ");
  blk("    (GYRO_Z_LVL is the world-frame z angular rate - bounded,     ");
  blk("     no cos(pitch) singularity - clamped to yaw_rate_clamp)      ");
  blk("  IMU_STATE - bundled coherent snapshot at a single timestamp    ");
  blk("  Power/health (unsuffixed):                                     ");
  blk("    NVGR_VOLTAGE, NVGR_CURRENT, NVGR_ROLLING_POWER               ");
  blk("    NVGR_THRUST_LEFT, NVGR_THRUST_RIGHT (+_WIRE variants)        ");
  blk("    NVGR_THRUST_TIMEOUT - autonomous-mode command timeout        ");
  blk("    NVGR_RC_DEADMAN_ACTIVE - RC deadman state                    ");
  blk("    NVGR_RC_KILL - RC KILL neutral-lock engaged (CH5)            ");
  blk("    NVGR_ESC_ARMED - PWM output enable state                     ");
  blk("    NVGR_LEAK - leak detector state                              ");
  blk("    NVGR_HW_VERSION - detected Navigator/Pi hardware             ");
  blk("    NVGTR_IT_C, NVGTR_IP_KPA, RPI_TEMP - temps/pressure          ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("iBBNavigatorInterface", "gpl");
  exit(0);
}
