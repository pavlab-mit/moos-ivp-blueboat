/****************************************************************/
/*   NAME: Raymond Turrisi (orig.), Jeremy Wenger                */
/*   ORGN: MIT, Cambridge MA                                     */
/*   FILE: BBNavigatorInterface_Info.cpp                         */
/*   DATE: 2026-08-27 (command-pipeline rewrite)                 */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "BBNavigatorInterface_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  Navigator board interface for the BlueBoat. Two jobs:          ");
  blk("                                                                 ");
  blk("    BB_MIXED_CMD -> ESC pulses     final safety, ESC mapping,    ");
  blk("                                   PWM writer + watchdog, ESC    ");
  blk("                                   arm/disarm lifecycle          ");
  blk("                                                                 ");
  blk("    sensors                        AHRS, raw gyro / level-frame  ");
  blk("                                   yaw rate, ADC power, baro,    ");
  blk("                                   leak, NeoPixels               ");
  blk("                                                                 ");
  blk("  It is the ONLY process allowed to write the thruster PWM       ");
  blk("  interface, and the ONLY place electrical inversion happens.    ");
  blk("  Motor effort is physical and forward-positive everywhere       ");
  blk("  upstream.                                                      ");
  blk("                                                                 ");
  blk("  It does NOT arbitrate (pBBCommandArbiter), does NOT mix        ");
  blk("  (pThrustMix), does not filter, and applies no deadband.        ");
}

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
  mag("  --example, -e   ");  blk("  Show an example configuration.    ");
  mag("  --help, -h      ");  blk("  Show this message.                ");
  mag("  --interface, -i ");  blk("  Show publications/subscriptions.  ");
  mag("  --version, -v   ");  blk("  Show release version.             ");
  blk("                                                                ");
  exit(0);
}

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("iBBNavigatorInterface Example MOOS Configuration                ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = iBBNavigatorInterface                           ");
  blk("{                                                               ");
  blk("  AppTick   = 50                                                ");
  blk("  CommsTick = 50                                                ");
  blk("                                                                ");
  blk("  //=========================================================   ");
  blk("  // COMMAND PATH                                               ");
  blk("  //=========================================================   ");
  blk("  // Lease on the mixer. Start lenient; tighten once the        ");
  blk("  // measured NVGR_MIX_INPUT_AGE distribution justifies it.     ");
  blk("  mixed_cmd_timeout_sec  = 0.5                                  ");
  blk("                                                                ");
  blk("  // How long a committed actuator frame stays valid to the     ");
  blk("  // PWM thread. Past this the writer neutralises on its own,   ");
  blk("  // so a stalled or dead Iterate() cannot leave the props      ");
  blk("  // running on an old command.                                 ");
  blk("  actuator_frame_ttl_sec = 0.25                                 ");
  blk("                                                                ");
  blk("  //=========================================================   ");
  blk("  // ESC ENDPOINTS -- the only place inversion is allowed       ");
  blk("  //=========================================================   ");
  blk("  // BlueBoat120, from the ArduRover 4.7 parameter file.        ");
  blk("  // NOTE trim is 1510, not 1500: the 10 us is real and using   ");
  blk("  // 1500 is a permanent idle bias.                             ");
  blk("  // right = SERVO1 (not reversed), left = SERVO3 (reversed).   ");
  blk("  left_esc_min_us   = 1100                                      ");
  blk("  left_esc_trim_us  = 1510                                      ");
  blk("  left_esc_max_us   = 1900                                      ");
  blk("  left_esc_reversed = true                                      ");
  blk("  right_esc_min_us   = 1100                                     ");
  blk("  right_esc_trim_us  = 1510                                     ");
  blk("  right_esc_max_us   = 1900                                     ");
  blk("  right_esc_reversed = false                                    ");
  blk("                                                                ");
  blk("  left_thruster_pin  = 14      // 1-indexed board channel       ");
  blk("  right_thruster_pin = 16                                       ");
  blk("  thruster_enabled   = true                                     ");
  blk("                                                                ");
  blk("  //=========================================================   ");
  blk("  // ESC LIFECYCLE                                              ");
  blk("  //=========================================================   ");
  blk("  // Arm on launch with the documented 2 s neutral hold. The    ");
  blk("  // hold runs IN the PWM thread, writing neutral every cycle,  ");
  blk("  // because an ESC arms on a signal it can see.                ");
  blk("  initialize_esc = true                                         ");
  blk("                                                                ");
  blk("  // true: safe shutdown also cuts the signal (OE high). The    ");
  blk("  // fleet default is false -- pin state PERSISTS after exit,   ");
  blk("  // so the safe state is neutral-on-the-wire, not absence of   ");
  blk("  // signal.                                                     ");
  blk("  disarm_on_exit = false                                        ");
  blk("                                                                ");
  blk("  //=========================================================   ");
  blk("  // RC DEADMAN -- opt-in blanket, default OFF                  ");
  blk("  //=========================================================   ");
  blk("  // If enabled, loss of the RC link safes the boat REGARDLESS  ");
  blk("  // of mode -- including a healthy autonomous mission. It is   ");
  blk("  // for running experimental code, which is why it is enforced ");
  blk("  // here rather than in the arbiter: routing it through the    ");
  blk("  // code under test would defeat it.                           ");
  blk("  //                                                             ");
  blk("  // This is NOT the source lease. That lives in the arbiter    ");
  blk("  // and is never disableable.                                  ");
  blk("  rc_deadman_enabled     = false                                ");
  blk("  rc_deadman_timeout_sec = 2.0                                  ");
  blk("                                                                ");
  blk("  //=========================================================   ");
  blk("  // POWER / ENVIRONMENT                                        ");
  blk("  //=========================================================   ");
  blk("  nbats                 = 4    // only 4 is calibrated          ");
  blk("  voltage_scale         = 11.132                                ");
  blk("  voltage_offset        = 0.0                                   ");
  blk("  rolling_window_period = 2.0                                   ");
  blk("                                                                ");
  blk("  //=========================================================   ");
  blk("  // AHRS                                                       ");
  blk("  //=========================================================   ");
  blk("  sample_rate = 150     // Hz; NVGR_SENSOR_RATE_HZ reports the  ");
  blk("                        // rate ACHIEVED, which is the number   ");
  blk("                        // to believe                            ");
  blk("  use_mag     = false   // off pending on-water characterisation");
  blk("  // ahrs_kp = 2.20 / ahrs_ti = 2.65 (library defaults if unset)");
  blk("  mag_ak_cal_file = /home/pi/mag_cal/$(VNAME)/mag/mag_cal_nav.dat");
  blk("  // imu_cal_file = /home/pi/mag_cal/$(VNAME)/imu/imu_cal.txt   ");
  blk("  yaw_rate_clamp  = 3.0                                         ");
  blk("  roll_offset = 0.0 / pitch_offset = 0.0 / yaw_offset = 0.0     ");
  blk("  declination_deg = -14.058                                     ");
  blk("  ahrs_pub_suffix = AHRS                                        ");
  blk("  imu_pub_suffix  = IMU                                         ");
  blk("}                                                               ");
  blk("                                                                ");
  blk("RETIRED parameters -- ignored, with a config warning:           ");
  blk("  max_thrust, min_thrust, thruster_dead_band, thruster_alpha,   ");
  blk("  left_thruster_invert, right_thruster_invert, pwm_min_us,      ");
  blk("  pwm_max_us, thrust_command_timeout, rc_deadman_timeout,       ");
  blk("  rc_thrust_limit_enable, teleop_command_timeout,               ");
  blk("  rc_stick_convention, theta_b, turn_scale                      ");
  blk("                                                                ");
  exit(0);
}

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
  blk("  BB_MIXED_CMD      The ONLY command input. Physical left/right ");
  blk("                    effort, forward-positive, with the complete ");
  blk("                    source -> decision -> mix lineage.          ");
  blk("                                                                ");
  blk("  Hardware sidebands -- plain booleans, deliberately NOT fields ");
  blk("  inside a contract. Both bypass the command path so they still ");
  blk("  work when it has failed:                                      ");
  blk("    RC_KILL_ASSERTED  neutral-lock. ESCs stay ARMED, so release ");
  blk("                      recovers instantly. Distinct from disarm. ");
  blk("    RC_LINK_ALIVE     link presence, watched by the opt-in      ");
  blk("                      deadman.                                   ");
  blk("                                                                ");
  blk("  NVGR_DISARM       true  = signal cut (OE high, orderly        ");
  blk("                            channel all-off first).             ");
  blk("                    false = re-arm, mid-mission, costing the    ");
  blk("                            2 s hold. Serves the handset's      ");
  blk("                            disarm/arm request.                 ");
  blk("  MISSION_COMPLETE  requests shutdown.                          ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  NVGR_ACTUATOR_TRACE  The end of the lineage chain. Given one  ");
  blk("                       of these, a log query resolves backwards ");
  blk("                       to exactly one mixer result, one         ");
  blk("                       arbitration decision, and one stick      ");
  blk("                       movement or controller output.           ");
  blk("                                                                ");
  blk("  Applied output -- TELEMETRY, never a command input:           ");
  blk("    NVGR_APPLIED_LEFT / _RIGHT     physical effort              ");
  blk("    NVGR_PWM_LEFT_US / _RIGHT_US   electrical, post-reversal    ");
  blk("    NVGR_MIX_INPUT_AGE                                          ");
  blk("    NVGR_STOP_REASON                                            ");
  blk("    NVGR_ARM_STATE       DISARMED | ARMING | ARMED              ");
  blk("    NVGR_PWM_ARMED / NVGR_RC_KILL / NVGR_ESC_ARMED              ");
  blk("    NVGR_PWM_WATCHDOG_COUNT   frames that outlived their TTL    ");
  blk("    NVGR_MIX_INPUT_REJECT_REASON                                ");
  blk("                                                                ");
  blk("  Sensors:                                                      ");
  blk("    NAV_ROLL_<sfx> / NAV_PITCH_<sfx> / NAV_YAW_<sfx>            ");
  blk("    NAV_HEADING_<sfx>                                           ");
  blk("    GYRO_X_<sfx> / GYRO_Y_<sfx> / GYRO_Z_<sfx>                  ");
  blk("    GYRO_Z_LVL_<sfx>   world-frame yaw rate, no cos(pitch)      ");
  blk("                       singularity                              ");
  blk("    NVGR_SENSOR_RATE_HZ   ACHIEVED sampling rate                ");
  blk("    NVGR_IMU_READ_ERRORS / NVGR_IMU_GLITCHES                    ");
  blk("    NVGR_ROLLING_VOLTAGE / _CURRENT / _POWER                    ");
  blk("    NVGTR_IT_C / NVGTR_IP_KPA / NVGR_LEAK                       ");
  blk("                                                                ");
  blk("STOP REASONS (NVGR_STOP_REASON):                                ");
  blk("------------------------------------                            ");
  blk("  Local, in priority order -- these outrank anything upstream   ");
  blk("  so the log names the operator's action rather than its        ");
  blk("  downstream symptom:                                           ");
  blk("    SHUTDOWN, HARDWARE_FAULT, PWM_DISARMED, RC_KILL, RC_DEADMAN ");
  blk("  This app's own input:                                         ");
  blk("    NAV_INPUT_INVALID  never heard from the mixer               ");
  blk("    NAV_INPUT_STALE    mixer went quiet                         ");
  blk("  Otherwise the upstream reason is propagated unchanged         ");
  blk("  (RC_STALE, TELEOP_ESTOP, AUTONOMY_ALL_STOP, ...).             ");
  blk("                                                                ");
  exit(0);
}

void showReleaseInfoAndExit()
{
  showReleaseInfo("iBBNavigatorInterface", "gpl");
  exit(0);
}
