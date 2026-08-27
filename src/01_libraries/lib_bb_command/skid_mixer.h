/*************************************************************
 *  skid_mixer -- ArduRover 4.7 skid-steer allocation, ported.
 *
 *  Converts semantic surge/yaw into physical left/right motor
 *  effort using ArduPilot's asymmetric-thrust mixer. Pure: no
 *  MOOS, no hardware, no state, no I/O.
 *
 *  Source of truth:
 *    ArduPilot  libraries/AR_Motors/AP_MotorsUGV.cpp
 *               AP_MotorsUGV::output_skid_steering()
 *
 *  This is NOT left = surge + yaw / right = surge - yaw followed
 *  by normalisation. It uses an asymmetric feasible region
 *  derived from MOT_THST_ASYM, a tunable steering-vs-throttle
 *  saturation priority (MOT_STR_THR_MIX), and a post-mix reverse
 *  compensation. See docs/ibb_navigator_command_pipeline.md §9.
 *
 *  Sign convention (docs/ibb_navigator_command_pipeline.md §3):
 *    surge > 0  = forward
 *    yaw   > 0  = starboard / clockwise
 *    effort > 0 = physical FORWARD thrust on that side
 *  Electrical inversion belongs in esc_mapper, never here.
 *
 *  Author: Jeremy Wenger
 *************************************************************/

#ifndef BB_SKID_MIXER_HEADER
#define BB_SKID_MIXER_HEADER

#include <string>

namespace bb {

struct SkidMixerParams
{
  // MOT_THST_ASYM: forward thrust is this many times stronger than
  // reverse thrust at equal motor command. Values below 1 are
  // clamped up to 1 by the mixer, matching ArduPilot.
  double thrust_asymmetry = 1.6;

  // MOT_STR_THR_MIX: saturation priority.
  //   0.0 = preserve throttle, sacrifice steering
  //   0.5 = scale both proportionally (fair)
  //   1.0 = preserve steering, sacrifice throttle
  double steering_throttle_mix = 0.6;

  // Reject nonsense at config time rather than mid-mission.
  // Returns "" if valid, else a human-readable reason.
  std::string validate() const;
};

struct AllocationResult
{
  // Primary output: physical motor effort, percent, forward-positive.
  double left_effort  = 0.0;
  double right_effort = 0.0;

  // Everything below is trace detail. It exists so a log can
  // explain WHY a given pair came out, without re-running the
  // mixer. Named to match the ArduPilot locals where possible.
  double throttle_scaled        = 0.0;  // t, after clamp + saturation
  double steering_scaled        = 0.0;  // s, after clamp + saturation
  double lower_throttle_limit   = 0.0;  // l = -1/A
  double best_steering_throttle = 0.0;  // b = (1+l)/2
  double steering_range         = 0.0;
  double saturation_value       = 0.0;  // q
  double fair_scaler            = 0.0;  // f = 1/q, 0 when unsaturated
  bool   saturated              = false;

  // ArduPilot's limit flags, reproduced.
  bool limit_steer_left     = false;
  bool limit_steer_right    = false;
  bool limit_throttle_lower = false;
  bool limit_throttle_upper = false;
};

// Allocate. surge_pct and yaw_pct are percent in [-100, 100];
// out-of-range inputs are clamped exactly as ArduPilot clamps
// them, not rejected. Non-finite input yields a neutral result
// with all limit flags set -- fail closed (invariant 12).
AllocationResult skid_mix(double surge_pct, double yaw_pct,
                          const SkidMixerParams& params);

} // namespace bb

#endif
