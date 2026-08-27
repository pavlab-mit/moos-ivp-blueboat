/*************************************************************
 *  skid_mixer -- see skid_mixer.h
 *
 *  PORTING NOTE. This follows the ArduPilot branch structure
 *  line for line on purpose. It is deliberately NOT simplified:
 *  the saturation law has four branches whose boundaries matter,
 *  and a "cleaner" algebraic rewrite is how parity gets lost.
 *  If you are tempted to collapse a branch, add a golden vector
 *  first and prove it.
 *
 *  Deviations from ArduPilot, all deliberate:
 *    1. double instead of float. We are not on an STM32 and the
 *       extra precision costs nothing. Golden vectors are
 *       generated in float, so tests allow a small tolerance --
 *       see test/test_skid_mixer.cpp for the argument.
 *    2. Non-finite input fails closed instead of propagating NaN.
 *       ArduPilot trusts its callers; we do not (invariant 12).
 *    3. Wheel-rate control, MOT_THR_MIN, MOT_THST_EXPO and
 *       MOT_REV_DELAY are not ported. All four are identity at
 *       the BlueBoat120 defaults; porting disabled code would be
 *       untested code.
 *************************************************************/

#include "skid_mixer.h"

#include <cmath>
#include <cfloat>

namespace bb {

namespace {

double clampd(double v, double lo, double hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ArduPilot AP_Math is_negative/is_positive: an epsilon band around
// zero, not a bare sign test. Reproduced because the asymmetry
// correction and the steering-sign branch both key off it, and a
// bare (x < 0) differs from ArduPilot for denormal-scale values.
bool ap_is_negative(double v) { return v <= -1.0 * FLT_EPSILON; }
bool ap_is_positive(double v) { return v >=  1.0 * FLT_EPSILON; }

// ArduPilot AP_Math linear_interpolate(), including its clamping
// at both ends. Only the (input_low <= input_high) case is used
// here, but the guard is kept so the helper behaves identically
// if it is reused.
double ap_linear_interpolate(double output_low, double output_high,
                             double input_value,
                             double input_low, double input_high)
{
  if (input_low > input_high) {
    double t;
    t = input_low;   input_low   = input_high;   input_high  = t;
    t = output_low;  output_low  = output_high;  output_high = t;
  }
  if (input_value <= input_low)  return output_low;
  if (input_value >= input_high) return output_high;
  const double p = (input_value - input_low) / (input_high - input_low);
  return output_low + p * (output_high - output_low);
}

AllocationResult neutral_failed_closed()
{
  AllocationResult r;
  r.left_effort = 0.0;
  r.right_effort = 0.0;
  r.limit_steer_left = r.limit_steer_right = true;
  r.limit_throttle_lower = r.limit_throttle_upper = true;
  return r;
}

} // namespace

std::string SkidMixerParams::validate() const
{
  if (!std::isfinite(thrust_asymmetry))
    return "thrust_asymmetry is not finite";
  if (thrust_asymmetry < 1.0)
    return "thrust_asymmetry must be >= 1.0 (forward thrust is never weaker than reverse)";
  if (!std::isfinite(steering_throttle_mix))
    return "steering_throttle_mix is not finite";
  if (steering_throttle_mix < 0.0 || steering_throttle_mix > 1.0)
    return "steering_throttle_mix must be within [0, 1]";
  return "";
}

AllocationResult skid_mix(double surge_pct, double yaw_pct,
                          const SkidMixerParams& params)
{
  // Invariant 12: non-finite input invalidates the whole result.
  if (!std::isfinite(surge_pct) || !std::isfinite(yaw_pct) ||
      !std::isfinite(params.thrust_asymmetry) ||
      !std::isfinite(params.steering_throttle_mix)) {
    return neutral_failed_closed();
  }

  AllocationResult r;

  // ArduPilot receives steering as [-4500, 4500] and divides by
  // 4500; we carry percent and divide by 100. Identical maths,
  // and the clamp lands in the same place.
  double steering_scaled = clampd(yaw_pct,   -100.0, 100.0) * 0.01;
  double throttle_scaled = clampd(surge_pct, -100.0, 100.0) * 0.01;

  // Set the input-driven limit flags before any shaping, matching
  // AP_MotorsUGV::set_limits_from_input().
  if (yaw_pct   <= -100.0) r.limit_steer_left     = true;
  if (yaw_pct   >=  100.0) r.limit_steer_right    = true;
  if (surge_pct <= -100.0) r.limit_throttle_lower = true;
  if (surge_pct >=  100.0) r.limit_throttle_upper = true;

  // Asymmetric feasible region. Reverse propulsion is assumed to
  // deliver 1/A of forward thrust for the same motor command, so
  // the usable pre-compensation domain is [-1/A, 1], not [-1, 1].
  const double thrust_asymmetry      = (params.thrust_asymmetry > 1.0)
                                          ? params.thrust_asymmetry : 1.0;
  const double lower_throttle_limit  = -1.0 / thrust_asymmetry;
  const double best_steering_throttle = (1.0 + lower_throttle_limit) * 0.5;

  // Steering authority available at this throttle. Below the
  // sweet spot the mixer can only pull throttle DOWN toward the
  // lower limit, so the range shrinks; above it, the full range
  // is always reachable.
  double steering_range;
  if (throttle_scaled < best_steering_throttle) {
    const double t_floor = (throttle_scaled > 0.0) ? throttle_scaled : 0.0;
    steering_range = t_floor - lower_throttle_limit;
  } else {
    steering_range = 1.0 - best_steering_throttle;
  }

  if (steering_scaled > steering_range) {
    r.limit_steer_right = true;
    steering_scaled = steering_range;
  } else if (steering_scaled < -steering_range) {
    r.limit_steer_left = true;
    steering_scaled = -steering_range;
  }
  if (throttle_scaled > 1.0) {
    r.limit_throttle_upper = true;
    throttle_scaled = 1.0;
  } else if (throttle_scaled < lower_throttle_limit) {
    r.limit_throttle_lower = true;
    throttle_scaled = lower_throttle_limit;
  }

  // Throttle alone fits and steering alone fits. Do they fit
  // together? lower_throttle_limit is negative, so dividing by it
  // flips the comparison: min_output/l exceeds 1 exactly when
  // min_output drops below the lower limit.
  const double max_output = throttle_scaled + std::fabs(steering_scaled);
  const double min_output = throttle_scaled - std::fabs(steering_scaled);
  const double saturation_value =
      (max_output > (min_output / lower_throttle_limit))
          ? max_output
          : (min_output / lower_throttle_limit);

  r.saturation_value = saturation_value;

  if (saturation_value > 1.0) {
    r.saturated = true;

    const double steering_scaled_orig = steering_scaled;
    const double throttle_scaled_orig = throttle_scaled;

    const double str_thr_mix = clampd(params.steering_throttle_mix, 0.0, 1.0);
    const double fair_scaler = 1.0 / saturation_value;
    r.fair_scaler = fair_scaler;

    if (str_thr_mix >= 0.5) {
      // Prioritise steering. At mix = 0.5 this is the fair scale;
      // at 1.0 steering is untouched and throttle absorbs all of
      // the saturation.
      steering_scaled *= ap_linear_interpolate(fair_scaler, 1.0,
                                               str_thr_mix, 0.5, 1.0);
      if (throttle_scaled >= best_steering_throttle) {
        throttle_scaled = 1.0 - std::fabs(steering_scaled);
      } else {
        throttle_scaled = std::fabs(steering_scaled) + lower_throttle_limit;
      }
    } else {
      // Prioritise throttle; steering absorbs the saturation.
      throttle_scaled *= ap_linear_interpolate(fair_scaler, 1.0,
                                               0.5 - str_thr_mix, 0.0, 0.5);
      const double steering_sign = ap_is_positive(steering_scaled) ? 1.0 : -1.0;
      if (throttle_scaled >= best_steering_throttle) {
        steering_scaled = (1.0 - throttle_scaled) * steering_sign;
      } else {
        steering_scaled = (throttle_scaled - lower_throttle_limit) * steering_sign;
      }
    }

    if (std::fabs(steering_scaled) < std::fabs(steering_scaled_orig)) {
      if (ap_is_negative(steering_scaled_orig)) r.limit_steer_left  = true;
      if (ap_is_positive(steering_scaled_orig)) r.limit_steer_right = true;
    }
    if (std::fabs(throttle_scaled) < std::fabs(throttle_scaled_orig)) {
      if (ap_is_negative(throttle_scaled_orig)) r.limit_throttle_lower = true;
      if (ap_is_positive(throttle_scaled_orig)) r.limit_throttle_upper = true;
    }
  }

  double motor_left  = throttle_scaled + steering_scaled;
  double motor_right = throttle_scaled - steering_scaled;

  // Reverse compensation. A negative effective-thrust request is
  // multiplied back up by A so the delivered force approximates
  // what was asked for, given reverse is the weaker direction.
  if (ap_is_negative(motor_right)) motor_right *= thrust_asymmetry;
  if (ap_is_negative(motor_left))  motor_left  *= thrust_asymmetry;

  r.throttle_scaled        = throttle_scaled;
  r.steering_scaled        = steering_scaled;
  r.lower_throttle_limit   = lower_throttle_limit;
  r.best_steering_throttle = best_steering_throttle;
  r.steering_range         = steering_range;
  r.left_effort            = 100.0 * motor_left;
  r.right_effort           = 100.0 * motor_right;

  return r;
}

} // namespace bb
