/*************************************************************
 *  gen_golden_vectors -- emit ArduRover skid-mixer reference
 *  vectors as CSV, for use as the lib_bb_command test fixture.
 *
 *  WHY THIS EXISTS
 *  ---------------
 *  Claiming "ArduPilot parity" is worthless if the reference is
 *  our own port. This file is a TRANSCRIPTION of the real
 *  ArduPilot function, kept as close to the original text as a
 *  standalone build allows, so the transcription itself can be
 *  diffed against upstream by eye.
 *
 *  UPSTREAM SOURCE
 *    repo:  ArduPilot/ardupilot
 *    file:  libraries/AR_Motors/AP_MotorsUGV.cpp
 *    func:  AP_MotorsUGV::output_skid_steering()
 *    local: /Users/jerwenger/dev/ardupilot @ 09c814147c
 *
 *  Helpers transcribed from libraries/AP_Math:
 *    linear_interpolate(), is_negative(), is_positive(),
 *    constrain_float()
 *
 *  EDITS MADE, and only these:
 *    - SRV_Channels::set_output_* replaced by returning the two
 *      motor values (we want the mix, not the servo layer).
 *    - The limit.* struct became four local bools.
 *    - The !armed early-out is dropped; a disarmed mixer emits no
 *      mix at all, so there is nothing to tabulate.
 *    - float arithmetic is PRESERVED. ArduPilot runs float32 and
 *      the fixture must capture float32 behaviour, including its
 *      rounding. lib_bb_command computes in double and the tests
 *      allow a tolerance for exactly this reason.
 *
 *  REGENERATE
 *    c++ -std=c++14 -O2 -o gen_golden gen_golden_vectors.cpp
 *    ./gen_golden > ../golden/ardurover_skid_golden.csv
 *
 *  Author: Jeremy Wenger
 *************************************************************/

#include <cstdio>
#include <cmath>
#include <cfloat>

// ---- AP_Math transcriptions -------------------------------

static float constrain_float(float amt, float low, float high)
{
  if (std::isnan(amt)) return (low + high) * 0.5f;
  if (amt < low)  return low;
  if (amt > high) return high;
  return amt;
}

static bool is_negative(float v) { return v <= (-1.0f * FLT_EPSILON); }
static bool is_positive(float v) { return v >= ( 1.0f * FLT_EPSILON); }

static float linear_interpolate(float output_low, float output_high,
                                float input_value,
                                float input_low, float input_high)
{
  if (input_low > input_high) {
    float t;
    t = input_low;  input_low  = input_high;  input_high  = t;
    t = output_low; output_low = output_high; output_high = t;
  }
  if (input_value <= input_low)  return output_low;
  if (input_value >= input_high) return output_high;
  float p = (input_value - input_low) / (input_high - input_low);
  return output_low + p * (output_high - output_low);
}

#define MAX(a,b) ((a)>(b)?(a):(b))

// ---- output_skid_steering(), transcribed -------------------

struct MixOut {
  float motor_left, motor_right;
  bool steer_left, steer_right, throttle_lower, throttle_upper;
};

static MixOut output_skid_steering(float steering, float throttle,
                                   float thrust_asymmetry_param,
                                   float steering_throttle_mix_param)
{
  MixOut out;
  out.steer_left = out.steer_right = false;
  out.throttle_lower = out.throttle_upper = false;

  // set_limits_from_input(), armed == true
  const float throttle_max = 100.0f;
  out.steer_left     |= (steering <= -4500.0f);
  out.steer_right    |= (steering >=  4500.0f);
  out.throttle_lower |= (throttle <= -throttle_max);
  out.throttle_upper |= (throttle >=  throttle_max);

  steering = constrain_float(steering, -4500.0f, 4500.0f);

  float steering_scaled = steering / 4500.0f;
  float throttle_scaled = throttle * 0.01f;

  const float thrust_asymmetry = MAX(thrust_asymmetry_param, 1.0);
  const float lower_throttle_limit = -1.0 / thrust_asymmetry;

  const float best_steering_throttle = (1.0 + lower_throttle_limit) * 0.5;
  float steering_range;
  if (throttle_scaled < best_steering_throttle) {
    steering_range = MAX(throttle_scaled, 0.0) - lower_throttle_limit;
  } else {
    steering_range = 1 - best_steering_throttle;
  }

  if (steering_scaled > steering_range) {
    out.steer_right = true;
    steering_scaled = steering_range;
  } else if (steering_scaled < -steering_range) {
    out.steer_left = true;
    steering_scaled = -steering_range;
  }
  if (throttle_scaled > 1.0) {
    out.throttle_upper = true;
    throttle_scaled = 1.0;
  } else if (throttle_scaled < lower_throttle_limit) {
    out.throttle_lower = true;
    throttle_scaled = lower_throttle_limit;
  }

  const float max_output = throttle_scaled + fabsf(steering_scaled);
  const float min_output = throttle_scaled - fabsf(steering_scaled);

  const float saturation_value = MAX(max_output, min_output / lower_throttle_limit);
  if (saturation_value > 1.0f) {
    const float steering_scaled_orig = steering_scaled;
    const float throttle_scaled_orig = throttle_scaled;

    const float str_thr_mix = constrain_float(steering_throttle_mix_param, 0.0f, 1.0f);
    const float fair_scaler = 1.0f / saturation_value;
    if (str_thr_mix >= 0.5f) {
      steering_scaled *= linear_interpolate(fair_scaler, 1.0f, str_thr_mix, 0.5f, 1.0f);
      if (throttle_scaled >= best_steering_throttle) {
        throttle_scaled = 1.0 - fabsf(steering_scaled);
      } else {
        throttle_scaled = fabsf(steering_scaled) + lower_throttle_limit;
      }
    } else {
      throttle_scaled *= linear_interpolate(fair_scaler, 1.0f, 0.5f - str_thr_mix, 0.0f, 0.5f);
      const float steering_sign = is_positive(steering_scaled) ? 1.0 : -1.0;
      if (throttle_scaled >= best_steering_throttle) {
        steering_scaled = (1.0 - throttle_scaled) * steering_sign;
      } else {
        steering_scaled = (throttle_scaled - lower_throttle_limit) * steering_sign;
      }
    }

    if (fabsf(steering_scaled) < fabsf(steering_scaled_orig)) {
      out.steer_left  |= is_negative(steering_scaled_orig);
      out.steer_right |= is_positive(steering_scaled_orig);
    }
    if (fabsf(throttle_scaled) < fabsf(throttle_scaled_orig)) {
      out.throttle_lower |= is_negative(throttle_scaled_orig);
      out.throttle_upper |= is_positive(throttle_scaled_orig);
    }
  }

  float motor_left  = throttle_scaled + steering_scaled;
  float motor_right = throttle_scaled - steering_scaled;

  if (is_negative(motor_right)) motor_right *= thrust_asymmetry;
  if (is_negative(motor_left))  motor_left  *= thrust_asymmetry;

  out.motor_left  = 100.0f * motor_left;
  out.motor_right = 100.0f * motor_right;
  return out;
}

// ---- fixture generation ------------------------------------

int main()
{
  // Parameter sweep: the BlueBoat values plus the branch
  // boundaries. str_thr_mix must cross 0.5 in both directions --
  // that is the branch the design doc calls out as the one most
  // likely to be "simplified" wrongly.
  const float asym[]  = {1.0f, 1.6f, 2.5f};
  const float mix[]   = {0.0f, 0.3f, 0.5f, 0.6f, 1.0f};

  printf("surge_pct,yaw_pct,thrust_asymmetry,steering_throttle_mix,"
         "left_effort,right_effort,"
         "limit_steer_left,limit_steer_right,"
         "limit_throttle_lower,limit_throttle_upper\n");

  for (float A : asym) {
    for (float m : mix) {
      // Coarse grid over the full square, plus a fine pass near
      // the interesting boundaries: zero crossings, t == b
      // (0.1875 at A=1.6), and the q == 1 saturation onset.
      for (int si = -110; si <= 110; si += 5) {
        for (int yi = -110; yi <= 110; yi += 5) {
          const float surge = (float)si;
          const float yaw   = (float)yi;
          // ArduPilot's steering domain is +/-4500, ours is
          // +/-100; convert on the way in so the transcription
          // stays byte-identical to upstream.
          MixOut o = output_skid_steering(yaw * 45.0f, surge, A, m);
          printf("%.6f,%.6f,%.6f,%.6f,%.9g,%.9g,%d,%d,%d,%d\n",
                 surge, yaw, A, m,
                 o.motor_left, o.motor_right,
                 o.steer_left ? 1 : 0, o.steer_right ? 1 : 0,
                 o.throttle_lower ? 1 : 0, o.throttle_upper ? 1 : 0);
        }
      }
      // Fine sweep around the throttle sweet spot and the
      // steering-range knee.
      for (int si = -40; si <= 40; si += 1) {
        for (int yi = -100; yi <= 100; yi += 10) {
          const float surge = si * 1.0f;
          const float yaw   = (float)yi;
          MixOut o = output_skid_steering(yaw * 45.0f, surge, A, m);
          printf("%.6f,%.6f,%.6f,%.6f,%.9g,%.9g,%d,%d,%d,%d\n",
                 surge, yaw, A, m,
                 o.motor_left, o.motor_right,
                 o.steer_left ? 1 : 0, o.steer_right ? 1 : 0,
                 o.throttle_lower ? 1 : 0, o.throttle_upper ? 1 : 0);
        }
      }
    }
  }
  return 0;
}
