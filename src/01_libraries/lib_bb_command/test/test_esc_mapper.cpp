/*************************************************************
 * Unit tests for esc_mapper.
 *
 * The endpoints under test are the ones this refactor adopts:
 * 1100 / 1510 / 1900 us, replacing the previous 800 / 1500 /
 * 2200. That old mapping put every command above |57.1| outside
 * the Basic ESC 500's documented input range -- measured at 8.0%
 * of all single-side commands in the via 5 Aug log. These tests
 * pin the new mapping so that cannot come back.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "esc_mapper.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace bb;

static int g_failures = 0;
static int g_checks   = 0;

static void check(bool cond, const std::string& what)
{
  g_checks++;
  if (!cond) { g_failures++; fprintf(stderr, "FAIL: %s\n", what.c_str()); }
}

static void check_near(double got, double want, double tol, const std::string& what)
{
  g_checks++;
  if (std::fabs(got - want) > tol) {
    g_failures++;
    fprintf(stderr, "FAIL: %s (got %.6f want %.6f)\n", what.c_str(), got, want);
  }
}

// BlueBoat120 channel assignment, from the ArduRover 4.7 params:
// right = SERVO1 not reversed, left = SERVO3 reversed.
static EscChannelConfig right_cfg()
{
  EscChannelConfig c;             // 1100 / 1510 / 1900
  c.reversed = false;
  return c;
}
static EscChannelConfig left_cfg()
{
  EscChannelConfig c;
  c.reversed = true;
  return c;
}

static void test_config_validation()
{
  check(right_cfg().validate().empty(), "BlueBoat defaults validate");

  EscChannelConfig bad;
  bad.min_us = 1600;   // min > trim
  check(!bad.validate().empty(), "min > trim rejected");

  EscChannelConfig bad2;
  bad2.max_us = 1400;  // max < trim
  check(!bad2.validate().empty(), "max < trim rejected");

  EscChannelConfig bad3;
  bad3.trim_us = std::nan("");
  check(!bad3.validate().empty(), "non-finite trim rejected");

  EscChannelConfig bad4;
  bad4.min_us = 100;   // outside plausible servo band
  check(!bad4.validate().empty(), "implausible endpoint rejected");
}

static void test_endpoints_and_neutral()
{
  const EscChannelConfig r = right_cfg();
  const EscChannelConfig l = left_cfg();

  check_near(esc_map(0.0, r).pulse_us, 1510.0, 1e-9, "right neutral = trim");
  check_near(esc_map(0.0, l).pulse_us, 1510.0, 1e-9, "left neutral = trim");
  check_near(esc_neutral_us(r), 1510.0, 1e-9, "esc_neutral_us agrees with trim");

  // Physical forward on both sides. The pulses differ because the
  // left channel is electrically reversed -- that asymmetry IS
  // the wiring, and it must appear here and nowhere else.
  check_near(esc_map( 100.0, r).pulse_us, 1900.0, 1e-9, "right full forward");
  check_near(esc_map( 100.0, l).pulse_us, 1100.0, 1e-9, "left full forward");
  check_near(esc_map(-100.0, r).pulse_us, 1100.0, 1e-9, "right full reverse");
  check_near(esc_map(-100.0, l).pulse_us, 1900.0, 1e-9, "left full reverse");
}

static void test_piecewise_spans_are_unequal()
{
  const EscChannelConfig r = right_cfg();
  // 390 us above trim, 410 us below. A single symmetric scale
  // would misplace one end by 20 us.
  check_near(esc_map( 50.0, r).pulse_us, 1510.0 + 195.0, 1e-9, "right +50%");
  check_near(esc_map(-50.0, r).pulse_us, 1510.0 - 205.0, 1e-9, "right -50%");
}

static void test_clamping()
{
  const EscChannelConfig r = right_cfg();

  EscOutput hi = esc_map(150.0, r);
  check(hi.clamped, "over-range flagged");
  check_near(hi.pulse_us, 1900.0, 1e-9, "over-range clamps to max");

  EscOutput lo = esc_map(-150.0, r);
  check(lo.clamped, "under-range flagged");
  check_near(lo.pulse_us, 1100.0, 1e-9, "under-range clamps to min");

  EscOutput ok = esc_map(42.0, r);
  check(!ok.clamped, "in-range not flagged");
}

static void test_nonfinite_fails_closed()
{
  const EscChannelConfig r = right_cfg();
  const double bad[] = {std::nan(""), HUGE_VAL, -HUGE_VAL};
  for (size_t i = 0; i < 3; ++i) {
    EscOutput o = esc_map(bad[i], r);
    check_near(o.pulse_us, 1510.0, 1e-9, "non-finite effort -> trim");
    check(o.clamped, "non-finite effort flagged");
  }
}

// The whole point of the endpoint change: the full command range
// now lands inside the ESC's documented 1100-1900 window, so no
// command is physically unreachable.
static void test_full_range_is_reachable()
{
  const EscChannelConfig cfgs[2] = {right_cfg(), left_cfg()};
  for (int c = 0; c < 2; ++c) {
    for (int e = -100; e <= 100; ++e) {
      EscOutput o = esc_map((double)e, cfgs[c]);
      check(o.pulse_us >= 1100.0 - 1e-9 && o.pulse_us <= 1900.0 + 1e-9,
            "pulse stays inside the ESC's documented range");
      check(!o.clamped, "no in-range command is clamped");
    }
  }
}

// Monotonicity: more physical forward effort must never produce
// less forward pulse travel. Catches a sign error in reversal.
static void test_monotonic_in_physical_effort()
{
  const EscChannelConfig cfgs[2] = {right_cfg(), left_cfg()};
  for (int c = 0; c < 2; ++c) {
    const bool rev = cfgs[c].reversed;
    double prev = esc_map(-100.0, cfgs[c]).pulse_us;
    for (int e = -99; e <= 100; ++e) {
      const double cur = esc_map((double)e, cfgs[c]).pulse_us;
      const bool ok = rev ? (cur <= prev + 1e-9) : (cur >= prev - 1e-9);
      if (!ok) { check(false, "monotonic in physical effort"); return; }
      prev = cur;
    }
  }
  g_checks++;
}

int main()
{
  printf("test_esc_mapper\n");
  test_config_validation();
  test_endpoints_and_neutral();
  test_piecewise_spans_are_unequal();
  test_clamping();
  test_nonfinite_fails_closed();
  test_full_range_is_reachable();
  test_monotonic_in_physical_effort();

  if (g_failures) {
    fprintf(stderr, "\n%d/%d checks FAILED\n", g_failures, g_checks);
    return 1;
  }
  printf("PASS (%d checks)\n", g_checks);
  return 0;
}
