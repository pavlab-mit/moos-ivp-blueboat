/*************************************************************
 * Unit tests for SlewLimiter.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "slew_limiter.h"

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

// BlueBoat: MOT_SLEWRATE = 200 %/s at a 50 Hz control cycle.
static const double kRate = 200.0;
static const double kDt   = 0.02;

static void test_step_size_at_50hz()
{
  SlewLimiter s(kRate);
  // 200 %/s * 0.02 s = 4 percentage points per cycle.
  check_near(s.update(100.0, kDt), 4.0, 1e-9, "one cycle from zero");
  check_near(s.update(100.0, kDt), 8.0, 1e-9, "two cycles");
  check(s.limited(), "still rate-limited");
}

static void test_full_scale_timing()
{
  // Zero to full forward in 0.5 s; full reverse to full forward
  // in 1.0 s. These are the numbers quoted in the design doc, so
  // pin them rather than trusting the arithmetic.
  SlewLimiter s(kRate);
  int cycles = 0;
  while (s.state() < 100.0 - 1e-9 && cycles < 1000) { s.update(100.0, kDt); ++cycles; }
  check_near(cycles * kDt, 0.5, 1e-9, "zero to full forward in 0.5 s");

  SlewLimiter t(kRate);
  t.reset(-100.0);
  cycles = 0;
  while (t.state() < 100.0 - 1e-9 && cycles < 1000) { t.update(100.0, kDt); ++cycles; }
  check_near(cycles * kDt, 1.0, 1e-9, "full reverse to full forward in 1.0 s");
}

static void test_passes_through_when_within_rate()
{
  SlewLimiter s(kRate);
  check_near(s.update(2.0, kDt), 2.0, 1e-9, "small request passes through");
  check(!s.limited(), "small request not flagged limited");
}

static void test_symmetric()
{
  SlewLimiter up(kRate), down(kRate);
  const double a = up.update(100.0, kDt);
  const double b = down.update(-100.0, kDt);
  check_near(a, -b, 1e-9, "limit is symmetric in sign");
}

// Invariant 7: a hard stop must bypass the limiter entirely.
static void test_reset_is_immediate()
{
  SlewLimiter s(kRate);
  for (int i = 0; i < 10; ++i) s.update(100.0, kDt);
  check(s.state() > 30.0, "ramped up before the stop");
  s.reset();
  check_near(s.state(), 0.0, 1e-9, "reset is instant, not slewed");
  check(!s.limited(), "reset clears the limited flag");
}

static void test_reset_to_value_seeds_state()
{
  // Used on a source handoff: carry the applied common effort so
  // an operator taking manual control does not get a drop to zero
  // and a ramp back up (plan section 12(c)).
  SlewLimiter s(kRate);
  s.reset(55.0);
  check_near(s.state(), 55.0, 1e-9, "seeded state");
  check_near(s.update(100.0, kDt), 59.0, 1e-9, "ramps on from the seed");
}

static void test_nonfinite_holds_state()
{
  SlewLimiter s(kRate);
  s.update(50.0, 1.0);           // plenty of credit, lands on 50
  const double held = s.state();

  check_near(s.update(std::nan(""), kDt), held, 1e-9, "NaN request holds state");
  check_near(s.update(HUGE_VAL, kDt),     held, 1e-9, "inf request holds state");
  check_near(s.update(10.0, std::nan("")), held, 1e-9, "NaN dt holds state");
  check_near(s.state(), held, 1e-9, "state never poisoned");
}

static void test_nonpositive_dt_earns_no_credit()
{
  // Two calls in one tick must not move the output twice.
  SlewLimiter s(kRate);
  s.update(100.0, kDt);
  const double after_one = s.state();
  s.update(100.0, 0.0);
  check_near(s.state(), after_one, 1e-9, "zero dt earns nothing");
  s.update(100.0, -0.02);
  check_near(s.state(), after_one, 1e-9, "negative dt earns nothing");
}

// No ArduPilot equivalent; see the header comment for why.
static void test_stall_cannot_grant_unbounded_step()
{
  SlewLimiter s(kRate, 0.5);   // max_dt = 0.5 s
  s.update(100.0, 30.0);       // app was descheduled for 30 s
  check_near(s.state(), 100.0, 1e-9, "0.5 s of credit is 100 points, request reached");

  SlewLimiter t(kRate, 0.1);   // tighter clamp
  t.update(100.0, 30.0);
  check_near(t.state(), 20.0, 1e-9, "long stall clamped to max_dt of credit");
}

static void test_zero_rate_disables_limiting()
{
  SlewLimiter s(0.0);
  check_near(s.update(100.0, kDt), 100.0, 1e-9, "rate 0 disables limiting");
  SlewLimiter n(-5.0);
  check_near(n.update(-100.0, kDt), -100.0, 1e-9, "negative rate disables limiting");
}

static void test_validate()
{
  check(SlewLimiter(200.0, 0.5).validate().empty(), "sane config validates");
  check(!SlewLimiter(200.0, 0.0).validate().empty(), "max_dt 0 rejected");
  check(!SlewLimiter(std::nan(""), 0.5).validate().empty(), "NaN rate rejected");
}

int main()
{
  printf("test_slew_limiter\n");
  test_step_size_at_50hz();
  test_full_scale_timing();
  test_passes_through_when_within_rate();
  test_symmetric();
  test_reset_is_immediate();
  test_reset_to_value_seeds_state();
  test_nonfinite_holds_state();
  test_nonpositive_dt_earns_no_credit();
  test_stall_cannot_grant_unbounded_step();
  test_zero_rate_disables_limiting();
  test_validate();

  if (g_failures) {
    fprintf(stderr, "\n%d/%d checks FAILED\n", g_failures, g_checks);
    return 1;
  }
  printf("PASS (%d checks)\n", g_checks);
  return 0;
}
