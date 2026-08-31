/*************************************************************
 * Unit tests for bb::Pid.
 *
 * The scenarios are the incidents: the 43 s resume-windup gap
 * and the railed-pivot windup from the 2026-08-27 handling
 * block, plus the lifecycle semantics ScalarPID cannot express
 * (freeze-with-live-P, external rails, bounded dt credit).
 * ScalarPID-parity itself is pinned separately in
 * test_pid_scalar_parity.cpp against the real class.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "pid.h"

#include <cmath>
#include <cstdio>
#include <limits>
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
    fprintf(stderr, "FAIL: %s (got %.9f want %.9f)\n", what.c_str(), got, want);
  }
}

// Live-config-shaped loop: the yaw-rate loop's 60/40/0 at 20 Hz.
static const double kDt = 0.05;

static Pid make_yawrate_loop()
{
  Pid p;
  p.set_gains(60.0, 40.0, 0.0);
  p.set_limits(50.0, 100.0);
  return p;
}

//---------------------------------------------------------
static void test_p_only()
{
  Pid p;
  p.set_gains(2.0, 0.0, 0.0);
  p.set_limits(0.0, 100.0);
  check_near(p.step(3.0, kDt), 6.0, 1e-12, "P-only output kp*e");
  check_near(p.i_term(), 0.0, 1e-12, "P-only accrues no integral");
}

//---------------------------------------------------------
static void test_integral_accumulation_output_units()
{
  Pid p = make_yawrate_loop();
  p.step(0.1, kDt);   // first step establishes state: no increment
  check_near(p.i_term(), 0.0, 1e-12, "first step integrates nothing");
  p.step(0.1, kDt);   // I += 40*0.1*0.05 = 0.2
  p.step(0.1, kDt);
  check_near(p.i_term(), 0.4, 1e-12, "I accumulates ki*e*dt in output units");
  check_near(p.output(), 60.0*0.1 + 0.4, 1e-12, "output includes this tick's increment");
}

static void test_integral_limit()
{
  Pid p = make_yawrate_loop();
  p.set_limits(0.5, 1000.0);          // tiny I cap, huge output cap
  for (int i = 0; i < 100; ++i) p.step(1.0, kDt);
  check_near(p.i_term(), 0.5, 1e-12, "I clamps at integral_limit");
  Pid n = make_yawrate_loop();
  n.set_limits(0.5, 1000.0);
  for (int i = 0; i < 100; ++i) n.step(-1.0, kDt);
  check_near(n.i_term(), -0.5, 1e-12, "I clamp is symmetric");
}

static void test_ki_zero_clears_integral()
{
  // ScalarPID semantics, kept deliberately: no Ki, no memory.
  Pid p = make_yawrate_loop();
  p.step(1.0, kDt);
  p.step(1.0, kDt);
  check(p.i_term() > 0.0, "integral present before Ki change");
  p.set_gains(60.0, 0.0, 0.0);
  p.step(1.0, kDt);
  check_near(p.i_term(), 0.0, 1e-12, "Ki <= 0 zeroes the accumulated integral");
}

//---------------------------------------------------------
// The 27 Aug incident: 43 s of RC repositioning, then one tick.
// ScalarPID integrated Ki*e*dt across the whole gap and slammed
// the integral to its limit; with max_dt the gap earns at most one
// bounded increment.

static void test_gap_credit_bounded()
{
  // Error kept small (0.02) so the OUTPUT stays off its rail: this
  // isolates the dt-credit bound from the tracking anti-windup,
  // which would otherwise (correctly) refuse the wall on its own.
  const double e = 0.02;
  Pid p = make_yawrate_loop();
  p.set_max_dt(0.5);
  p.step(e, kDt);
  p.step(e, kDt);
  double i_before = p.i_term();
  p.step(e, 43.0);                     // the gap arrives as one huge dt
  double increment = p.i_term() - i_before;
  check(increment <= 40.0 * e * 0.5 + 1e-12,
        "gap increment bounded by ki*e*max_dt");
  check(increment > 0.0, "gap still earns one normal-tick credit");

  Pid legacy = make_yawrate_loop();    // max_dt unset = legacy
  legacy.step(e, kDt);
  legacy.step(e, kDt);
  double li = legacy.i_term();
  legacy.step(e, 43.0);
  check(legacy.i_term() - li > 40.0 * e * 0.5,
        "without max_dt the gap integrates in full (legacy parity)");
}

static void test_negative_dt_no_integration()
{
  Pid p = make_yawrate_loop();
  p.step(1.0, kDt);
  p.step(1.0, kDt);
  double i0 = p.i_term();
  double out = p.step(1.0, -5.0);      // clock went backwards
  check_near(p.i_term(), i0, 1e-12, "negative dt integrates nothing");
  check(std::isfinite(out), "negative dt still yields a finite output");
}

//---------------------------------------------------------
static void test_freeze_keeps_p_live()
{
  Pid p = make_yawrate_loop();
  p.step(0.5, kDt);
  p.step(0.5, kDt);
  double i0 = p.i_term();
  p.set_integrate_enable(false);
  double out = p.step(1.0, kDt);
  check_near(p.i_term(), i0, 1e-12, "frozen integral does not move");
  check_near(out, 60.0*1.0 + i0 + p.d_term(), 1e-12,
             "P responds to fresh error while frozen");
  p.set_integrate_enable(true);
  p.step(1.0, kDt);
  check(p.i_term() > i0, "integration resumes when re-enabled");
}

//---------------------------------------------------------
// Rollback anti-windup at the loop's own output rail: the railed
// pivot. Error saturates the output; the integral must not climb
// to its limit while the rail holds, and must unwind immediately
// when the error reverses.

static void test_antiwindup_off_is_legacy_at_rails()
{
  // Default (AW off): ScalarPID-verbatim -- the integral climbs to
  // its own limit behind the wall. This is what keeps flags-off
  // parity replay exact even on logs that rail.
  Pid p = make_yawrate_loop();       // i_lim 50, out_lim 100
  for (int i = 0; i < 200; ++i) p.step(10.0, kDt);
  check_near(p.i_term(), 50.0, 1e-9,
             "AW off: integral parks at i_lim behind the rail (legacy)");
}

static void test_rollback_at_own_rail()
{
  Pid p = make_yawrate_loop();
  p.set_antiwindup_enable(true);
  for (int i = 0; i < 200; ++i) p.step(10.0, kDt);   // P alone rails (600 > 100)
  check(p.saturated(), "output is railed");
  check_near(p.i_term(), 0.0, 1e-9,
             "integral does not climb while railed in its direction");

  double out = p.step(-0.5, kDt);                    // error reverses
  check(out < 100.0, "output leaves the rail on the first reversed tick");
  check(p.i_term() < 0.0, "unwinding increments are always allowed");
}

static void test_tracking_parks_at_rail_authority()
{
  // kp small, ki dominant, rail at 10: the integral may supply the
  // rail's worth of authority (P=5 -> I parks at 5) but not climb
  // toward its own 50-unit limit behind the wall.
  Pid p;
  p.set_gains(1.0, 40.0, 0.0);
  p.set_limits(50.0, 10.0);
  p.set_antiwindup_enable(true);
  for (int i = 0; i < 100; ++i) p.step(5.0, kDt);
  check_near(p.output(), 10.0, 1e-12, "output rails");
  check_near(p.i_term(), 5.0, 1e-9, "I parks at exactly the rail's worth");
  double out = p.step(-0.1, kDt);
  check(out < 10.0, "off the rail on the first reversed tick");
}

static void test_increment_away_from_rail_kept()
{
  // Railed positive by a big P term but error already negative:
  // the increment reduces |out| and must be kept.
  Pid p;
  p.set_gains(0.0, 40.0, 0.0);
  p.set_limits(50.0, 10.0);
  p.set_antiwindup_enable(true);
  for (int i = 0; i < 100; ++i) p.step(1.0, kDt);    // wind I to +10 rail
  double i0 = p.i_term();
  p.step(-1.0, kDt);
  check(p.i_term() < i0, "negative error unwinds a positive-railed loop");
}

//---------------------------------------------------------
static void test_external_saturation()
{
  Pid p = make_yawrate_loop();
  p.set_external_saturation(+1);       // downstream railed positive
  p.step(1.0, kDt);
  check_near(p.i_term(), 0.0, 1e-12,
             "no increment into an external positive rail");
  p.step(-1.0, kDt);
  check(p.i_term() < 0.0, "unwinding past an external rail is allowed");

  p.set_external_saturation(0);
  double i0 = p.i_term();
  p.step(1.0, kDt);
  check(p.i_term() > i0, "clearing the external rail resumes integration");
}

//---------------------------------------------------------
static void test_fail_closed()
{
  Pid p = make_yawrate_loop();
  double good = p.step(1.0, kDt);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  check_near(p.step(nan, kDt), good, 1e-12, "NaN error holds the output");
  check_near(p.step(1.0, nan), good, 1e-12, "NaN dt holds the output");
  check_near(p.step(1.0, nan, kDt), good, 1e-12, "NaN d_error holds the output");
  double i0 = p.i_term();
  p.step(nan, kDt);
  check_near(p.i_term(), i0, 1e-12, "NaN input integrates nothing");
}

//---------------------------------------------------------
static void test_derivative()
{
  Pid p;
  p.set_gains(0.0, 0.0, 2.0);
  p.set_limits(0.0, 1000.0);
  p.step(1.0, kDt);
  check_near(p.d_term(), 0.0, 1e-12, "first tick has no derivative");
  p.step(2.0, kDt);
  check_near(p.d_term(), 2.0 * (1.0/kDt), 1e-12, "backward difference");
  p.step(2.0, 0.0);
  check_near(p.d_term(), 2.0 * (1.0/kDt), 1e-12, "dt=0 holds the derivative");

  Pid q;
  q.set_gains(0.0, 0.0, 2.0);
  q.set_limits(0.0, 1000.0);
  q.step(5.0, -3.0, kDt);              // caller-supplied d_error
  check_near(q.d_term(), -6.0, 1e-12, "caller-supplied derivative used as-is");
}

//---------------------------------------------------------
static void test_reset()
{
  Pid p = make_yawrate_loop();
  p.step(1.0, kDt); p.step(2.0, kDt);
  p.reset();
  check_near(p.i_term(), 0.0, 1e-12, "reset clears I");
  check_near(p.output(), 0.0, 1e-12, "reset clears the held output");
  p.step(2.0, kDt);
  check_near(p.d_term(), 0.0, 1e-12, "no derivative on first tick after reset");

  // Errors kept small enough that the output never rails, so the
  // formula check is exact.
  Pid q = make_yawrate_loop();
  q.step(0.5, kDt); q.step(0.8, kDt);
  q.reset_integral();
  check_near(q.i_term(), 0.0, 1e-12, "reset_integral clears only I");
  double out = q.step(0.5, kDt);
  check_near(out, 60.0*0.5 + q.i_term() + q.d_term(), 1e-12,
             "P/I/D reassemble cleanly after reset_integral");
}

//---------------------------------------------------------
static void test_validate()
{
  Pid p = make_yawrate_loop();
  check(p.validate() == "", "live-shaped config validates");
  Pid q;
  q.set_gains(1.0, 5.0, 0.0);
  q.set_limits(0.0, 100.0);
  check(q.validate() != "", "Ki>0 with zero integral_limit is rejected");
}

//---------------------------------------------------------
int main()
{
  test_p_only();
  test_integral_accumulation_output_units();
  test_integral_limit();
  test_ki_zero_clears_integral();
  test_gap_credit_bounded();
  test_negative_dt_no_integration();
  test_freeze_keeps_p_live();
  test_antiwindup_off_is_legacy_at_rails();
  test_rollback_at_own_rail();
  test_tracking_parks_at_rail_authority();
  test_increment_away_from_rail_kept();
  test_external_saturation();
  test_fail_closed();
  test_derivative();
  test_reset();
  test_validate();

  printf("test_pid: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
