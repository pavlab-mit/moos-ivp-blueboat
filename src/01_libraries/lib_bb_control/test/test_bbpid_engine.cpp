/*************************************************************
 * Unit tests for BBPIDEngine -- the cascade around bb::Pid.
 *
 * Configured like the live plug (plug_pBBPID.moos, rad/s yaw
 * path) and driven like the handling_block mission: staircase
 * commands, step-and-hold, the 180 pivot, the RC-repositioning
 * gap. These are the brief's section 3 cases, as checks.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "BBPIDEngine.h"

#include <cmath>
#include <cstdio>
#include <string>

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

// Live plug shape: 20 Hz, rad/s yaw path, 0.4363 rad/s cap.
static const double kDt        = 0.05;
static const double kMaxYawRate = 0.4363;

static BBPIDEngine make_live_engine()
{
  BBPIDEngine e;
  e.setSpeedGains(1.0, 2.0, 0.0);
  e.setHeadingGains(0.8, 0.0, 0.0);
  e.setYawRateGains(60.0, 40.0, 0.0);
  e.setSpeedLimits(100.0, 100.0);
  e.setHeadingLimits(kMaxYawRate, kMaxYawRate);
  e.setYawRateLimits(50.0, 100.0);
  e.setAllowReverse(true);
  e.setYawRateScale(1.0);              // feedback already rad/s; engine negates
  e.setDesYawRateFilter(0.0);          // raw FF: deterministic checks
  return e;
}

// One tick, boat held stationary (nav frozen). Returns des yaw rate.
static void tick(BBPIDEngine& e, double t, double des_spd, double des_hdg,
                 double nav_spd = 0.0, double nav_hdg = 0.0,
                 double nav_yr = 0.0)
{
  double thrust = 0.0, rudder = 0.0;
  e.update(t, des_spd, nav_spd, des_hdg, nav_hdg, nav_yr, thrust, rudder);
}

//---------------------------------------------------------
// Legacy staircase: a helm ticking the command 5 deg per second.
// The held derivative between changes is the FEATURE -- with the
// hold-expiry configured at 2x the cadence it must not fire.

static void test_staircase_ff_alive()
{
  BBPIDEngine e = make_live_engine();
  e.setHeadingGains(0.0, 0.0, 0.0);    // isolate the reference FF
  e.setFFHoldTime(2.0);                // 2x the 1 Hz staircase cadence

  double t = 1000.0, hdg = 0.0;
  tick(e, t, 0.0, hdg);
  // 10 s staircase: +5 deg every 1.0 s = 0.0873 rad/s implied.
  for (int i = 1; i <= 200; ++i) {
    t += kDt;
    if (i % 20 == 0) hdg += 5.0;
    tick(e, t, 0.0, hdg);
  }
  check_near(e.getDesiredYawRate(), 5.0 * (M_PI/180.0), 1e-6,
             "staircase FF holds the implied rate between changes");
}

//---------------------------------------------------------
// Step-and-hold: the 27 Aug standing-error defect. After the
// command settles, the held derivative must decay to a true zero
// instead of forcing a standing error of ff/kp (6.4 deg measured).

static void test_step_and_hold_ff_expires()
{
  BBPIDEngine e = make_live_engine();
  e.setHeadingGains(0.0, 0.0, 0.0);    // watch the FF alone
  e.setFFHoldTime(2.0);

  double t = 1000.0;
  tick(e, t, 0.0, 0.0);
  for (int i = 0; i < 20; ++i) { t += kDt; tick(e, t, 0.0, 0.0); }
  t += kDt; tick(e, t, 0.0, 60.0);     // one 60 deg step...
  check(std::fabs(e.getDesiredYawRate()) > 0.0,
        "step implies a rate at the change tick");
  for (int i = 0; i < 100; ++i) { t += kDt; tick(e, t, 0.0, 60.0); }  // ...held 5 s
  check_near(e.getDesiredYawRate(), 0.0, 1e-12,
             "held command decays FF to a true zero after ff_hold_time");

  // Legacy mode: same drive, hold-forever, the standing bias remains.
  BBPIDEngine l = make_live_engine();
  l.setHeadingGains(0.0, 0.0, 0.0);
  double tl = 1000.0;
  tick(l, tl, 0.0, 0.0);
  for (int i = 0; i < 20; ++i) { tl += kDt; tick(l, tl, 0.0, 0.0); }
  tl += kDt; tick(l, tl, 0.0, 60.0);
  for (int i = 0; i < 100; ++i) { tl += kDt; tick(l, tl, 0.0, 60.0); }
  check(std::fabs(l.getDesiredYawRate()) > 0.0,
        "legacy default still holds the derivative (parity)");
}

//---------------------------------------------------------
// The 180 pivot. With the step guard, a repositioning step
// publishes no FF at all: the desired rate is pure feedback,
// railed at the full turn budget -- not FF-cancelled to 2 deg/s.

static void test_180_pivot_full_rate()
{
  BBPIDEngine e = make_live_engine();
  e.setFFStepLimitDeg(150.0);

  double t = 1000.0;
  tick(e, t, 0.0, 0.0);
  for (int i = 0; i < 20; ++i) { t += kDt; tick(e, t, 0.0, 0.0); }
  t += kDt; tick(e, t, 0.0, 180.0);    // exact 180: FF sign would be noise
  check_near(std::fabs(e.getDesiredYawRate()), kMaxYawRate, 1e-9,
             "180 step commands the FULL turn budget");
  // (The legacy defect itself -- FF sign flipping against the railed
  // feedback at exact 180 -- is a floating-point coin flip on
  // sin(pi) rounding and cannot be pinned deterministically. What
  // CAN be pinned is that the guard removes the coin: below.)

  // Guard on, FF path isolated: a repositioning step publishes NO
  // reference FF at all, while an ordinary trajectory step does.
  BBPIDEngine g = make_live_engine();
  g.setHeadingGains(0.0, 0.0, 0.0);    // silence feedback: watch FF alone
  g.setFFStepLimitDeg(150.0);
  double tg = 1000.0;
  tick(g, tg, 0.0, 0.0);
  for (int i = 0; i < 20; ++i) { tg += kDt; tick(g, tg, 0.0, 0.0); }
  tg += kDt; tick(g, tg, 0.0, 160.0);  // beyond the limit: repositioning
  check_near(g.getDesiredYawRate(), 0.0, 1e-12,
             "guarded: repositioning step publishes no FF");

  BBPIDEngine h = make_live_engine();
  h.setHeadingGains(0.0, 0.0, 0.0);
  h.setFFStepLimitDeg(150.0);
  double th = 1000.0;
  tick(h, th, 0.0, 0.0);
  for (int i = 0; i < 20; ++i) { th += kDt; tick(h, th, 0.0, 0.0); }
  th += kDt; tick(h, th, 0.0, 100.0);  // ordinary step: FF present
  check(std::fabs(h.getDesiredYawRate()) > 0.0,
        "guarded: ordinary trajectory step keeps its FF");
}

//---------------------------------------------------------
// FF cap: a large step over a short cadence implies a rate far
// beyond the turn budget; the FF itself must be capped so feedback
// keeps authority inside the clamp.

static void test_ff_capped_at_budget()
{
  BBPIDEngine e = make_live_engine();
  e.setHeadingGains(0.0, 0.0, 0.0);
  double t = 1000.0;
  tick(e, t, 0.0, 0.0);
  t += kDt; tick(e, t, 0.0, 120.0);    // 120 deg in 50 ms ~= 42 rad/s implied
  check(std::fabs(e.getDesiredYawRate()) <= kMaxYawRate + 1e-12,
        "reference FF alone cannot exceed max_yawrate");
}

//---------------------------------------------------------
// Quadratic speed-FF term (session 3): the hull's drag is
// 9.2v + 10.1v|v| (rc_cal), which a linear cv cannot fit at both
// ends of the staircase. cvv defaults to 0 = legacy bit-identical;
// nonzero adds cvv*v*|v*| (signed, so reverse commands get
// reverse FF).

static BBPIDEngine make_speed_ff_engine()
{
  BBPIDEngine e = make_live_engine();
  e.setFeedforwardEnable(true);
  e.setFeedforwardSpeedEnable(true);
  return e;
}

static void test_ff_speed_quadratic_term()
{
  double t = 1000.0;

  // cvv omitted (3-arg call) == cvv 0: legacy FF value exactly.
  BBPIDEngine legacy = make_speed_ff_engine();
  legacy.setFeedforwardSpeed(0.0, 9.2, 0.0);
  tick(legacy, t, 1.5, 0.0, 1.5);
  check_near(legacy.getFFThrust(), 9.2 * 1.5, 1e-12,
             "cvv default 0: FF is the legacy linear value");

  // Quadratic term present: FF = cv*v + cvv*v^2 at the drag-curve
  // coefficients matches the measured steady demand (~34 at 1.5).
  BBPIDEngine quad = make_speed_ff_engine();
  quad.setFeedforwardSpeed(0.0, 9.2, 0.0, 10.1);
  tick(quad, t, 1.5, 0.0, 1.5);
  check_near(quad.getFFThrust(), 9.2 * 1.5 + 10.1 * 1.5 * 1.5, 1e-12,
             "cvv adds the quadratic drag term");

  // Signed: a reverse command gets reverse FF, not forward.
  BBPIDEngine rev = make_speed_ff_engine();
  rev.setFeedforwardSpeed(0.0, 9.2, 0.0, 10.1);
  tick(rev, t, -1.0, 0.0, -1.0);
  check_near(rev.getFFThrust(), -9.2 - 10.1, 1e-12,
             "cvv term is v*|v|: reverse command -> reverse FF");
}

//---------------------------------------------------------
// The 43 s RC-repositioning gap, engine-level: with max_dt the
// speed integral resumes sane instead of railed (surge -13.7).

static void test_gap_bounded_at_engine_level()
{
  BBPIDEngine e = make_live_engine();
  e.setMaxDt(0.5);
  double t = 1000.0;
  tick(e, t, 1.5, 0.0);
  t += kDt; tick(e, t, 1.5, 0.0);      // integrating a 1.5 m/s error
  t += 43.0; tick(e, t, 1.5, 0.0);     // the gap
  check(std::fabs(e.speedITerm()) <= 2.0 * 1.5 * (kDt + 0.5) + 1e-9,
        "speed integral bounded across the gap");
}

//---------------------------------------------------------
// Authority gate at the engine boundary: frozen integrals with
// live P, resumed on release.

static void test_integrate_gate()
{
  BBPIDEngine e = make_live_engine();
  double t = 1000.0;
  tick(e, t, 1.5, 0.0);
  for (int i = 0; i < 20; ++i) { t += kDt; tick(e, t, 1.5, 0.0); }
  double i0 = e.speedITerm();
  check(i0 > 0.0, "integral accrues under autonomy");

  e.setIntegrateEnable(false);          // kill window / RC takeover
  for (int i = 0; i < 100; ++i) { t += kDt; tick(e, t, 1.5, 0.0); }
  check_near(e.speedITerm(), i0, 1e-12, "integral frozen under the gate");
  check(e.getThrust() > 0.0, "P still drives while frozen");

  e.setIntegrateEnable(true);
  t += kDt; tick(e, t, 1.5, 0.0);
  check(e.speedITerm() > i0, "integration resumes on release");
}

//---------------------------------------------------------
// Mixer saturation reported from the app: the loop demanding more
// than the mixer can allocate stops integrating the excess.

static void test_external_saturation_stops_excess()
{
  BBPIDEngine e = make_live_engine();
  e.setAntiWindup(true);               // rail reporting is AW-family
  double t = 1000.0;
  tick(e, t, 1.5, 0.0);
  for (int i = 0; i < 5; ++i) { t += kDt; tick(e, t, 1.5, 0.0); }
  double i0 = e.speedITerm();

  e.setExternalSaturation(+1, 0);       // mixer railed on positive surge
  for (int i = 0; i < 50; ++i) { t += kDt; tick(e, t, 1.5, 0.0); }
  check_near(e.speedITerm(), i0, 1e-12,
             "positive-surge rail stops positive-error integration");

  e.setExternalSaturation(0, 0);
  t += kDt; tick(e, t, 1.5, 0.0);
  check(e.speedITerm() > i0, "cleared rail resumes integration");
}

//---------------------------------------------------------
// Internal post-FF rail: identified FF pushes thrust to the clamp;
// the speed loop must not wind the difference the water never sees.

static void test_post_ff_rail_observed()
{
  BBPIDEngine e = make_live_engine();
  e.setAntiWindup(true);
  e.setFeedforwardEnable(true);
  e.setFeedforwardSpeed(95.0, 0.0, 0.0);   // c0 alone nearly rails thrust
  double t = 1000.0;
  tick(e, t, 1.5, 0.0);
  for (int i = 0; i < 200; ++i) { t += kDt; tick(e, t, 1.5, 0.0); }
  check_near(e.getThrust(), 100.0, 1e-9, "thrust railed by FF + PID");
  // I may supply up to the rail's worth (5 units over c0), never more:
  check(e.speedITerm() <= 5.0 + 1.5 + 1e-9,
        "speed integral parks near the rail's worth, not at i_lim");
}

//---------------------------------------------------------
int main()
{
  test_staircase_ff_alive();
  test_step_and_hold_ff_expires();
  test_180_pivot_full_rate();
  test_ff_capped_at_budget();
  test_ff_speed_quadratic_term();
  test_gap_bounded_at_engine_level();
  test_integrate_gate();
  test_external_saturation_stops_excess();
  test_post_ff_rail_observed();

  printf("test_bbpid_engine: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
