/*************************************************************
 * bb::Pid vs moos-ivp ScalarPID -- the parity oracle.
 *
 * The parity contract (pid.h): on a CLEAN tick -- integration
 * enabled, no external saturation, dt under max_dt, output
 * inside its limit, Kd = 0 (all live BlueBoat gain sets) --
 * bb::Pid must reproduce ScalarPID's arithmetic exactly. The
 * moos-ivp dependency is retired from the runtime and kept
 * here, as the test oracle that pins the water-proven
 * semantics.
 *
 * Where the two must DIVERGE is asserted just as hard: at a
 * rail the old class parks its integral at the limit and the
 * new one refuses the increment, so the new loop must come off
 * the rail no later than the old one.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "pid.h"
#include "ScalarPID.h"

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
    fprintf(stderr, "FAIL: %s (got %.12f want %.12f)\n", what.c_str(), got, want);
  }
}

// Deterministic pseudo-random error stream (no libc rand: the
// sequence must be identical everywhere the test runs).
static unsigned long g_seed = 12345;
static double lcg_pm1()
{
  g_seed = g_seed * 1103515245ul + 12345ul;
  return ((double)((g_seed >> 16) & 0x7fff) / 16383.5) - 1.0;
}

struct LoopSpec {
  const char* name;
  double kp, ki, kd;
  double i_lim, out_lim;
  double err_scale;       // keep the loop off its rails for the clean test
};

// The three live gain sets from plug_pBBPID.moos (all Kd = 0).
static const LoopSpec kLoops[] = {
  { "speed  (1,2,0 / il 100, ol 100)", 1.0,  2.0, 0.0, 100.0, 100.0, 1.0  },
  { "heading(0.8,0,0 / il .44, ol .44)", 0.8, 0.0, 0.0, 0.4363, 0.4363, 0.3 },
  { "yawrate(60,40,0 / il 50, ol 100)", 60.0, 40.0, 0.0, 50.0, 100.0, 0.01 },
};

//---------------------------------------------------------
// Clean segments: tick-for-tick exact equality over 2000 ticks of
// wandering error at 20 Hz, including dt jitter (MOOS is not a
// fixed-rate scheduler and neither is the parity claim).

static void test_clean_segment_parity()
{
  for (unsigned s = 0; s < sizeof(kLoops)/sizeof(kLoops[0]); ++s) {
    const LoopSpec& L = kLoops[s];

    ScalarPID old_pid;
    old_pid.SetGains(L.kp, L.kd, L.ki);          // ScalarPID order: Kp, Kd, Ki
    old_pid.SetLimits(L.i_lim, L.out_lim);

    bb::Pid new_pid;
    new_pid.set_gains(L.kp, L.ki, L.kd);
    new_pid.set_limits(L.i_lim, L.out_lim);
    // max_dt, gate, external saturation: all left at legacy defaults.

    double t = 1000.0;
    double err = 0.0;
    for (int i = 0; i < 2000; ++i) {
      double dt = 0.05 * (1.0 + 0.2 * lcg_pm1());       // 40-60 ms jitter
      t += dt;
      err += 0.1 * L.err_scale * lcg_pm1();             // random walk
      if (err >  L.err_scale) err =  L.err_scale;       // stay off the rails
      if (err < -L.err_scale) err = -L.err_scale;

      double want = 0.0;
      old_pid.Run(err, t, want);
      double got = new_pid.step(err, dt);

      if (std::fabs(got - want) > 1e-9) {
        check_near(got, want, 1e-9,
                   std::string("clean parity, loop ") + L.name +
                   " tick " + std::to_string(i));
        return;   // one detailed failure beats 2000 identical ones
      }
    }
    check(true, std::string("clean parity 2000 ticks, loop ") + L.name);
  }
}

//---------------------------------------------------------
// At a rail both classes clamp the OUTPUT identically; the state
// underneath differs by design. Drive both hard onto the rail,
// then reverse the error and count ticks to leave it: the
// anti-windup loop must recover no later, and the old one's
// parked integral is the reason.

static void test_rail_divergence_and_recovery()
{
  // Gains shaped so the INTEGRAL is what rails the output: kp
  // small, ki dominant, i_lim well past out_lim. The oracle winds
  // its integral to 50 against a 10-unit rail; the new loop stops
  // committing increments the moment the output rails.
  const double kp = 1.0, ki = 40.0;
  const double i_lim = 50.0, out_lim = 10.0;
  const double dt = 0.05;

  ScalarPID old_pid;
  old_pid.SetGains(kp, 0.0, ki);
  old_pid.SetLimits(i_lim, out_lim);
  bb::Pid new_pid;
  new_pid.set_gains(kp, ki, 0.0);
  new_pid.set_limits(i_lim, out_lim);
  new_pid.set_antiwindup_enable(true);

  // AW OFF must be oracle-exact even ON the rail -- that is what
  // makes a flags-off replay of a log that saturates auditable.
  bb::Pid legacy_pid;
  legacy_pid.set_gains(kp, ki, 0.0);
  legacy_pid.set_limits(i_lim, out_lim);

  double t = 1000.0;
  double old_out = 0.0;
  for (int i = 0; i < 100; ++i) {           // 5 s of e = +5, railed
    t += dt;
    old_pid.Run(5.0, t, old_out);
    new_pid.step(5.0, dt);
    double lo = legacy_pid.step(5.0, dt);
    check_near(lo, old_out, 1e-9, "AW off matches oracle at the rail");
    if (std::fabs(lo - old_out) > 1e-9) return;
  }
  check_near(legacy_pid.i_term(), i_lim, 1e-9,
             "AW off: integral parked at i_lim, like the oracle");
  check_near(old_out, out_lim, 1e-12, "oracle railed");
  check_near(new_pid.output(), out_lim, 1e-12, "new loop railed identically");
  check(new_pid.i_term() <= out_lim + 1e-9,
        "new integral stopped at the rail, not at i_lim");
  check(old_pid.getKI() > 0, "oracle sanity");

  // Error reverses, gently: -0.1. The oracle must burn its parked
  // integral down (~200 ticks = 10 s of full reverse rudder the
  // boat did not ask for); the new loop responds immediately.
  int old_ticks = -1, new_ticks = -1;
  for (int i = 0; i < 500; ++i) {
    t += dt;
    old_pid.Run(-0.1, t, old_out);
    double new_out = new_pid.step(-0.1, dt);
    if (old_ticks < 0 && old_out < out_lim - 1e-9) old_ticks = i;
    if (new_ticks < 0 && new_out < out_lim - 1e-9) new_ticks = i;
    if (old_ticks >= 0 && new_ticks >= 0) break;
  }
  check(new_ticks == 0, "new loop leaves the rail on the FIRST reversed tick");
  check(old_ticks > 50,
        "oracle stays railed for seconds while its integral unwinds");
}

//---------------------------------------------------------
// The resume-windup incident, in miniature: identical loops, a
// 43 s hole in the time base, one error sample after. The oracle
// integrates the hole; max_dt credits one tick.

static void test_gap_divergence()
{
  const double ki = 2.0, i_lim = 100.0, out_lim = 100.0;

  ScalarPID old_pid;
  old_pid.SetGains(1.0, 0.0, ki);
  old_pid.SetLimits(i_lim, out_lim);
  bb::Pid new_pid;
  new_pid.set_gains(1.0, ki, 0.0);
  new_pid.set_limits(i_lim, out_lim);
  new_pid.set_max_dt(0.5);

  double t = 1000.0, old_out = 0.0;
  t += 0.05; old_pid.Run(1.5, t, old_out); new_pid.step(1.5, 0.05);
  t += 43.0; old_pid.Run(1.5, t, old_out); new_pid.step(1.5, 43.0);

  // Oracle: I ~= 2*1.5*(0.05+43) clamped to 100 -> way past sane.
  check(old_out >= 80.0, "oracle integrated the whole gap");
  check(new_pid.i_term() <= ki * 1.5 * (0.05 + 0.5) + 1e-9,
        "max_dt bounded the gap to one tick's credit");
}

//---------------------------------------------------------
int main()
{
  test_clean_segment_parity();
  test_rail_divergence_and_recovery();
  test_gap_divergence();

  printf("test_pid_scalar_parity: %d checks, %d failures\n",
         g_checks, g_failures);
  return g_failures ? 1 : 0;
}
