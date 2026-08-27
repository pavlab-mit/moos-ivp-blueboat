/*************************************************************
 * Unit tests for ArmSequencer.
 *
 * This state machine's predecessor panicked the ESCs on
 * alternating launches for months. The tests that matter are the
 * ones asserting that EVERY exit from ARMING and ARMED goes
 * through DISARM -- because DISARM is where the caller performs
 * the orderly channel all-off, and skipping it is what arms the
 * PCA9685 RESTART trap for the next arm attempt.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "arm_sequencer.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace bb;

static int g_failures = 0;
static int g_checks   = 0;

static void check(bool cond, const std::string& what)
{
  g_checks++;
  if (!cond) { g_failures++; fprintf(stderr, "FAIL: %s\n", what.c_str()); }
}
static void check_act(ArmAction got, ArmAction want, const std::string& what)
{
  g_checks++;
  if (got != want) {
    g_failures++;
    fprintf(stderr, "FAIL: %s (got %s want %s)\n",
            what.c_str(), to_string(got), to_string(want));
  }
}
static void check_state(ArmState got, ArmState want, const std::string& what)
{
  g_checks++;
  if (got != want) {
    g_failures++;
    fprintf(stderr, "FAIL: %s (got %s want %s)\n",
            what.c_str(), to_string(got), to_string(want));
  }
}

static ArmSequencerConfig cfg(double hold = 2.0, bool skip = false)
{
  ArmSequencerConfig c;
  c.arm_hold_sec = hold;
  c.skip_hold = skip;
  return c;
}

// Drive the sequencer at 50 Hz for `secs`, collecting actions.
static std::vector<ArmAction> run(ArmSequencer& s, double& t, double secs,
                                  bool want, bool hw)
{
  std::vector<ArmAction> out;
  const int n = (int)(secs * 50.0);
  for (int i = 0; i < n; ++i) {
    out.push_back(s.update(t, want, hw));
    t += 0.02;
  }
  return out;
}

static bool contains(const std::vector<ArmAction>& v, ArmAction a)
{
  for (ArmAction x : v) if (x == a) return true;
  return false;
}
static int count_of(const std::vector<ArmAction>& v, ArmAction a)
{
  int n = 0; for (ArmAction x : v) if (x == a) ++n; return n;
}

//---------------------------------------------------------

static void test_starts_disarmed_and_idle()
{
  ArmSequencer s(cfg());
  double t = 100.0;
  check_state(s.state(), ArmState::DISARMED, "starts disarmed");
  check_act(s.update(t, false, true), ArmAction::IDLE, "no request -> idle");
  check(!s.armed(), "not armed");
  check(s.arm_cycles() == 0, "no arm cycles yet");
}

static void test_full_arm_takes_the_hold()
{
  ArmSequencer s(cfg(2.0));
  double t = 100.0;

  check_act(s.update(t, true, true), ArmAction::BEGIN_ARM, "request -> BEGIN_ARM");
  check_state(s.state(), ArmState::ARMING, "now arming");
  check(!s.armed(), "not armed during the hold");
  t += 0.02;

  // Through the hold: every cycle must write neutral. An ESC arms
  // on a signal it can see, not on a sleep somewhere else.
  auto during = run(s, t, 1.9, true, true);
  check(count_of(during, ArmAction::HOLD_NEUTRAL) == (int)during.size(),
        "every cycle of the hold writes neutral");
  check(!s.armed(), "still not armed at 1.92 s");

  auto after = run(s, t, 0.5, true, true);
  check(s.armed(), "armed after the hold elapses");
  check(s.arm_cycles() == 1, "one completed arm cycle");
  check(contains(after, ArmAction::DRIVE), "then drives");
}

static void test_hold_duration_is_honoured()
{
  ArmSequencer s(cfg(2.0));
  double t = 100.0;
  s.update(t, true, true);
  t += 0.02;
  int cycles = 0;
  while (!s.armed() && cycles < 1000) { s.update(t, true, true); t += 0.02; ++cycles; }
  const double elapsed = cycles * 0.02;
  check(elapsed >= 1.94 && elapsed <= 2.06,
        "hold is 2 s within one cycle (" + std::to_string(elapsed) + ")");
}

// The tests that matter. Every exit from ARMING or ARMED must
// emit DISARM, because that is where the caller latches the
// channels off. An exit that skips it leaves them running and
// arms the RESTART trap for the next arm.

static void test_withdrawn_request_during_hold_still_disarms()
{
  ArmSequencer s(cfg(2.0));
  double t = 100.0;
  s.update(t, true, true);          // BEGIN_ARM
  t += 0.02;
  run(s, t, 0.5, true, true);       // partway through the hold

  const ArmAction a = s.update(t, /*want=*/false, true);
  check_act(a, ArmAction::DISARM,
            "aborting mid-hold emits DISARM, not a silent return");
  check_state(s.state(), ArmState::DISARMED, "back to disarmed");
  check(s.arm_cycles() == 0, "an aborted hold is not a completed arm cycle");
}

static void test_hardware_failing_during_hold_disarms()
{
  ArmSequencer s(cfg(2.0));
  double t = 100.0;
  s.update(t, true, true);
  t += 0.02;
  run(s, t, 0.5, true, true);

  const ArmAction a = s.update(t, true, /*hardware_ok=*/false);
  check_act(a, ArmAction::DISARM,
            "hardware failing mid-hold cuts the output it already enabled");
  check_state(s.state(), ArmState::DISARMED, "disarmed");
}

static void test_disarm_request_while_armed()
{
  ArmSequencer s(cfg(0.0));         // no hold, for brevity
  double t = 100.0;
  s.update(t, true, true); t += 0.02;
  run(s, t, 0.2, true, true);
  check(s.armed(), "armed");

  check_act(s.update(t, false, true), ArmAction::DISARM, "request -> DISARM");
  check_state(s.state(), ArmState::DISARMED, "disarmed");
  check_act(s.update(t, false, true), ArmAction::IDLE, "then idle");
}

// The RC handset's disarm/arm cycle, mid-mission. This is the
// path that bench-passed 2026-08-14 and that an earlier draft of
// the rewrite deleted.
static void test_full_disarm_rearm_cycle()
{
  ArmSequencer s(cfg(2.0));
  double t = 100.0;

  s.update(t, true, true); t += 0.02;
  run(s, t, 2.2, true, true);
  check(s.armed(), "armed first time");

  check_act(s.update(t, false, true), ArmAction::DISARM, "operator disarms");
  t += 0.02;
  run(s, t, 0.5, false, true);
  check(!s.armed(), "stays disarmed");

  // Re-arm, mid-mission, no relaunch.
  check_act(s.update(t, true, true), ArmAction::BEGIN_ARM, "operator re-arms");
  t += 0.02;
  run(s, t, 2.2, true, true);
  check(s.armed(), "armed again");
  check(s.arm_cycles() == 2, "two completed arm cycles");
}

// A transient bus error must NOT cut the signal and cost a 2 s
// re-arm mid-mission. ActuatorStage already holds the props at
// neutral on HARDWARE_FAULT, which is safe and instantly
// recoverable; disarming here would be neither.
static void test_hardware_blip_while_armed_does_not_disarm()
{
  ArmSequencer s(cfg(0.0));
  double t = 100.0;
  s.update(t, true, true); t += 0.02;
  run(s, t, 0.2, true, true);
  check(s.armed(), "armed");

  const ArmAction a = s.update(t, true, /*hardware_ok=*/false);
  check_act(a, ArmAction::DRIVE,
            "a hardware blip while ARMED does not cut the signal");
  check(s.armed(), "stays armed; ActuatorStage holds neutral instead");
}

static void test_hardware_absent_never_begins_an_arm()
{
  ArmSequencer s(cfg(2.0));
  double t = 100.0;
  auto acts = run(s, t, 3.0, /*want=*/true, /*hardware_ok=*/false);
  check(!contains(acts, ArmAction::BEGIN_ARM),
        "no arm sequence is begun on a board that is not there");
  check(count_of(acts, ArmAction::IDLE) == (int)acts.size(), "idle throughout");
  check_state(s.state(), ArmState::DISARMED, "still disarmed");
}

static void test_skip_hold_still_transits_arming()
{
  // initialize_esc = false. One path to ARMED, always.
  ArmSequencer s(cfg(2.0, /*skip=*/true));
  double t = 100.0;
  check_act(s.update(t, true, true), ArmAction::BEGIN_ARM, "begins");
  check_state(s.state(), ArmState::ARMING, "transits ARMING even when skipping");
  t += 0.02;
  check_act(s.update(t, true, true), ArmAction::HOLD_NEUTRAL, "one hold cycle");
  check(s.armed(), "armed immediately after");
  check(s.arm_cycles() == 1, "counted as an arm cycle");
}

static void test_nonfinite_clock_cannot_complete_a_hold()
{
  ArmSequencer s(cfg(2.0));
  double t = 100.0;
  s.update(t, true, true);
  t += 0.02;
  run(s, t, 0.5, true, true);

  for (int i = 0; i < 500; ++i)
    check_act(s.update(std::nan(""), true, true), ArmAction::HOLD_NEUTRAL,
              "a bad clock holds position rather than advancing");
  check(!s.armed(), "an untrusted clock never completes the hold");
}

static void test_config_validation()
{
  check(cfg().validate().empty(), "sane config validates");
  ArmSequencerConfig c; c.arm_hold_sec = -1.0;
  check(!c.validate().empty(), "negative hold rejected");
  c.arm_hold_sec = std::nan("");
  check(!c.validate().empty(), "NaN hold rejected");
}

int main()
{
  printf("test_arm_sequencer\n");
  test_starts_disarmed_and_idle();
  test_full_arm_takes_the_hold();
  test_hold_duration_is_honoured();
  test_withdrawn_request_during_hold_still_disarms();
  test_hardware_failing_during_hold_disarms();
  test_disarm_request_while_armed();
  test_full_disarm_rearm_cycle();
  test_hardware_blip_while_armed_does_not_disarm();
  test_hardware_absent_never_begins_an_arm();
  test_skip_hold_still_transits_arming();
  test_nonfinite_clock_cannot_complete_a_hold();
  test_config_validation();

  if (g_failures) {
    fprintf(stderr, "\n%d/%d checks FAILED\n", g_failures, g_checks);
    return 1;
  }
  printf("PASS (%d checks)\n", g_checks);
  return 0;
}
