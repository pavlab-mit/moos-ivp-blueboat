/*************************************************************
 * Unit tests for ActuatorStage -- the last safety word before
 * the water.
 *
 * The ordering tests matter most. Local safety is evaluated
 * BEFORE the command, so a kill or a hardware fault cannot be
 * masked by a perfectly good command arriving in the same cycle.
 * Every "X wins over a valid command" test below is pinning that.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "actuator_stage.h"

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
static void check_stop(StopReason got, StopReason want, const std::string& what)
{
  g_checks++;
  if (got != want) {
    g_failures++;
    fprintf(stderr, "FAIL: %s (got %s want %s)\n",
            what.c_str(), to_string(got), to_string(want));
  }
}
static void check_near(double got, double want, double tol, const std::string& what)
{
  g_checks++;
  if (std::fabs(got - want) > tol) {
    g_failures++;
    fprintf(stderr, "FAIL: %s (got %.3f want %.3f)\n", what.c_str(), got, want);
  }
}

static ActuatorConfig good_cfg()
{
  ActuatorConfig c;
  c.left.reversed  = true;    // SERVO3
  c.right.reversed = false;   // SERVO1
  return c;
}

// A healthy, armed, un-killed boat with a live RC link.
static NavigatorSafetyState healthy()
{
  NavigatorSafetyState s;
  s.pwm_armed        = true;
  s.hardware_healthy = true;
  s.shutdown_requested = false;
  s.rc_kill_asserted = false;
  s.rc_link_lost     = false;
  return s;
}

static std::string mixed(unsigned long long seq, double l, double r,
                         bool hard_stop = false, const char* stop = "NONE",
                         const char* epoch = "mix1")
{
  char b[512];
  snprintf(b, sizeof(b),
           "v=1,mixer_model=ARDUROVER_4_7_SKID,mix_epoch=%s,mix_seq=%llu,"
           "mix_time=%.3f,decision_epoch=arb1,decision_seq=%llu,"
           "selected=RC,hard_stop=%d,stop=%s,"
           "source_producer=iRCInterface,source_epoch=rc1,source_seq=%llu,"
           "surge_in=0,yaw_in=0,surge_shaped=0,yaw_shaped=0,"
           "left_effort=%.3f,right_effort=%.3f,q=0,f=0,limits=0000",
           epoch, seq, (double)seq * 0.02, seq, hard_stop ? 1 : 0, stop, seq, l, r);
  return b;
}

//---------------------------------------------------------

static void test_config_validation()
{
  check(good_cfg().validate().empty(), "sane config validates");
  ActuatorConfig c = good_cfg();
  c.left.min_us = 1600;                       // min > trim
  check(!c.validate().empty(), "bad ESC endpoints rejected");
  ActuatorConfig d = good_cfg();
  d.rc_deadman_enabled = true;
  d.rc_deadman_timeout_sec = 0.0;
  check(!d.validate().empty(), "deadman with no timeout rejected");
}

static void test_normal_drive()
{
  ActuatorStage a(good_cfg());
  MixedMailbox mb;
  mb.accept(mixed(1, 100.0, 100.0), 100.0);

  ActuatorFrame f = a.update(100.01, mb, healthy(), 100.0);
  check(!f.neutral, "healthy boat with a fresh command drives");
  check_stop(f.stop_reason, StopReason::NONE, "no stop reason");
  // Physical forward on both sides; the pulses differ because the
  // left channel is electrically reversed. That asymmetry IS the
  // wiring and must appear only here.
  check_near(f.left_pwm_us,  1100.0, 1e-6, "left full forward (reversed)");
  check_near(f.right_pwm_us, 1900.0, 1e-6, "right full forward");
  check(!f.local_stop, "not a local stop");
}

static void test_lineage_survives_even_a_stop()
{
  ActuatorStage a(good_cfg());
  MixedMailbox mb;
  mb.accept(mixed(77, 0.0, 0.0, true, "RC_STALE"), 100.0);

  ActuatorFrame f = a.update(100.01, mb, healthy(), 100.0);
  check(f.neutral, "stopped");
  check_stop(f.stop_reason, StopReason::RC_STALE, "upstream reason propagated");
  check(f.source_seq == 77, "source lineage present on a STOPPED frame");
  check(f.mix_seq == 77, "mix lineage present");
  check(f.decision_seq == 77, "decision lineage present");
  check(f.source_producer == "iRCInterface", "producer named");
}

// --- local safety beats a perfectly good command -----------

static void test_kill_beats_a_valid_command()
{
  ActuatorStage a(good_cfg());
  MixedMailbox mb;
  mb.accept(mixed(1, 80.0, 80.0), 100.0);

  NavigatorSafetyState s = healthy();
  s.rc_kill_asserted = true;

  ActuatorFrame f = a.update(100.01, mb, s, 100.0);
  check(f.neutral, "kill neutralises");
  check_stop(f.stop_reason, StopReason::RC_KILL, "reason is RC_KILL");
  check(f.local_stop, "flagged as a local stop");
  check_near(f.left_pwm_us, 0.0, 1e-9, "no pulse computed for a stopped frame");
}

static void test_disarmed_beats_a_valid_command()
{
  ActuatorStage a(good_cfg());
  MixedMailbox mb;
  mb.accept(mixed(1, 80.0, 80.0), 100.0);
  NavigatorSafetyState s = healthy();
  s.pwm_armed = false;
  ActuatorFrame f = a.update(100.01, mb, s, 100.0);
  check_stop(f.stop_reason, StopReason::PWM_DISARMED, "disarmed wins");
}

static void test_hardware_fault_beats_a_valid_command()
{
  ActuatorStage a(good_cfg());
  MixedMailbox mb;
  mb.accept(mixed(1, 80.0, 80.0), 100.0);
  NavigatorSafetyState s = healthy();
  s.hardware_healthy = false;
  ActuatorFrame f = a.update(100.01, mb, s, 100.0);
  check_stop(f.stop_reason, StopReason::HARDWARE_FAULT, "hardware fault wins");
}

static void test_shutdown_is_the_highest_priority()
{
  ActuatorStage a(good_cfg());
  MixedMailbox mb;
  mb.accept(mixed(1, 80.0, 80.0), 100.0);
  NavigatorSafetyState s = healthy();
  s.shutdown_requested = true;
  s.rc_kill_asserted   = true;   // both asserted
  s.hardware_healthy   = false;
  ActuatorFrame f = a.update(100.01, mb, s, 100.0);
  check_stop(f.stop_reason, StopReason::SHUTDOWN,
             "shutdown outranks every other local stop");
}

// The ordering test that actually bites.
//
// Every other local-safety test pairs the fault with a VALID
// command, and under that pairing both orderings return the same
// reason -- which is why moving the local block after the
// upstream block passed the whole suite until this was added.
// The ordering is only observable when BOTH are failing at once.
static void test_local_reason_wins_over_a_simultaneous_upstream_fault()
{
  ActuatorStage a(good_cfg());
  MixedMailbox mb;
  mb.accept(mixed(1, 80.0, 80.0), 100.0);

  NavigatorSafetyState s = healthy();
  s.rc_kill_asserted = true;

  // Kill asserted AND the mixer has gone quiet. Both are true;
  // only one is the cause.
  ActuatorFrame f = a.update(101.0, mb, s, 101.0);   // timeout 0.5
  check(f.neutral, "stopped either way");
  check_stop(f.stop_reason, StopReason::RC_KILL,
             "the operator's action is reported, not the downstream symptom");
  check(f.local_stop, "flagged local");

  // Same for a hardware fault racing a stale input.
  NavigatorSafetyState h = healthy();
  h.hardware_healthy = false;
  ActuatorFrame g = a.update(101.0, mb, h, 101.0);
  check_stop(g.stop_reason, StopReason::HARDWARE_FAULT,
             "hardware fault reported over a simultaneous stale input");
}

// --- the opt-in deadman (plan decision (d)) ----------------

static void test_deadman_off_by_default()
{
  ActuatorStage a(good_cfg());          // deadman disabled
  MixedMailbox mb;
  mb.accept(mixed(1, 50.0, 50.0), 100.0);
  NavigatorSafetyState s = healthy();
  s.rc_link_lost = true;                // handset gone

  ActuatorFrame f = a.update(100.01, mb, s, 0.0);
  check(!f.neutral,
        "with the deadman off, RC link loss does NOT stop an autonomous boat");
}

static void test_deadman_stops_regardless_of_mode()
{
  ActuatorConfig c = good_cfg();
  c.rc_deadman_enabled = true;
  c.rc_deadman_timeout_sec = 2.0;
  ActuatorStage a(c);

  MixedMailbox mb;
  // AUTONOMY is driving -- the deadman must stop it anyway.
  std::string m = mixed(1, 50.0, 50.0);
  size_t p = m.find("selected=RC");
  m.replace(p, 11, "selected=AUTONOMY");
  mb.accept(m, 100.0);

  NavigatorSafetyState s = healthy();
  s.rc_link_lost = true;

  ActuatorFrame f = a.update(100.01, mb, s, 100.0);
  check(f.neutral, "enabled deadman stops the boat");
  check_stop(f.stop_reason, StopReason::RC_DEADMAN, "reason is RC_DEADMAN");
  check(f.local_stop, "local stop");
}

static void test_deadman_trips_on_age_not_only_on_the_flag()
{
  ActuatorConfig c = good_cfg();
  c.rc_deadman_enabled = true;
  c.rc_deadman_timeout_sec = 2.0;
  ActuatorStage a(c);
  MixedMailbox mb;
  mb.accept(mixed(1, 50.0, 50.0), 100.0);

  NavigatorSafetyState s = healthy();   // link_lost false...
  // ...but nothing good has been heard for 5 s. A wedged reader
  // that stopped updating the flag must not defeat the deadman.
  ActuatorFrame f = a.update(105.0, mb, s, 100.0);
  check_stop(f.stop_reason, StopReason::RC_DEADMAN,
             "stale RC observation trips the deadman even with the flag clear");
}

static void test_deadman_never_seen_rc_fails_closed()
{
  ActuatorConfig c = good_cfg();
  c.rc_deadman_enabled = true;
  ActuatorStage a(c);
  MixedMailbox mb;
  mb.accept(mixed(1, 50.0, 50.0), 100.0);
  NavigatorSafetyState s = healthy();
  ActuatorFrame f = a.update(100.01, mb, s, /*never*/ 0.0);
  check_stop(f.stop_reason, StopReason::RC_DEADMAN,
             "deadman enabled but RC never seen -> stop, not drive");
}

// --- upstream failures, three distinct reasons -------------

static void test_upstream_failures_are_distinguishable()
{
  {
    ActuatorStage a(good_cfg());
    MixedMailbox mb;                    // nothing ever accepted
    ActuatorFrame f = a.update(100.0, mb, healthy(), 100.0);
    check_stop(f.stop_reason, StopReason::NAV_INPUT_INVALID,
               "never heard from the mixer");
  }
  {
    ActuatorStage a(good_cfg());
    MixedMailbox mb;
    mb.accept(mixed(1, 50.0, 50.0), 100.0);
    ActuatorFrame f = a.update(101.0, mb, healthy(), 101.0);  // timeout 0.5
    check_stop(f.stop_reason, StopReason::NAV_INPUT_STALE,
               "mixer went quiet");
  }
  {
    ActuatorStage a(good_cfg());
    MixedMailbox mb;
    mb.accept(mixed(1, 0.0, 0.0, true, "AUTONOMY_ALL_STOP"), 100.0);
    ActuatorFrame f = a.update(100.01, mb, healthy(), 100.0);
    check_stop(f.stop_reason, StopReason::AUTONOMY_ALL_STOP,
               "the mixer's reason is propagated, not overwritten");
  }
}

static void test_duplicate_mixed_frames_go_stale()
{
  ActuatorStage a(good_cfg());
  MixedMailbox mb;
  mb.accept(mixed(9, 40.0, 40.0), 100.0);
  check(!a.update(100.01, mb, healthy(), 100.0).neutral, "driving");

  for (int i = 1; i <= 40; ++i)
    mb.accept(mixed(9, 40.0, 40.0), 100.0 + i * 0.02);   // same seq

  ActuatorFrame f = a.update(100.9, mb, healthy(), 100.9);
  check_stop(f.stop_reason, StopReason::NAV_INPUT_STALE,
             "a repeating mix sequence does not keep the props turning");
}

// --- the TTL the PWM writer enforces -----------------------

static void test_frame_carries_a_ttl()
{
  ActuatorConfig c = good_cfg();
  c.actuator_frame_ttl_sec = 0.25;
  ActuatorStage a(c);
  MixedMailbox mb;
  mb.accept(mixed(1, 50.0, 50.0), 100.0);

  ActuatorFrame f = a.update(100.01, mb, healthy(), 100.0);
  check_near(f.committed_at, 100.01, 1e-9, "committed_at stamped");
  check_near(f.expires_at, 100.26, 1e-9, "expires_at = committed + ttl");
  check(f.expires_at > f.committed_at, "ttl is positive");
}

static void test_contradictory_mixed_frame_rejected()
{
  // hard_stop=1 with nonzero effort never reaches the stage: the
  // parser refuses it. This is the last parse before the water.
  MixedMailbox mb;
  const std::string bad = mixed(1, 50.0, 50.0, true, "RC_STALE");
  check(mb.accept(bad, 100.0) == AcceptResult::REJECTED,
        "stop-with-effort rejected at the mailbox");
  check(!mb.has_mixed(), "and never becomes the snapshot");
}

static void test_trace_carries_the_whole_chain()
{
  ActuatorStage a(good_cfg());
  MixedMailbox mb;
  mb.accept(mixed(31, 25.0, 25.0), 100.0);
  const std::string t = serialize_actuator_trace(a.update(100.01, mb, healthy(), 100.0));

  check(t.find("source_seq=31") != std::string::npos, "trace has source seq");
  check(t.find("decision_seq=31") != std::string::npos, "trace has decision seq");
  check(t.find("mix_seq=31") != std::string::npos, "trace has mix seq");
  check(t.find("left_pwm_us=") != std::string::npos, "trace has the pulse");
  check(t.find("stop=NONE") != std::string::npos, "trace has the stop reason");
}

int main()
{
  printf("test_actuator_stage\n");
  test_config_validation();
  test_normal_drive();
  test_lineage_survives_even_a_stop();
  test_kill_beats_a_valid_command();
  test_disarmed_beats_a_valid_command();
  test_hardware_fault_beats_a_valid_command();
  test_shutdown_is_the_highest_priority();
  test_local_reason_wins_over_a_simultaneous_upstream_fault();
  test_deadman_off_by_default();
  test_deadman_stops_regardless_of_mode();
  test_deadman_trips_on_age_not_only_on_the_flag();
  test_deadman_never_seen_rc_fails_closed();
  test_upstream_failures_are_distinguishable();
  test_duplicate_mixed_frames_go_stale();
  test_frame_carries_a_ttl();
  test_contradictory_mixed_frame_rejected();
  test_trace_carries_the_whole_chain();

  if (g_failures) {
    fprintf(stderr, "\n%d/%d checks FAILED\n", g_failures, g_checks);
    return 1;
  }
  printf("PASS (%d checks)\n", g_checks);
  return 0;
}
