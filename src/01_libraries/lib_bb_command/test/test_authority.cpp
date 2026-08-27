/*************************************************************
 * Unit tests for the authority state machine.
 *
 * This is the safety-critical file in lib_bb_command. Every test
 * below is either a row of design doc section 10's transition
 * table, a requirement from its section 15, or one of the
 * decisions in docs/control_refactor_plan.md section 12.
 *
 * The tests that matter most are the fail-closed ones: a manual
 * owner losing its link must STOP the boat, never hand it to
 * autonomy. An operator whose radio just died is not expressing
 * a preference for autonomous operation.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "authority.h"

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
static void check_eq(StopReason got, StopReason want, const std::string& what)
{
  g_checks++;
  if (got != want) {
    g_failures++;
    fprintf(stderr, "FAIL: %s (got %s want %s)\n",
            what.c_str(), to_string(got), to_string(want));
  }
}
static void check_src(CommandSource got, CommandSource want, const std::string& what)
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
    fprintf(stderr, "FAIL: %s (got %.4f want %.4f)\n", what.c_str(), got, want);
  }
}

//---------------------------------------------------------
// Frame builders

static std::string rc_frame(unsigned long long seq, const char* mode,
                            double surge, double yaw,
                            bool valid = true, double authority = 100.0,
                            const char* epoch = "rc1")
{
  char b[320];
  snprintf(b, sizeof(b),
           "v=1,producer=iRCInterface,epoch=%s,seq=%llu,source_time=%.2f,"
           "valid=%d,surge=%.2f,yaw=%.2f,authority_limit=%.1f,"
           "mode=%s,kill=0,link=VALID,deadman_switch=DISABLED",
           epoch, seq, (double)seq * 0.02, valid ? 1 : 0, surge, yaw,
           authority, mode);
  return b;
}

static std::string teleop_frame(unsigned long long seq, bool claim,
                                double surge, double yaw,
                                bool estop = false, bool valid = true,
                                const char* epoch = "tl1")
{
  char b[320];
  snprintf(b, sizeof(b),
           "v=1,producer=iTeleop,epoch=%s,seq=%llu,source_time=%.2f,"
           "valid=%d,surge=%.2f,yaw=%.2f,session=s1,claim=%d,estop=%d",
           epoch, seq, (double)seq * 0.02, valid ? 1 : 0, surge, yaw,
           claim ? 1 : 0, estop ? 1 : 0);
  return b;
}

static std::string auto_frame(unsigned long long seq, double surge, double yaw,
                              bool valid = true, const char* epoch = "au1",
                              double authority = -1.0)
{
  char b[384];
  // authority < 0 means "omit the field", which is the normal
  // autonomy case. A value is emitted only by the test that
  // proves the arbiter IGNORES it for autonomy -- omitting it
  // would make that test pass vacuously, since an absent
  // authority_limit defaults to 100 and capping at 100 is a
  // no-op. (Found by mutation testing: applying the cap to
  // autonomy passed the whole suite until this was added.)
  if (authority < 0.0) {
    snprintf(b, sizeof(b),
             "v=1,producer=pBBPID,epoch=%s,seq=%llu,source_time=%.2f,"
             "valid=%d,surge=%.2f,yaw=%.2f,controller=pBBPID,"
             "control_mode=HEADING_SPEED",
             epoch, seq, (double)seq * 0.02, valid ? 1 : 0, surge, yaw);
  } else {
    snprintf(b, sizeof(b),
             "v=1,producer=pBBPID,epoch=%s,seq=%llu,source_time=%.2f,"
             "valid=%d,surge=%.2f,yaw=%.2f,authority_limit=%.1f,"
             "controller=pBBPID,control_mode=HEADING_SPEED",
             epoch, seq, (double)seq * 0.02, valid ? 1 : 0, surge, yaw, authority);
  }
  return b;
}

// A rig holding the three mailboxes plus the arbiter, so each
// test reads as a scenario rather than as plumbing.
struct Rig
{
  CommandMailbox rc{CommandSource::RC};
  CommandMailbox teleop{CommandSource::TELEOP};
  CommandMailbox autonomy{CommandSource::AUTONOMY};
  ArbiterConfig cfg;
  AuthorityArbiter arb;
  SafetyInputs safety;

  Rig() : arb(ArbiterConfig(), "arb-test") {}
  explicit Rig(const ArbiterConfig& c) : cfg(c), arb(c, "arb-test") {}

  AuthorityDecision step(double now) {
    return arb.decide(now, rc, teleop, autonomy, safety);
  }
};

//---------------------------------------------------------

static void test_config_validation()
{
  ArbiterConfig c;
  check(c.validate().empty(), "default config validates");
  c.rc_timeout_sec = 0.0;
  check(!c.validate().empty(), "zero timeout rejected");
  c.rc_timeout_sec = std::nan("");
  check(!c.validate().empty(), "NaN timeout rejected");
}

// Startup: nothing has ever been heard from anyone.
static void test_startup_stops()
{
  Rig r;
  AuthorityDecision d = r.step(100.0);
  check(d.hard_stop, "startup is a stop");
  check_src(d.selected_source, CommandSource::NONE, "startup selects nobody");
  check_eq(d.stop_reason, StopReason::AUTONOMY_INVALID,
           "startup falls through to autonomy, which has produced nothing");
  check_near(d.surge, 0.0, 1e-9, "startup surge zero");
  check_near(d.yaw, 0.0, 1e-9, "startup yaw zero");
  check(d.decision_seq == 1, "decision_seq starts at 1");
}

static void test_autonomy_drives_when_nobody_else_wants_it()
{
  Rig r;
  r.autonomy.accept(auto_frame(1, 30.0, -10.0), 100.0);
  AuthorityDecision d = r.step(100.05);
  check(!d.hard_stop, "autonomy drives");
  check_src(d.selected_source, CommandSource::AUTONOMY, "autonomy selected");
  check_near(d.surge, 30.0, 1e-9, "surge passed through");
  check_near(d.yaw, -10.0, 1e-9, "yaw passed through");
  check(d.source_producer == "pBBPID", "lineage producer copied");
  check(d.source_seq == 1, "lineage seq copied");
}

// Design doc 10: autonomy -> fresh valid RC manual request -> RC.
static void test_rc_takes_over_from_autonomy()
{
  Rig r;
  r.autonomy.accept(auto_frame(1, 30.0, 0.0), 100.0);
  AuthorityDecision a = r.step(100.05);
  check_src(a.selected_source, CommandSource::AUTONOMY, "autonomy first");

  r.rc.accept(rc_frame(1, "MANUAL", 60.0, 20.0), 100.10);
  AuthorityDecision b = r.step(100.12);
  check_src(b.selected_source, CommandSource::RC, "RC takes the boat");
  check(!b.hard_stop, "no stop on takeover");
  check(b.authority_changed, "authority change flagged");
  check_near(b.surge, 60.0, 1e-9, "RC surge");
  check(b.source_producer == "iRCInterface", "lineage switched to RC");
}

// Invariant 3: sticks alone are not a request for authority.
static void test_rc_sticks_without_manual_mode_do_not_take_authority()
{
  Rig r;
  r.autonomy.accept(auto_frame(1, 25.0, 0.0), 100.0);
  // Full deflection, but the mode switch is in AUTO.
  r.rc.accept(rc_frame(1, "NON_MANUAL", 100.0, 100.0), 100.01);
  AuthorityDecision d = r.step(100.02);
  check_src(d.selected_source, CommandSource::AUTONOMY,
            "a knocked stick in AUTO does not steal the boat");
  check_near(d.surge, 25.0, 1e-9, "autonomy still driving");
  check(!d.rc.requesting_authority, "RC not requesting");
}

// Invariant 6, and the most important test in this file.
static void test_rc_loss_in_manual_stops_rather_than_falling_through()
{
  Rig r;
  r.autonomy.accept(auto_frame(1, 40.0, 0.0), 100.0);
  r.rc.accept(rc_frame(1, "MANUAL", 50.0, 0.0), 100.0);
  check_src(r.step(100.01).selected_source, CommandSource::RC, "RC has the boat");

  // The handset dies. Autonomy keeps publishing perfectly good
  // commands the whole time.
  for (int i = 2; i < 200; ++i)
    r.autonomy.accept(auto_frame(i, 40.0, 0.0), 100.0 + i * 0.05);

  AuthorityDecision d = r.step(101.5);   // well past rc_timeout 1.0
  check(d.hard_stop, "RC stale in MANUAL is a STOP");
  check(d.fail_closed, "flagged as fail-closed");
  check_eq(d.stop_reason, StopReason::RC_STALE, "reason is RC_STALE");
  check_src(d.selected_source, CommandSource::NONE,
            "and NOT a silent handover to autonomy");
  check_near(d.surge, 0.0, 1e-9, "surge zeroed");
  check(d.autonomy.fresh && d.autonomy.valid,
        "autonomy was fresh and valid and still did not get the boat");
}

static void test_rc_invalid_in_manual_stops()
{
  Rig r;
  r.rc.accept(rc_frame(1, "MANUAL", 50.0, 0.0, /*valid=*/false), 100.0);
  AuthorityDecision d = r.step(100.01);
  check(d.hard_stop, "RC invalid in MANUAL stops");
  check_eq(d.stop_reason, StopReason::RC_INVALID, "reason is RC_INVALID");
  check(d.fail_closed, "fail-closed flagged");
}

// Design doc 10: RC leaves manual, teleop has an active claim.
static void test_teleop_takes_over_when_rc_leaves_manual()
{
  Rig r;
  r.rc.accept(rc_frame(1, "MANUAL", 50.0, 0.0), 100.0);
  r.teleop.accept(teleop_frame(1, /*claim=*/true, 20.0, 5.0), 100.0);
  check_src(r.step(100.01).selected_source, CommandSource::RC, "RC outranks teleop");

  r.rc.accept(rc_frame(2, "NON_MANUAL", 0.0, 0.0), 100.05);
  AuthorityDecision d = r.step(100.06);
  check_src(d.selected_source, CommandSource::TELEOP, "teleop takes over");
  check_near(d.surge, 20.0, 1e-9, "teleop surge");
}

static void test_teleop_claim_required()
{
  Rig r;
  r.autonomy.accept(auto_frame(1, 15.0, 0.0), 100.0);
  // Valid teleop command, but no claim: presence is not a claim.
  r.teleop.accept(teleop_frame(1, /*claim=*/false, 90.0, 0.0), 100.0);
  AuthorityDecision d = r.step(100.01);
  check_src(d.selected_source, CommandSource::AUTONOMY,
            "an unclaimed teleop command does not take the boat");
}

static void test_teleop_stale_with_claim_stops()
{
  Rig r;
  r.autonomy.accept(auto_frame(1, 15.0, 0.0), 100.0);
  r.teleop.accept(teleop_frame(1, true, 20.0, 0.0), 100.0);
  check_src(r.step(100.01).selected_source, CommandSource::TELEOP, "teleop driving");

  for (int i = 2; i < 100; ++i)
    r.autonomy.accept(auto_frame(i, 15.0, 0.0), 100.0 + i * 0.05);

  AuthorityDecision d = r.step(102.0);   // past teleop_timeout 1.5
  check(d.hard_stop, "hung iTeleop with a latched claim stops the boat");
  check_eq(d.stop_reason, StopReason::TELEOP_STALE, "reason is TELEOP_STALE");
  check_src(d.selected_source, CommandSource::NONE, "no fallthrough to autonomy");
  check(d.fail_closed, "fail-closed flagged");
}

static void test_teleop_estop()
{
  Rig r;
  r.teleop.accept(teleop_frame(1, true, 50.0, 0.0, /*estop=*/true), 100.0);
  AuthorityDecision d = r.step(100.01);
  check(d.hard_stop, "teleop estop stops");
  check_eq(d.stop_reason, StopReason::TELEOP_ESTOP, "reason is TELEOP_ESTOP");
}

// Design doc 10: explicit claim release lets autonomy resume.
static void test_teleop_release_returns_to_autonomy()
{
  Rig r;
  r.autonomy.accept(auto_frame(1, 15.0, 0.0), 100.0);
  r.teleop.accept(teleop_frame(1, true, 20.0, 0.0), 100.0);
  check_src(r.step(100.01).selected_source, CommandSource::TELEOP, "teleop driving");

  r.teleop.accept(teleop_frame(2, /*claim=*/false, 0.0, 0.0), 100.05);
  r.autonomy.accept(auto_frame(2, 15.0, 0.0), 100.05);
  AuthorityDecision d = r.step(100.06);
  check_src(d.selected_source, CommandSource::AUTONOMY, "autonomy resumes on release");
  check(!d.hard_stop, "release is not a stop");
}

// ALL_STOP gates autonomy only -- it must never paralyse a rescue.
static void test_all_stop_blocks_autonomy_but_not_rc()
{
  Rig r;
  r.autonomy.accept(auto_frame(1, 40.0, 0.0), 100.0);
  r.safety.autonomy_all_stop = true;

  AuthorityDecision a = r.step(100.01);
  check(a.hard_stop, "ALL_STOP stops autonomy");
  check_eq(a.stop_reason, StopReason::AUTONOMY_ALL_STOP, "reason is ALL_STOP");

  // Operator grabs the boat. ALL_STOP is still latched.
  r.rc.accept(rc_frame(1, "MANUAL", 70.0, 10.0), 100.05);
  AuthorityDecision b = r.step(100.06);
  check_src(b.selected_source, CommandSource::RC,
            "RC rescue works with ALL_STOP latched");
  check(!b.hard_stop, "rescue is not blocked");
  check_near(b.surge, 70.0, 1e-9, "full RC authority during rescue");
}

// Decision (b): kill is enforced at the Navigator. The arbiter
// records it and keeps deciding, so the log still explains what
// the command path was doing while the hardware sat neutral.
static void test_rc_kill_is_trace_only_here()
{
  Rig r;
  r.rc.accept(rc_frame(1, "MANUAL", 60.0, 0.0), 100.0);
  r.safety.rc_kill_asserted = true;

  AuthorityDecision d = r.step(100.01);
  check(d.rc_kill_observed, "kill recorded for the trace");
  check_src(d.selected_source, CommandSource::RC,
            "arbiter does not gate on kill -- the Navigator enforces it");
  check(!d.hard_stop, "no arbiter-level stop on kill");
}

// Decision (e): the pre-first-frame window must not strand a boat
// launched without a handset.
static void test_autonomy_allowed_before_any_rc_frame()
{
  Rig r;
  r.autonomy.accept(auto_frame(1, 20.0, 0.0), 100.0);
  AuthorityDecision d = r.step(100.01);
  check_src(d.selected_source, CommandSource::AUTONOMY,
            "no RC ever seen is not-manual, autonomy proceeds");
  check(!d.rc.ever_seen, "RC genuinely never seen");
}

static void test_autonomy_can_be_gated_before_first_rc()
{
  ArbiterConfig c;
  c.allow_autonomy_before_first_rc = false;
  Rig r(c);
  r.autonomy.accept(auto_frame(1, 20.0, 0.0), 100.0);
  AuthorityDecision d = r.step(100.01);
  check(d.hard_stop, "gated deployment refuses to run without a handset");
  check_eq(d.stop_reason, StopReason::STARTUP, "reason is STARTUP");

  // One valid RC frame in AUTO is enough to unblock it forever.
  r.rc.accept(rc_frame(1, "NON_MANUAL", 0.0, 0.0), 100.02);
  r.autonomy.accept(auto_frame(2, 20.0, 0.0), 100.02);
  AuthorityDecision e = r.step(100.03);
  check_src(e.selected_source, CommandSource::AUTONOMY,
            "one RC frame unblocks autonomy");
}

// The manual authority cap is an operator affordance, not a
// mission limiter.
static void test_authority_limit_applies_to_manual_only()
{
  Rig r;
  r.rc.accept(rc_frame(1, "MANUAL", 100.0, 50.0, true, /*authority=*/40.0), 100.0);
  AuthorityDecision a = r.step(100.01);
  check_near(a.surge, 40.0, 1e-9, "RC surge scaled by authority_limit");
  check_near(a.yaw,   20.0, 1e-9, "RC yaw scaled by authority_limit");

  // Autonomy carrying an explicit authority_limit. The arbiter
  // must IGNORE it: the cap is an operator affordance for manual
  // driving, and a mission is either permitted to run or it is
  // not. Without a value here this assertion is vacuous.
  r.rc.accept(rc_frame(2, "NON_MANUAL", 0.0, 0.0), 100.05);
  r.autonomy.accept(auto_frame(1, 100.0, 50.0, true, "au1", /*authority=*/40.0),
                    100.05);
  AuthorityDecision b = r.step(100.06);
  check_near(b.surge, 100.0, 1e-9,
             "autonomy is never capped, even when it sends an authority_limit");
  check_near(b.yaw,    50.0, 1e-9, "autonomy yaw uncapped");
}

// A repeated broker frame must not keep a dead controller alive.
static void test_duplicate_autonomy_frames_do_not_hold_authority()
{
  Rig r;
  r.autonomy.accept(auto_frame(7, 30.0, 0.0), 100.0);
  check_src(r.step(100.01).selected_source, CommandSource::AUTONOMY, "driving");

  // Controller dies; broker keeps re-sending the same sequence.
  for (int i = 1; i <= 60; ++i)
    r.autonomy.accept(auto_frame(7, 30.0, 0.0), 100.0 + i * 0.05);

  AuthorityDecision d = r.step(103.0);   // past autonomy_timeout 2.0
  check(d.hard_stop, "a repeating sequence goes stale on schedule");
  check_eq(d.stop_reason, StopReason::AUTONOMY_STALE, "reason is AUTONOMY_STALE");
}

static void test_autonomy_epoch_restart()
{
  Rig r;
  r.autonomy.accept(auto_frame(50000, 30.0, 0.0), 100.0);
  check_src(r.step(100.01).selected_source, CommandSource::AUTONOMY, "driving");

  // Back seat reboots: new epoch, sequence restarts at 1.
  r.autonomy.accept(auto_frame(1, 25.0, 0.0, true, "au2"), 101.0);
  AuthorityDecision d = r.step(101.01);
  check_src(d.selected_source, CommandSource::AUTONOMY, "new epoch accepted");
  check(d.source_epoch == "au2", "lineage carries the new epoch");
  check_near(d.surge, 25.0, 1e-9, "new epoch's command is used");
}

static void test_stop_clears_and_selection_resumes()
{
  Rig r;
  r.rc.accept(rc_frame(1, "MANUAL", 50.0, 0.0), 100.0);
  check(!r.step(100.01).hard_stop, "driving");

  AuthorityDecision stopped = r.step(102.0);   // RC went stale
  check(stopped.hard_stop, "stopped");

  // Operator's link comes back.
  r.rc.accept(rc_frame(2, "MANUAL", 30.0, 0.0), 102.1);
  AuthorityDecision resumed = r.step(102.11);
  check(!resumed.hard_stop, "selection resumes when the stop condition clears");
  check_src(resumed.selected_source, CommandSource::RC, "RC drives again");
  check(resumed.stop_reason_changed, "stop-reason transition flagged");
}

static void test_decision_sequence_advances_every_cycle()
{
  Rig r;
  uint64_t last = 0;
  for (int i = 0; i < 25; ++i) {
    AuthorityDecision d = r.step(100.0 + i * 0.02);
    check(d.decision_seq == last + 1, "decision_seq advances on every cycle");
    last = d.decision_seq;
  }
  // Including hard-stop cycles -- the trace must be gapless, or a
  // log reader cannot tell a stopped arbiter from a dead one.
  check(last == 25, "25 cycles produced 25 sequence numbers");
}

static void test_serialisation_round_trips_key_fields()
{
  Rig r;
  r.rc.accept(rc_frame(42, "MANUAL", 12.5, -7.25), 100.0);
  AuthorityDecision d = r.step(100.01);
  const std::string wire = serialize_decision(d);
  check(wire.find("selected=RC") != std::string::npos, "wire names the source");
  check(wire.find("stop=NONE") != std::string::npos, "wire names the stop reason");
  check(wire.find("source_seq=42") != std::string::npos, "wire carries lineage seq");
  check(wire.find("hard_stop=0") != std::string::npos, "wire carries stop flag");
}

int main()
{
  printf("test_authority\n");
  test_config_validation();
  test_startup_stops();
  test_autonomy_drives_when_nobody_else_wants_it();
  test_rc_takes_over_from_autonomy();
  test_rc_sticks_without_manual_mode_do_not_take_authority();
  test_rc_loss_in_manual_stops_rather_than_falling_through();
  test_rc_invalid_in_manual_stops();
  test_teleop_takes_over_when_rc_leaves_manual();
  test_teleop_claim_required();
  test_teleop_stale_with_claim_stops();
  test_teleop_estop();
  test_teleop_release_returns_to_autonomy();
  test_all_stop_blocks_autonomy_but_not_rc();
  test_rc_kill_is_trace_only_here();
  test_autonomy_allowed_before_any_rc_frame();
  test_autonomy_can_be_gated_before_first_rc();
  test_authority_limit_applies_to_manual_only();
  test_duplicate_autonomy_frames_do_not_hold_authority();
  test_autonomy_epoch_restart();
  test_stop_clears_and_selection_resumes();
  test_decision_sequence_advances_every_cycle();
  test_serialisation_round_trips_key_fields();

  if (g_failures) {
    fprintf(stderr, "\n%d/%d checks FAILED\n", g_failures, g_checks);
    return 1;
  }
  printf("PASS (%d checks)\n", g_checks);
  return 0;
}
