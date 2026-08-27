#include "scenarios.h"

#include "actuator_stage.h"
#include "arm_sequencer.h"
#include "authority.h"
#include "mixer_stage.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace bb { namespace scenarios {

namespace {

// ---- wire builders --------------------------------------

std::string rc_frame(unsigned long long seq, const char* mode,
                     double surge, double yaw, bool valid = true,
                     const char* epoch = "rc1")
{
  char b[320];
  snprintf(b, sizeof(b),
           "v=1,producer=iRCInterface,epoch=%s,seq=%llu,source_time=%.3f,"
           "valid=%d,surge=%.2f,yaw=%.2f,authority_limit=100,"
           "mode=%s,kill=0,link=VALID,deadman_switch=DISABLED",
           epoch, seq, (double)seq * 0.02, valid ? 1 : 0, surge, yaw, mode);
  return b;
}

std::string auto_frame(unsigned long long seq, double surge, double yaw,
                       bool valid = true, const char* epoch = "au1")
{
  char b[320];
  snprintf(b, sizeof(b),
           "v=1,producer=pBBPID,epoch=%s,seq=%llu,source_time=%.3f,"
           "valid=%d,surge=%.2f,yaw=%.2f,controller=pBBPID,"
           "control_mode=HEADING_SPEED",
           epoch, seq, (double)seq * 0.02, valid ? 1 : 0, surge, yaw);
  return b;
}

std::string teleop_frame(unsigned long long seq, bool claim, double surge,
                         double yaw, bool estop = false)
{
  char b[320];
  snprintf(b, sizeof(b),
           "v=1,producer=iTeleop,epoch=tl1,seq=%llu,source_time=%.3f,"
           "valid=1,surge=%.2f,yaw=%.2f,session=s1,claim=%d,estop=%d",
           seq, (double)seq * 0.02, surge, yaw, claim ? 1 : 0, estop ? 1 : 0);
  return b;
}

// ---- the pipeline under test ----------------------------
//
// The REAL objects, wired exactly as the three apps wire them.
// If this diverges from the apps it stops proving anything, so
// the construction is deliberately boring.

struct Pipeline
{
  ArbiterConfig  arb_cfg;
  MixerConfig    mix_cfg;
  ActuatorConfig act_cfg;

  CommandMailbox rc{CommandSource::RC};
  CommandMailbox teleop{CommandSource::TELEOP};
  CommandMailbox autonomy{CommandSource::AUTONOMY};
  DecisionMailbox selected;
  MixedMailbox    mixed;

  AuthorityArbiter *arb = nullptr;
  MixerStage       *mix = nullptr;
  ActuatorStage    *act = nullptr;

  SafetyInputs          arb_safety;
  NavigatorSafetyState  nav_safety;
  double last_rc_good = 0.0;

  Pipeline()
  {
    mix_cfg.mixer_model = "ARDUROVER_4_7_SKID";
    act_cfg.left.reversed  = true;
    act_cfg.right.reversed = false;
    arb = new AuthorityArbiter(arb_cfg, "arbS");
    mix = new MixerStage(mix_cfg, "mixS");
    act = new ActuatorStage(act_cfg);

    nav_safety.pwm_armed = true;
    nav_safety.hardware_healthy = true;
    nav_safety.rc_link_lost = false;
  }
  ~Pipeline() { delete arb; delete mix; delete act; }

  // One 50 Hz cycle through all three stages, over the wire at
  // every boundary -- serialise and re-parse, exactly as MOOS
  // would. A structure passed by reference would skip the very
  // parsers this is meant to exercise.
  ActuatorFrame step(double now, double dt = 0.02)
  {
    const AuthorityDecision d =
        arb->decide(now, rc, teleop, autonomy, arb_safety);
    selected.accept(serialize_decision(d), now);

    const MixedCommand m = mix->update(now, dt, selected);
    mixed.accept(serialize_mixed(m), now);

    return act->update(now, mixed, nav_safety, last_rc_good);
  }
};

Result pass(const std::string& n) { Result r; r.name = n; r.passed = true; return r; }
Result fail(const std::string& n, const std::string& d)
{ Result r; r.name = n; r.passed = false; r.detail = d; return r; }

Result expect_stop(const std::string& name, const ActuatorFrame& f,
                   StopReason want)
{
  if (f.stop_reason == want && f.neutral) return pass(name);
  char b[256];
  snprintf(b, sizeof(b), "got stop=%s neutral=%d, expected stop=%s neutral=1",
           to_string(f.stop_reason), f.neutral ? 1 : 0, to_string(want));
  return fail(name, b);
}

Result expect_driving(const std::string& name, const ActuatorFrame& f)
{
  if (!f.neutral && f.stop_reason == StopReason::NONE) return pass(name);
  char b[256];
  snprintf(b, sizeof(b), "expected driving, got stop=%s neutral=%d",
           to_string(f.stop_reason), f.neutral ? 1 : 0);
  return fail(name, b);
}

// ---- scenarios ------------------------------------------

Result sc_autonomy_drives(bool)
{
  Pipeline p;
  double t = 100.0;
  ActuatorFrame f;
  for (int i = 1; i <= 60; ++i) {
    p.autonomy.accept(auto_frame(i, 60.0, 0.0), t);
    f = p.step(t);
    t += 0.02;
  }
  return expect_driving("autonomy drives end to end", f);
}

// The one the whole contract exists for.
Result sc_rc_stale_in_manual_does_not_fall_through(bool)
{
  Pipeline p;
  double t = 100.0;
  for (int i = 1; i <= 30; ++i) {
    p.autonomy.accept(auto_frame(i, 40.0, 0.0), t);
    p.rc.accept(rc_frame(i, "MANUAL", 70.0, 10.0), t);
    p.step(t);
    t += 0.02;
  }
  // RC dies. Autonomy keeps publishing perfectly good commands.
  ActuatorFrame f;
  for (int i = 31; i <= 200; ++i) {
    p.autonomy.accept(auto_frame(i, 40.0, 0.0), t);
    f = p.step(t);
    t += 0.02;
  }
  if (!f.neutral)
    return fail("RC stale in MANUAL stops", "boat still driving");
  if (f.stop_reason != StopReason::RC_STALE)
    return fail("RC stale in MANUAL stops",
                std::string("stop=") + to_string(f.stop_reason) +
                " -- expected RC_STALE, NOT a handover to autonomy");
  return pass("RC stale in MANUAL stops, no fallthrough to autonomy");
}

Result sc_teleop_stale_with_claim(bool)
{
  Pipeline p;
  double t = 100.0;
  for (int i = 1; i <= 30; ++i) {
    p.autonomy.accept(auto_frame(i, 30.0, 0.0), t);
    p.teleop.accept(teleop_frame(i, true, 25.0, 0.0), t);
    p.step(t); t += 0.02;
  }
  ActuatorFrame f;
  for (int i = 31; i <= 250; ++i) {
    p.autonomy.accept(auto_frame(i, 30.0, 0.0), t);
    f = p.step(t); t += 0.02;
  }
  return expect_stop("teleop stale with a held claim stops", f,
                     StopReason::TELEOP_STALE);
}

Result sc_teleop_estop(bool)
{
  Pipeline p;
  double t = 100.0;
  ActuatorFrame f;
  for (int i = 1; i <= 20; ++i) {
    p.teleop.accept(teleop_frame(i, true, 40.0, 0.0, /*estop=*/true), t);
    f = p.step(t); t += 0.02;
  }
  return expect_stop("teleop E-STOP stops", f, StopReason::TELEOP_ESTOP);
}

Result sc_repeated_sequence_goes_stale(bool)
{
  Pipeline p;
  double t = 100.0;
  p.autonomy.accept(auto_frame(7, 50.0, 0.0), t);
  for (int i = 0; i < 10; ++i) { p.step(t); t += 0.02; }
  // Controller hangs; the broker keeps re-sending seq 7.
  ActuatorFrame f;
  for (int i = 0; i < 250; ++i) {
    p.autonomy.accept(auto_frame(7, 50.0, 0.0), t);
    f = p.step(t); t += 0.02;
  }
  return expect_stop("a repeating broker sequence does not keep the boat alive",
                     f, StopReason::AUTONOMY_STALE);
}

Result sc_producer_restart(bool)
{
  Pipeline p;
  double t = 100.0;
  for (int i = 1; i <= 20; ++i) {
    p.autonomy.accept(auto_frame(50000 + i, 45.0, 0.0), t);
    p.step(t); t += 0.02;
  }
  // Back seat reboots: new epoch, sequence restarts at 1. Under a
  // seq-must-increase rule alone this source could never return.
  ActuatorFrame f;
  for (int i = 1; i <= 20; ++i) {
    p.autonomy.accept(auto_frame(i, 45.0, 0.0, true, "au2"), t);
    f = p.step(t); t += 0.02;
  }
  if (f.source_epoch != "au2")
    return fail("producer restart is accepted",
                "lineage still on the old epoch: " + f.source_epoch);
  return expect_driving("producer restart accepted, new epoch in lineage", f);
}

Result sc_out_of_order_rejected(bool)
{
  Pipeline p;
  double t = 100.0;
  for (int i = 1; i <= 20; ++i) {
    p.autonomy.accept(auto_frame(100 + i, 35.0, 0.0), t);
    p.step(t); t += 0.02;
  }
  const uint64_t before = p.autonomy.snapshot().seq;
  p.autonomy.accept(auto_frame(5, 99.0, 0.0), t);   // stale duplicate-ish
  if (p.autonomy.snapshot().seq != before)
    return fail("out-of-order rejected", "snapshot moved backwards");
  if (p.autonomy.out_of_order_count() != 1)
    return fail("out-of-order rejected", "not counted");
  return pass("out-of-order sequence rejected and counted");
}

Result sc_rc_kill_while_driving(bool)
{
  Pipeline p;
  double t = 100.0;
  ActuatorFrame f;
  for (int i = 1; i <= 30; ++i) {
    p.autonomy.accept(auto_frame(i, 55.0, 0.0), t);
    f = p.step(t); t += 0.02;
  }
  if (f.neutral) return fail("RC kill while driving", "was not driving first");

  p.nav_safety.rc_kill_asserted = true;
  for (int i = 31; i <= 35; ++i) {
    p.autonomy.accept(auto_frame(i, 55.0, 0.0), t);
    f = p.step(t); t += 0.02;
  }
  if (f.stop_reason != StopReason::RC_KILL || !f.neutral)
    return fail("RC kill while driving",
                std::string("stop=") + to_string(f.stop_reason));

  // And it recovers instantly -- kill is a neutral lock, not a
  // disarm. This is why kill and NVGR_DISARM are different things.
  p.nav_safety.rc_kill_asserted = false;
  for (int i = 36; i <= 40; ++i) {
    p.autonomy.accept(auto_frame(i, 55.0, 0.0), t);
    f = p.step(t); t += 0.02;
  }
  return expect_driving("RC kill stops, and releasing it recovers at once", f);
}

Result sc_deadman_blanket(bool)
{
  Pipeline p;
  p.act_cfg.rc_deadman_enabled = true;
  p.act_cfg.rc_deadman_timeout_sec = 2.0;
  delete p.act; p.act = new ActuatorStage(p.act_cfg);

  double t = 100.0;
  p.last_rc_good = t;
  ActuatorFrame f;
  for (int i = 1; i <= 30; ++i) {
    p.autonomy.accept(auto_frame(i, 50.0, 0.0), t);
    p.last_rc_good = t;
    f = p.step(t); t += 0.02;
  }
  if (f.neutral) return fail("deadman blanket", "was not driving first");

  // Handset goes away. Autonomy is still perfectly healthy -- the
  // deadman stops it anyway, which is the entire point.
  p.nav_safety.rc_link_lost = true;
  for (int i = 31; i <= 60; ++i) {
    p.autonomy.accept(auto_frame(i, 50.0, 0.0), t);
    f = p.step(t); t += 0.02;
  }
  return expect_stop("enabled deadman stops a healthy autonomous boat", f,
                     StopReason::RC_DEADMAN);
}

Result sc_mixer_goes_quiet(bool)
{
  // The Navigator's own input failing, distinct from anything
  // upstream deciding to stop.
  Pipeline p;
  double t = 100.0;
  for (int i = 1; i <= 30; ++i) {
    p.autonomy.accept(auto_frame(i, 40.0, 0.0), t);
    p.step(t); t += 0.02;
  }
  // Stop feeding the actuator stage entirely.
  ActuatorFrame f;
  for (int i = 0; i < 60; ++i) {
    f = p.act->update(t, p.mixed, p.nav_safety, p.last_rc_good);
    t += 0.02;
  }
  return expect_stop("mixer going quiet neutralises at the Navigator", f,
                     StopReason::NAV_INPUT_STALE);
}

Result sc_all_stop_blocks_autonomy_not_rc(bool)
{
  Pipeline p;
  double t = 100.0;
  p.arb_safety.autonomy_all_stop = true;
  ActuatorFrame f;
  for (int i = 1; i <= 20; ++i) {
    p.autonomy.accept(auto_frame(i, 50.0, 0.0), t);
    f = p.step(t); t += 0.02;
  }
  if (f.stop_reason != StopReason::AUTONOMY_ALL_STOP)
    return fail("ALL_STOP gates autonomy",
                std::string("stop=") + to_string(f.stop_reason));

  // Manual rescue must still work with ALL_STOP latched.
  for (int i = 21; i <= 60; ++i) {
    p.rc.accept(rc_frame(i, "MANUAL", 65.0, 0.0), t);
    p.autonomy.accept(auto_frame(i, 50.0, 0.0), t);
    f = p.step(t); t += 0.02;
  }
  return expect_driving("ALL_STOP blocks autonomy but never the RC rescue", f);
}

Result sc_lineage_resolves(bool)
{
  Pipeline p;
  double t = 100.0;
  ActuatorFrame f;
  for (int i = 1; i <= 40; ++i) {
    p.rc.accept(rc_frame(i, "MANUAL", 50.0, 25.0), t);
    f = p.step(t); t += 0.02;
  }
  if (f.neutral) return fail("lineage resolves", "not driving");
  if (f.source_producer != "iRCInterface" || f.source_epoch != "rc1" ||
      f.source_seq == 0 || f.decision_seq == 0 || f.mix_seq == 0)
    return fail("lineage resolves", "chain incomplete at the actuator");
  return pass("every actuator frame resolves to one source sample");
}

Result sc_arm_disarm_rearm(bool)
{
  // The RC handset's disarm/arm cycle, mid-mission.
  ArmSequencerConfig c; c.arm_hold_sec = 2.0;
  ArmSequencer s(c);
  double t = 100.0;
  int holds = 0;

  if (s.update(t, true, true) != ArmAction::BEGIN_ARM)
    return fail("arm/disarm/re-arm cycle", "no BEGIN_ARM");
  t += 0.02;
  while (!s.armed() && t < 105.0) {
    if (s.update(t, true, true) == ArmAction::HOLD_NEUTRAL) ++holds;
    t += 0.02;
  }
  if (!s.armed()) return fail("arm/disarm/re-arm cycle", "never armed");
  if (holds < 90) return fail("arm/disarm/re-arm cycle", "hold too short");

  if (s.update(t, false, true) != ArmAction::DISARM)
    return fail("arm/disarm/re-arm cycle", "disarm did not emit DISARM");
  t += 0.02;
  if (s.update(t, true, true) != ArmAction::BEGIN_ARM)
    return fail("arm/disarm/re-arm cycle", "re-arm did not begin");
  t += 0.02;
  while (!s.armed() && t < 110.0) { s.update(t, true, true); t += 0.02; }
  if (!s.armed() || s.arm_cycles() != 2)
    return fail("arm/disarm/re-arm cycle", "second arm did not complete");
  return pass("arm -> disarm -> re-arm, mid-mission, no relaunch");
}

Result sc_garbage_on_the_wire(bool)
{
  Pipeline p;
  double t = 100.0;
  for (int i = 1; i <= 20; ++i) {
    p.autonomy.accept(auto_frame(i, 40.0, 0.0), t);
    p.step(t); t += 0.02;
  }
  const uint64_t before = p.autonomy.snapshot().seq;
  const char* junk[] = {
    "", "garbage", "v=2,producer=p,epoch=a,seq=1,source_time=0,valid=1,surge=0,yaw=0",
    "v=1,producer=p,epoch=a,seq=1,source_time=0,valid=1,surge=nan,yaw=0",
    "v=1,producer=p,epoch=a,seq=1,source_time=0,valid=1,surge=500,yaw=0",
    "v=1,producer=p,epoch=a,seq=1,source_time=0,valid=1,surge=1,yaw=0,surge=2",
  };
  for (const char* j : junk) p.autonomy.accept(j, t);
  if (p.autonomy.snapshot().seq != before)
    return fail("garbage rejected", "snapshot was disturbed");
  if (p.autonomy.reject_count() != 6)
    return fail("garbage rejected", "reject count wrong");
  return pass("malformed / bad-version / NaN / out-of-range / dup-key all rejected");
}

} // namespace

std::vector<Result> run_all(bool verbose)
{
  std::vector<Result> out;
  out.push_back(sc_autonomy_drives(verbose));
  out.push_back(sc_rc_stale_in_manual_does_not_fall_through(verbose));
  out.push_back(sc_teleop_stale_with_claim(verbose));
  out.push_back(sc_teleop_estop(verbose));
  out.push_back(sc_repeated_sequence_goes_stale(verbose));
  out.push_back(sc_producer_restart(verbose));
  out.push_back(sc_out_of_order_rejected(verbose));
  out.push_back(sc_rc_kill_while_driving(verbose));
  out.push_back(sc_deadman_blanket(verbose));
  out.push_back(sc_mixer_goes_quiet(verbose));
  out.push_back(sc_all_stop_blocks_autonomy_not_rc(verbose));
  out.push_back(sc_lineage_resolves(verbose));
  out.push_back(sc_arm_disarm_rearm(verbose));
  out.push_back(sc_garbage_on_the_wire(verbose));
  return out;
}

}} // namespace
