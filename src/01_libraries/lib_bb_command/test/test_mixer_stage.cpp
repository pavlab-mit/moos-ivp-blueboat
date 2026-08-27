/*************************************************************
 * Unit tests for MixerStage and the BB_SELECTED_CMD contract.
 *
 * The behaviours pinned here that are easiest to break by
 * "tidying up" later:
 *   - yaw is never slewed, only common throttle is;
 *   - a hard stop bypasses the limiter AND resets it;
 *   - three different input failures get three different stop
 *     reasons, because they point at different faults;
 *   - a source handoff carries the slew state by default.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "mixer_stage.h"

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
    fprintf(stderr, "FAIL: %s (got %.4f want %.4f)\n", what.c_str(), got, want);
  }
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

static MixerConfig good_config()
{
  MixerConfig c;
  c.mixer_model = "ARDUROVER_4_7_SKID";
  return c;
}

// A BB_SELECTED_CMD line as the arbiter would emit it.
static std::string sel(unsigned long long seq, const char* selected,
                       double surge, double yaw,
                       bool hard_stop = false, const char* stop = "NONE",
                       const char* epoch = "arb1")
{
  char b[384];
  snprintf(b, sizeof(b),
           "v=1,decision_epoch=%s,decision_seq=%llu,decision_time=%.3f,"
           "selected=%s,hard_stop=%d,stop=%s,"
           "source_producer=pBBPID,source_epoch=au1,source_seq=%llu,"
           "surge=%.3f,yaw=%.3f",
           epoch, seq, (double)seq * 0.02, selected, hard_stop ? 1 : 0, stop,
           seq, surge, yaw);
  return b;
}

//---------------------------------------------------------

static void test_config_requires_explicit_model()
{
  MixerConfig c;
  check(!c.validate().empty(), "mixer_model has no default and is required");
  c.mixer_model = "ARDUROVER_4_7_SKID";
  check(c.validate().empty(), "named model validates");
  c.selected_cmd_timeout_sec = 0.0;
  check(!c.validate().empty(), "zero input timeout rejected");
}

static void test_decision_parse_round_trip()
{
  DecisionParseResult r = parse_decision(sel(7, "RC", 42.0, -8.0));
  check(r.ok, "canonical BB_SELECTED_CMD parses");
  if (!r.ok) return;
  check(r.decision.decision_epoch == "arb1", "decision_epoch");
  check(r.decision.decision_seq == 7, "decision_seq");
  check(r.decision.selected == CommandSource::RC, "selected");
  check(!r.decision.hard_stop, "hard_stop");
  check_near(r.decision.surge, 42.0, 1e-9, "surge");
  check(r.decision.source_seq == 7, "lineage seq");
}

static void test_decision_parse_fails_closed()
{
  check(!parse_decision("").ok, "empty rejected");
  check(!parse_decision("garbage").ok, "garbage rejected");
  check(!parse_decision(sel(1, "WHO", 0, 0)).ok, "unknown source rejected");

  // A frame claiming a stop while carrying thrust is
  // self-contradictory; refuse rather than pick a half to trust.
  DecisionParseResult r =
      parse_decision(sel(1, "RC", 50.0, 0.0, /*hard_stop=*/true, "RC_STALE"));
  check(!r.ok, "hard_stop with nonzero command rejected");
  check(r.reject_reason == reject::kOutOfRange, "reason is out_of_range");

  // Missing decision_epoch: the lease cannot work without it.
  check(!parse_decision("v=1,decision_seq=1,decision_time=0,selected=RC,"
                        "hard_stop=0,stop=NONE,surge=0,yaw=0").ok,
        "missing decision_epoch rejected");
}

static void test_normal_mix()
{
  MixerStage m(good_config(), "mix1");
  DecisionMailbox mb;
  mb.accept(sel(1, "AUTONOMY", 50.0, 0.0), 100.0);

  MixedCommand out = m.update(100.01, 0.02, mb);
  check(!out.hard_stop, "normal cycle is not a stop");
  check_stop(out.stop_reason, StopReason::NONE, "no stop reason");
  check(out.mix_seq == 1, "mix_seq starts at 1");
  check(out.mixer_model == "ARDUROVER_4_7_SKID", "model named on the wire");
  // First cycle: slew allows 200 * 0.02 = 4 points from zero.
  check_near(out.surge_shaped, 4.0, 1e-9, "common throttle slew-limited");
  check(out.slew_limited, "slew flagged");
  check_near(out.left_effort, out.right_effort, 1e-9, "no yaw -> equal sides");
}

// The behaviour this whole component exists for.
static void test_yaw_is_not_slewed()
{
  MixerStage m(good_config(), "mix1");
  DecisionMailbox mb;
  // Full yaw from a standing start, in one cycle.
  mb.accept(sel(1, "RC", 0.0, 100.0), 100.0);
  MixedCommand out = m.update(100.01, 0.02, mb);

  check_near(out.yaw_shaped, 100.0, 1e-9,
             "yaw reaches full deflection immediately -- it is NOT slewed");
  check_near(out.surge_shaped, 0.0, 1e-9, "common throttle unchanged at zero");
  // ArduRover pivot: steering clamps to 0.625, reverse side to -100.
  check_near(out.left_effort,   62.5, 1e-6, "pivot left effort");
  check_near(out.right_effort, -100.0, 1e-6, "pivot right effort");
}

static void test_surge_ramps_over_cycles()
{
  MixerStage m(good_config(), "mix1");
  DecisionMailbox mb;
  double last = 0.0;
  for (int i = 1; i <= 25; ++i) {
    mb.accept(sel(i, "AUTONOMY", 100.0, 0.0), 100.0 + i * 0.02);
    MixedCommand out = m.update(100.0 + i * 0.02, 0.02, mb);
    check(out.surge_shaped >= last - 1e-9, "surge is monotonic while ramping");
    last = out.surge_shaped;
  }
  check_near(last, 100.0, 1e-9, "0 to full in 25 cycles at 50 Hz = 0.5 s");
}

// Three different faults, three different reasons.
static void test_input_failures_are_distinguishable()
{
  {
    MixerStage m(good_config(), "mix1");
    DecisionMailbox mb;   // nothing ever accepted
    MixedCommand out = m.update(100.0, 0.02, mb);
    check(out.hard_stop, "no arbiter at all -> stop");
    check_stop(out.stop_reason, StopReason::MIXER_INPUT_INVALID,
               "never heard from the arbiter");
  }
  {
    MixerStage m(good_config(), "mix1");
    DecisionMailbox mb;
    mb.accept(sel(1, "AUTONOMY", 50.0, 0.0), 100.0);
    MixedCommand out = m.update(101.0, 0.02, mb);   // timeout is 0.5
    check(out.hard_stop, "arbiter went quiet -> stop");
    check_stop(out.stop_reason, StopReason::MIXER_INPUT_STALE,
               "arbiter stopped publishing");
  }
  {
    MixerStage m(good_config(), "mix1");
    DecisionMailbox mb;
    mb.accept(sel(1, "NONE", 0.0, 0.0, true, "RC_STALE"), 100.0);
    MixedCommand out = m.update(100.01, 0.02, mb);
    check(out.hard_stop, "arbiter's own stop is honoured");
    check_stop(out.stop_reason, StopReason::RC_STALE,
               "the arbiter's reason is propagated, not overwritten");
  }
}

// Invariant 7.
static void test_hard_stop_bypasses_and_resets_slew()
{
  MixerStage m(good_config(), "mix1");
  DecisionMailbox mb;

  for (int i = 1; i <= 25; ++i) {
    mb.accept(sel(i, "AUTONOMY", 100.0, 0.0), 100.0 + i * 0.02);
    m.update(100.0 + i * 0.02, 0.02, mb);
  }
  check_near(m.slew_state(), 100.0, 1e-9, "ramped to full");

  mb.accept(sel(26, "NONE", 0.0, 0.0, true, "AUTONOMY_ALL_STOP"), 100.52);
  MixedCommand out = m.update(100.52, 0.02, mb);

  check_near(out.left_effort,  0.0, 1e-9, "stop is immediate on the left");
  check_near(out.right_effort, 0.0, 1e-9, "stop is immediate on the right");
  check_near(m.slew_state(), 0.0, 1e-9, "limiter state reset, not ramped down");

  // Resuming starts from rest.
  mb.accept(sel(27, "AUTONOMY", 100.0, 0.0), 100.54);
  MixedCommand res = m.update(100.54, 0.02, mb);
  check_near(res.surge_shaped, 4.0, 1e-9, "resume ramps from zero");
}

// Plan decision (c).
static void test_handoff_carries_slew_state_by_default()
{
  MixerStage m(good_config(), "mix1");
  DecisionMailbox mb;

  for (int i = 1; i <= 25; ++i) {
    mb.accept(sel(i, "AUTONOMY", 60.0, 0.0), 100.0 + i * 0.02);
    m.update(100.0 + i * 0.02, 0.02, mb);
  }
  const double before = m.slew_state();
  check(before > 55.0, "autonomy had the boat up to speed");

  // Operator takes over at the same commanded speed.
  mb.accept(sel(26, "RC", 60.0, 0.0), 100.52);
  MixedCommand out = m.update(100.52, 0.02, mb);
  check(out.handoff, "handoff detected");
  check_near(out.surge_shaped, 60.0, 1e-9,
             "no drop to zero on takeover -- state carried");
}

static void test_handoff_can_reset_slew()
{
  MixerConfig c = good_config();
  c.slew_reset_on_handoff = true;
  MixerStage m(c, "mix1");
  DecisionMailbox mb;

  for (int i = 1; i <= 25; ++i) {
    mb.accept(sel(i, "AUTONOMY", 60.0, 0.0), 100.0 + i * 0.02);
    m.update(100.0 + i * 0.02, 0.02, mb);
  }
  mb.accept(sel(26, "RC", 60.0, 0.0), 100.52);
  MixedCommand out = m.update(100.52, 0.02, mb);
  check(out.handoff, "handoff detected");
  check_near(out.surge_shaped, 4.0, 1e-9, "configured to restart from rest");
}

static void test_lineage_is_copied_unchanged()
{
  MixerStage m(good_config(), "mixEPOCH");
  DecisionMailbox mb;
  mb.accept(sel(4242, "RC", 10.0, 5.0), 100.0);
  MixedCommand out = m.update(100.01, 0.02, mb);

  check(out.decision_epoch == "arb1", "decision epoch copied");
  check(out.decision_seq == 4242, "decision seq copied");
  check(out.source_producer == "pBBPID", "source producer copied");
  check(out.source_epoch == "au1", "source epoch copied");
  check(out.source_seq == 4242, "source seq copied");
  check(out.mix_epoch == "mixEPOCH", "mixer stamps its own epoch");
}

static void test_mix_sequence_advances_on_stop_cycles_too()
{
  MixerStage m(good_config(), "mix1");
  DecisionMailbox mb;   // nothing: every cycle is a stop
  for (int i = 1; i <= 10; ++i) {
    MixedCommand out = m.update(100.0 + i * 0.02, 0.02, mb);
    check(out.mix_seq == (uint64_t)i,
          "mix_seq advances even while stopped, so the trace has no gaps");
  }
}

static void test_duplicate_decision_goes_stale()
{
  MixerStage m(good_config(), "mix1");
  DecisionMailbox mb;
  mb.accept(sel(5, "AUTONOMY", 40.0, 0.0), 100.0);
  check(!m.update(100.01, 0.02, mb).hard_stop, "driving");

  // Arbiter hangs; something keeps re-publishing its last output.
  for (int i = 1; i <= 40; ++i)
    mb.accept(sel(5, "AUTONOMY", 40.0, 0.0), 100.0 + i * 0.02);

  MixedCommand out = m.update(100.9, 0.02, mb);
  check(out.hard_stop, "a repeating decision does not keep the mixer alive");
  check_stop(out.stop_reason, StopReason::MIXER_INPUT_STALE, "reason is stale");
}

static void test_serialisation_carries_the_chain()
{
  MixerStage m(good_config(), "mix1");
  DecisionMailbox mb;
  mb.accept(sel(9, "RC", 30.0, 20.0), 100.0);
  const std::string wire = serialize_mixed(m.update(100.01, 0.02, mb));

  check(wire.find("mixer_model=ARDUROVER_4_7_SKID") != std::string::npos,
        "wire names the mixer model");
  check(wire.find("decision_seq=9") != std::string::npos, "wire carries decision seq");
  check(wire.find("source_seq=9") != std::string::npos, "wire carries source seq");
  check(wire.find("mix_seq=1") != std::string::npos, "wire carries mix seq");
  check(wire.find("selected=RC") != std::string::npos, "wire carries the source");
}

int main()
{
  printf("test_mixer_stage\n");
  test_config_requires_explicit_model();
  test_decision_parse_round_trip();
  test_decision_parse_fails_closed();
  test_normal_mix();
  test_yaw_is_not_slewed();
  test_surge_ramps_over_cycles();
  test_input_failures_are_distinguishable();
  test_hard_stop_bypasses_and_resets_slew();
  test_handoff_carries_slew_state_by_default();
  test_handoff_can_reset_slew();
  test_lineage_is_copied_unchanged();
  test_mix_sequence_advances_on_stop_cycles_too();
  test_duplicate_decision_goes_stale();
  test_serialisation_carries_the_chain();

  if (g_failures) {
    fprintf(stderr, "\n%d/%d checks FAILED\n", g_failures, g_checks);
    return 1;
  }
  printf("PASS (%d checks)\n", g_checks);
  return 0;
}
