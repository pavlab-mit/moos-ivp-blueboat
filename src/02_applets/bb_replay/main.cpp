/*************************************************************
 *  bb_replay -- offline replay of the BlueBoat command pipeline
 *  against a recorded .alog.
 *
 *  This is Tier 2 of the test strategy
 *  (docs/control_refactor_plan.md section 8.2). With no on-water
 *  shadow phase, replay is the gate between a unit-tested
 *  library and props in the water.
 *
 *  Modes:
 *    --mode=mixer   Replay logged DESIRED_THRUST/DESIRED_RUDDER
 *                   through the new ArduRover SkidMixer and
 *                   compare against the DESIRED_THRUST_L/R the
 *                   old pThrustMix actually produced. Answers
 *                   "what changes on the water when we swap
 *                   mixers", using real commands rather than a
 *                   guess.
 *
 *    --mode=esc     Compare the OLD pulse mapping (800/1500/2200)
 *                   against the NEW one (1100/1510/1900) over the
 *                   logged commands, and count how many land
 *                   outside the ESC's documented input range.
 *
 *    --mode=scan    List variables and rates. Orientation only.
 *
 *  Not yet implemented: full-pipeline replay against
 *  NVGR_ACTUATOR_TRACE, which needs pBBCommandArbiter (P3) and
 *  logs that contain the new contracts. The reader and the
 *  comparison scaffolding here are what that will be built on.
 *
 *  Build:  ./build.sh
 *  Run:    ./bin/bb_replay <file.alog> --mode=mixer
 *
 *  Author: Jeremy Wenger
 *************************************************************/

#include "alog_reader.h"

#include "esc_mapper.h"
#include "skid_mixer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace bb;

//---------------------------------------------------------
// Small stats helper. Percentiles over a copy, because these
// samples are cheap and clarity beats an in-place partition.

struct Stats
{
  std::vector<double> v;
  void add(double x) { v.push_back(x); }
  size_t n() const { return v.size(); }
  double pct(double q)
  {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    size_t i = (size_t)(q * (v.size() - 1));
    return v[i];
  }
  double max() { return v.empty() ? 0.0 : pct(1.0); }
  double mean() const
  {
    if (v.empty()) return 0.0;
    double s = 0.0;
    for (size_t i = 0; i < v.size(); ++i) s += v[i];
    return s / v.size();
  }
};

//---------------------------------------------------------

static void print_hdr(const char* title)
{
  printf("\n");
  printf("=====================================================================\n");
  printf("  %s\n", title);
  printf("=====================================================================\n");
}

//---------------------------------------------------------
// --mode=scan

static int mode_scan(const AlogLog& log)
{
  print_hdr("Variables");
  const std::vector<std::string> vars = log.variables();
  const double span = log.last_time() - log.first_time();
  printf("  %-28s %10s %9s\n", "VARIABLE", "SAMPLES", "RATE(Hz)");
  for (size_t i = 0; i < vars.size(); ++i) {
    const AlogSeries& s = log.series(vars[i]);
    const double rate = (span > 0.0) ? (s.size() / span) : 0.0;
    printf("  %-28s %10zu %9.2f\n", vars[i].c_str(), s.size(), rate);
  }
  printf("\n  span %.1f s (%.1f min), %ld lines read\n",
         span, span / 60.0, log.lines_read());
  return 0;
}

//---------------------------------------------------------
// --mode=mixer
//
// Pairing: for each logged mixer OUTPUT sample, take the most
// recent mixer INPUT at or before it. That is what pThrustMix
// itself saw -- a MOOS subscriber holds the last value received
// until the next arrives -- so the comparison is against the same
// inputs the old mixer actually acted on, not an interpolation we
// invented.

static int mode_mixer(const AlogLog& log)
{
  const char* kThrust = "DESIRED_THRUST";
  const char* kRudder = "DESIRED_RUDDER";
  const char* kOutL   = "DESIRED_THRUST_L";
  const char* kOutR   = "DESIRED_THRUST_R";

  if (!log.has(kOutL) || !log.has(kOutR)) {
    fprintf(stderr, "error: log has no %s/%s to compare against\n", kOutL, kOutR);
    return 2;
  }
  if (!log.has(kThrust) || !log.has(kRudder)) {
    fprintf(stderr, "error: log has no %s/%s mixer inputs\n", kThrust, kRudder);
    return 2;
  }

  SkidMixerParams params;   // BlueBoat defaults: A=1.6, m=0.6
  const std::string cfg_err = params.validate();
  if (!cfg_err.empty()) { fprintf(stderr, "bad params: %s\n", cfg_err.c_str()); return 2; }

  const AlogSeries& outL = log.series(kOutL);
  const AlogSeries& outR = log.series(kOutR);

  // Pair L and R BY INDEX, not by timestamp.
  //
  // pThrustMix publishes them back to back in one Iterate:
  //   Notify(L); Notify(R);
  // so R for tick N carries a timestamp fractionally LATER than L
  // for tick N. Looking R up with at_or_before(t_L) therefore
  // returns tick N-1's value -- a systematic one-tick skew on one
  // side only, which manufactures differentials that were never
  // commanded. An earlier version of this tool did exactly that
  // and reported 102 spurious turn-direction disagreements.
  if (outL.size() != outR.size()) {
    fprintf(stderr, "warning: %s has %zu samples, %s has %zu -- "
                    "index pairing may skew; falling back to timestamp pairing\n",
            kOutL, outL.size(), kOutR, outR.size());
  }
  const bool pair_by_index = (outL.size() == outR.size());

  // Widest tolerated gap between a paired L and R. One 16 Hz tick
  // is 62 ms; anything beyond a third of that is not the same
  // publish and should not be paired.
  const double kPairWindow = 0.020;
  long pair_window_violations = 0;

  Stats dl, dr;              // |new - old| per side
  Stats old_mag, new_mag;    // |command| magnitude per side
  long paired = 0;
  long sign_flip = 0;        // new and old disagree on a motor's DIRECTION
  long turn_flip = 0;        // new and old disagree on TURN direction
  long old_sat = 0, new_sat = 0;
  long turn_flip_steady = 0, turn_flip_transient = 0;
  long flip_astern = 0, flip_small_yaw = 0;
  long new_more_diff = 0;    // new commands a larger left/right differential
  Stats diff_old, diff_new;  // |L-R|, the yaw authority actually commanded

  // Regime split: is the difference concentrated where it matters?
  Stats dl_turning, dl_straight;

  for (size_t i = 0; i < outL.size(); ++i) {
    const double t = outL[i].time;

    const AlogSample* sr = nullptr;
    if (pair_by_index) {
      sr = &outR[i];
      if (std::fabs(sr->time - t) > kPairWindow) { ++pair_window_violations; continue; }
    } else {
      sr = log.at_or_before(kOutR, t);
    }
    const AlogSample* th = log.at_or_before(kThrust, t);
    const AlogSample* ru = log.at_or_before(kRudder, t);
    if (!sr || !th || !ru) continue;
    if (!outL[i].numeric || !sr->numeric || !th->numeric || !ru->numeric) continue;

    const double old_l = outL[i].value;
    const double old_r = sr->value;
    const double surge = th->value;
    const double yaw   = ru->value;

    const AllocationResult a = skid_mix(surge, yaw, params);

    dl.add(std::fabs(a.left_effort  - old_l));
    dr.add(std::fabs(a.right_effort - old_r));

    old_mag.add(std::fabs(old_l));
    old_mag.add(std::fabs(old_r));
    new_mag.add(std::fabs(a.left_effort));
    new_mag.add(std::fabs(a.right_effort));

    if ((a.left_effort  > 0) != (old_l > 0) &&
        std::fabs(a.left_effort) > 1.0 && std::fabs(old_l) > 1.0) ++sign_flip;
    if ((a.right_effort > 0) != (old_r > 0) &&
        std::fabs(a.right_effort) > 1.0 && std::fabs(old_r) > 1.0) ++sign_flip;

    // Was the yaw command STEADY around this sample? The mixer
    // input is up to one tick stale relative to the logged
    // output, so during a fast rudder reversal the paired input
    // may not be the one the old mixer acted on. A flip found
    // during a reversal says nothing about the two mixers; a flip
    // found while the command was steady does.
    bool yaw_steady = true;
    {
      const AlogSample* before = log.at_or_before(kRudder, t - 0.15);
      const AlogSample* after  = log.at_or_before(kRudder, t + 0.15);
      if (!before || !after || !before->numeric || !after->numeric) {
        yaw_steady = false;
      } else {
        const bool sign_stable =
            (before->value > 0) == (yaw > 0) && (after->value > 0) == (yaw > 0);
        const bool magnitude_stable =
            std::fabs(before->value - yaw) < 5.0 &&
            std::fabs(after->value  - yaw) < 5.0;
        yaw_steady = sign_stable && magnitude_stable;
      }
    }

    const double old_diff = old_l - old_r;
    const double new_diff = a.left_effort - a.right_effort;
    if ((old_diff > 0) != (new_diff > 0) &&
        std::fabs(old_diff) > 1.0 && std::fabs(new_diff) > 1.0) {
      if (!yaw_steady) ++turn_flip_transient;
      else             ++turn_flip_steady;
      if (turn_flip < 12) {
        printf("    flip t=%8.2f surge=%7.2f yaw=%7.2f | old L=%7.2f R=%7.2f (d=%7.2f)"
               " | new L=%7.2f R=%7.2f (d=%7.2f)\n",
               t, surge, yaw, old_l, old_r, old_diff,
               a.left_effort, a.right_effort, new_diff);
        printf("         yaw %s, surge %s\n",
               yaw_steady ? "STEADY" : "in transition",
               surge < -5.0 ? "ASTERN" : "ahead");
      }
      if (surge < -5.0) ++flip_astern;
      if (std::fabs(yaw) < 10.0) ++flip_small_yaw;
      ++turn_flip;
    }

    diff_old.add(std::fabs(old_diff));
    diff_new.add(std::fabs(new_diff));
    if (std::fabs(new_diff) > std::fabs(old_diff) + 1e-6) ++new_more_diff;

    if (std::fabs(old_l) >= 99.9 || std::fabs(old_r) >= 99.9) ++old_sat;
    if (std::fabs(a.left_effort) >= 99.9 || std::fabs(a.right_effort) >= 99.9) ++new_sat;

    if (std::fabs(yaw) > 10.0) dl_turning.add(std::fabs(a.left_effort - old_l));
    else                       dl_straight.add(std::fabs(a.left_effort - old_l));

    ++paired;
  }

  print_hdr("Mixer A/B: new ArduRover SkidMixer vs logged pThrustMix output");
  printf("  (turn-direction disagreements printed above, first 12)\n");
  printf("  paired samples            : %ld  (%s)\n", paired,
         pair_by_index ? "L/R paired by index" : "L/R paired by timestamp");
  if (pair_window_violations)
    printf("  dropped, L/R too far apart: %ld\n", pair_window_violations);
  if (!paired) return 2;
  printf("  mixer params              : A=%.2f  m=%.2f\n",
         params.thrust_asymmetry, params.steering_throttle_mix);

  printf("\n  Per-side command difference |new - old|, percent of full scale\n");
  printf("    left   median %6.2f   p90 %6.2f   p99 %6.2f   max %6.2f\n",
         dl.pct(0.50), dl.pct(0.90), dl.pct(0.99), dl.max());
  printf("    right  median %6.2f   p90 %6.2f   p99 %6.2f   max %6.2f\n",
         dr.pct(0.50), dr.pct(0.90), dr.pct(0.99), dr.max());

  printf("\n  Split by regime (left side)\n");
  printf("    turning (|rudder|>10)  n=%7zu  median %6.2f  p90 %6.2f\n",
         dl_turning.n(), dl_turning.pct(0.50), dl_turning.pct(0.90));
  printf("    near-straight          n=%7zu  median %6.2f  p90 %6.2f\n",
         dl_straight.n(), dl_straight.pct(0.50), dl_straight.pct(0.90));

  printf("\n  Yaw authority commanded, |L-R|\n");
  printf("    old  median %6.2f   p90 %6.2f   max %6.2f\n",
         diff_old.pct(0.50), diff_old.pct(0.90), diff_old.max());
  printf("    new  median %6.2f   p90 %6.2f   max %6.2f\n",
         diff_new.pct(0.50), diff_new.pct(0.90), diff_new.max());
  printf("    new commands MORE differential in %ld/%ld samples (%.1f%%)\n",
         new_more_diff, paired, 100.0 * new_more_diff / paired);

  printf("\n  Saturation (either side at full scale)\n");
  printf("    old %ld (%.2f%%)   new %ld (%.2f%%)\n",
         old_sat, 100.0 * old_sat / paired, new_sat, 100.0 * new_sat / paired);

  printf("\n  Direction disagreements (both magnitudes > 1)\n");
  printf("    per-motor sign flips  : %ld (%.3f%%)\n",
         sign_flip, 100.0 * sign_flip / (2.0 * paired));
  printf("    TURN direction flips  : %ld (%.3f%%)\n",
         turn_flip, 100.0 * turn_flip / paired);
  printf("      of which, yaw command in transition : %ld  (pairing artefact)\n",
         turn_flip_transient);
  printf("      of which, yaw command steady        : %ld\n", turn_flip_steady);
  printf("      of which, commanded astern          : %ld  (neither mixer models\n",
         flip_astern);
  printf("                                              reverse steering sign)\n");
  printf("      of which, |yaw| < 10                : %ld\n", flip_small_yaw);
  if (turn_flip_steady > paired / 1000)
    printf("    ^ INVESTIGATE: steady-command turn disagreements are not noise\n");

  return 0;
}

//---------------------------------------------------------
// --mode=esc
//
// The endpoint change is the one part of this refactor that
// alters what the ESC sees for an UNCHANGED command, so quantify
// it directly on real commands rather than reasoning about it.

static int mode_esc(const AlogLog& log)
{
  const char* vars[2] = {"DESIRED_THRUST_L", "DESIRED_THRUST_R"};
  if (!log.has(vars[0]) || !log.has(vars[1])) {
    fprintf(stderr, "error: log has no DESIRED_THRUST_L/R\n");
    return 2;
  }

  // Old behaviour, from iBBNavigatorInterface::setPinPulseWidth
  // with the shipped defaults pwm_min_us=800, pwm_max_us=2200.
  const double old_center = 1500.0, old_span = (2200.0 - 800.0) / 2.0;

  EscChannelConfig right; right.reversed = false;
  EscChannelConfig left;  left.reversed  = true;

  long n = 0, out_of_range = 0, at_full = 0;
  Stats old_pulse_dev, new_pulse_dev;

  for (int c = 0; c < 2; ++c) {
    const AlogSeries& s = log.series(vars[c]);
    const EscChannelConfig& cfg = (c == 0) ? left : right;
    for (size_t i = 0; i < s.size(); ++i) {
      if (!s[i].numeric) continue;
      const double cmd = s[i].value;

      double old_us = old_center + (cmd / 100.0) * old_span;
      if (old_us < 800.0)  old_us = 800.0;
      if (old_us > 2200.0) old_us = 2200.0;

      const EscOutput neu = esc_map(cmd, cfg);

      if (old_us < 1100.0 || old_us > 1900.0) ++out_of_range;
      if (std::fabs(cmd) >= 99.9) ++at_full;

      old_pulse_dev.add(std::fabs(old_us - 1500.0));
      new_pulse_dev.add(std::fabs(neu.pulse_us - 1510.0));
      ++n;
    }
  }

  print_hdr("ESC mapping: old 800/1500/2200 vs new 1100/1510/1900");
  printf("  single-side commands      : %ld\n", n);
  if (!n) return 2;
  printf("  OLD mapping outside the ESC's documented 1100-1900 us window:\n");
  printf("    %ld  (%.2f%%)\n", out_of_range, 100.0 * out_of_range / n);
  printf("  commands at full scale (|cmd| >= 99.9): %ld (%.2f%%)\n",
         at_full, 100.0 * at_full / n);
  printf("\n  Pulse deviation from that mapping's own neutral\n");
  printf("    old  median %7.1f us   p99 %7.1f us   max %7.1f us\n",
         old_pulse_dev.pct(0.50), old_pulse_dev.pct(0.99), old_pulse_dev.max());
  printf("    new  median %7.1f us   p99 %7.1f us   max %7.1f us\n",
         new_pulse_dev.pct(0.50), new_pulse_dev.pct(0.99), new_pulse_dev.max());
  printf("\n  Under the NEW mapping every command in [-100,100] lands inside\n");
  printf("  1100-1900 us by construction -- see test_esc_mapper.\n");
  return 0;
}

//---------------------------------------------------------

static void usage()
{
  printf("usage: bb_replay <file.alog> [--mode=mixer|esc|scan]\n");
  printf("\n");
  printf("  --mode=mixer  new SkidMixer vs the logged pThrustMix output\n");
  printf("  --mode=esc    old vs new ESC pulse mapping over logged commands\n");
  printf("  --mode=scan   list variables and rates\n");
}

int main(int argc, char** argv)
{
  if (argc < 2) { usage(); return 1; }

  std::string path;
  std::string mode = "mixer";
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "-h" || a == "--help") { usage(); return 0; }
    else if (a.compare(0, 7, "--mode=") == 0) mode = a.substr(7);
    else path = a;
  }
  if (path.empty()) { usage(); return 1; }

  std::set<std::string> wanted;
  if (mode != "scan") {
    wanted.insert("DESIRED_THRUST");
    wanted.insert("DESIRED_RUDDER");
    wanted.insert("DESIRED_THRUST_L");
    wanted.insert("DESIRED_THRUST_R");
    wanted.insert("NAV_SPEED");
    wanted.insert("THRUSTMIX_DERATE");
  }

  AlogLog log;
  std::string err;
  printf("reading %s ...\n", path.c_str());
  if (!log.load(path, wanted, err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }
  printf("  %ld lines, span %.1f s\n", log.lines_read(),
         log.last_time() - log.first_time());

  if (mode == "scan")  return mode_scan(log);
  if (mode == "mixer") return mode_mixer(log);
  if (mode == "esc")   return mode_esc(log);

  fprintf(stderr, "unknown mode: %s\n", mode.c_str());
  usage();
  return 1;
}
