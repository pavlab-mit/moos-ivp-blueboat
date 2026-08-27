/*************************************************************
 * Unit tests for the ArduRover 4.7 skid-steer mixer port.
 *
 * The centrepiece is a replay of ~56k golden vectors generated
 * from a transcription of the real ArduPilot function (see
 * tools/gen_golden_vectors.cpp). Hand-written expectations
 * would only prove the port matches our own reading of the
 * algorithm; the CSV proves it matches ArduPilot's arithmetic.
 *
 * ON TOLERANCE. The fixture is generated in float32, because
 * that is what ArduPilot runs. This library computes in double.
 * The two therefore agree to float precision, not exactly, and
 * the comparison uses an absolute tolerance on effort expressed
 * in percent. 2e-3 %% of full scale is roughly 8e-3 us of pulse
 * width -- two orders of magnitude below the 1 us the ESC can
 * actually resolve. A tighter tolerance would be testing
 * float32 rounding, not behaviour.
 *
 * ON LIMIT FLAGS AT BOUNDARIES. The flags come from STRICT
 * inequalities (t < b, |s| > steering_range, q > 1). At an exact
 * tie, float32 and double land on opposite sides -- e.g. at
 * surge=30, yaw=-70, A=2.5 the library computes |s| and the
 * steering range as 0.70000000000000007 and 0.69999999999999996
 * while ArduPilot gets 0.69999998807907104 for both, so one
 * clamps and the other does not. The EFFORTS still agree to well
 * inside tolerance; only the "did it clamp or merely touch the
 * limit" bookkeeping differs, and at a tie neither answer is
 * more correct.
 *
 * Rather than loosen the flag check everywhere, each row is
 * probed for how close it sits to a decision boundary. Every
 * flag comes from a strict inequality against a threshold; if
 * the margin on any of them is smaller than float32 can resolve,
 * the two precisions may legitimately disagree and that row's
 * flags are exempted AND COUNTED. The test fails if the exempt
 * fraction exceeds 1%, so this cannot quietly grow into a
 * blanket excuse for a real flag bug.
 *
 * A worked example of why the margin has to be checked rather
 * than the inputs nudged: at surge=15, A=2.5, ArduPilot computes
 * 15.0f*0.01f as 0.14999999106 (rounding down) while the
 * steering range rounds up to 0.55000001192. Their difference
 * lands just past the lower throttle limit, so q comes out
 * 1.0000001 and the saturation branch runs, setting
 * throttle_upper. In double the same quantities cancel to
 * exactly 1.0 and the branch does not run. Nudging surge or yaw
 * does not reveal this, because the steering value is clamped
 * and does not move -- which is why an earlier version of this
 * test missed a whole band of yaw values at that surge.
 *
 * Convention follows navigator-cpp and lib_crsf: print
 * "FAIL: ..." and return non-zero, else "PASS" and 0.
 *
 * Build:  ./build.sh --unit_tests
 * Run:    ./bin/test_skid_mixer      (or: ctest)
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "skid_mixer.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace bb;

static int g_failures = 0;
static int g_checks   = 0;

static void check(bool cond, const std::string& what)
{
  g_checks++;
  if (!cond) {
    g_failures++;
    if (g_failures <= 25)
      fprintf(stderr, "FAIL: %s\n", what.c_str());
    else if (g_failures == 26)
      fprintf(stderr, "FAIL: (further failures suppressed)\n");
  }
}

static void check_near(double got, double want, double tol,
                       const std::string& what)
{
  const bool ok = std::fabs(got - want) <= tol;
  if (!ok) {
    char buf[256];
    snprintf(buf, sizeof(buf), "%s (got %.9g want %.9g, tol %.1e)",
             what.c_str(), got, want, tol);
    check(false, buf);
  } else {
    g_checks++;
  }
}

// Percent-of-full-scale tolerance; see header comment.
static const double kEffortTol = 2e-3;

//---------------------------------------------------------
// The worked examples from
// docs/ibb_navigator_command_pipeline.md section 9.6.
//
// These are hand-derived in the design doc and independently
// reproduced by the generator. Keeping them as explicit
// assertions means a broken CSV path cannot silently turn the
// golden replay into a no-op.

static void test_design_doc_examples()
{
  SkidMixerParams p;   // BlueBoat defaults: A=1.6, m=0.6

  struct Case { double surge, yaw, left, right; const char* note; };
  const Case cases[] = {
    { 50.0,    0.0,   50.0,     50.0,   "straight ahead, no saturation" },
    {  0.0,  100.0,   62.5,   -100.0,   "pivot: steering clamps to 0.625" },
    {-100.0,   0.0, -100.0,   -100.0,   "full astern after reverse compensation" },
    { 80.0,   40.0,  100.0,  30.6667,   "q=1.2, steering priority" },
    { 80.0,   80.0,  100.0,    -19.2,   "q=1.6, right motor driven astern" },
  };

  for (size_t i = 0; i < sizeof(cases)/sizeof(cases[0]); ++i) {
    const Case& c = cases[i];
    AllocationResult r = skid_mix(c.surge, c.yaw, p);
    check_near(r.left_effort,  c.left,  1e-3,
               std::string("design doc 9.6 left: ") + c.note);
    check_near(r.right_effort, c.right, 1e-3,
               std::string("design doc 9.6 right: ") + c.note);
  }

  // The pivot case is the clearest physical statement the mixer
  // makes, so pin the reasoning and not just the numbers: the
  // forward motor is deliberately commanded LOWER than the
  // reverse one, because reverse is the weaker direction.
  AllocationResult pivot = skid_mix(0.0, 100.0, p);
  check(pivot.left_effort > 0.0 && pivot.right_effort < 0.0,
        "pivot: opposite thrust directions");
  check(std::fabs(pivot.left_effort) < std::fabs(pivot.right_effort),
        "pivot: forward side commanded less than reverse side");
}

//---------------------------------------------------------
// Feasible region constants, A = 1.6.

static void test_feasible_region()
{
  SkidMixerParams p;
  AllocationResult r = skid_mix(0.0, 0.0, p);

  check_near(r.lower_throttle_limit,   -0.625,  1e-9, "l = -1/A");
  check_near(r.best_steering_throttle,  0.1875, 1e-9, "b = (1+l)/2");

  // Below the sweet spot steering authority grows with throttle;
  // above it, it is flat. Pin both sides of the knee.
  AllocationResult at_zero = skid_mix(0.0, 5.0, p);
  check_near(at_zero.steering_range, 0.625, 1e-9,
             "steering range at zero throttle");

  AllocationResult above = skid_mix(50.0, 5.0, p);
  check_near(above.steering_range, 0.8125, 1e-9,
             "steering range above the sweet spot");
}

//---------------------------------------------------------
// Invariant 12: non-finite in, fail closed.

static void test_nonfinite_fails_closed()
{
  SkidMixerParams p;
  const double nan_v = std::nan("");
  const double inf_v = HUGE_VAL;

  const double bad[] = {nan_v, inf_v, -inf_v};
  for (size_t i = 0; i < 3; ++i) {
    AllocationResult a = skid_mix(bad[i], 0.0, p);
    check(a.left_effort == 0.0 && a.right_effort == 0.0,
          "non-finite surge -> neutral");
    AllocationResult b = skid_mix(0.0, bad[i], p);
    check(b.left_effort == 0.0 && b.right_effort == 0.0,
          "non-finite yaw -> neutral");
  }

  SkidMixerParams bad_p;
  bad_p.thrust_asymmetry = nan_v;
  AllocationResult c = skid_mix(50.0, 0.0, bad_p);
  check(c.left_effort == 0.0 && c.right_effort == 0.0,
        "non-finite param -> neutral");
}

//---------------------------------------------------------
// Symmetry: mirroring yaw mirrors the motor pair. This is a
// property the saturation branches could plausibly break
// asymmetrically, and it is cheap to assert everywhere.

static void test_yaw_mirror_symmetry()
{
  SkidMixerParams p;
  for (int s = -100; s <= 100; s += 10) {
    for (int y = 0; y <= 100; y += 10) {
      AllocationResult pos = skid_mix(s, y, p);
      AllocationResult neg = skid_mix(s, -y, p);
      char buf[128];
      snprintf(buf, sizeof(buf), "mirror at surge=%d yaw=%d", s, y);
      check_near(pos.left_effort,  neg.right_effort, 1e-9,
                 std::string("left/right ") + buf);
      check_near(pos.right_effort, neg.left_effort,  1e-9,
                 std::string("right/left ") + buf);
    }
  }
}

//---------------------------------------------------------
// Property sweep: whatever the inputs, the outputs must be
// finite and inside the commandable range. Insurance against a
// porting typo in a branch the golden grid happens to miss.

static void test_output_envelope_property()
{
  const double asym[] = {1.0, 1.6, 2.5, 4.0};
  const double mix[]  = {0.0, 0.25, 0.5, 0.6, 0.75, 1.0};

  for (size_t ai = 0; ai < 4; ++ai) {
    for (size_t mi = 0; mi < 6; ++mi) {
      SkidMixerParams p;
      p.thrust_asymmetry = asym[ai];
      p.steering_throttle_mix = mix[mi];
      for (int s = -120; s <= 120; s += 3) {
        for (int y = -120; y <= 120; y += 3) {
          AllocationResult r = skid_mix(s, y, p);
          if (!std::isfinite(r.left_effort) || !std::isfinite(r.right_effort)) {
            check(false, "non-finite output from finite input");
            return;
          }
          if (r.left_effort  < -100.0001 || r.left_effort  > 100.0001 ||
              r.right_effort < -100.0001 || r.right_effort > 100.0001) {
            char buf[192];
            snprintf(buf, sizeof(buf),
                     "output outside [-100,100]: surge=%d yaw=%d A=%.2f m=%.2f "
                     "-> L=%.4f R=%.4f",
                     s, y, asym[ai], mix[mi], r.left_effort, r.right_effort);
            check(false, buf);
            return;
          }
        }
      }
    }
  }
  g_checks++;   // the sweep itself is the assertion
}

//---------------------------------------------------------
// Is this row's FLAG outcome below float32's ability to decide?
//
// Every limit flag comes from a strict inequality. Measure the
// margin on each of them; if the tightest is under the threshold
// below, float32 and double can land on opposite sides and the
// flag is not a meaningful cross-precision assertion. Efforts
// are still compared strictly for every row.
//
// Threshold: float32 epsilon is ~1.2e-7 on quantities of order
// 1, and these margins accumulate a handful of operations, so
// 1e-6 is about an order of magnitude of headroom over the
// resolution limit and still far tighter than any real logic
// error would be.
static bool flags_are_precision_sensitive(double surge, double yaw,
                                          const SkidMixerParams& p)
{
  const double kMargin = 1e-6;

  const AllocationResult r = skid_mix(surge, yaw, p);

  // Pre-shaping inputs, as the mixer sees them.
  double t = surge * 0.01;
  double s = yaw   * 0.01;
  if (t < -1.0) t = -1.0;  if (t > 1.0) t = 1.0;
  if (s < -1.0) s = -1.0;  if (s > 1.0) s = 1.0;

  const double margins[] = {
    std::fabs(r.saturation_value - 1.0),            // saturation onset
    std::fabs(t - r.best_steering_throttle),        // steering-range branch
    std::fabs(std::fabs(s) - r.steering_range),     // steering clamp
    std::fabs(t - 1.0),                             // throttle upper clamp
    std::fabs(t - r.lower_throttle_limit),          // throttle lower clamp
  };

  for (size_t i = 0; i < sizeof(margins)/sizeof(margins[0]); ++i)
    if (margins[i] < kMargin) return true;

  return false;
}

//---------------------------------------------------------
// Golden replay against the ArduPilot transcription.

static void test_golden_vectors()
{
  const std::string path = std::string(BB_GOLDEN_DIR) + "/ardurover_skid_golden.csv";
  FILE* f = fopen(path.c_str(), "r");
  if (!f) {
    check(false, "cannot open golden fixture: " + path);
    return;
  }

  char line[512];
  if (!fgets(line, sizeof(line), f)) {   // header
    check(false, "golden fixture is empty");
    fclose(f);
    return;
  }

  long rows = 0;
  long effort_mismatch = 0;
  long flag_mismatch = 0;
  long flag_exempt = 0;

  while (fgets(line, sizeof(line), f)) {
    double surge, yaw, asym, mix, want_l, want_r;
    int fl_sl, fl_sr, fl_tl, fl_tu;
    const int n = sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d,%d",
                         &surge, &yaw, &asym, &mix, &want_l, &want_r,
                         &fl_sl, &fl_sr, &fl_tl, &fl_tu);
    if (n != 10) continue;

    SkidMixerParams p;
    p.thrust_asymmetry = asym;
    p.steering_throttle_mix = mix;

    AllocationResult r = skid_mix(surge, yaw, p);

    if (std::fabs(r.left_effort  - want_l) > kEffortTol ||
        std::fabs(r.right_effort - want_r) > kEffortTol) {
      if (effort_mismatch < 10) {
        fprintf(stderr,
                "FAIL: golden mismatch surge=%.1f yaw=%.1f A=%.2f m=%.2f | "
                "got L=%.6f R=%.6f want L=%.6f R=%.6f\n",
                surge, yaw, asym, mix,
                r.left_effort, r.right_effort, want_l, want_r);
      }
      ++effort_mismatch;
    }

    // Limit flags are part of the contract: pThrustMix publishes
    // them as trace, and a wrong flag means a log that lies about
    // why the boat did what it did. Compare strictly, except
    // where the outcome is a floating-point coin toss.
    const bool flags_differ =
        r.limit_steer_left     != (fl_sl != 0) ||
        r.limit_steer_right    != (fl_sr != 0) ||
        r.limit_throttle_lower != (fl_tl != 0) ||
        r.limit_throttle_upper != (fl_tu != 0);

    if (flags_differ) {
      if (flags_are_precision_sensitive(surge, yaw, p)) {
        ++flag_exempt;
      } else {
        if (flag_mismatch < 10) {
          fprintf(stderr,
                  "FAIL: golden flag mismatch surge=%.1f yaw=%.1f A=%.2f m=%.2f | "
                  "got sl=%d sr=%d tl=%d tu=%d want sl=%d sr=%d tl=%d tu=%d\n",
                  surge, yaw, asym, mix,
                  r.limit_steer_left, r.limit_steer_right,
                  r.limit_throttle_lower, r.limit_throttle_upper,
                  fl_sl, fl_sr, fl_tl, fl_tu);
        }
        ++flag_mismatch;
      }
    }
    ++rows;
  }
  fclose(f);

  const double exempt_pct = rows ? (100.0 * flag_exempt / rows) : 0.0;
  printf("  golden vectors replayed: %ld\n", rows);
  printf("  flag rows exempt as precision-sensitive: %ld (%.3f%%)\n",
         flag_exempt, exempt_pct);
  check(rows > 10000, "golden fixture has a plausible row count");
  check(effort_mismatch == 0, "all golden efforts match ArduPilot");
  check(flag_mismatch == 0,   "all non-boundary limit flags match ArduPilot");
  check(exempt_pct < 1.0,
        "precision-sensitive flag rows stay a rounding-error minority");
  g_checks += 2 * rows;   // count the replay honestly
}

//---------------------------------------------------------

int main()
{
  printf("test_skid_mixer\n");
  test_design_doc_examples();
  test_feasible_region();
  test_nonfinite_fails_closed();
  test_yaw_mirror_symmetry();
  test_output_envelope_property();
  test_golden_vectors();

  if (g_failures) {
    fprintf(stderr, "\n%d/%d checks FAILED\n", g_failures, g_checks);
    return 1;
  }
  printf("PASS (%d checks)\n", g_checks);
  return 0;
}
