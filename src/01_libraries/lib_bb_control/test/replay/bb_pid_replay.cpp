/*************************************************************
 * bb_pid_replay -- offline parity replay of pBBPID on a
 * recorded .alog (brief section 3: "old vs new engine on the
 * existing block log: identical on clean segments, divergent
 * only where the fixes bite").
 *
 * Three engines run side by side on the logged input stream:
 *
 *   legacy : LegacyBBPIDEngine, extracted verbatim from git
 *            HEAD (the code that ran on the water 27 Aug),
 *            ScalarPID underneath.
 *   new/off: BBPIDEngine on bb::Pid, every lifecycle fix at
 *            its legacy default.
 *   new/on : ff_hold_time=2, ff_step_limit=150, max_dt=0.5,
 *            integrate gate driven by the LOGGED
 *            BB_CMD_AUTHORITY / NVGR_STOP_REASON stream.
 *
 * Tick discipline: the app published BBPID_CMD_STALE on every
 * iterate that passed hold-off, BEFORE deciding stale/active,
 * so each CMD_STALE=false event marks exactly one
 * engine.update() call. Inputs are applied at their logged
 * arrival times; the tick uses the CMD_STALE timestamp (the
 * real call used MOOSTime() a few hundred microseconds later
 * -- this is why legacy-vs-LOG is a fidelity check with
 * tolerance, while legacy-vs-new is exact: both replayed
 * engines see identical inputs at identical times).
 *
 * Expected result:
 *   legacy vs log     : small residuals (timestamp jitter only)
 *   new/off vs legacy : 0 on every tick where legacy's output
 *                       was not railed (the parity contract);
 *                       divergence at rails is the always-on
 *                       tracking anti-windup, reported separately
 *   new/on  vs legacy : divergence at settles, big steps, gaps,
 *                       rails, and RC-authority windows -- and
 *                       nowhere else
 *
 * Usage:
 *   bb_pid_replay <file.alog> [out.csv]
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "BBPIDEngine.h"
#include "LegacyBBPIDEngine.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

//---------------------------------------------------------
// The exact pBBPID config from the run's ._moos snapshot
// (LOG_YIP_27_8_2026_____16_59_38). NOTE no yawrate_scale line
// was present: the default 1.0 applied (GYRO_Z_LVL_IMU already
// rad/s), engine negates internally.

template <class ENGINE>
static void configure(ENGINE& e)
{
  e.setSpeedGains(1.0, 2.0, 0.0);
  e.setHeadingGains(0.8, 0.0, 0.0);
  e.setYawRateGains(60.0, 40.0, 0.0);
  e.setSpeedLimits(100.0, 100.0);          // speed_integral_limit, max_thrust
  e.setHeadingLimits(0.4363, 0.4363);      // max_yawrate as both
  e.setYawRateLimits(50.0, 100.0);         // yawrate_integral_limit, max_rudder
  e.setMaxYawRate(0.4363);
  e.setDesYawRateFilter(0.3);
  e.setFeedforwardEnable(true);
  e.setFeedforwardSpeedEnable(true);
  e.setFeedforwardYawEnable(true);
  e.setFeedforwardSpeed(0.0, 7.5, 30.0);   // c0, cv, crr
  e.setFeedforwardYaw(0.0, 20.0, 30.0);    // d0, dr, dvr
  e.setFeedforwardRudderScale(1.0);
  e.setYawPriorityGain(0.5);
  e.setYawPriorityKnee(0.6);
  e.setMinSpeedFrac(0.5);
  e.setDerateFilter(0.5);
  e.setAllowReverse(true);
  e.setRudderPolarity(1.0);
  e.setYawRateScale(1.0);
  e.setYawRateDerive(false);               // yawrate_source = external
  e.enableGainSchedule(false);
}

//---------------------------------------------------------
struct Stats {
  std::vector<double> v;
  void add(double x) { v.push_back(std::fabs(x)); }
  double pct(double p) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    size_t i = (size_t)(p * (v.size() - 1));
    return v[i];
  }
  double maxv() { return v.empty() ? 0.0 : *std::max_element(v.begin(), v.end()); }
};

struct Divergence {
  double t_start = 0.0, t_end = 0.0;
  double peak = 0.0;
  int    ticks = 0;
};

// Collapse per-tick exceedances into human-readable windows.
static void collect(std::vector<Divergence>& out, double t, double mag,
                    double thresh, double join_gap = 1.0)
{
  if (mag <= thresh) return;
  if (!out.empty() && (t - out.back().t_end) < join_gap) {
    out.back().t_end = t;
    out.back().ticks++;
    if (mag > out.back().peak) out.back().peak = mag;
  }
  else {
    Divergence d; d.t_start = d.t_end = t; d.peak = mag; d.ticks = 1;
    out.push_back(d);
  }
}

//---------------------------------------------------------
int main(int argc, char** argv)
{
  if (argc < 2) {
    fprintf(stderr, "usage: bb_pid_replay <file.alog> [out.csv]\n");
    return 2;
  }
  std::ifstream in(argv[1]);
  if (!in) { fprintf(stderr, "cannot open %s\n", argv[1]); return 2; }
  FILE* csv = nullptr;
  if (argc > 2) {
    csv = fopen(argv[2], "w");
    if (!csv) { fprintf(stderr, "cannot open %s\n", argv[2]); return 2; }
    fprintf(csv, "t,log_thrust,log_rudder,leg_thrust,leg_rudder,"
                 "off_thrust,off_rudder,on_thrust,on_rudder,"
                 "gate_open,leg_railed\n");
  }

  LegacyBBPIDEngine legacy;  configure(legacy);
  BBPIDEngine       new_off; configure(new_off);
  BBPIDEngine       new_on;  configure(new_on);
  new_on.setFFHoldTime(2.0);
  new_on.setFFStepLimitDeg(150.0);
  new_on.setMaxDt(0.5);
  new_on.setAntiWindup(true);

  // Latest-value input state (what OnNewMail would have cached)
  double des_speed = 0, des_heading = 0, nav_speed = 0, nav_heading = 0;
  double gyro_z = 0;
  bool have_ds = false, have_dh = false, have_ns = false, have_nh = false;
  std::string authority = "", stop_reason = "";

  // Pending comparison: after an active tick, the next logged
  // DESIRED_THRUST / DESIRED_RUDDER from pBBPID are that tick's
  // published outputs.
  bool   awaiting_thrust = false, awaiting_rudder = false;
  double tick_t = 0, leg_th = 0, leg_rd = 0, off_th = 0, off_rd = 0;
  double on_th = 0, on_rd = 0;
  bool   tick_gate = true, tick_railed = false;

  long ticks = 0, ticks_railed = 0, ticks_gated = 0, skipped_incomplete = 0;
  Stats fid_th, fid_rd;                    // legacy vs log (harness fidelity)
  double off_max_clean_th = 0, off_max_clean_rd = 0;   // parity claim
  std::vector<Divergence> on_div;          // new/on vs legacy windows

  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '%') continue;
    std::istringstream ss(line);
    double t; std::string var, src, val;
    if (!(ss >> t >> var >> src >> val)) continue;

    if      (var == "DESIRED_SPEED")     { des_speed = atof(val.c_str());  have_ds = true; }
    else if (var == "DESIRED_HEADING")   { des_heading = atof(val.c_str()); have_dh = true; }
    else if (var == "NAV_SPEED")         { nav_speed = atof(val.c_str());  have_ns = true; }
    else if (var == "GPS_HEADING_DGNSS") { nav_heading = atof(val.c_str()); have_nh = true; }
    else if (var == "GYRO_Z_LVL_IMU")    { gyro_z = atof(val.c_str()); }
    else if (var == "BB_CMD_AUTHORITY")  { authority = val; }
    else if (var == "NVGR_STOP_REASON")  { stop_reason = val; }
    else if (var == "DESIRED_THRUST" && src == "pBBPID" && awaiting_thrust) {
      awaiting_thrust = false;
      fid_th.add(atof(val.c_str()) - leg_th);
      // The CSV row opens here and completes on the rudder line,
      // which the app publishes immediately after.
      if (csv) fprintf(csv, "%.5f,%.6f,", t, atof(val.c_str()));
    }
    else if (var == "DESIRED_RUDDER" && src == "pBBPID" && awaiting_rudder) {
      awaiting_rudder = false;
      fid_rd.add(atof(val.c_str()) - leg_rd);
      if (csv)
        fprintf(csv, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d\n",
                atof(val.c_str()), leg_th, leg_rd, off_th, off_rd,
                on_th, on_rd, tick_gate ? 1 : 0, tick_railed ? 1 : 0);
    }
    else if (var == "BBPID_CMD_STALE" && src == "pBBPID") {
      // std::to_string(bool) on the boat wrote "0"/"1", not
      // "true"/"false": 0 = actively commanding, 1 = stale.
      if (val != "0") continue;            // stale tick: no engine update ran
      if (!(have_ds && have_dh && have_ns && have_nh)) { skipped_incomplete++; continue; }

      ticks++;
      legacy.update (t, des_speed, nav_speed, des_heading, nav_heading, gyro_z, leg_th, leg_rd);
      new_off.update(t, des_speed, nav_speed, des_heading, nav_heading, gyro_z, off_th, off_rd);

      tick_gate = (authority == "AUTONOMY") && (stop_reason == "NONE");
      if (!tick_gate) ticks_gated++;
      new_on.setIntegrateEnable(tick_gate);
      new_on.update (t, des_speed, nav_speed, des_heading, nav_heading, gyro_z, on_th, on_rd);

      // Parity accounting. With every fix (anti-windup included)
      // config-gated, new/off must match legacy on EVERY tick --
      // railed ones too. Railed ticks are still counted for context.
      tick_railed = (std::fabs(leg_th) >= 100.0 - 1e-9) ||
                    (std::fabs(leg_rd) >= 100.0 - 1e-9);
      if (tick_railed)
        ticks_railed++;
      off_max_clean_th = std::max(off_max_clean_th, std::fabs(off_th - leg_th));
      off_max_clean_rd = std::max(off_max_clean_rd, std::fabs(off_rd - leg_rd));

      double on_mag = std::max(std::fabs(on_th - leg_th), std::fabs(on_rd - leg_rd));
      collect(on_div, t, on_mag, 0.5);

      awaiting_thrust = awaiting_rudder = true;
      tick_t = t;
    }
  }

  printf("bb_pid_replay: %s\n", argv[1]);
  printf("  ticks replayed        : %ld  (skipped pre-nav: %ld)\n", ticks, skipped_incomplete);
  printf("  ticks legacy railed   : %ld\n", ticks_railed);
  printf("  ticks gate closed     : %ld  (authority/stop says not ours)\n", ticks_gated);
  printf("\n");
  printf("  HARNESS FIDELITY (legacy replay vs logged outputs; timestamp jitter only)\n");
  printf("    thrust |d|  p50 %.4f  p99 %.4f  max %.4f\n",
         fid_th.pct(0.50), fid_th.pct(0.99), fid_th.maxv());
  printf("    rudder |d|  p50 %.4f  p99 %.4f  max %.4f\n",
         fid_rd.pct(0.50), fid_rd.pct(0.99), fid_rd.maxv());
  printf("\n");
  printf("  PARITY new/off vs legacy (claim: 0 on EVERY tick, railed included)\n");
  printf("    max |d| thrust %.3e   rudder %.3e\n", off_max_clean_th, off_max_clean_rd);
  printf("\n");
  printf("  DIVERGENCE new/on vs legacy (fixes biting; |d| > 0.5 windows)\n");
  for (size_t i = 0; i < on_div.size(); ++i)
    printf("    t %8.1f .. %8.1f  (%4d ticks)  peak |d| %.2f\n",
           on_div[i].t_start, on_div[i].t_end, on_div[i].ticks, on_div[i].peak);
  if (on_div.empty())
    printf("    (none)\n");
  if (csv) { fclose(csv); printf("\n  csv: %s\n", argv[2]); }

  // Exit status is the parity claim.
  bool pass = (off_max_clean_th < 1e-9) && (off_max_clean_rd < 1e-9) && (ticks > 0);
  printf("\n  PARITY: %s\n", pass ? "PASS" : "FAIL");
  return pass ? 0 : 1;
}
