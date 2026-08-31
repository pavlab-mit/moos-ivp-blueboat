/*************************************************************
 * dgps_ekf_replay -- offline A/B replay of pBB_DGPS_EKF on a
 * recorded .alog (checkpoint brief section 5: the GPS lever arm
 * and the unsigned-speed runaway).
 *
 * Two models run side by side on the logged input stream:
 *
 *   base : lever arm zero, measurement TS stripped -- replicates
 *          the pre-change filter exactly (re-application incl.)
 *   new  : lever-arm corrected measurements, TS-gated fusion,
 *          signed NAV_SURGE derived from the state
 *
 * Tick discipline: the app publishes NAV_X on every iterate in
 * which NAV was valid, so each logged NAV_X from pBB_DGPS_EKF
 * marks one predict+update tick. Mail (GNSS_STATE, gyro) is
 * applied at logged arrival times, exactly as OnNewMail cached
 * it ahead of the real Iterate.
 *
 * Reference signed surge is finite-differenced from the LOGGED
 * NAV_X/NAV_Y projected on the LOGGED NAV_HEADING (~0.4 s
 * window) -- the same construction the 31 Aug analysis used to
 * demonstrate the reversal, so the reversed window's numbers
 * are directly comparable. (It carries the lever-arm tangential
 * term during pivots; it is a reference, not truth.)
 *
 * Expected on LOG_YIP_31_8_2026_____12_05_45:
 *   base vs log      : small residuals (timestamp jitter only)
 *   new speed, pivots: collapses toward 0 where base holds ~0.25
 *   new surge, t 476+: tracks ref surge through zero to -1.48
 *                      while base speed reports +|v|
 *   straight phases  : base and new essentially identical
 *
 * Usage:
 *   dgps_ekf_replay <file.alog> [out.csv]
 *       [--lever=x,y]      body frame, m (default 0,0)
 *       [--origin=lat,lon] geodesy origin (default: parsed from
 *                          the ._moos snapshot beside the alog)
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "BB_DGPS_EKF_Model.hpp"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

typedef BB_DGPS_EKF_Model Model;

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

// Parse the app's GNSS_STATE string (same fields as parseGPSState).
static bool parse_gnss_state(const std::string& sval, Model::GPSMeasurement& gps)
{
  gps = Model::GPSMeasurement();
  std::stringstream ss(sval);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    size_t eq = tok.find('=');
    if (eq == std::string::npos) continue;
    std::string key = tok.substr(0, eq);
    std::string val = tok.substr(eq + 1);
    if      (key == "LAT")       gps.nav_lat = atof(val.c_str());
    else if (key == "LON")       gps.nav_lon = atof(val.c_str());
    else if (key == "SPD")       gps.speed = atof(val.c_str());
    else if (key == "COG")       gps.cog = atof(val.c_str());
    else if (key == "HDG")       gps.heading = atof(val.c_str());
    else if (key == "HDG_ACC")   gps.heading_acc = atof(val.c_str());
    else if (key == "HDG_VALID") gps.heading_valid = (val == "true");
    else if (key == "FIX")       gps.fix_type = atoi(val.c_str());
    else if (key == "HDOP")      gps.hdop = atof(val.c_str());
    else if (key == "H_ACC")     gps.h_acc = atof(val.c_str());
    else if (key == "TS")        gps.timestamp = atof(val.c_str());
  }
  gps.gps_lock = (gps.fix_type >= 2);
  return gps.isValid();
}

// Pull LatOrigin/LongOrigin from the ._moos snapshot beside the alog.
static bool origin_from_moos(const std::string& alog_path,
                             double& lat, double& lon)
{
  std::string moos_path = alog_path;
  size_t dot = moos_path.rfind(".alog");
  if (dot == std::string::npos) return false;
  moos_path.replace(dot, 5, "._moos");
  std::ifstream in(moos_path.c_str());
  if (!in) return false;

  bool got_lat = false, got_lon = false;
  std::string line;
  while (std::getline(in, line) && !(got_lat && got_lon)) {
    // Strip comments, find "key = value"
    size_t cmt = line.find("//");
    if (cmt != std::string::npos) line = line.substr(0, cmt);
    size_t eq = line.find('=');
    if (eq == std::string::npos) continue;
    std::string key = line.substr(0, eq);
    key.erase(std::remove_if(key.begin(), key.end(), ::isspace), key.end());
    for (size_t i = 0; i < key.size(); i++) key[i] = ::tolower(key[i]);
    if (key == "latorigin")  { lat = atof(line.c_str() + eq + 1); got_lat = true; }
    if (key == "longorigin") { lon = atof(line.c_str() + eq + 1); got_lon = true; }
  }
  return got_lat && got_lon;
}

static double wrap180(double deg)
{
  while (deg > 180.0) deg -= 360.0;
  while (deg < -180.0) deg += 360.0;
  return deg;
}

//---------------------------------------------------------
int main(int argc, char** argv)
{
  const char* alog_path = nullptr;
  const char* csv_path = nullptr;
  Model::LeverArm lever;
  double origin_lat = 0.0, origin_lon = 0.0;
  bool have_origin = false;

  for (int i = 1; i < argc; i++) {
    if (strncmp(argv[i], "--lever=", 8) == 0) {
      if (sscanf(argv[i] + 8, "%lf,%lf", &lever.x, &lever.y) != 2) {
        fprintf(stderr, "bad --lever (want x,y)\n"); return 2;
      }
    }
    else if (strncmp(argv[i], "--origin=", 9) == 0) {
      if (sscanf(argv[i] + 9, "%lf,%lf", &origin_lat, &origin_lon) != 2) {
        fprintf(stderr, "bad --origin (want lat,lon)\n"); return 2;
      }
      have_origin = true;
    }
    else if (!alog_path) alog_path = argv[i];
    else if (!csv_path)  csv_path = argv[i];
  }
  if (!alog_path) {
    fprintf(stderr,
        "usage: dgps_ekf_replay <file.alog> [out.csv] [--lever=x,y] [--origin=lat,lon]\n");
    return 2;
  }

  if (!have_origin && !origin_from_moos(alog_path, origin_lat, origin_lon)) {
    fprintf(stderr, "no --origin and no LatOrigin in ._moos beside alog\n");
    return 2;
  }

  CMOOSGeodesy geodesy;
  if (!geodesy.Initialise(origin_lat, origin_lon)) {
    fprintf(stderr, "geodesy init failed\n"); return 2;
  }

  std::ifstream in(alog_path);
  if (!in) { fprintf(stderr, "cannot open %s\n", alog_path); return 2; }
  FILE* csv = nullptr;
  if (csv_path) {
    csv = fopen(csv_path, "w");
    if (!csv) { fprintf(stderr, "cannot open %s\n", csv_path); return 2; }
    fprintf(csv, "t,log_x,log_y,log_heading,log_speed,"
                 "base_x,base_y,base_heading,base_speed,"
                 "new_x,new_y,new_heading,new_speed,new_surge,new_sway,"
                 "ref_surge,gyro_z\n");
  }

  Model base, model;                    // base: pre-change replica
  Model::LeverArm zero_lever;

  // Latest-mail state
  Model::GPSMeasurement latest_gps;     // raw antenna-frame, as parsed
  bool have_gps = false;
  double last_gps_time = -1e9, last_gyro_time = -1e9;
  double gyro_z = 0.0;

  // Logged EKF outputs (tick markers + fidelity reference)
  double log_x = 0.0, log_y = 0.0, log_heading = 0.0, log_speed = 0.0;
  bool have_log_y = false, have_log_h = false, have_log_s = false;
  double last_tick_t = -1.0;

  // Reference signed surge from logged positions (~0.4 s window)
  struct PosSample { double t, x, y; };
  std::deque<PosSample> pos_hist;

  Stats fid_speed, fid_heading, fid_x, fid_y;   // base vs log
  Stats dv_speed;                               // new vs base, |speed diff|
  Stats surge_err;                              // new surge vs ref (|v|>0.4)
  long ticks = 0, fused = 0, skipped = 0;
  double t_min_new_surge = 0.0, min_new_surge = 0.0;

  const double FRESH = 5.0;

  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '%') continue;
    std::istringstream ss(line);
    double t; std::string var, src;
    if (!(ss >> t >> var >> src)) continue;
    std::string val;
    std::getline(ss, val);
    // strip leading whitespace
    size_t b = val.find_first_not_of(" \t");
    val = (b == std::string::npos) ? "" : val.substr(b);

    if (var == "GNSS_STATE") {
      Model::GPSMeasurement gps;
      if (parse_gnss_state(val, gps)) {
        // geodesy: LatLong2LocalGrid(lat, lon, north(y), east(x))
        double x, y;
        if (geodesy.LatLong2LocalGrid(gps.nav_lat, gps.nav_lon, y, x)) {
          gps.nav_x = x;
          gps.nav_y = y;
        }
        latest_gps = gps;
        have_gps = true;
        last_gps_time = t;
      }
    }
    else if (var == "GYRO_Z_LVL_IMU") {
      gyro_z = atof(val.c_str());
      last_gyro_time = t;
    }
    else if (src != "pBB_DGPS_EKF") {
      continue;
    }
    else if (var == "NAV_Y")       { log_y = atof(val.c_str()); have_log_y = true; }
    else if (var == "NAV_HEADING") { log_heading = atof(val.c_str()); have_log_h = true; }
    else if (var == "NAV_SPEED")   { log_speed = atof(val.c_str()); have_log_s = true; }
    else if (var == "NAV_X") {
      // One logged NAV_X == one real iterate that published: the tick.
      log_x = atof(val.c_str());
      if (!have_gps) continue;

      bool gps_fresh = (t - last_gps_time) < FRESH;
      bool gyro_fresh = (t - last_gyro_time) < FRESH;

      // --- base: pre-change replica (no lever, TS stripped) ---
      Model::GPSMeasurement gps_base = latest_gps;
      gps_base.timestamp = 0.0;                    // always fuse (old behavior)

      // --- new: lever-arm corrected, TS-gated ---
      Model::GPSMeasurement gps_hull = latest_gps;
      if (gps_fresh && gps_hull.isValid()) {
        double phi_cart;
        if (model.isInitialized())
          phi_cart = model.getHeading();
        else if (gps_hull.heading_valid && gps_hull.heading_acc > 0 &&
                 gps_hull.heading_acc < 10.0)
          phi_cart = Model::compassToCartesian(gps_hull.heading);
        else
          phi_cart = Model::compassToCartesian(gps_hull.cog);
        Model::correctLeverArm(gps_hull, gyro_z, gyro_fresh, phi_cart, lever);
      }

      if (!base.isInitialized() && gps_fresh && gps_base.isValid()) {
        base.initialize(gps_base);
        model.initialize(gps_hull);
        last_tick_t = t;
        continue;
      }
      if (!base.isInitialized()) continue;

      double dt = (last_tick_t > 0) ? (t - last_tick_t) : 0.0625;
      dt = std::max(0.001, std::min(dt, 1.0));
      last_tick_t = t;

      double gz = gyro_fresh ? gyro_z : 0.0;
      base.predict(dt, gz);
      model.predict(dt, gz);
      if (gps_fresh && gps_base.isValid()) {
        base.updateGPS(gps_base, 0.3);
        if (model.updateGPS(gps_hull, 0.3)) fused++; else skipped++;
      }
      ticks++;

      // Reference signed surge from LOGGED positions
      double ref_surge = 0.0;
      bool have_ref = false;
      if (have_log_y && have_log_h) {
        PosSample psamp; psamp.t = t; psamp.x = log_x; psamp.y = log_y;
        pos_hist.push_back(psamp);
        while (pos_hist.size() > 2 && (t - pos_hist.front().t) > 0.45)
          pos_hist.pop_front();
        if ((t - pos_hist.front().t) > 0.2) {
          double ddt = t - pos_hist.front().t;
          double vx = (log_x - pos_hist.front().x) / ddt;
          double vy = (log_y - pos_hist.front().y) / ddt;
          double phi = Model::compassToCartesian(log_heading);
          ref_surge = vx * std::cos(phi) + vy * std::sin(phi);
          have_ref = true;
        }
      }

      // Accounting
      if (have_log_s) fid_speed.add(base.getSpeed() - log_speed);
      if (have_log_h) fid_heading.add(wrap180(base.getHeadingCompass() - log_heading));
      fid_x.add(base.getX() - log_x);
      if (have_log_y) fid_y.add(base.getY() - log_y);
      dv_speed.add(model.getSpeed() - base.getSpeed());
      if (have_ref && std::fabs(ref_surge) > 0.4)
        surge_err.add(model.getSurge() - ref_surge);
      if (model.getSurge() < min_new_surge) {
        min_new_surge = model.getSurge();
        t_min_new_surge = t;
      }

      if (csv)
        fprintf(csv, "%.5f,%.3f,%.3f,%.2f,%.3f,"
                     "%.3f,%.3f,%.2f,%.3f,"
                     "%.3f,%.3f,%.2f,%.3f,%.3f,%.3f,"
                     "%.3f,%.4f\n",
                t, log_x, log_y, log_heading, log_speed,
                base.getX(), base.getY(), base.getHeadingCompass(), base.getSpeed(),
                model.getX(), model.getY(), model.getHeadingCompass(),
                model.getSpeed(), model.getSurge(), model.getSway(),
                have_ref ? ref_surge : 0.0, gz);
    }
  }

  printf("dgps_ekf_replay: %s\n", alog_path);
  printf("  lever arm             : (%.3f fwd, %.3f stbd) m\n", lever.x, lever.y);
  printf("  origin                : %.7f, %.7f\n", origin_lat, origin_lon);
  printf("  ticks replayed        : %ld\n", ticks);
  printf("  epochs fused / skips  : %ld / %ld (TS gate)\n", fused, skipped);
  printf("\n");
  printf("  BASE FIDELITY (pre-change replica vs logged outputs)\n");
  printf("    speed   |d|  p50 %.4f  p99 %.4f  max %.4f\n",
         fid_speed.pct(0.50), fid_speed.pct(0.99), fid_speed.maxv());
  printf("    heading |d|  p50 %.4f  p99 %.4f  max %.4f\n",
         fid_heading.pct(0.50), fid_heading.pct(0.99), fid_heading.maxv());
  printf("    x       |d|  p50 %.4f  p99 %.4f  max %.4f\n",
         fid_x.pct(0.50), fid_x.pct(0.99), fid_x.maxv());
  printf("    y       |d|  p50 %.4f  p99 %.4f  max %.4f\n",
         fid_y.pct(0.50), fid_y.pct(0.99), fid_y.maxv());
  printf("\n");
  printf("  NEW vs BASE speed |d|  p50 %.4f  p90 %.4f  max %.4f\n",
         dv_speed.pct(0.50), dv_speed.pct(0.90), dv_speed.maxv());
  printf("  NEW surge vs ref (|ref|>0.4)  p50 %.4f  p90 %.4f  max %.4f  (n=%zu)\n",
         surge_err.pct(0.50), surge_err.pct(0.90), surge_err.maxv(),
         surge_err.v.size());
  printf("  NEW min surge         : %.3f m/s at t %.1f\n",
         min_new_surge, t_min_new_surge);
  if (csv) { fclose(csv); printf("\n  csv: %s\n", csv_path); }

  return ticks > 0 ? 0 : 1;
}
