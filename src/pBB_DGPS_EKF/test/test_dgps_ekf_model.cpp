/*************************************************************
 * Unit tests for BB_DGPS_EKF_Model.
 *
 * The scenarios are the 2026-08-31 incidents (checkpoint brief
 * section 5): the GPS lever arm painting ~0.25 m/s of phantom
 * speed onto a stationary pivoting hull, the rectified speed
 * that made spd=0 an unstable equilibrium in reverse, and the
 * GNSS epoch re-application that collapses covariance.
 *
 * Conventions under test are anchored to analysis/rc_cal.py:
 * lateral offset d starboard-positive, pivot contamination on
 * surge = -radians(yawrate_compass) * d.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "BB_DGPS_EKF_Model.hpp"

#include <cmath>
#include <cstdio>
#include <string>

typedef BB_DGPS_EKF_Model Model;

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
    fprintf(stderr, "FAIL: %s (got %.9f want %.9f)\n", what.c_str(), got, want);
  }
}

// Build the antenna-frame measurement a receiver would report for a
// hull with the given true state: the exact forward model the
// correction must invert. All angles compass degrees.
static Model::GPSMeasurement antenna_meas(double hull_x, double hull_y,
                                          double heading_deg,
                                          double vx_hull, double vy_hull,
                                          double gyro_z_cart,
                                          const Model::LeverArm& lever)
{
  double phi = Model::compassToCartesian(heading_deg);
  double c = std::cos(phi), s = std::sin(phi);
  double rx = lever.x * c + lever.y * s;
  double ry = lever.x * s - lever.y * c;

  // v_ant = v_hull + omega x r ; z_hat x (rx, ry) = (-ry, rx)
  double vx = vx_hull + gyro_z_cart * (-ry);
  double vy = vy_hull + gyro_z_cart * ( rx);

  Model::GPSMeasurement gps;
  gps.nav_x = hull_x + rx;
  gps.nav_y = hull_y + ry;
  gps.speed = std::sqrt(vx * vx + vy * vy);
  gps.cog = Model::cartesianToCompass(std::atan2(vy, vx));
  gps.heading = heading_deg;
  gps.heading_valid = true;
  gps.heading_acc = 0.5;
  gps.h_acc = 0.05;
  gps.fix_type = 3;
  gps.gps_lock = true;
  gps.nav_lat = 42.0;    // non-zero so isValid() passes
  gps.nav_lon = -71.0;
  return gps;
}

// The yip geometry and the 31 Aug pivot numbers.
static const double kYipLeverY = -0.356;   // antennas on the PORT pontoon

//---------------------------------------------------------
// The incident case: hull dead stationary, pivoting at 40 deg/s
// (compass CW+). Antenna paints ~0.25 m/s of phantom speed;
// correction must take it to zero.
static void test_pivot_phantom_speed()
{
  Model::LeverArm lever; lever.x = 0.0; lever.y = kYipLeverY;
  double yr_compass = 40.0;                        // deg/s, CW+
  double gyro_z = -yr_compass * M_PI / 180.0;      // CCW+ rad/s

  Model::GPSMeasurement gps =
      antenna_meas(10.0, 20.0, 0.0, 0.0, 0.0, gyro_z, lever);

  // Uncorrected: |omega|*|d| phantom speed -- the measured ~0.25 m/s
  check_near(gps.speed, std::fabs(gyro_z * kYipLeverY), 1e-12,
             "pivot: antenna phantom speed = |omega*d|");
  check_near(gps.speed, 0.2486, 5e-4, "pivot: phantom ~0.25 m/s at 40 deg/s");

  // rc_cal.py convention: surge contamination = -radians(yr)*d = gyro_z*d
  double gamma = Model::compassToCartesian(gps.cog);
  double phi   = Model::compassToCartesian(0.0);
  double surge_ant = gps.speed * std::cos(gamma - phi);
  check_near(surge_ant, gyro_z * kYipLeverY, 1e-12,
             "pivot: antenna surge matches rc_cal -radians(yr)*d");

  Model::correctLeverArm(gps, gyro_z, true, phi, lever);
  check_near(gps.speed, 0.0, 1e-9, "pivot: corrected speed is zero");
  check_near(gps.nav_x, 10.0, 1e-12, "pivot: corrected x is hull x");
  check_near(gps.nav_y, 20.0, 1e-12, "pivot: corrected y is hull y");
}

//---------------------------------------------------------
// Straight line, no rotation: velocity untouched, position shifted
// by exactly the rotated lever arm.
static void test_straight_line()
{
  Model::LeverArm lever; lever.x = 0.2; lever.y = kYipLeverY;
  // Heading east, moving east at 1.5 m/s
  Model::GPSMeasurement gps =
      antenna_meas(0.0, 0.0, 90.0, 1.5, 0.0, 0.0, lever);

  double phi = Model::compassToCartesian(90.0);
  check_near(gps.speed, 1.5, 1e-12, "straight: antenna speed = hull speed");

  Model::correctLeverArm(gps, 0.0, true, phi, lever);
  check_near(gps.speed, 1.5, 1e-12, "straight: speed unchanged");
  check_near(gps.cog, 90.0, 1e-9, "straight: cog unchanged");
  check_near(gps.nav_x, 0.0, 1e-12, "straight: x recovered");
  check_near(gps.nav_y, 0.0, 1e-12, "straight: y recovered");
}

//---------------------------------------------------------
// Turn while underway: the 0.16 m/s bias at 25 deg/s from the brief.
static void test_turn_underway()
{
  Model::LeverArm lever; lever.x = 0.0; lever.y = kYipLeverY;
  double yr_compass = 25.0;
  double gyro_z = -yr_compass * M_PI / 180.0;

  // Heading north at 1.5 m/s
  Model::GPSMeasurement gps =
      antenna_meas(5.0, -3.0, 0.0, 0.0, 1.5, gyro_z, lever);

  // Antenna surge bias: gyro_z*d = +0.155 m/s (brief: ~0.16 at 25 deg/s)
  double phi = Model::compassToCartesian(0.0);
  double gamma = Model::compassToCartesian(gps.cog);
  double surge_ant = gps.speed * std::cos(gamma - phi);
  check_near(surge_ant - 1.5, gyro_z * kYipLeverY, 1e-9,
             "turn: surge bias = gyro_z*d while underway");

  Model::correctLeverArm(gps, gyro_z, true, phi, lever);
  check_near(gps.speed, 1.5, 1e-9, "turn: corrected speed = hull speed");
  check_near(gps.cog, 0.0, 1e-6, "turn: corrected cog = hull course");
  check_near(gps.nav_x, 5.0, 1e-12, "turn: x recovered");
  check_near(gps.nav_y, -3.0, 1e-12, "turn: y recovered");
}

//---------------------------------------------------------
// Zero lever arm must be a bit-exact no-op (regression guard: the
// default config reproduces pre-change behavior exactly).
static void test_zero_lever_noop()
{
  Model::LeverArm zero;
  Model::GPSMeasurement gps;
  gps.nav_x = 1.234; gps.nav_y = -5.678;
  gps.speed = 0.9; gps.cog = 123.4;

  Model::GPSMeasurement before = gps;
  Model::correctLeverArm(gps, -0.7, true, 0.3, zero);
  check(gps.nav_x == before.nav_x && gps.nav_y == before.nav_y &&
        gps.speed == before.speed && gps.cog == before.cog,
        "zero lever arm: measurement bit-identical");
}

//---------------------------------------------------------
// Stale gyro: position still corrected, velocity left alone (a stale
// omega would inject a wrong correction).
static void test_stale_gyro()
{
  Model::LeverArm lever; lever.x = 0.0; lever.y = kYipLeverY;
  double gyro_z = -40.0 * M_PI / 180.0;
  Model::GPSMeasurement gps =
      antenna_meas(0.0, 0.0, 0.0, 0.0, 0.0, gyro_z, lever);
  double phantom = gps.speed;
  double phi = Model::compassToCartesian(0.0);

  Model::correctLeverArm(gps, gyro_z, false /*gyro stale*/, phi, lever);
  check_near(gps.nav_x, 0.0, 1e-12, "stale gyro: position still corrected (x)");
  check_near(gps.nav_y, 0.0, 1e-12, "stale gyro: position still corrected (y)");
  check_near(gps.speed, phantom, 1e-12, "stale gyro: velocity untouched");
}

//---------------------------------------------------------
// Signed surge/sway projections from the state (no fusion needed:
// the explicit initialize overload sets the state directly).
static void test_surge_sign()
{
  double phi = Model::compassToCartesian(0.0);   // heading north

  // Ahead at 1.2: gamma = phi
  Model m1; m1.initialize(0, 0, phi, phi, 1.2);
  check_near(m1.getSurge(), 1.2, 1e-12, "surge: ahead is positive");
  check_near(m1.getSway(), 0.0, 1e-12, "surge: no sway ahead");
  check_near(m1.getSpeed(), 1.2, 1e-12, "surge: speed stays magnitude");

  // Astern at 1.48 (the runaway's settled speed): COG south, heading north
  double gamma_astern = Model::compassToCartesian(180.0);
  Model m2; m2.initialize(0, 0, phi, gamma_astern, 1.48);
  check_near(m2.getSurge(), -1.48, 1e-9, "surge: astern is negative");
  check_near(m2.getSpeed(), 1.48, 1e-12, "surge: speed still rectified");

  // Pure starboard drift: heading north, COG east
  double gamma_stbd = Model::compassToCartesian(90.0);
  Model m3; m3.initialize(0, 0, phi, gamma_stbd, 0.5);
  check_near(m3.getSurge(), 0.0, 1e-9, "sway: no surge drifting abeam");
  check_near(m3.getSway(), 0.5, 1e-9, "sway: starboard drift is positive");

  // Pure port drift
  double gamma_port = Model::compassToCartesian(270.0);
  Model m4; m4.initialize(0, 0, phi, gamma_port, 0.5);
  check_near(m4.getSway(), -0.5, 1e-9, "sway: port drift is negative");
}

//---------------------------------------------------------
// The runaway mechanism itself: with spd=0 commanded and the boat
// slipping astern, spdErr = cmd - feedback must go POSITIVE (recovery)
// on the surge channel where the rectified channel goes negative
// (more reverse -- the positive feedback of the 31 Aug incident).
static void test_runaway_error_sign()
{
  double phi = Model::compassToCartesian(0.0);
  double gamma_astern = Model::compassToCartesian(180.0);
  Model m; m.initialize(0, 0, phi, gamma_astern, 0.75);

  double cmd = 0.0;
  double err_rectified = cmd - m.getSpeed();
  double err_surge     = cmd - m.getSurge();
  check(err_rectified < 0.0, "runaway: rectified error commands more reverse");
  check(err_surge > 0.0, "runaway: surge error commands recovery");
}

//---------------------------------------------------------
// TS gate: each GNSS epoch fuses once; a repeat is skipped and
// leaves state AND covariance untouched (no covariance collapse).
static void test_ts_gate()
{
  Model::LeverArm lever;  // zero
  Model::GPSMeasurement gps =
      antenna_meas(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, lever);
  gps.timestamp = 100.0;

  Model m;
  m.initialize(gps);
  check(m.isInitialized(), "ts: initialized");

  // The same epoch arriving again right after init must be skipped.
  check(!m.updateGPS(gps, 0.3), "ts: init epoch not re-fused");

  // Advance, then a fresh epoch fuses.
  m.predict(0.0625, 0.0);
  gps.timestamp = 100.1;
  gps.nav_x = 0.05;
  check(m.updateGPS(gps, 0.3), "ts: new epoch fuses");

  // Re-applying it repeatedly must change nothing at all.
  double x0 = m.getX(), y0 = m.getY();
  double sx0 = m.getStdX(), ss0 = m.getStdSpeed(), sp0 = m.getStdPhi();
  for (int i = 0; i < 5; i++)
    check(!m.updateGPS(gps, 0.3), "ts: repeat epoch skipped");
  check(m.getX() == x0 && m.getY() == y0, "ts: state untouched by repeats");
  check(m.getStdX() == sx0 && m.getStdSpeed() == ss0 && m.getStdPhi() == sp0,
        "ts: covariance untouched by repeats (no collapse)");

  // TS-less sources (timestamp 0) keep the old always-fuse behavior.
  Model::GPSMeasurement gps_nots =
      antenna_meas(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, lever);
  gps_nots.timestamp = 0.0;
  Model m2;
  m2.initialize(gps_nots);
  check(m2.updateGPS(gps_nots, 0.3), "ts: TS-less measurement fuses (1)");
  check(m2.updateGPS(gps_nots, 0.3), "ts: TS-less measurement fuses (2)");
}

//---------------------------------------------------------
// End-to-end mini-scenario: stationary hull, pivot commanded from
// rest (the 31 Aug trigger). Corrected filter must NOT develop the
// phantom speed the uncorrected one does.
static void test_filter_pivot_scenario()
{
  Model::LeverArm lever; lever.x = 0.0; lever.y = kYipLeverY;
  double yr_compass = 40.0;
  double gyro_z = -yr_compass * M_PI / 180.0;

  Model corrected, uncorrected;
  double heading = 0.0;
  double ts = 1000.0;

  for (int i = 0; i < 100; i++) {                 // 10 s at 10 Hz GPS
    Model::GPSMeasurement ant =
        antenna_meas(0.0, 0.0, heading, 0.0, 0.0, gyro_z, lever);
    ant.timestamp = ts;

    Model::GPSMeasurement hull = ant;             // corrected copy
    double phi = corrected.isInitialized()
                     ? corrected.getHeading()
                     : Model::compassToCartesian(heading);
    Model::correctLeverArm(hull, gyro_z, true, phi, lever);

    if (!corrected.isInitialized()) {
      corrected.initialize(hull);
      uncorrected.initialize(ant);
    } else {
      corrected.predict(0.1, gyro_z);
      uncorrected.predict(0.1, gyro_z);
      corrected.updateGPS(hull, 0.3);
      uncorrected.updateGPS(ant, 0.3);
    }

    heading += yr_compass * 0.1;
    ts += 0.1;
  }

  check(uncorrected.getSpeed() > 0.2,
        "scenario: uncorrected filter shows phantom pivot speed");
  check_near(corrected.getSpeed(), 0.0, 0.02,
             "scenario: corrected filter stays near zero speed");
  check(std::fabs(corrected.getSurge()) < 0.02,
        "scenario: corrected surge near zero through pivot");
}

//---------------------------------------------------------
int main()
{
  test_pivot_phantom_speed();
  test_straight_line();
  test_turn_underway();
  test_zero_lever_noop();
  test_stale_gyro();
  test_surge_sign();
  test_runaway_error_sign();
  test_ts_gate();
  test_filter_pivot_scenario();

  printf("test_dgps_ekf_model: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
