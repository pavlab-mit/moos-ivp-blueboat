/*************************************************************
      Orgn: MIT, Cambridge MA
      File: sample_navigator/main.cpp
   Last Ed: 2026-07-26 (navigator-cpp port)
     Brief:
        Navigator sensor sampler / calibration data collector.
        Streams raw + calibrated IMU/mag samples to a CSV whose
        column format is consumed by:
            scripts/get_bb_nav_gyro_cal.py   (gyro bias)
            scripts/get_bb_nav_mag_cal.py    (mag hard/soft iron)

        Workflows:
          Gyro cal:  boat STATIONARY, mission STOPPED (sensors are
                     single-owner):
              sample_navigator --duration 30
              python3 get_bb_nav_gyro_cal.py nav_sample_*.csv
          Mag cal:   slow full rotations (the compass dance):
              sample_navigator --duration 180
              python3 get_bb_nav_mag_cal.py nav_sample_*.csv

        --gyro-cal / --mag-cal LOAD existing calibration files so
        the nav_*_cal_* columns and attitude reflect them (same
        file formats the interface app reads).

        Ported from the navigator-lib original: the MCC UDP
        broadcast and Madgwick filter are gone; attitude columns
        come from navigator-cpp's built-in Allgeuer estimator.
*************************************************************/

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>

#include "nav_bindings.h"

using namespace std;

static Navigator g_nav;
static volatile sig_atomic_t g_interrupted = 0;
static void sigHandler(int) { g_interrupted = 1; }

struct GyroBias {
  bool valid = false;
  double bx = 0, by = 0, bz = 0;
};

struct MagCal {
  bool valid = false;
  double hard[3] = {0, 0, 0};
  double soft[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};  // row-major
};

// Accepts both formats (matches BBNavigatorInterface::readImuCalFile):
//   gyro_bias = x,y,z      |      bias_x = v / bias_y = v / bias_z = v
static GyroBias loadGyroBias(const string &path) {
  GyroBias gb;
  ifstream f(path);
  if (!f.is_open()) return gb;
  string line;
  while (getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    size_t eq = line.find('=');
    if (eq == string::npos) continue;
    string key = line.substr(0, eq);
    string val = line.substr(eq + 1);
    key.erase(remove_if(key.begin(), key.end(), ::isspace), key.end());
    if (key == "gyro_bias") {
      if (sscanf(val.c_str(), " %lf , %lf , %lf", &gb.bx, &gb.by, &gb.bz) == 3)
        gb.valid = true;
    } else if (key == "bias_x") { gb.bx = atof(val.c_str()); gb.valid = true; }
    else if (key == "bias_y")   { gb.by = atof(val.c_str()); gb.valid = true; }
    else if (key == "bias_z")   { gb.bz = atof(val.c_str()); gb.valid = true; }
  }
  return gb;
}

// Format (matches BBNavigatorInterface::readMagCalFile):
//   b = bx,by,bz
//   A = a11,...,a33  (row-major)
static MagCal loadMagCal(const string &path) {
  MagCal mc;
  ifstream f(path);
  if (!f.is_open()) return mc;
  string line;
  bool have_b = false, have_a = false;
  while (getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    size_t eq = line.find('=');
    if (eq == string::npos) continue;
    string key = line.substr(0, eq);
    string val = line.substr(eq + 1);
    key.erase(remove_if(key.begin(), key.end(), ::isspace), key.end());
    if (key == "b") {
      if (sscanf(val.c_str(), " %lf , %lf , %lf",
                 &mc.hard[0], &mc.hard[1], &mc.hard[2]) == 3)
        have_b = true;
    } else if (key == "A") {
      if (sscanf(val.c_str(),
                 " %lf , %lf , %lf , %lf , %lf , %lf , %lf , %lf , %lf",
                 &mc.soft[0], &mc.soft[1], &mc.soft[2],
                 &mc.soft[3], &mc.soft[4], &mc.soft[5],
                 &mc.soft[6], &mc.soft[7], &mc.soft[8]) == 9)
        have_a = true;
    }
  }
  mc.valid = have_b && have_a;
  return mc;
}

static void mat3_mult(const double A[9], const double B[9], double C[9]) {
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      C[3 * r + c] = A[3 * r] * B[c] + A[3 * r + 1] * B[3 + c] +
                     A[3 * r + 2] * B[6 + c];
}

static void mat3_vec(const double A[9], const double v[3], double out[3]) {
  for (int r = 0; r < 3; r++)
    out[r] = A[3 * r] * v[0] + A[3 * r + 1] * v[1] + A[3 * r + 2] * v[2];
}

static void showHelpAndExit() {
  printf(
      "Usage: sample_navigator [options]\n"
      "  -d, --duration <s>        Sample duration (default: run until Ctrl-C)\n"
      "  -r, --rate <hz>           Sample rate (default: 50)\n"
      "  -o, --output <path>       Output CSV (default: ./nav_sample_<ts>.csv)\n"
      "  --gyro-cal <path>         Load gyro_bias.dat so calibrated columns apply it\n"
      "  --mag-cal <path>          Load mag_cal_nav.dat so calibrated columns apply it\n"
      "  --roll-offset <deg>       Mounting roll offset (default 0)\n"
      "  --pitch-offset <deg>      Mounting pitch offset (default 0)\n"
      "  --yaw-offset <deg>        Mounting yaw offset (default 0)\n"
      "  -h, --help                Show this help\n"
      "\n"
      "Calibration workflows (STOP the mission first - sensors are single-owner):\n"
      "  Gyro: boat stationary:  sample_navigator -d 30\n"
      "        then: python3 scripts/get_bb_nav_gyro_cal.py nav_sample_*.csv\n"
      "  Mag:  slow rotations:   sample_navigator -d 180\n"
      "        then: python3 scripts/get_bb_nav_mag_cal.py nav_sample_*.csv\n");
  exit(0);
}

int main(int ac, char *av[]) {
  double duration = -1.0;  // <0 => until Ctrl-C
  double rate = 50.0;
  string output_path;
  string gyro_cal_path, mag_cal_path;
  double roll_off = 0, pitch_off = 0, yaw_off = 0;

  for (int i = 1; i < ac; i++) {
    string argi = av[i];
    auto need = [&](const char *what) -> string {
      if (++i >= ac) { cerr << what << " requires a value\n"; exit(1); }
      return av[i];
    };
    if (argi == "-h" || argi == "--help") showHelpAndExit();
    else if (argi == "-d" || argi == "--duration") duration = stod(need("--duration"));
    else if (argi.find("--duration=") == 0) duration = stod(argi.substr(11));
    else if (argi == "-r" || argi == "--rate") rate = stod(need("--rate"));
    else if (argi.find("--rate=") == 0) rate = stod(argi.substr(7));
    else if (argi == "-o" || argi == "--output") output_path = need("--output");
    else if (argi.find("--output=") == 0) output_path = argi.substr(9);
    else if (argi == "--gyro-cal") gyro_cal_path = need("--gyro-cal");
    else if (argi.find("--gyro-cal=") == 0) gyro_cal_path = argi.substr(11);
    else if (argi == "--mag-cal") mag_cal_path = need("--mag-cal");
    else if (argi.find("--mag-cal=") == 0) mag_cal_path = argi.substr(10);
    else if (argi == "--roll-offset") roll_off = stod(need("--roll-offset"));
    else if (argi == "--pitch-offset") pitch_off = stod(need("--pitch-offset"));
    else if (argi == "--yaw-offset") yaw_off = stod(need("--yaw-offset"));
    else {
      cerr << "Unhandled argument: " << argi << " (see --help)\n";
      return 1;
    }
  }

  if (rate <= 0 || rate > 500) { cerr << "rate must be in (0, 500] Hz\n"; return 1; }

  if (output_path.empty()) {
    char ts[32];
    time_t t = time(nullptr);
    strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", localtime(&t));
    output_path = string("nav_sample_") + ts + ".csv";
  }

  signal(SIGINT, sigHandler);
  signal(SIGTERM, sigHandler);

  GyroBias gb;
  if (!gyro_cal_path.empty()) gb = loadGyroBias(gyro_cal_path);
  MagCal mc;
  if (!mag_cal_path.empty()) mc = loadMagCal(mag_cal_path);

  {
    string err = g_nav.init();
    if (!err.empty())
      cerr << "navigator init warnings: " << err << endl;
  }
  g_nav.ahrs_set_mag_calib(0.0, 0.0, 0.0);  // attitude columns are IMU-only
  g_nav.ahrs_reset(true, true);

  // Mounting rotation R_offset = R_yaw * R_pitch * R_roll
  const double rr = roll_off * M_PI / 180.0;
  const double pr = pitch_off * M_PI / 180.0;
  const double yr = yaw_off * M_PI / 180.0;
  const double R_roll[9] = {1, 0, 0, 0, cos(rr), -sin(rr), 0, sin(rr), cos(rr)};
  const double R_pitch[9] = {cos(pr), 0, sin(pr), 0, 1, 0, -sin(pr), 0, cos(pr)};
  const double R_yaw[9] = {cos(yr), -sin(yr), 0, sin(yr), cos(yr), 0, 0, 0, 1};
  double R_tmp[9], R_off[9];
  mat3_mult(R_pitch, R_roll, R_tmp);
  mat3_mult(R_yaw, R_tmp, R_off);

  FILE *csv = fopen(output_path.c_str(), "w");
  if (!csv) { cerr << "Failed to open: " << output_path << endl; return 1; }
  // EXACT column format consumed by get_bb_nav_gyro_cal.py / get_bb_nav_mag_cal.py
  fprintf(csv,
          "timestamp_s,"
          "nav_mag_x_ut,nav_mag_y_ut,nav_mag_z_ut,"
          "nav_accel_x_ms2,nav_accel_y_ms2,nav_accel_z_ms2,"
          "nav_gyro_x_rads,nav_gyro_y_rads,nav_gyro_z_rads,"
          "nav_mag_cal_x_ut,nav_mag_cal_y_ut,nav_mag_cal_z_ut,"
          "nav_gyro_cal_x_rads,nav_gyro_cal_y_rads,nav_gyro_cal_z_rads,"
          "nav_roll_rad,nav_pitch_rad,nav_yaw_rad\n");

  printf("sample_navigator (navigator-cpp) -> %s\n", output_path.c_str());
  if (duration > 0) printf("  Duration: %.0f s, Rate: %.0f Hz\n", duration, rate);
  else              printf("  Duration: until Ctrl-C, Rate: %.0f Hz\n", rate);
  printf("  Gyro cal: %s\n", gb.valid ? gyro_cal_path.c_str() : "none (raw = cal)");
  printf("  Mag cal:  %s\n", mc.valid ? mag_cal_path.c_str() : "none (raw = cal)");

  const double nominal_dt = 1.0 / rate;
  auto start = chrono::high_resolution_clock::now();
  auto prev = start;
  auto last_print = start;
  long count = 0, errors = 0;

  while (!g_interrupted) {
    auto now = chrono::high_resolution_clock::now();
    double elapsed = chrono::duration<double>(now - start).count();
    if (duration > 0 && elapsed >= duration) break;

    NavAxisData mag_raw, acc_raw, gyro_raw;
    string em = g_nav.read_mag_ak09915(mag_raw);
    string ea = g_nav.read_accel(acc_raw);
    string eg = g_nav.read_gyro(gyro_raw);

    if (ea.empty() && eg.empty()) {
      // Mounting rotation (mag zeroed if its read failed)
      double mag_v[3] = {0, 0, 0}, acc_v[3], gyro_v[3];
      double m_in[3] = {mag_raw.x, mag_raw.y, mag_raw.z};
      double a_in[3] = {acc_raw.x, acc_raw.y, acc_raw.z};
      double g_in[3] = {gyro_raw.x, gyro_raw.y, gyro_raw.z};
      if (em.empty()) mat3_vec(R_off, m_in, mag_v);
      mat3_vec(R_off, a_in, acc_v);
      mat3_vec(R_off, g_in, gyro_v);

      double gyro_cal[3] = {gyro_v[0], gyro_v[1], gyro_v[2]};
      if (gb.valid) {
        gyro_cal[0] -= gb.bx;
        gyro_cal[1] -= gb.by;
        gyro_cal[2] -= gb.bz;
      }

      double mag_cal[3] = {mag_v[0], mag_v[1], mag_v[2]};
      if (mc.valid) {
        double centered[3] = {mag_v[0] - mc.hard[0], mag_v[1] - mc.hard[1],
                              mag_v[2] - mc.hard[2]};
        mat3_vec(mc.soft, centered, mag_cal);
      }

      double dt = chrono::duration<double>(now - prev).count();
      if (dt < 0.5 * nominal_dt) dt = 0.5 * nominal_dt;
      if (dt > 3.0 * nominal_dt) dt = 3.0 * nominal_dt;
      g_nav.ahrs_update(dt, gyro_cal[0], gyro_cal[1], gyro_cal[2],
                        acc_v[0], acc_v[1], acc_v[2]);
      NavAttitudeData att;
      g_nav.ahrs_get_attitude(att);

      fprintf(csv,
              "%.4f,"
              "%.4f,%.4f,%.4f,"
              "%.5f,%.5f,%.5f,"
              "%.6f,%.6f,%.6f,"
              "%.4f,%.4f,%.4f,"
              "%.6f,%.6f,%.6f,"
              "%.5f,%.5f,%.5f\n",
              elapsed,
              mag_v[0], mag_v[1], mag_v[2],
              acc_v[0], acc_v[1], acc_v[2],
              gyro_v[0], gyro_v[1], gyro_v[2],
              mag_cal[0], mag_cal[1], mag_cal[2],
              gyro_cal[0], gyro_cal[1], gyro_cal[2],
              (double)att.roll, (double)att.pitch, (double)att.yaw);
      count++;
    } else {
      errors++;
    }

    prev = now;
    if (chrono::duration<double>(now - last_print).count() > 1.0) {
      printf("\r  t=%6.1fs  samples=%ld  errors=%ld   ", elapsed, count, errors);
      fflush(stdout);
      last_print = now;
    }

    this_thread::sleep_for(chrono::microseconds((long)(1e6 / rate)));
  }

  fclose(csv);
  printf("\nWrote %ld samples (%ld read errors) to %s\n", count, errors,
         output_path.c_str());
  if (count < 10) {
    fprintf(stderr, "error: too few valid samples - IMU unreadable? "
                    "(is the mission still running?)\n");
    g_nav.shutdown();
    return 1;
  }
  g_nav.shutdown();
  return 0;
}
