/*************************************************************
      File: bb_attitude/main.cpp
   Last Ed: 2026-07-24 (navigator-cpp port)
     Brief:
        BlueBoat attitude sampling applet used by bb_init.sh's
        pitch gate. Samples the IMU for --duration seconds
        through navigator-cpp's built-in Allgeuer attitude
        estimator and prints the averaged roll/pitch.

        Output contract (parsed by bb_init.sh):
          verbose:  "roll_deg=%.2f pitch_deg=%.2f"
          plain:    "%.2f %.2f"

        navigator-cpp reads raw chip axes identically on
        Navigator V1 and V2 boards (the old per-version gravity
        sign flip was an artifact of navigator-rs's per-version
        software transforms), so the default mounting offsets
        are zero for both. Override with --roll-offset etc. if a
        boat's board is mounted unusually.
*************************************************************/

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "nav_bindings.h"

using namespace std;

static Navigator g_nav;

void showHelpAndExit() {
    cout << "Usage: ./bb_attitude [options]" << endl;
    cout << "Options:" << endl;
    cout << "  -h, --help              Show this help message" << endl;
    cout << "  -d, --duration <s>      Set sampling duration in seconds (default: 1.0 s)" << endl;
    cout << "  --roll-offset <deg>     Set roll offset in degrees (default: 0)" << endl;
    cout << "  --pitch-offset <deg>    Set pitch offset in degrees (default: 0)" << endl;
    cout << "  --yaw-offset <deg>      Set yaw offset in degrees (default: 0)" << endl;
    cout << "  -v, --verbose           Enable verbose output during sampling" << endl;
    cout << "  --raw                   Dump raw accel/gyro reads each sample (diagnostic)" << endl;
    cout << endl;
    cout << "Note: Arguments can use space or '=' (e.g., -d 2.0, --duration=2.0)" << endl;
    exit(0);
}

static void mat3_mult(const double A[9], const double B[9], double C[9]) {
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            C[3 * r + c] = A[3 * r + 0] * B[0 + c] +
                           A[3 * r + 1] * B[3 + c] +
                           A[3 * r + 2] * B[6 + c];
}

static void mat3_vec(const double A[9], const double v[3], double out[3]) {
    for (int r = 0; r < 3; r++)
        out[r] = A[3 * r] * v[0] + A[3 * r + 1] * v[1] + A[3 * r + 2] * v[2];
}

int main(int ac, char *av[]) {
    // Default values
    double sample_frequency_hz = 150.0;
    double duration = 1.0;  // Default 1 second sampling
    bool verbose = false;
    bool raw = false;       // Dump unmodified read_accel()/read_gyro() per sample

    // With navigator-cpp the raw chip frame is the same on V1 and V2
    // boards; no per-version flip needed.
    double roll_offset_deg = 0.0;
    double pitch_offset_deg = 0.0;
    double yaw_offset_deg = 0.0;

    // Parse user arguments
    for (int i = 1; i < ac; i++) {
        string argi = av[i];
        if ((argi == "-h") || (argi == "--help")) {
            showHelpAndExit();
        }
        else if (argi.find("--duration=") == 0) {
            duration = stod(argi.substr(11));
        }
        else if ((argi == "-d") || (argi == "--duration")) {
            if (++i >= ac) {
                cerr << "Error: " << argi << " requires a value" << endl;
                exit(1);
            }
            duration = stod(av[i]);
        }
        else if (argi.find("--roll-offset=") == 0) {
            roll_offset_deg = stod(argi.substr(14));
        }
        else if (argi == "--roll-offset") {
            if (++i >= ac) {
                cerr << "Error: " << argi << " requires a value" << endl;
                exit(1);
            }
            roll_offset_deg = stod(av[i]);
        }
        else if (argi.find("--pitch-offset=") == 0) {
            pitch_offset_deg = stod(argi.substr(15));
        }
        else if (argi == "--pitch-offset") {
            if (++i >= ac) {
                cerr << "Error: " << argi << " requires a value" << endl;
                exit(1);
            }
            pitch_offset_deg = stod(av[i]);
        }
        else if (argi.find("--yaw-offset=") == 0) {
            yaw_offset_deg = stod(argi.substr(13));
        }
        else if (argi == "--yaw-offset") {
            if (++i >= ac) {
                cerr << "Error: " << argi << " requires a value" << endl;
                exit(1);
            }
            yaw_offset_deg = stod(av[i]);
        }
        else if ((argi == "-v") || (argi == "--verbose")) {
            verbose = true;
        }
        else if (argi == "--raw") {
            raw = true;
        }
        else {
            cerr << "Unhandled argument: " << argi << endl;
            cerr << "Use --help for usage information" << endl;
            exit(1);
        }
    }

    // Initialize Navigator. Board revision and Pi model are detected
    // at runtime; init() reports (rather than throws) sensor issues.
    {
        std::string err = g_nav.init();
        if (!err.empty())
            cerr << "navigator init warnings: " << err << endl;
    }

    // No mag for the pitch gate; quick-learn converges roll/pitch from
    // the accelerometer within the stabilization window.
    g_nav.ahrs_set_mag_calib(0.0, 0.0, 0.0);
    g_nav.ahrs_reset(true, true);

    // Mounting-offset rotation R_offset = R_yaw * R_pitch * R_roll
    const double ro = roll_offset_deg * M_PI / 180.0;
    const double po = pitch_offset_deg * M_PI / 180.0;
    const double yo = yaw_offset_deg * M_PI / 180.0;
    const double R_roll[9] = {1, 0, 0,
                              0, cos(ro), -sin(ro),
                              0, sin(ro), cos(ro)};
    const double R_pitch[9] = {cos(po), 0, sin(po),
                               0, 1, 0,
                               -sin(po), 0, cos(po)};
    const double R_yaw[9] = {cos(yo), -sin(yo), 0,
                             sin(yo), cos(yo), 0,
                             0, 0, 1};
    double R_tmp[9], R_offset[9];
    mat3_mult(R_pitch, R_roll, R_tmp);
    mat3_mult(R_yaw, R_tmp, R_offset);

    auto start = std::chrono::high_resolution_clock::now();
    auto now = start;
    auto prev = start;
    double elapsed = 0.0;

    int sample_count = 0;
    double final_roll = 0.0;
    double final_pitch = 0.0;

    // Allow filter to stabilize for first 20% of duration
    double stabilization_time = duration * 0.2;
    const double nominal_dt = 1.0 / sample_frequency_hz;

    while (elapsed < duration) {
        now = std::chrono::high_resolution_clock::now();
        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;

        // Read IMU data only (no magnetometer)
        NavAxisData imu_sensor, gyro_sensor;
        std::string err_a = g_nav.read_accel(imu_sensor);
        std::string err_g = g_nav.read_gyro(gyro_sensor);
        if (!err_a.empty() || !err_g.empty()) {
            // A failed read must never contribute a fake-zero sample;
            // skip and keep trying for the rest of the window.
            if (raw)
                printf("read error: %s%s\n", err_a.c_str(), err_g.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(1000.0 / sample_frequency_hz)));
            prev = now;
            continue;
        }

        // Diagnostic: dump the RAW sensor reads (pre-rotation, pre-filter).
        // A healthy accel reads |a| ~ 9.81 m/s^2; a stationary gyro ~0 rad/s.
        if (raw) {
            double accel_mag = sqrt(imu_sensor.x*imu_sensor.x +
                                    imu_sensor.y*imu_sensor.y +
                                    imu_sensor.z*imu_sensor.z);
            printf("accel=[% .4f % .4f % .4f] |a|=% .4f  gyro=[% .4f % .4f % .4f]\n",
                   imu_sensor.x, imu_sensor.y, imu_sensor.z, accel_mag,
                   gyro_sensor.x, gyro_sensor.y, gyro_sensor.z);
        }

        // Apply rotation offsets
        double gyro_v[3] = {gyro_sensor.x, gyro_sensor.y, gyro_sensor.z};
        double acc_v[3] = {imu_sensor.x, imu_sensor.y, imu_sensor.z};
        double gyro_b[3], acc_b[3];
        mat3_vec(R_offset, gyro_v, gyro_b);
        mat3_vec(R_offset, acc_v, acc_b);

        double dt = std::chrono::duration<double>(now - prev).count();
        if (dt < 0.5 * nominal_dt) dt = 0.5 * nominal_dt;
        if (dt > 3.0 * nominal_dt) dt = 3.0 * nominal_dt;

        g_nav.ahrs_update(dt, gyro_b[0], gyro_b[1], gyro_b[2],
                          acc_b[0], acc_b[1], acc_b[2]);

        NavAttitudeData att;
        g_nav.ahrs_get_attitude(att);
        double roll = att.roll * 180.0 / M_PI;
        double pitch = att.pitch * 180.0 / M_PI;

        // Only start averaging after stabilization period
        if (elapsed > stabilization_time) {
            final_roll += roll;
            final_pitch += pitch;
            sample_count++;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000.0 / sample_frequency_hz)));
        prev = now;
    }

    // Fail CLOSED if we averaged (almost) nothing. If the IMU was
    // unreadable (e.g. a boot-time udev permission transient), every
    // read fails and sample_count stays 0 - printing an averaged
    // "pitch_deg=0.00" here would hand bb_init.sh's pitch gate a
    // confidently-wrong "level" answer and launch a boat that may not
    // be in the water. Emit no parseable pitch and exit nonzero so
    // the gate's retry/fail path engages instead.
    const int min_samples = 10;
    if (sample_count < min_samples) {
        fprintf(stderr,
                "error: only %d valid IMU samples collected (need >= %d); "
                "IMU unreadable or heavily corrupted\n",
                sample_count, min_samples);
        g_nav.shutdown();
        return 1;
    }

    final_roll /= sample_count;
    final_pitch /= sample_count;

    // Output final result
    if (verbose) {
        printf("roll_deg=%.2f pitch_deg=%.2f\n", final_roll, final_pitch);
    } else {
        printf("%.2f %.2f\n", final_roll, final_pitch);
    }

    g_nav.shutdown();
    return 0;
}
