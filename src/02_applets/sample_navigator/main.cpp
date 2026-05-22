/*************************************************************
      Name: Raymond Turrisi
      File: sample_navigator/main.cpp
     Brief:
       Standalone CLI applet for sampling the Navigator's IMU
       (accel, gyro, mag) to CSV. Runs Madgwick IMU-only filter
       for roll/pitch/yaw estimates.

       Logs both raw and calibrated sensor data. If calibration
       files are provided (--gyro-cal, --mag-cal) or exist in
       ~/system_data/{hostname}/nav/, calibrated columns reflect
       the corrections. Otherwise calibrated == raw.

     Usage:
       sample_navigator --duration 120
       sample_navigator --duration 30 --gyro-cal ~/system_data/zoe-bb/nav/gyro_bias.dat
       sample_navigator --duration 180 --mag-cal ~/system_data/zoe-bb/nav/mag_cal_nav.dat
*************************************************************/

#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstring>
#include <csignal>
#include <string>
#include <vector>
#include <sys/stat.h>
#include <unistd.h>
#include "MadgwickAHRS.h"
#include <armadillo>
#include "bindings.h"
#include "LPF.hpp"
#include "networkbroker.hpp"

using namespace std;

volatile sig_atomic_t g_interrupted = 0;
void signalHandler(int) { g_interrupted = 1; }

// ---------------------------------------------------------------------------
// Calibration file parsing
// ---------------------------------------------------------------------------
struct GyroBias {
    double bx = 0, by = 0, bz = 0;
    bool valid = false;
};

struct MagCal {
    arma::vec3 hard_iron = {0, 0, 0};
    arma::mat33 soft_iron = arma::eye<arma::mat>(3, 3);
    bool valid = false;
};

GyroBias loadGyroBias(const string &path) {
    GyroBias gb;
    ifstream f(path);
    if (!f.is_open()) return gb;

    string line;
    while (getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (line.find("bias_x") != string::npos) {
            size_t eq = line.find('=');
            if (eq != string::npos) gb.bx = stod(line.substr(eq + 1));
        } else if (line.find("bias_y") != string::npos) {
            size_t eq = line.find('=');
            if (eq != string::npos) gb.by = stod(line.substr(eq + 1));
        } else if (line.find("bias_z") != string::npos) {
            size_t eq = line.find('=');
            if (eq != string::npos) gb.bz = stod(line.substr(eq + 1));
        }
    }
    gb.valid = true;
    return gb;
}

MagCal loadMagCal(const string &path) {
    MagCal mc;
    ifstream f(path);
    if (!f.is_open()) return mc;

    string line;
    while (getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        line.erase(0, line.find_first_not_of(" \t"));

        if (line.substr(0, 2) == "b " || line.substr(0, 4) == "b = ") {
            size_t eq = line.find('=');
            if (eq == string::npos) continue;
            string vals = line.substr(eq + 1);
            vals.erase(0, vals.find_first_not_of(" \t"));
            istringstream iss(vals);
            double x, y, z;
            char c1, c2;
            if (iss >> x >> c1 >> y >> c2 >> z)
                mc.hard_iron = {x, y, z};
        } else if (line.substr(0, 2) == "A " || line.substr(0, 4) == "A = ") {
            size_t eq = line.find('=');
            if (eq == string::npos) continue;
            string vals = line.substr(eq + 1);
            for (char &c : vals) if (c == ',') c = ' ';
            istringstream iss(vals);
            vector<double> v;
            double val;
            while (iss >> val && v.size() < 9) v.push_back(val);
            if (v.size() == 9) {
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        mc.soft_iron(i, j) = v[i * 3 + j];
                mc.valid = true;
            }
        }
    }
    return mc;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
string getDefaultCalDir() {
    char hostname[64];
    memset(hostname, 0, sizeof(hostname));
    gethostname(hostname, sizeof(hostname));
    string home = getenv("HOME") ? getenv("HOME") : "/tmp";
    return home + "/system_data/" + hostname + "/nav";
}

string getTimestamp() {
    auto now = chrono::system_clock::now();
    time_t t = chrono::system_clock::to_time_t(now);
    struct tm *lt = localtime(&t);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", lt);
    return string(buf);
}

void showHelp() {
    printf(
        "sample_navigator — Navigator sensor sampling applet\n\n"
        "Usage: sample_navigator [OPTIONS]\n\n"
        "Options:\n"
        "  -h, --help                Show this help\n"
        "  -d, --duration <s>        Sampling duration in seconds (default: 60)\n"
        "  -r, --rate <hz>           Sample rate in Hz (default: 150)\n"
        "  -o, --output_file <path>  Output CSV path (default: nav_sample_<ts>.csv)\n"
        "  --gyro-cal <path>         Path to gyro_bias.dat (default: ~/system_data/{hostname}/nav/)\n"
        "  --mag-cal <path>          Path to mag_cal_nav.dat (default: ~/system_data/{hostname}/nav/)\n"
        "  --roll-offset <deg>       Mounting roll offset (default: 180, upside-down)\n"
        "  --pitch-offset <deg>      Mounting pitch offset (default: 0)\n"
        "  --yaw-offset <deg>        Mounting yaw offset (default: 0)\n"
        "  --use-mag                 Use magnetometer in Madgwick filter for heading (requires mag cal)\n"
        "  --declination <deg>       Magnetic declination in degrees (default: 0)\n"
        "  --beta <val>              Madgwick filter gain (default: 0.1, lower=trust gyro more)\n"
        "  --tau <val>               LPF smoothing time constant (default: 0.075)\n"
        "  --mcc                     Broadcast MCC protocol over UDP (indefinite if no --duration)\n"
        "  --mcc-addr <addr>         MCC UDP target address (default: 127.0.0.1)\n"
        "  --mcc-port <port>         MCC UDP target port (default: 50100)\n"
        "  -v, --verbose             Verbose startup output\n\n"
        "Examples:\n"
        "  sample_navigator --duration 120\n"
        "  sample_navigator --duration 30 -o gyro_capture.csv\n"
        "  sample_navigator --gyro-cal ~/system_data/zoe-bb/nav/gyro_bias.dat --duration 180\n"
        "  sample_navigator --mcc --mcc-addr 192.168.10.1\n"
    );
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int ac, char *av[]) {
    double duration = -1.0;  // negative = use default or indefinite
    bool duration_set = false;
    double rate = 150.0;
    double beta = 0.1;
    double tau = 0.075;
    double roll_offset_deg = 180.0;
    double pitch_offset_deg = 0.0;
    double yaw_offset_deg = 0.0;
    bool verbose = false;
    bool use_mag = false;
    double declination_deg = 0.0;
    bool do_mcc = false;
    string mcc_addr = "127.0.0.1";
    int mcc_port = 50100;
    string output_path = "";
    string gyro_cal_path = "";
    string mag_cal_path = "";

    for (int i = 1; i < ac; i++) {
        string argi = av[i];
        if (argi == "-h" || argi == "--help") { showHelp(); return 0; }
        else if (argi.find("--duration=") == 0) { duration = stod(argi.substr(11)); duration_set = true; }
        else if (argi == "-d" || argi == "--duration") { if (++i >= ac) { cerr << argi << " requires a value\n"; return 1; } duration = stod(av[i]); duration_set = true; }
        else if (argi.find("--rate=") == 0) rate = stod(argi.substr(7));
        else if (argi == "-r" || argi == "--rate") { if (++i >= ac) { cerr << argi << " requires a value\n"; return 1; } rate = stod(av[i]); }
        else if (argi.find("--output_file=") == 0) output_path = argi.substr(14);
        else if (argi == "-o" || argi == "--output_file") { if (++i >= ac) { cerr << argi << " requires a value\n"; return 1; } output_path = av[i]; }
        else if (argi.find("--gyro-cal=") == 0) gyro_cal_path = argi.substr(11);
        else if (argi == "--gyro-cal") { if (++i >= ac) { cerr << argi << " requires a value\n"; return 1; } gyro_cal_path = av[i]; }
        else if (argi.find("--mag-cal=") == 0) mag_cal_path = argi.substr(10);
        else if (argi == "--mag-cal") { if (++i >= ac) { cerr << argi << " requires a value\n"; return 1; } mag_cal_path = av[i]; }
        else if (argi.find("--roll-offset=") == 0) roll_offset_deg = stod(argi.substr(14));
        else if (argi == "--roll-offset") { if (++i >= ac) { cerr << argi << " requires a value\n"; return 1; } roll_offset_deg = stod(av[i]); }
        else if (argi.find("--pitch-offset=") == 0) pitch_offset_deg = stod(argi.substr(15));
        else if (argi == "--pitch-offset") { if (++i >= ac) { cerr << argi << " requires a value\n"; return 1; } pitch_offset_deg = stod(av[i]); }
        else if (argi.find("--yaw-offset=") == 0) yaw_offset_deg = stod(argi.substr(13));
        else if (argi == "--yaw-offset") { if (++i >= ac) { cerr << argi << " requires a value\n"; return 1; } yaw_offset_deg = stod(av[i]); }
        else if (argi.find("--beta=") == 0) beta = stod(argi.substr(7));
        else if (argi == "--beta") { if (++i >= ac) { cerr << argi << " requires a value\n"; return 1; } beta = stod(av[i]); }
        else if (argi.find("--tau=") == 0) tau = stod(argi.substr(6));
        else if (argi == "--tau") { if (++i >= ac) { cerr << argi << " requires a value\n"; return 1; } tau = stod(av[i]); }
        else if (argi == "--use-mag") use_mag = true;
        else if (argi.find("--declination=") == 0) declination_deg = stod(argi.substr(14));
        else if (argi == "--declination") { if (++i >= ac) { cerr << argi << " requires a value\n"; return 1; } declination_deg = stod(av[i]); }
        else if (argi == "-v" || argi == "--verbose") verbose = true;
        else if (argi == "--mcc") do_mcc = true;
        else if (argi.find("--mcc-addr=") == 0) mcc_addr = argi.substr(11);
        else if (argi == "--mcc-addr") { if (++i >= ac) { cerr << argi << " requires a value\n"; return 1; } mcc_addr = av[i]; }
        else if (argi.find("--mcc-port=") == 0) mcc_port = stoi(argi.substr(11));
        else if (argi == "--mcc-port") { if (++i >= ac) { cerr << argi << " requires a value\n"; return 1; } mcc_port = stoi(av[i]); }
        else { cerr << "Unknown argument: " << argi << "\nUse --help for usage.\n"; return 1; }
    }

    // Default duration: 60s, or indefinite if --mcc without explicit --duration
    bool run_indefinite = false;
    if (!duration_set) {
        if (do_mcc) {
            run_indefinite = true;
            duration = 0;  // unused when indefinite
        } else {
            duration = 60.0;
        }
    }

    // Signal handler for Ctrl-C
    struct sigaction sa;
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);

    // Output path: current directory with timestamp, or user-specified
    string ts = getTimestamp();
    if (output_path.empty())
        output_path = "nav_sample_" + ts + ".csv";

    // Load calibration files: explicit path > default system_data path
    string default_cal_dir = getDefaultCalDir();

    GyroBias gb;
    if (!gyro_cal_path.empty())
        gb = loadGyroBias(gyro_cal_path);
    else
        gb = loadGyroBias(default_cal_dir + "/gyro_bias.dat");

    MagCal mc;
    if (!mag_cal_path.empty())
        mc = loadMagCal(mag_cal_path);
    else
        mc = loadMagCal(default_cal_dir + "/mag_cal_nav.dat");

    // Resolve actual paths used for reporting
    string gyro_path_used = gyro_cal_path.empty() ? (default_cal_dir + "/gyro_bias.dat") : gyro_cal_path;
    string mag_path_used = mag_cal_path.empty() ? (default_cal_dir + "/mag_cal_nav.dat") : mag_cal_path;

    printf("sample_navigator — %s\n", output_path.c_str());
    if (run_indefinite)
        printf("  Duration: indefinite (Ctrl-C to stop), Rate: %.0f Hz\n", rate);
    else
        printf("  Duration: %.1f s, Rate: %.0f Hz\n", duration, rate);
    if (gb.valid)
        printf("  Gyro cal: loaded (%s)\n", gyro_path_used.c_str());
    else
        printf("  Gyro cal: not found (%s) — using uncalibrated\n", gyro_path_used.c_str());
    if (mc.valid)
        printf("  Mag cal:  loaded (%s)\n", mag_path_used.c_str());
    else
        printf("  Mag cal:  not found (%s) — using uncalibrated\n", mag_path_used.c_str());
    if (use_mag) {
        if (mc.valid)
            printf("  Mag heading: enabled (declination=%.1f°)\n", declination_deg);
        else
            printf("  Mag heading: --use-mag set but no mag cal loaded — using IMU-only\n");
    }
    if (do_mcc)
        printf("  MCC broadcast: %s:%d\n", mcc_addr.c_str(), mcc_port);
    if (verbose)
        printf("  Mount offsets: R=%.1f° P=%.1f° Y=%.1f°\n",
               roll_offset_deg, pitch_offset_deg, yaw_offset_deg);

    // Mounting rotation
    double rr = roll_offset_deg * M_PI / 180.0;
    double pr = pitch_offset_deg * M_PI / 180.0;
    double yr = yaw_offset_deg * M_PI / 180.0;
    arma::mat R_roll = {{1,0,0},{0,cos(rr),-sin(rr)},{0,sin(rr),cos(rr)}};
    arma::mat R_pitch = {{cos(pr),0,sin(pr)},{0,1,0},{-sin(pr),0,cos(pr)}};
    arma::mat R_yaw = {{cos(yr),-sin(yr),0},{sin(yr),cos(yr),0},{0,0,1}};
    arma::mat R_offset = R_yaw * R_pitch * R_roll;

    // Madgwick filter (IMU-only, no mag in fusion)
    Madgwick filter(beta, rate);
    filter.begin(rate);
    LPFSmoother roll_smoother(tau);
    LPFSmoother pitch_smoother(tau);

    // Initialize Navigator hardware
    init();

    // Setup MCC UDP broadcaster
    SocketBroker *mcc_broker = nullptr;
    if (do_mcc) {
        mcc_broker = new SocketBroker(mcc_addr, mcc_port);
        string err = mcc_broker->open();
        if (!err.empty()) {
            cerr << "Failed to open MCC UDP socket: " << err << endl;
            delete mcc_broker;
            return 1;
        }
    }

    // Open CSV
    FILE *csv = fopen(output_path.c_str(), "w");
    if (!csv) { cerr << "Failed to open: " << output_path << endl; return 1; }

    fprintf(csv,
        "timestamp_s,"
        "nav_mag_x_ut,nav_mag_y_ut,nav_mag_z_ut,"
        "nav_accel_x_ms2,nav_accel_y_ms2,nav_accel_z_ms2,"
        "nav_gyro_x_rads,nav_gyro_y_rads,nav_gyro_z_rads,"
        "nav_mag_cal_x_ut,nav_mag_cal_y_ut,nav_mag_cal_z_ut,"
        "nav_gyro_cal_x_rads,nav_gyro_cal_y_rads,nav_gyro_cal_z_rads,"
        "nav_roll_rad,nav_pitch_rad,nav_yaw_rad\n");

    auto start = chrono::high_resolution_clock::now();
    auto prev = start;
    auto last_print = start;
    int interval_us = static_cast<int>(1e6 / rate);
    int count = 0;
    double print_interval = 0.2;  // 5 Hz terminal output

    while (!g_interrupted) {
        auto now = chrono::high_resolution_clock::now();
        double elapsed = chrono::duration<double>(now - start).count();
        if (!run_indefinite && elapsed >= duration) break;

        double dt = chrono::duration<double>(now - prev).count();
        prev = now;

        // Read raw sensors
        AxisData mag_raw = read_mag();
        AxisData acc_raw = read_accel();
        AxisData gyro_raw = read_gyro();

        // Apply mounting rotation
        arma::vec mag_v = R_offset * arma::vec({mag_raw.x, mag_raw.y, mag_raw.z});
        arma::vec acc_v = R_offset * arma::vec({acc_raw.x, acc_raw.y, acc_raw.z});
        arma::vec gyro_v = R_offset * arma::vec({gyro_raw.x, gyro_raw.y, gyro_raw.z});

        // Calibrated values
        arma::vec gyro_cal = gyro_v;
        if (gb.valid) {
            gyro_cal[0] -= gb.bx;
            gyro_cal[1] -= gb.by;
            gyro_cal[2] -= gb.bz;
        }

        arma::vec mag_cal = mag_v;
        if (mc.valid) {
            arma::vec centered = mag_v - mc.hard_iron;
            mag_cal = mc.soft_iron * centered;
        }

        // Madgwick update
        if (use_mag && mc.valid) {
            filter.update(gyro_cal[0], gyro_cal[1], gyro_cal[2],
                          acc_v[0], acc_v[1], acc_v[2],
                          mag_cal[0], mag_cal[1], mag_cal[2]);
        } else {
            filter.updateIMU(gyro_cal[0], gyro_cal[1], gyro_cal[2],
                             acc_v[0], acc_v[1], acc_v[2]);
        }

        double roll_deg = filter.getRoll();
        double pitch_deg = filter.getPitch();
        double yaw_deg = filter.getYaw();
        if (use_mag && mc.valid)
            yaw_deg = fmod(yaw_deg + declination_deg + 360.0, 360.0);

        roll_smoother.update180(roll_deg, dt);
        pitch_smoother.update180(pitch_deg, dt);

        double roll_rad = roll_smoother.getNextState() * M_PI / 180.0;
        double pitch_rad = pitch_smoother.getNextState() * M_PI / 180.0;
        double yaw_rad = yaw_deg * M_PI / 180.0;

        // Write CSV row
        fprintf(csv, "%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f\n",
                elapsed,
                mag_v[0], mag_v[1], mag_v[2],
                acc_v[0], acc_v[1], acc_v[2],
                gyro_v[0], gyro_v[1], gyro_v[2],
                mag_cal[0], mag_cal[1], mag_cal[2],
                gyro_cal[0], gyro_cal[1], gyro_cal[2],
                roll_rad, pitch_rad, yaw_rad);

        // MCC broadcast: |ts,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw*
        // ts is time since epoch per MCC protocol spec
        if (mcc_broker) {
            double epoch_ts = chrono::duration<double>(
                chrono::system_clock::now().time_since_epoch()).count();
            char mcc_buf[256];
            int mcc_n = snprintf(mcc_buf, sizeof(mcc_buf),
                "|%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f*",
                epoch_ts,
                acc_v[0], acc_v[1], acc_v[2],
                gyro_cal[0], gyro_cal[1], gyro_cal[2],
                mag_v[0], mag_v[1], mag_v[2],
                roll_rad, pitch_rad, yaw_rad);
            mcc_broker->send(string(mcc_buf, mcc_n));
        }

        count++;

        // Terminal output at 5 Hz
        double since_print = chrono::duration<double>(now - last_print).count();
        if (since_print >= print_interval) {
            last_print = now;
            double mag_mag = sqrt(mag_v[0]*mag_v[0] + mag_v[1]*mag_v[1] + mag_v[2]*mag_v[2]);
            double gyro_mag = sqrt(gyro_v[0]*gyro_v[0] + gyro_v[1]*gyro_v[1] + gyro_v[2]*gyro_v[2]);
            if (run_indefinite)
                printf("\r[%6.1fs] R:%6.1f P:%6.1f Y:%6.1f  |B|:%6.1f uT  |w|:%5.2f rad/s  (%d)",
                   elapsed,
                   roll_smoother.getNextState(), pitch_smoother.getNextState(), yaw_deg,
                   mag_mag, gyro_mag, count);
            else
                printf("\r[%6.1f/%0.0fs] R:%6.1f P:%6.1f Y:%6.1f  |B|:%6.1f uT  |w|:%5.2f rad/s  (%d)",
                   elapsed, duration,
                   roll_smoother.getNextState(), pitch_smoother.getNextState(), yaw_deg,
                   mag_mag, gyro_mag, count);
            fflush(stdout);
        }

        this_thread::sleep_for(chrono::microseconds(interval_us));
    }

    fclose(csv);
    if (mcc_broker) delete mcc_broker;
    printf("\n%s (%d samples, %.1fs)\n", output_path.c_str(), count, duration);

    return 0;
}
