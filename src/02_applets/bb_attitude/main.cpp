#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <atomic>
#include <cmath>
#include "MadgwickAHRS.h"
#include <armadillo>
#include <string>
#include "bindings.h"
#include "LPF.hpp"

using namespace std;

void showHelpAndExit() {
    cout << "Usage: ./bb_attitude [options]" << endl;
    cout << "Options:" << endl;
    cout << "  -h, --help              Show this help message" << endl;
    cout << "  -d, --duration <s>      Set sampling duration in seconds (default: 1.0 s)" << endl;
    cout << "  --roll-offset <deg>     Set roll offset in degrees (default: 180)" << endl;
    cout << "  --pitch-offset <deg>    Set pitch offset in degrees (default: 0)" << endl;
    cout << "  --yaw-offset <deg>      Set yaw offset in degrees (default: 0)" << endl;
    cout << "  -v, --verbose           Enable verbose output during sampling" << endl;
    cout << endl;
    cout << "Note: Arguments can use space or '=' (e.g., -d 2.0, --duration=2.0)" << endl;
    exit(0);
}


int main(int ac, char *av[]) {
    // Default values
    double sample_frequency_hz = 150.0;
    double duration = 1.0;  // Default 1 second sampling
    double beta = 0.1;      // Lower beta for IMU-only mode
    double tau = 0.075;     // Smoothing parameter
    bool verbose = false;
    
    // BlueBoat specific defaults (upside down mounting)
    double roll_offset_deg = 180.0;   // Upside down
    double pitch_offset_deg = 0.0;
    double yaw_offset_deg = 0.0;

    // Parse user arguments
    for (int i = 1; i < ac; i++) {
        string argi = av[i];
        if ((argi == "-h") || (argi == "--help")) {
            showHelpAndExit();
        }
        // Duration parsing
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
        // Roll offset parsing
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
        // Pitch offset parsing
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
        // Yaw offset parsing
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
        // Verbose flag
        else if ((argi == "-v") || (argi == "--verbose")) {
            verbose = true;
        }
        else {
            cerr << "Unhandled argument: " << argi << endl;
            cerr << "Use --help for usage information" << endl;
            exit(1);
        }
    }

    Madgwick filter(beta, sample_frequency_hz);
    filter.begin(sample_frequency_hz);
    LPFSmoother roll_smoother = LPFSmoother(tau);
    LPFSmoother pitch_smoother = LPFSmoother(tau);

    auto start = std::chrono::high_resolution_clock::now();
    auto now = start;
    auto prev = start;
    double elapsed = 0.0;

    // Initialize Navigator
    init();


    // Convert to radians
    double roll_offset_rad = roll_offset_deg * M_PI / 180.0;
    double pitch_offset_rad = pitch_offset_deg * M_PI / 180.0;
    double yaw_offset_rad = yaw_offset_deg * M_PI / 180.0;

    // Rotation matrix for roll (x-axis)
    arma::mat R_roll = {
        {1, 0, 0},
        {0, cos(roll_offset_rad), -sin(roll_offset_rad)},
        {0, sin(roll_offset_rad), cos(roll_offset_rad)}
    };

    // Rotation matrix for pitch (y-axis)
    arma::mat R_pitch = {
        {cos(pitch_offset_rad), 0, sin(pitch_offset_rad)},
        {0, 1, 0},
        {-sin(pitch_offset_rad), 0, cos(pitch_offset_rad)}
    };

    // Rotation matrix for yaw (z-axis)
    arma::mat R_yaw = {
        {cos(yaw_offset_rad), -sin(yaw_offset_rad), 0},
        {sin(yaw_offset_rad), cos(yaw_offset_rad), 0},
        {0, 0, 1}
    };

    arma::mat R_offset = R_yaw * R_pitch * R_roll;

    int i = 0;
    int sample_count = 0;
    double final_roll = 0.0;
    double final_pitch = 0.0;
    
    // Allow filter to stabilize for first 20% of duration
    double stabilization_time = duration * 0.2;
    
    while (elapsed < duration) {
        now = std::chrono::high_resolution_clock::now();
        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
        
        // Read IMU data only (no magnetometer)
        AxisData imu_sensor = read_accel();
        AxisData gyro_sensor = read_gyro();

        arma::vec gyro = {gyro_sensor.x, gyro_sensor.y, gyro_sensor.z};
        arma::vec acc = {imu_sensor.x, imu_sensor.y, imu_sensor.z};

        // Apply rotation offsets
        gyro = R_offset * gyro;
        acc = R_offset * acc;

        double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - prev).count();

        // Use IMU-only update
        filter.updateIMU(gyro[0], gyro[1], gyro[2],
                        acc[0], acc[1], acc[2]);

        double roll = filter.getRoll();
        double pitch = filter.getPitch();
        
        roll_smoother.update180(roll, dt/1000);
        pitch_smoother.update180(pitch, dt/1000);

        roll = roll_smoother.getNextState();
        pitch = pitch_smoother.getNextState();

        // Only start averaging after stabilization period
        if (elapsed > stabilization_time) {
            final_roll += roll;
            final_pitch += pitch;
            sample_count++;
        }

        i++;

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000.0 / sample_frequency_hz)));
        prev = now;
    }
    
    // Calculate final averaged values
    if (sample_count > 0) {
        final_roll /= sample_count;
        final_pitch /= sample_count;
    }
    
    // Output final result
    if (verbose) {
        printf("roll_deg=%.2f pitch_deg=%.2f\n", final_roll, final_pitch);
    } else {
        printf("%.2f %.2f\n", final_roll, final_pitch);
    }

    return 0;
}