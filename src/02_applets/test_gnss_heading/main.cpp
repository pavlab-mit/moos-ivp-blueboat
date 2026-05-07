/*************************************************************
      Name: GNSS Heading Test Utility
      Orgn: MIT, Cambridge MA
      File: test_gnss_heading/main.cpp
   Last Ed: 2026-04-26
     Brief:
       Interactive test utility for GNSS heading from a Unicore
       UM982 dual-antenna receiver. Connects over serial and
       displays real-time heading, baseline, carrier solution,
       and position data in a clear terminal format.

       (Earlier revisions also supported the u-blox ZED-F9P via
       UBX. blueboat ships with the UM982 only, so the UBX path
       was removed for a standalone build.)
*************************************************************/

#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <chrono>
#include <getopt.h>
#include <csignal>
#include <cmath>
#include "UnicoreParser.h"

volatile sig_atomic_t g_running = 1;

void signalHandler(int signum) {
    g_running = 0;
}

void print_usage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [OPTIONS]" << std::endl;
    std::cout << std::endl;
    std::cout << "Test utility for GNSS heading from a UM982 dual-antenna receiver." << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -h, --help             Display this help message" << std::endl;
    std::cout << "  -p, --port PORT        Specify serial port (default: /dev/ttyUSB0)" << std::endl;
    std::cout << "  -b, --baud BAUDRATE    Specify baud rate (default: 460800)" << std::endl;
    std::cout << "  -i, --interval MS      Update interval in milliseconds (default: 200)" << std::endl;
    std::cout << "  -c, --compact          Compact single-line output mode" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << program_name << " -p /dev/ttyUSB0           # Default settings" << std::endl;
    std::cout << "  " << program_name << " -c -i 100                 # Compact output at 10Hz" << std::endl;
}

void clearScreen() {
    std::cout << "\033[2J\033[H";
}

std::string carrSolnColor(const std::string& soln) {
    if (soln == "FIXED" || soln == "NARROW_INT" || soln == "WIDE_INT") {
        return "\033[32m";  // Green
    } else if (soln == "FLOAT" || soln == "NARROW_FLOAT" || soln == "WIDE_FLOAT") {
        return "\033[33m";  // Yellow
    } else if (soln == "NONE" || soln == "SINGLE") {
        return "\033[31m";  // Red
    }
    return "\033[0m";  // Reset
}

void printUM982Heading(UnicoreParser& parser, bool compact) {
    const auto& nav = parser.getBestNavA();
    const auto& hdg = parser.getUniHeadingA();

    double heading = hdg.heading;
    double baseline = hdg.baseline;
    double headingAcc = hdg.headingAcc;
    bool headingValid = hdg.headingValid;
    std::string carrSoln = hdg.posType;
    double pitch = hdg.pitch;

    if (compact) {
        std::cout << "\r"
                  << "Fix:" << std::setw(8) << nav.fixType << " | "
                  << carrSolnColor(carrSoln) << "Carr:" << std::setw(12) << carrSoln << "\033[0m | "
                  << "Hdg:" << std::fixed << std::setprecision(2) << std::setw(7) << heading << "° | "
                  << "Acc:" << std::setprecision(2) << std::setw(5) << headingAcc << "° | "
                  << "Base:" << std::setprecision(3) << std::setw(6) << baseline << "m | "
                  << "Pitch:" << std::setprecision(1) << std::setw(5) << pitch << "° | "
                  << "SV:" << std::setw(2) << static_cast<int>(nav.numUsed)
                  << "   ";
        std::cout.flush();
    } else {
        clearScreen();

        std::cout << "┌─────────────────────────────────────────────────────────────────┐" << std::endl;
        std::cout << "│           GNSS HEADING TEST - UM982 (Unicore)                   │" << std::endl;
        std::cout << "├─────────────────────────────────────────────────────────────────┤" << std::endl;

        // Heading section
        std::cout << "│ HEADING                                                         │" << std::endl;
        std::cout << "│   Heading:      " << carrSolnColor(carrSoln)
                  << std::fixed << std::setprecision(3) << std::setw(10) << heading << "°\033[0m"
                  << "  (" << (headingValid ? "VALID" : "INVALID") << ")";
        std::cout << std::string(23 - (headingValid ? 5 : 7), ' ') << "│" << std::endl;

        std::cout << "│   Accuracy:     " << std::setprecision(3) << std::setw(10) << headingAcc << "°"
                  << std::string(34, ' ') << "│" << std::endl;

        std::cout << "│   Pitch:        " << std::setprecision(2) << std::setw(10) << pitch << "°"
                  << std::string(34, ' ') << "│" << std::endl;

        std::cout << "│   Baseline:     " << std::setprecision(4) << std::setw(10) << baseline << " m"
                  << std::string(33, ' ') << "│" << std::endl;

        std::cout << "│   Position Type:" << carrSolnColor(carrSoln)
                  << std::setw(12) << carrSoln << "\033[0m"
                  << std::string(32, ' ') << "│" << std::endl;

        std::cout << "├─────────────────────────────────────────────────────────────────┤" << std::endl;

        // Position section
        std::cout << "│ POSITION                                                        │" << std::endl;
        std::cout << "│   Latitude:     " << std::setprecision(7) << std::setw(14) << nav.lat << "°"
                  << std::string(30, ' ') << "│" << std::endl;
        std::cout << "│   Longitude:    " << std::setprecision(7) << std::setw(14) << nav.lon << "°"
                  << std::string(30, ' ') << "│" << std::endl;
        std::cout << "│   Altitude:     " << std::setprecision(2) << std::setw(10) << nav.height << " m"
                  << std::string(33, ' ') << "│" << std::endl;
        std::cout << "│   H Accuracy:   " << std::setprecision(3) << std::setw(10) << nav.hAcc << " m"
                  << std::string(33, ' ') << "│" << std::endl;
        std::cout << "│   Fix Type:     " << std::setw(12) << nav.fixType
                  << std::string(32, ' ') << "│" << std::endl;

        std::cout << "├─────────────────────────────────────────────────────────────────┤" << std::endl;

        // Quality section
        std::cout << "│ QUALITY                                                         │" << std::endl;
        std::cout << "│   Satellites:   " << std::setw(10) << static_cast<int>(nav.numUsed)
                  << " (tracked: " << static_cast<int>(nav.numTracked) << ")"
                  << std::string(21, ' ') << "│" << std::endl;
        std::cout << "│   Sol Status:   " << std::setw(12) << nav.solStatus
                  << std::string(32, ' ') << "│" << std::endl;
        std::cout << "│   Diff Age:     " << std::setprecision(1) << std::setw(10) << nav.diffAge << " s"
                  << std::string(33, ' ') << "│" << std::endl;

        std::cout << "└─────────────────────────────────────────────────────────────────┘" << std::endl;
        std::cout << "Press Ctrl+C to exit" << std::endl;
    }
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);

    std::string port = "/dev/ttyUSB0";
    int baudrate = 460800;
    int update_interval = 200;
    bool compact = false;

    static struct option long_options[] = {
        {"help",     no_argument,       0, 'h'},
        {"port",     required_argument, 0, 'p'},
        {"baud",     required_argument, 0, 'b'},
        {"interval", required_argument, 0, 'i'},
        {"compact",  no_argument,       0, 'c'},
        {0, 0, 0, 0}
    };

    int opt;
    int option_index = 0;

    while ((opt = getopt_long(argc, argv, "hp:b:i:c", long_options, &option_index)) != -1) {
        switch (opt) {
            case 'h':
                print_usage(argv[0]);
                return 0;
            case 'p':
                port = optarg;
                break;
            case 'b':
                baudrate = std::stoi(optarg);
                break;
            case 'i':
                update_interval = std::stoi(optarg);
                break;
            case 'c':
                compact = true;
                break;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    UnicoreParser unicore(port, baudrate);
    if (!unicore.connect()) {
        std::cerr << "Failed to connect to UM982 on " << port << std::endl;
        return 1;
    }

    std::cout << "Connected. Waiting for data..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    while (g_running) {
        auto new_data = unicore.update();
        if (new_data[UnicoreMessageType::BESTNAVA] || new_data[UnicoreMessageType::UNIHEADINGA]) {
            printUM982Heading(unicore, compact);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(update_interval));
    }

    std::cout << std::endl << "Shutting down..." << std::endl;
    unicore.disconnect();
    return 0;
}
