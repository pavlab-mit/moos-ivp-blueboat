/*************************************************************
      Name: GNSS Heading Test Utility
      Orgn: MIT, Cambridge MA
      File: test_gnss_heading/main.cpp
   Last Ed: 2024-12-22
     Brief:
       Interactive test utility for GNSS heading from dual-antenna
       receivers. Supports both ZED-F9P (UBX) and UM982 (Unicore).

       Displays real-time heading, baseline, carrier solution status,
       and position data in a clear terminal format.

*************************************************************/

#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <chrono>
#include <getopt.h>
#include <csignal>
#include <cmath>
#include "UBXParser.h"
#include "UnicoreParser.h"

volatile sig_atomic_t g_running = 1;

void signalHandler(int signum) {
    g_running = 0;
}

void print_usage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [OPTIONS]" << std::endl;
    std::cout << std::endl;
    std::cout << "Test utility for GNSS heading from dual-antenna receivers." << std::endl;
    std::cout << "Supports ZED-F9P (UBX protocol) and UM982 (Unicore protocol)." << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -h, --help             Display this help message" << std::endl;
    std::cout << "  -p, --port PORT        Specify serial port (default: /dev/ttyACM0)" << std::endl;
    std::cout << "  -b, --baud BAUDRATE    Specify baud rate (default: 115200)" << std::endl;
    std::cout << "  -i, --interval MS      Update interval in milliseconds (default: 200)" << std::endl;
    std::cout << "  --zed                  Force ZED-F9P (UBX) mode" << std::endl;
    std::cout << "  --um982                Force UM982 (Unicore) mode" << std::endl;
    std::cout << "  -c, --compact          Compact single-line output mode" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << program_name << " -p /dev/ttyACM0           # Auto-detect receiver" << std::endl;
    std::cout << "  " << program_name << " --zed -b 230400           # Force ZED-F9P mode" << std::endl;
    std::cout << "  " << program_name << " --um982 -b 460800         # Force UM982 mode" << std::endl;
    std::cout << "  " << program_name << " -c -i 100                 # Compact output at 10Hz" << std::endl;
}

enum class GNSSType {
    UNKNOWN,
    ZED_F9P,
    UM982
};

GNSSType detectGNSSType(const std::string& port, int baudrate) {
    std::cout << "Auto-detecting GNSS type..." << std::endl;

    // Try UBX first (ZED-F9P)
    UBXParser ubx(port, baudrate);
    if (ubx.connect()) {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < 2.0) {
            auto result = ubx.update();
            if (result[UBXMessageType::NAV_PVT] || result[UBXMessageType::NAV_RELPOSNED]) {
                ubx.disconnect();
                return GNSSType::ZED_F9P;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        ubx.disconnect();
    }

    // Try Unicore (UM982) at 460800
    UnicoreParser unicore(port, 460800);
    if (unicore.connect()) {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < 2.0) {
            auto result = unicore.update();
            if (result[UnicoreMessageType::BESTNAVA] || result[UnicoreMessageType::UNIHEADINGA]) {
                unicore.disconnect();
                return GNSSType::UM982;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        unicore.disconnect();
    }

    return GNSSType::UNKNOWN;
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

void printZEDHeading(UBXParser& parser, bool compact) {
    auto [fix_type, fix_str] = parser.get_fix_type();
    const auto& relpos = parser.get_relposned();
    auto dop = parser.get_dop();

    double heading = relpos.heading;
    double baseline = relpos.baseline;
    double headingAcc = relpos.headingAcc;
    bool headingValid = relpos.relPosHeadingValid;
    std::string carrSoln = relpos.carrSolnStr;

    // Compute pitch from relative position
    double pitch = 0;
    if (relpos.relPosValid && baseline > 0.1) {
        double relPosD_m = (relpos.relPosD + relpos.relPosHPD * 0.01) / 100.0;
        pitch = std::asin(-relPosD_m / baseline) * 180.0 / M_PI;
    }

    if (compact) {
        std::cout << "\r"
                  << "Fix:" << std::setw(6) << fix_str << " | "
                  << carrSolnColor(carrSoln) << "Carr:" << std::setw(6) << carrSoln << "\033[0m | "
                  << "Hdg:" << std::fixed << std::setprecision(2) << std::setw(7) << heading << "° | "
                  << "Acc:" << std::setprecision(2) << std::setw(5) << headingAcc << "° | "
                  << "Base:" << std::setprecision(3) << std::setw(6) << baseline << "m | "
                  << "Pitch:" << std::setprecision(1) << std::setw(5) << pitch << "° | "
                  << "SV:" << std::setw(2) << static_cast<int>(parser.get_num_satellites())
                  << "   ";
        std::cout.flush();
    } else {
        clearScreen();

        std::cout << "┌─────────────────────────────────────────────────────────────────┐" << std::endl;
        std::cout << "│           GNSS HEADING TEST - ZED-F9P (UBX)                     │" << std::endl;
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

        std::cout << "│   Carrier Soln: " << carrSolnColor(carrSoln)
                  << std::setw(10) << carrSoln << "\033[0m"
                  << std::string(34, ' ') << "│" << std::endl;

        std::cout << "├─────────────────────────────────────────────────────────────────┤" << std::endl;

        // Position section
        std::cout << "│ POSITION                                                        │" << std::endl;
        std::cout << "│   Latitude:     " << std::setprecision(7) << std::setw(14) << parser.get_latitude() << "°"
                  << std::string(30, ' ') << "│" << std::endl;
        std::cout << "│   Longitude:    " << std::setprecision(7) << std::setw(14) << parser.get_longitude() << "°"
                  << std::string(30, ' ') << "│" << std::endl;
        std::cout << "│   Altitude:     " << std::setprecision(2) << std::setw(10) << parser.get_height_msl() << " m MSL"
                  << std::string(29, ' ') << "│" << std::endl;
        std::cout << "│   H Accuracy:   " << std::setprecision(3) << std::setw(10) << parser.get_h_accuracy() << " m"
                  << std::string(33, ' ') << "│" << std::endl;
        std::cout << "│   Fix Type:     " << std::setw(10) << fix_str
                  << std::string(34, ' ') << "│" << std::endl;

        std::cout << "├─────────────────────────────────────────────────────────────────┤" << std::endl;

        // Quality section
        std::cout << "│ QUALITY                                                         │" << std::endl;
        std::cout << "│   Satellites:   " << std::setw(10) << static_cast<int>(parser.get_num_satellites())
                  << std::string(34, ' ') << "│" << std::endl;
        std::cout << "│   HDOP:         " << std::setprecision(2) << std::setw(10) << dop["hDOP"]
                  << std::string(34, ' ') << "│" << std::endl;
        std::cout << "│   PDOP:         " << std::setprecision(2) << std::setw(10) << dop["pDOP"]
                  << std::string(34, ' ') << "│" << std::endl;

        // Relative position NED
        double relN = (relpos.relPosN + relpos.relPosHPN * 0.01) / 100.0;
        double relE = (relpos.relPosE + relpos.relPosHPE * 0.01) / 100.0;
        double relD = (relpos.relPosD + relpos.relPosHPD * 0.01) / 100.0;

        std::cout << "├─────────────────────────────────────────────────────────────────┤" << std::endl;
        std::cout << "│ RELATIVE POSITION (Rover to Moving Base)                        │" << std::endl;
        std::cout << "│   North:        " << std::setprecision(4) << std::setw(10) << relN << " m"
                  << std::string(33, ' ') << "│" << std::endl;
        std::cout << "│   East:         " << std::setprecision(4) << std::setw(10) << relE << " m"
                  << std::string(33, ' ') << "│" << std::endl;
        std::cout << "│   Down:         " << std::setprecision(4) << std::setw(10) << relD << " m"
                  << std::string(33, ' ') << "│" << std::endl;

        std::cout << "└─────────────────────────────────────────────────────────────────┘" << std::endl;
        std::cout << "Press Ctrl+C to exit" << std::endl;
    }
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
    // Set up signal handler
    signal(SIGINT, signalHandler);

    std::string port = "/dev/ttyACM0";
    int baudrate = 115200;
    int update_interval = 200;
    bool compact = false;
    GNSSType gnss_type = GNSSType::UNKNOWN;
    bool auto_detect = true;

    // Process command line arguments
    static struct option long_options[] = {
        {"help", no_argument, 0, 'h'},
        {"port", required_argument, 0, 'p'},
        {"baud", required_argument, 0, 'b'},
        {"interval", required_argument, 0, 'i'},
        {"compact", no_argument, 0, 'c'},
        {"zed", no_argument, 0, 'z'},
        {"um982", no_argument, 0, 'u'},
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
            case 'z':
                gnss_type = GNSSType::ZED_F9P;
                auto_detect = false;
                break;
            case 'u':
                gnss_type = GNSSType::UM982;
                auto_detect = false;
                baudrate = 460800;  // UM982 default
                break;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    // Auto-detect if needed
    if (auto_detect) {
        gnss_type = detectGNSSType(port, baudrate);
        if (gnss_type == GNSSType::UNKNOWN) {
            std::cerr << "Could not detect GNSS type. Use --zed or --um982 to specify." << std::endl;
            return 1;
        }
    }

    std::cout << "Detected: " << (gnss_type == GNSSType::ZED_F9P ? "ZED-F9P (UBX)" : "UM982 (Unicore)") << std::endl;

    // Connect to the appropriate receiver
    UBXParser* ubx = nullptr;
    UnicoreParser* unicore = nullptr;

    if (gnss_type == GNSSType::ZED_F9P) {
        ubx = new UBXParser(port, baudrate);
        if (!ubx->connect()) {
            std::cerr << "Failed to connect to ZED-F9P on " << port << std::endl;
            delete ubx;
            return 1;
        }
    } else {
        unicore = new UnicoreParser(port, baudrate);
        if (!unicore->connect()) {
            std::cerr << "Failed to connect to UM982 on " << port << std::endl;
            delete unicore;
            return 1;
        }
    }

    std::cout << "Connected. Waiting for data..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Main loop
    while (g_running) {
        if (ubx) {
            auto new_data = ubx->update();
            if (new_data[UBXMessageType::NAV_PVT] || new_data[UBXMessageType::NAV_RELPOSNED]) {
                printZEDHeading(*ubx, compact);
            }
        }

        if (unicore) {
            auto new_data = unicore->update();
            if (new_data[UnicoreMessageType::BESTNAVA] || new_data[UnicoreMessageType::UNIHEADINGA]) {
                printUM982Heading(*unicore, compact);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(update_interval));
    }

    // Cleanup
    std::cout << std::endl << "Shutting down..." << std::endl;

    if (ubx) {
        ubx->disconnect();
        delete ubx;
    }
    if (unicore) {
        unicore->disconnect();
        delete unicore;
    }

    return 0;
}
