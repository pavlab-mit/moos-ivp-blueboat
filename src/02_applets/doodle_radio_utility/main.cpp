/*************************************************************
      Name: J. Wenger
      Orgn: MIT, Cambridge MA
      File: doodle_radio_utility/main.cpp
   Last Ed:  2025-01-11
     Brief:
        Command line utility for interfacing with Doodle Labs
        SmartRadio devices using the DoodleRadioDriver library.
*************************************************************/

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <csignal>
#include <functional>
#include <atomic>
#include <iomanip>
#include <ctime>
#include "DoodleRadioDriver.h"

using namespace std;

DoodleRadioDriver* radio = nullptr;
atomic<bool> running(true);

void showHelpAndExit() {
    cout << "\nDoodle Labs SmartRadio Utility" << endl;
    cout << "==============================" << endl;
    cout << "This utility provides command-line access to Doodle Labs SmartRadio devices" << endl;
    cout << "using the JSON-RPC API over HTTPS. It allows configuration and monitoring" << endl;
    cout << "of radio parameters for mesh networking applications.\n" << endl;

    cout << "Usage: doodle_radio_utility [OPTIONS]\n" << endl;

    cout << "Options:" << endl;
    cout << "  -h, --help                    Display this help message and exit" << endl;
    cout << "  --ip <address>                Radio IP address (required)" << endl;
    cout << "  --username <user>             Login username (default: user)" << endl;
    cout << "  --password <pass>             Login password (default: DoodleSmartRadio)" << endl;
    cout << "  --system-info                 Get and display system information" << endl;
    cout << "  --wireless-info               Get and display wireless interface info" << endl;
    cout << "  --wifi-status                 Get and display WiFi configuration status" << endl;
    cout << "  --link-status                 Get and display link status" << endl;
    cout << "  --calibration-status          Get and display calibration status" << endl;
    cout << "  --system-status               Get and display system resource status" << endl;
    cout << "  --association-info            Get and display connected peer information" << endl;
    cout << "  --traffic-stats               Get and display network traffic statistics" << endl;
    cout << "  --gps-data                    Get and display GPS location data" << endl;
    cout << "  --gps-raw                     Get and display raw GPS output from gpspipe" << endl;
    cout << "  --gps-samples <n>             Get n samples of raw GPS data with summary" << endl;
    cout << "  --all-info                    Get and display all available information" << endl;
    cout << "  --debug-commands              Test which commands work on the radio" << endl;
    cout << "  --monitor <seconds>           Monitor status for specified seconds" << endl;
    cout << "  --monitor-link-status         Continuously monitor link status (Ctrl+C to stop)" << endl;
    cout << "  --monitor-gps                 Continuously monitor GPS data (Ctrl+C to stop)" << endl;
    cout << "  --refresh-interval <seconds>  Set refresh interval for monitoring (default: 5)" << endl;

    cout << "\nExamples:" << endl;
    cout << "  doodle_radio_utility --ip 10.223.71.187 --system-info" << endl;
    cout << "  doodle_radio_utility --ip 10.223.71.187 --all-info" << endl;
    cout << "  doodle_radio_utility --ip 10.223.71.187 --link-status" << endl;
    cout << "  doodle_radio_utility --ip 10.223.71.187 --gps-data" << endl;
    cout << "  doodle_radio_utility --ip 10.223.71.187 --gps-samples 50" << endl;
    cout << "  doodle_radio_utility --ip 10.223.71.187 --monitor-link-status" << endl;
    cout << "  doodle_radio_utility --ip 10.223.71.187 --monitor-gps --refresh-interval 1" << endl;
    cout << "  doodle_radio_utility --ip 10.223.71.187 --monitor 30" << endl;

    cout << "\nNotes:" << endl;
    cout << "  - The radio must be accessible via HTTPS" << endl;
    cout << "  - Default credentials are username='user', password='DoodleSmartRadio'" << endl;
    cout << "  - Self-signed certificates are automatically accepted" << endl;
    cout << "  - Use Ctrl+C to interrupt monitoring mode" << endl;

    exit(0);
}

void signalHandler(int signum) {
    cout << "\nCaught signal " << signum << ". Shutting down..." << endl;
    running = false;
}

void displaySystemInfo(const SystemInfo& info) {
    cout << "\n=== System Information ===" << endl;
    if (info.valid) {
        cout << "Model: " << info.model << endl;
        cout << "Hostname: " << info.hostname << endl;
        cout << "Firmware: " << info.firmware_version << endl;
        cout << "Uptime: " << info.uptime << endl;
    } else {
        cout << "System information not available or invalid" << endl;
    }
}

void displayWirelessInfo(const WirelessInfo& info) {
    cout << "\n=== Wireless Interface Information ===" << endl;
    if (info.valid) {
        cout << "Interface: " << info.interface_name << endl;
        cout << "Mode: " << info.mode << endl;
        cout << "Channel: " << info.channel << endl;
        cout << "SSID: " << info.ssid << endl;
        cout << "Encryption: " << info.encryption << endl;
    } else {
        cout << "Wireless information not available or invalid" << endl;
    }
}

void displayWiFiStatus(const WiFiStatus& status) {
    cout << "\n=== WiFi Configuration Status ===" << endl;
    if (status.valid) {
        cout << "Radio0 (2.4GHz Mesh):" << endl;
        cout << "  Channel: " << status.radio0.channel << endl;
        cout << "  Mode: " << status.radio0.mode << endl;
        cout << "  Mesh ID: " << status.radio0.ssid << endl;
        cout << "  Enabled: " << (status.radio0.enabled ? "Yes" : "No") << endl;
        
        cout << "Radio1 (5GHz AP):" << endl;
        cout << "  Channel: " << status.radio1.channel << endl;
        cout << "  Mode: " << status.radio1.mode << endl;
        cout << "  SSID: " << status.radio1.ssid << endl;
        cout << "  Enabled: " << (status.radio1.enabled ? "Yes" : "No") << endl;
    } else {
        cout << "WiFi status not available or invalid" << endl;
    }
}

void displayLinkStatus(const LinkStatus& status) {
    cout << "\n=== Link Status ===" << endl;
    if (status.valid) {
        cout << "Link State: " << status.link_state << endl;
        cout << "Noise Level: " << status.noise_level << " dBm" << endl;
        if (status.operating_channel > 0) {
            cout << "Operating Channel: " << status.operating_channel 
                 << " (" << status.operating_frequency << " MHz)" << endl;
        }
        
        // Display new fields if available
        if (status.channel_width > 0) {
            cout << "Channel Width: " << status.channel_width << " MHz" << endl;
        }
        if (status.activity_percent >= 0) {
            cout << "Activity: " << fixed << setprecision(1) 
                 << status.activity_percent << "% (air-time congestion)" << endl;
        }
        
        cout << "Connected Stations: " << status.stations.size() << endl;
        
        // Display detailed per-station information
        for (const auto& station : status.stations) {
            cout << "\n  Station: " << station.mac_address;
            if (!station.hostname.empty()) {
                cout << " (" << station.hostname << ")";
            }
            cout << endl;
            cout << "    RSSI: " << station.rssi << " dBm";
            
            // Show per-antenna RSSI if available
            if (!station.rssi_ant.empty()) {
                cout << " (Antennas: ";
                for (size_t i = 0; i < station.rssi_ant.size(); ++i) {
                    if (i > 0) cout << ", ";
                    cout << station.rssi_ant[i];
                }
                cout << " dBm)";
            }
            cout << endl;
            
            cout << "    SNR: " << station.snr << " dB" << endl;
            cout << "    Packet Loss: " << fixed << setprecision(1)
                 << station.packet_loss_ratio << "%" << endl;
            cout << "    TX: " << station.tx_bytes << " bytes, " 
                 << station.tx_packets << " packets, "
                 << station.tx_retries << " retries, " 
                 << station.tx_failed << " failed" << endl;
            cout << "    MCS: " << station.mcs << ", Inactive: " 
                 << station.inactive_ms << " ms" << endl;
            
            // Show mesh info if available
            if (!station.mesh_hop_status.empty()) {
                cout << "    Mesh: " << station.mesh_hop_status 
                     << ", TQ: " << station.mesh_tq 
                     << ", Last seen: " << station.mesh_last_seen_ms << " ms" << endl;
            }
        }
    } else {
        cout << "Link status not available or invalid" << endl;
    }
}

void displayCalibrationStatus(const CalibrationStatus& status) {
    cout << "\n=== Calibration Status ===" << endl;
    if (status.valid) {
        cout << "MCU Calibrated: " << (status.mcu_calibrated ? "Yes" : "No") << endl;
        cout << "Backup Available: " << (status.backup_available ? "Yes" : "No") << endl;
        cout << "Calibration Date: " << status.calibration_date << endl;
        cout << "Calibration Version: " << status.calibration_version << endl;
        cout << "Status Message: " << status.status_message << endl;
    } else {
        cout << "Calibration status not available or invalid" << endl;
    }
}

void displaySystemStatus(const SystemStatus& status) {
    cout << "\n=== System Status ===" << endl;
    if (status.valid) {
        cout << "CPU Load: " << status.cpu_load << "%" << endl;
        cout << "Memory Usage: " << status.memory_usage_percent << "%" << endl;
        cout << "Disk Usage: " << status.disk_usage_percent << "%" << endl;
        cout << "Temperature: " << status.temperature_celsius << "°C" << endl;
        cout << "Uptime: " << status.uptime << endl;
    } else {
        cout << "System status not available or invalid" << endl;
    }
}

void displayAssociationInfo(const AssociationInfo& info) {
    cout << "\n=== Association Information ===" << endl;
    if (info.valid) {
        cout << "Mesh Peers: " << info.mesh_peers.size() << endl;
        for (const auto& peer : info.mesh_peers) {
            cout << "  - MAC: " << peer.mac_address << ", Signal: " << peer.signal_strength << " dBm" << endl;
        }
        cout << "AP Clients: " << info.ap_clients.size() << endl;
        for (const auto& client : info.ap_clients) {
            cout << "  - MAC: " << client.mac_address << ", Signal: " << client.signal_strength << " dBm" << endl;
        }
        cout << "Total Connections: " << info.total_connections << endl;
    } else {
        cout << "Association information not available or invalid" << endl;
    }
}

void displayTrafficStats(const TrafficStats& stats) {
    cout << "\n=== Traffic Statistics ===" << endl;
    if (stats.valid) {
        cout << "Network Interfaces: " << stats.interfaces.size() << endl;
        for (const auto& iface : stats.interfaces) {
            cout << "  " << iface.interface_name << ":" << endl;
            cout << "    TX: " << iface.bytes_sent << " bytes, " << iface.packets_sent << " packets" << endl;
            cout << "    RX: " << iface.bytes_received << " bytes, " << iface.packets_received << " packets" << endl;
            cout << "    Errors: " << iface.errors << ", Drops: " << iface.drops << endl;
        }
        cout << "Total TX: " << stats.total_bytes_sent << " bytes" << endl;
        cout << "Total RX: " << stats.total_bytes_received << " bytes" << endl;
    } else {
        cout << "Traffic statistics not available or invalid" << endl;
    }
}

void displayGPSData(const GPSData& gps) {
    cout << "\n=== GPS Data ===" << endl;
    if (gps.valid) {
        cout << "Status: " << gps.status << endl;
        cout << "Position:" << endl;
        cout << "  Latitude:  " << fixed << setprecision(6) << gps.latitude << "°" << endl;
        cout << "  Longitude: " << fixed << setprecision(6) << gps.longitude << "°" << endl;
        cout << "  Altitude:  " << fixed << setprecision(1) << gps.altitude << " m" << endl;
        
        cout << "Motion:" << endl;
        cout << "  Speed:     " << fixed << setprecision(1) << gps.speed << " m/s (" 
             << (gps.speed * 3.6) << " km/h)" << endl;
        cout << "  Heading:   " << fixed << setprecision(1) << gps.heading << "°" << endl;
        cout << "  Climb:     " << fixed << setprecision(1) << gps.climb << " m/s" << endl;
        
        cout << "Quality:" << endl;
        cout << "  Satellites: " << gps.satellites_used << " used / " 
             << gps.satellites_visible << " visible" << endl;
        cout << "  HDOP:      " << fixed << setprecision(2) << gps.hdop << endl;
        cout << "  VDOP:      " << fixed << setprecision(2) << gps.vdop << endl;
        cout << "  PDOP:      " << fixed << setprecision(2) << gps.pdop << endl;
        
        if (gps.eph > 0 || gps.epv > 0) {
            cout << "  Errors:    H=" << fixed << setprecision(1) << gps.eph << "m"
                 << ", V=" << gps.epv << "m" << endl;
        }
        
        if (!gps.time.empty()) {
            cout << "Time: " << gps.time << endl;
        }
        
        if (!gps.device.empty()) {
            cout << "Device: " << gps.device << endl;
        }
    } else {
        cout << "GPS data not available or no fix" << endl;
        cout << "  Status: " << (gps.status.empty() ? "NO_FIX" : gps.status) << endl;
        if (gps.satellites_visible > 0) {
            cout << "  Satellites visible: " << gps.satellites_visible << endl;
        }
    }
}

void displayAllInfo() {
    cout << "\n=== Refreshing All Information ===" << endl;
    
    if (radio->refreshSystemInfo()) {
        displaySystemInfo(radio->getSystemInfo());
    }
    
    if (radio->refreshWirelessInfo()) {
        displayWirelessInfo(radio->getWirelessInfo());
    }
    
    if (radio->refreshWiFiStatus()) {
        displayWiFiStatus(radio->getWiFiStatus());
    }
    
    if (radio->refreshLinkStatus()) {
        displayLinkStatus(radio->getLinkStatus());
    }
    
    if (radio->refreshCalibrationStatus()) {
        displayCalibrationStatus(radio->getCalibrationStatus());
    }
    
    if (radio->refreshSystemStatus()) {
        displaySystemStatus(radio->getSystemStatus());
    }
    
    if (radio->refreshAssociationInfo()) {
        displayAssociationInfo(radio->getAssociationInfo());
    }
    
    if (radio->refreshTrafficStats()) {
        displayTrafficStats(radio->getTrafficStats());
    }
    
    if (radio->refreshGPSData()) {
        displayGPSData(radio->getGPSData());
    }
}

int main(int argc, char* argv[]) {
    string ip;
    string username = "user";
    string password = "DoodleSmartRadio";
    int monitor_duration = 0;
    int refresh_interval = 5;
    bool monitor_link_status = false;
    bool monitor_gps = false;
    vector<function<void()>> commands;

    signal(SIGINT, signalHandler);

    if (argc == 1) {
        showHelpAndExit();
    }

    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            showHelpAndExit();
        } else if (arg == "--ip" && i + 1 < argc) {
            ip = argv[++i];
        } else if (arg == "--username" && i + 1 < argc) {
            username = argv[++i];
        } else if (arg == "--password" && i + 1 < argc) {
            password = argv[++i];
        } else if (arg == "--system-info") {
            commands.push_back([&]() {
                if (radio->refreshSystemInfo()) {
                    displaySystemInfo(radio->getSystemInfo());
                } else {
                    cout << "Failed to get system info: " << radio->getLastError() << endl;
                }
            });
        } else if (arg == "--wireless-info") {
            commands.push_back([&]() {
                if (radio->refreshWirelessInfo()) {
                    displayWirelessInfo(radio->getWirelessInfo());
                } else {
                    cout << "Failed to get wireless info: " << radio->getLastError() << endl;
                }
            });
        } else if (arg == "--wifi-status") {
            commands.push_back([&]() {
                if (radio->refreshWiFiStatus()) {
                    displayWiFiStatus(radio->getWiFiStatus());
                } else {
                    cout << "Failed to get WiFi status: " << radio->getLastError() << endl;
                }
            });
        } else if (arg == "--link-status") {
            commands.push_back([&]() {
                if (radio->refreshLinkStatus()) {
                    displayLinkStatus(radio->getLinkStatus());
                } else {
                    cout << "Failed to get link status: " << radio->getLastError() << endl;
                }
            });
        } else if (arg == "--calibration-status") {
            commands.push_back([&]() {
                if (radio->refreshCalibrationStatus()) {
                    displayCalibrationStatus(radio->getCalibrationStatus());
                } else {
                    cout << "Failed to get calibration status: " << radio->getLastError() << endl;
                }
            });
        } else if (arg == "--system-status") {
            commands.push_back([&]() {
                if (radio->refreshSystemStatus()) {
                    displaySystemStatus(radio->getSystemStatus());
                } else {
                    cout << "Failed to get system status: " << radio->getLastError() << endl;
                }
            });
        } else if (arg == "--association-info") {
            commands.push_back([&]() {
                if (radio->refreshAssociationInfo()) {
                    displayAssociationInfo(radio->getAssociationInfo());
                } else {
                    cout << "Failed to get association info: " << radio->getLastError() << endl;
                }
            });
        } else if (arg == "--traffic-stats") {
            commands.push_back([&]() {
                if (radio->refreshTrafficStats()) {
                    displayTrafficStats(radio->getTrafficStats());
                } else {
                    cout << "Failed to get traffic stats: " << radio->getLastError() << endl;
                }
            });
        } else if (arg == "--gps-data") {
            commands.push_back([&]() {
                if (radio->refreshGPSData()) {
                    displayGPSData(radio->getGPSData());
                } else {
                    cout << "Failed to get GPS data: " << radio->getLastError() << endl;
                }
            });
        } else if (arg == "--gps-raw") {
            commands.push_back([&]() {
                cout << "\n=== Raw GPS Data from gpspipe ===" << endl;
                cout << "Executing: gpspipe -w -n 10" << endl;
                cout << "----------------------------------------" << endl;
                
                string output = radio->executeCommandWithOutput("gpspipe", {"-w", "-n", "10"});
                if (!output.empty()) {
                    cout << output << endl;
                    cout << "----------------------------------------" << endl;
                    
                    // Try to parse and show what we got
                    cout << "\nAttempting to parse JSON objects:" << endl;
                    std::istringstream iss(output);
                    std::string line;
                    int line_num = 0;
                    bool found_tpv = false;
                    int sky_count = 0;
                    
                    while (std::getline(iss, line)) {
                        line_num++;
                        if (line.empty()) continue;
                        
                        cout << "Line " << line_num << ": ";
                        try {
                            json j = json::parse(line);
                            if (j.contains("class")) {
                                string obj_class = j["class"].get<std::string>();
                                cout << "Found " << obj_class << " object";
                                
                                if (obj_class == "TPV") {
                                    found_tpv = true;
                                    if (j.contains("mode")) {
                                        cout << " (mode=" << j["mode"] << ")";
                                    }
                                    if (j.contains("lat") && j.contains("lon")) {
                                        cout << " - Has position!";
                                    }
                                } else if (obj_class == "SKY") {
                                    sky_count++;
                                    if (j.contains("satellites")) {
                                        auto sats = j["satellites"];
                                        int used = 0;
                                        for (const auto& sat : sats) {
                                            if (sat.contains("used") && sat["used"].get<bool>()) {
                                                used++;
                                            }
                                        }
                                        cout << " (" << used << " sats used)";
                                    }
                                }
                                cout << endl;
                            } else {
                                cout << "JSON object without 'class' field" << endl;
                            }
                        } catch (const json::parse_error& e) {
                            cout << "Not valid JSON: " << line.substr(0, 50) 
                                 << (line.length() > 50 ? "..." : "") << endl;
                        }
                    }
                    
                    cout << "\nSummary: Found " << sky_count << " SKY objects";
                    if (!found_tpv) {
                        cout << ", NO TPV (position) objects - GPS may not have a fix yet" << endl;
                        cout << "\nTry running with more samples: --gps-raw or wait for GPS fix" << endl;
                    } else {
                        cout << ", found TPV object(s)" << endl;
                    }
                } else {
                    cout << "Failed to get raw GPS output: " << radio->getLastError() << endl;
                }
            });
        } else if (arg == "--gps-samples" && i + 1 < argc) {
            int num_samples = stoi(argv[++i]);
            commands.push_back([&, num_samples]() {
                cout << "\n=== GPS Data Collection (" << num_samples << " samples) ===" << endl;
                cout << "Executing: gpspipe -w -n " << num_samples << endl;
                if (num_samples > 20) {
                    cout << "(This may take ~" << num_samples << " seconds)" << endl;
                }
                cout << "----------------------------------------" << endl;
                
                string output = radio->executeCommandWithOutput("gpspipe", {"-w", "-n", std::to_string(num_samples)});
                if (!output.empty()) {
                    // Don't print all raw output for large samples
                    if (num_samples <= 10) {
                        cout << output << endl;
                        cout << "----------------------------------------" << endl;
                    } else {
                        cout << "Received " << output.length() << " bytes of data" << endl;
                        cout << "----------------------------------------" << endl;
                    }
                    
                    // Parse and summarize
                    std::istringstream iss(output);
                    std::string line;
                    int tpv_count = 0;
                    int sky_count = 0;
                    int other_count = 0;
                    json last_tpv;
                    json last_sky;
                    bool has_position = false;
                    
                    while (std::getline(iss, line)) {
                        if (line.empty()) continue;
                        
                        try {
                            json j = json::parse(line);
                            if (j.contains("class")) {
                                string obj_class = j["class"].get<std::string>();
                                
                                if (obj_class == "TPV") {
                                    tpv_count++;
                                    last_tpv = j;
                                    if (j.contains("lat") && j.contains("lon")) {
                                        has_position = true;
                                    }
                                } else if (obj_class == "SKY") {
                                    sky_count++;
                                    last_sky = j;
                                } else {
                                    other_count++;
                                }
                            }
                        } catch (const json::parse_error& e) {
                            // Skip invalid lines
                        }
                    }
                    
                    cout << "\nSummary:" << endl;
                    cout << "  TPV (position) objects: " << tpv_count << endl;
                    cout << "  SKY (satellite) objects: " << sky_count << endl;
                    cout << "  Other objects: " << other_count << endl;
                    
                    if (tpv_count > 0 && !last_tpv.empty()) {
                        cout << "\nLast TPV object:" << endl;
                        cout << "  Mode: " << last_tpv.value("mode", 0);
                        switch(last_tpv.value("mode", 0)) {
                            case 0: cout << " (unknown)"; break;
                            case 1: cout << " (no fix)"; break;
                            case 2: cout << " (2D fix)"; break;
                            case 3: cout << " (3D fix)"; break;
                        }
                        cout << endl;
                        
                        if (last_tpv.contains("lat") && last_tpv.contains("lon")) {
                            cout << "  Position: " << std::fixed << std::setprecision(6) 
                                 << last_tpv["lat"].get<double>() << ", " 
                                 << last_tpv["lon"].get<double>() << endl;
                            if (last_tpv.contains("alt")) {
                                cout << "  Altitude: " << std::setprecision(1) 
                                     << last_tpv["alt"].get<double>() << " m" << endl;
                            }
                            if (last_tpv.contains("speed")) {
                                cout << "  Speed: " << std::setprecision(1) 
                                     << last_tpv["speed"].get<double>() << " m/s" << endl;
                            }
                        } else {
                            cout << "  No position in TPV object" << endl;
                        }
                    } else if (!has_position) {
                        cout << "\nNo position data found. GPS may not have a fix yet." << endl;
                    }
                    
                    if (sky_count > 0 && !last_sky.empty() && last_sky.contains("satellites")) {
                        auto sats = last_sky["satellites"];
                        int used = 0;
                        for (const auto& sat : sats) {
                            if (sat.contains("used") && sat["used"].get<bool>()) {
                                used++;
                            }
                        }
                        cout << "\nLast SKY object: " << used << "/" << sats.size() 
                             << " satellites (used/visible)" << endl;
                        
                        if (last_sky.contains("hdop")) {
                            cout << "  HDOP: " << last_sky["hdop"].get<double>() << endl;
                        }
                    }
                    
                    if (!has_position && sky_count > 0) {
                        cout << "\nSuggestion: Try more samples or wait for GPS to acquire fix" << endl;
                    }
                } else {
                    cout << "Failed to get GPS samples: " << radio->getLastError() << endl;
                }
            });
        } else if (arg == "--debug-commands") {
            commands.push_back([&]() {
                cout << "\n=== Debug: Testing raw commands ===" << endl;
                
                cout << "\n--- Testing cat /proc/loadavg ---" << endl;
                string loadavg = radio->executeCommandWithOutput("cat", {"/proc/loadavg"});
                if (!loadavg.empty()) {
                    cout << "Output: " << loadavg << endl;
                } else {
                    cout << "Failed: " << radio->getLastError() << endl;
                }
                
                cout << "\n--- Testing which gpsd ---" << endl;
                string gpsd_path = radio->executeCommandWithOutput("which", {"gpsd"});
                if (!gpsd_path.empty()) {
                    cout << "gpsd found at: " << gpsd_path << endl;
                } else {
                    cout << "gpsd not found" << endl;
                }
                
                cout << "\n--- Testing gpspipe availability ---" << endl;
                string gpspipe_path = radio->executeCommandWithOutput("which", {"gpspipe"});
                if (!gpspipe_path.empty()) {
                    cout << "gpspipe found at: " << gpspipe_path << endl;
                    
                    // Try to get one line of GPS data
                    cout << "\n--- Sample GPS output (gpspipe -w -n 1) ---" << endl;
                    string gps_sample = radio->executeCommandWithOutput("gpspipe", {"-w", "-n", "1"});
                    if (!gps_sample.empty()) {
                        cout << "GPS output: " << gps_sample << endl;
                    } else {
                        cout << "No GPS output received" << endl;
                    }
                } else {
                    cout << "gpspipe not found" << endl;
                }
            });
        } else if (arg == "--all-info") {
            commands.push_back([&]() {
                displayAllInfo();
            });
        } else if (arg == "--monitor" && i + 1 < argc) {
            monitor_duration = stoi(argv[++i]);
        } else if (arg == "--monitor-link-status") {
            monitor_link_status = true;
        } else if (arg == "--monitor-gps") {
            monitor_gps = true;
        } else if (arg == "--refresh-interval" && i + 1 < argc) {
            refresh_interval = stoi(argv[++i]);
        }
    }

    // Validate required parameters
    if (ip.empty()) {
        cerr << "Error: --ip parameter is required" << endl;
        return 1;
    }

    // Initialize radio driver
    radio = new DoodleRadioDriver();
    
    if (!radio->initialize(ip, username, password)) {
        cerr << "Failed to initialize radio driver: " << radio->getLastError() << endl;
        delete radio;
        return 1;
    }

    // Login to radio
    cout << "Connecting to radio at " << ip << "..." << endl;
    if (!radio->login()) {
        cerr << "Failed to login to radio: " << radio->getLastError() << endl;
        delete radio;
        return 1;
    }

    cout << "Successfully connected to radio" << endl;
    cout << radio->repr() << endl;

    // Execute commands
    for (const auto& cmd : commands) {
        cmd();
        this_thread::sleep_for(chrono::milliseconds(100));
    }

    // Monitor mode
    if (monitor_duration > 0) {
        cout << "\nMonitoring radio for " << monitor_duration << " seconds..." << endl;
        cout << "Refresh interval: " << refresh_interval << " seconds" << endl;
        cout << "Press Ctrl+C to stop monitoring" << endl;
        
        auto start_time = chrono::steady_clock::now();
        auto last_refresh = start_time;
        
        while (running) {
            auto now = chrono::steady_clock::now();
            auto elapsed = chrono::duration_cast<chrono::seconds>(now - start_time).count();
            
            if (elapsed >= monitor_duration) {
                break;
            }
            
            auto since_refresh = chrono::duration_cast<chrono::seconds>(now - last_refresh).count();
            if (since_refresh >= refresh_interval) {
                cout << "\n--- Status Update (T+" << elapsed << "s) ---" << endl;
                displayAllInfo();
                last_refresh = now;
            }
            
            this_thread::sleep_for(chrono::milliseconds(500));
        }
    }

    // Continuous link status monitoring
    if (monitor_link_status) {
        cout << "\n=== Continuous Link Status Monitoring ===" << endl;
        cout << "Refresh interval: " << refresh_interval << " seconds" << endl;
        cout << "Press Ctrl+C to stop monitoring\n" << endl;
        
        auto last_refresh = chrono::steady_clock::now();
        
        while (running) {
            auto now = chrono::steady_clock::now();
            auto since_refresh = chrono::duration_cast<chrono::seconds>(now - last_refresh).count();
            
            if (since_refresh >= refresh_interval) {
                // Clear screen and show timestamp
                cout << "\033[2J\033[H";  // Clear screen and move to top
                
                auto time_t = chrono::system_clock::to_time_t(chrono::system_clock::now());
                cout << "=== Link Status Monitor - " << ctime(&time_t);
                
                if (radio->refreshLinkStatus()) {
                    displayLinkStatus(radio->getLinkStatus());
                } else {
                    cout << "Failed to refresh link status: " << radio->getLastError() << endl;
                }
                
                cout << "\n(Ctrl+C to stop, refreshing every " << refresh_interval << "s)" << endl;
                last_refresh = now;
            }
            
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }

    // Continuous GPS monitoring
    if (monitor_gps) {
        cout << "\n=== Continuous GPS Monitoring ===" << endl;
        cout << "Refresh interval: " << refresh_interval << " seconds" << endl;
        cout << "Press Ctrl+C to stop monitoring\n" << endl;
        
        auto last_refresh = chrono::steady_clock::now();
        
        while (running) {
            auto now = chrono::steady_clock::now();
            auto since_refresh = chrono::duration_cast<chrono::seconds>(now - last_refresh).count();
            
            if (since_refresh >= refresh_interval) {
                // Clear screen and show timestamp
                cout << "\033[2J\033[H";  // Clear screen and move to top
                
                auto time_t = chrono::system_clock::to_time_t(chrono::system_clock::now());
                cout << "=== GPS Monitor - " << ctime(&time_t);
                
                if (radio->refreshGPSData()) {
                    displayGPSData(radio->getGPSData());
                } else {
                    cout << "Failed to refresh GPS data: " << radio->getLastError() << endl;
                }
                
                cout << "\n(Ctrl+C to stop, refreshing every " << refresh_interval << "s)" << endl;
                last_refresh = now;
            }
            
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }

    // Cleanup
    radio->disconnect();
    delete radio;
    
    cout << "\nUtility shut down successfully." << endl;
    return 0;
}