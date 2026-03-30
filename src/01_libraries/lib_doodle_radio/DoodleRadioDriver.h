/*************************************************************
      Name: J. Wenger
      Orgn: MIT, Cambridge MA
      File: DoodleRadioDriver.h
   Last Ed:  7/25/25
     Brief:
        Generic C++ driver interface for Doodle Labs SmartRadio
        JSON-RPC API. Provides DTOs for API responses and
        thread-safe methods for radio configuration and monitoring.
*************************************************************/

#pragma once

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <memory>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <sstream>
#include <chrono>
#include <regex>
#include <cmath>
#include <iomanip>

using json = nlohmann::json;

// Forward declarations
class DoodleRadioDriver;

/*************************************************************
 * SystemInfo DTO - System board information
 *************************************************************/
class SystemInfo {
public:
    std::string model;
    std::string hostname;
    std::string firmware_version;
    std::string system_time;
    std::string uptime;
    bool valid;

    SystemInfo();
    void fromJson(const json& j);
    std::string repr() const;
};

/*************************************************************
 * WirelessInfo DTO - Wireless interface information
 *************************************************************/
class WirelessInfo {
public:
    std::string interface_name;
    std::string mode;
    std::string ssid;
    std::string frequency;
    std::string channel;
    std::string tx_power;
    std::string encryption;
    double noise_level;
    bool valid;

    WirelessInfo();
    void fromString(const std::string& info_str);
    void fromJson(const json& j);
    std::string repr() const;
};

/*************************************************************
 * WiFiStatus DTO - WiFi configuration status
 *************************************************************/
class WiFiStatus {
public:
    struct RadioConfig {
        std::string channel;
        std::string mode;
        std::string ssid;
        bool enabled;
        
        RadioConfig();
    };

    RadioConfig radio0; // 2.4GHz Mesh
    RadioConfig radio1; // 5GHz AP
    bool valid;

    WiFiStatus();
    void fromJson(const json& j);
    std::string repr() const;
};

/*************************************************************
 * LinkStatus DTO - Network link status information
 *************************************************************/
class LinkStatus {
public:
    // Per-station link information (sta_stats)
    struct StationInfo {
        std::string mac_address;        // MAC address of the station
        std::string hostname;           // Hostname if known (populated externally)
        double rssi;                    // Received signal strength indication in dBm
        std::vector<double> rssi_ant;   // Per-antenna RSSI values
        double snr;                     // Signal-to-noise ratio (calculated from rssi - noise)
        double packet_loss_ratio;       // Packet loss percentage at PHY layer (0.0-100.0%)
                                       // Note: Lost packets are retransmitted, so this does not 
                                       // indicate packet loss at application layer
        uint64_t tx_bytes;             // Number of bytes transmitted 
        uint32_t tx_packets;           // Number of packets transmitted in this time interval
        uint32_t tx_retries;           // Number of packets retried in this time interval
        uint32_t tx_failed;            // Number of packets which failed in this time interval
        uint32_t inactive_ms;          // Time since a packet was received from station (milliseconds)
        int mcs;                       // MCS rate being used to transmit to this station
        
        // Mesh-specific info (mesh_stats)
        std::string mesh_hop_status;   // Whether node can be reached directly ("direct") or not
        int mesh_tq;                   // Transmit quality to reach this node (out of 255)
        uint32_t mesh_last_seen_ms;    // Time since mesh packet seen from this node (milliseconds)
        
        StationInfo();
    };
    
    std::string link_state;                // Connection state
    double noise_level;                     // Noise level in dBm
    std::vector<StationInfo> stations;      // Connected stations information
    int operating_channel;                  // Operating channel number (oper_channel)
    int operating_frequency;                // Operating frequency in MHz (oper_freq)
    int channel_width;                      // Channel bandwidth in MHz (chan_width)
    double activity_percent;                // Air-time congestion percentage (activity)
                                           // 100% = air time completely saturated
    std::chrono::system_clock::time_point last_update;
    bool valid;

    LinkStatus();
    void fromJson(const json& j);
    std::string repr() const;
};

/*************************************************************
 * HTTP Response Helper
 *************************************************************/
struct HTTPResponse {
    std::string data;
    long response_code;
    
    HTTPResponse();
};

/*************************************************************
 * CalibrationStatus DTO - Calibration information
 *************************************************************/
class CalibrationStatus {
public:
    bool mcu_calibrated;
    bool backup_available;
    std::string calibration_date;
    std::string calibration_version;
    std::string status_message;
    bool valid;

    CalibrationStatus();
    void fromJson(const json& j);
    std::string repr() const;
};

/*************************************************************
 * SystemStatus DTO - System resource information (sysinfo)
 *************************************************************/
class SystemStatus {
public:
    double cpu_load;                    // CPU load normalized to 65535 (reported over 1/5/15 min)
    double memory_usage_percent;        // Memory usage percentage (calculated from freemem)
    double disk_usage_percent;          // Disk usage percentage (-1.0 = not available)
    double temperature_celsius;         // Temperature in Celsius (-999.0 = not available)
    std::string uptime;                 // System uptime (calculated from localtime)
    bool valid;

    SystemStatus();
    void fromJson(const json& j);
    std::string repr() const;
};

/*************************************************************
 * AssociationInfo DTO - Connected peer information
 *************************************************************/
class AssociationInfo {
public:
    struct PeerInfo {
        std::string mac_address;
        std::string ip_address;
        double signal_strength;
        double tx_rate;
        double rx_rate;
        std::string connection_time;
        
        PeerInfo();
    };

    std::vector<PeerInfo> mesh_peers;
    std::vector<PeerInfo> ap_clients;
    int total_connections;
    bool valid;

    AssociationInfo();
    void fromJson(const json& j);
    std::string repr() const;
};

/*************************************************************
 * TrafficStats DTO - Network traffic statistics
 *************************************************************/
class TrafficStats {
public:
    struct InterfaceStats {
        std::string interface_name;
        uint64_t bytes_sent;
        uint64_t bytes_received;
        uint64_t packets_sent;
        uint64_t packets_received;
        uint64_t errors;
        uint64_t drops;
        
        InterfaceStats();
    };

    std::vector<InterfaceStats> interfaces;
    uint64_t total_bytes_sent;
    uint64_t total_bytes_received;
    std::chrono::system_clock::time_point last_update;
    bool valid;

    TrafficStats();
    void fromJson(const json& j);
    std::string repr() const;
};

/*************************************************************
 * GPSData DTO - GPS location and status information
 *************************************************************/
class GPSData {
public:
    // Position data
    double latitude;
    double longitude;
    double altitude;      // Altitude in meters
    
    // Accuracy data
    double hdop;          // Horizontal dilution of precision
    double vdop;          // Vertical dilution of precision
    double pdop;          // Position dilution of precision
    
    // Motion data
    double speed;         // Speed in m/s
    double heading;       // Track/course over ground in degrees
    double climb;         // Vertical speed in m/s
    
    // Time data
    std::string time;     // GPS timestamp in ISO format
    double ept;           // Estimated timestamp error
    
    // Status data
    int mode;             // NMEA mode: 0=unknown, 1=no fix, 2=2D, 3=3D
    std::string status;   // Human readable status: "NO_FIX", "2D_FIX", "3D_FIX"
    int satellites_used;  // Number of satellites used
    int satellites_visible; // Number of satellites visible
    
    // Quality indicators
    double eph;           // Estimated horizontal position error (meters)
    double epv;           // Estimated vertical position error (meters)
    double epc;           // Estimated climb error
    double eps;           // Estimated speed error
    double epd;           // Estimated track error
    
    // Device info
    std::string device;   // GPS device path
    std::string driver;   // GPS driver name
    
    bool valid;
    std::chrono::system_clock::time_point last_update;

    GPSData();
    void fromJson(const json& j);
    void fromGpspipeOutput(const std::string& gpspipe_data);
    std::string repr() const;
};

/*************************************************************
 * Main DoodleRadioDriver Class
 *************************************************************/
class DoodleRadioDriver {
public:
    DoodleRadioDriver();
    ~DoodleRadioDriver();

    // Initialization and connection management
    bool initialize(const std::string& ip, const std::string& username = "user", 
                   const std::string& password = "DoodleSmartRadio");
    bool login();
    void disconnect();
    void setTimeouts(long connection_timeout_seconds = 5, long request_timeout_seconds = 10);

    // Getters for cached data (thread-safe)
    SystemInfo getSystemInfo();
    WirelessInfo getWirelessInfo();
    WiFiStatus getWiFiStatus();
    LinkStatus getLinkStatus();
    CalibrationStatus getCalibrationStatus();
    SystemStatus getSystemStatus();
    AssociationInfo getAssociationInfo();
    TrafficStats getTrafficStats();
    GPSData getGPSData();

    // DTO refresh methods
    bool refreshSystemInfo();
    bool refreshWirelessInfo();
    bool refreshWiFiStatus();
    bool refreshLinkStatus();
    bool refreshCalibrationStatus();
    bool refreshSystemStatus();
    bool refreshAssociationInfo();
    bool refreshTrafficStats();
    bool refreshGPSData();
    bool refreshAllData();

    // Utility methods
    bool executeCommand(const std::string& command, 
                       const std::vector<std::string>& args = {});
    std::string executeCommandWithOutput(const std::string& command,
                                       const std::vector<std::string>& args = {});
    std::string getLastError() const;
    bool isConnected() const;
    std::string getSessionToken() const { return m_session_token; }

    // Debug and status
    std::string repr() const;

private:
    // Network and authentication
    std::string m_ip;
    std::string m_base_url;
    std::string m_username;
    std::string m_password;
    std::string m_session_token;
    int m_request_id;
    bool m_connected;

    // Cached data
    SystemInfo m_system_info;
    WirelessInfo m_wireless_info;
    WiFiStatus m_wifi_status;
    LinkStatus m_link_status;
    CalibrationStatus m_calibration_status;
    SystemStatus m_system_status;
    AssociationInfo m_association_info;
    TrafficStats m_traffic_stats;
    GPSData m_gps_data;

    // Thread safety
    mutable std::mutex m_mutex;
    std::string m_last_error;

    // HTTP client
    CURL* m_curl;
    long m_connection_timeout;
    long m_request_timeout;

    // Helper methods
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, HTTPResponse* response);
    HTTPResponse makeJsonRpcRequest(const std::string& method, const json& params);
    bool parseJsonRpcResponse(const HTTPResponse& response, json& result);
    void setLastError(const std::string& error);
    json createJsonRpcPayload(const std::string& method, const json& params);
};