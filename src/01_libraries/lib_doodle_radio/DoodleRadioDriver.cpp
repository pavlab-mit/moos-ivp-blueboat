/*************************************************************
      Name: J. Wenger
      Orgn: MIT, Cambridge MA
      File: DoodleRadioDriver.cpp
   Last Ed:  7/25/25
     Brief:
        Implementation of DoodleRadioDriver and DTO classes
*************************************************************/

#include "DoodleRadioDriver.h"

/*************************************************************
 * SystemInfo Implementation
 *************************************************************/
SystemInfo::SystemInfo() : valid(false) {}

void SystemInfo::fromJson(const json& j) {
    try {
        model = j.value("model", "");
        hostname = j.value("hostname", "");
        if (j.contains("release") && j["release"].is_object()) {
            firmware_version = j["release"].value("version", "");
        }
        system_time = j.value("system_time", "");
        uptime = j.value("uptime", "");
        valid = true;
    } catch (const std::exception& e) {
        valid = false;
        std::cerr << "Error parsing SystemInfo: " << e.what() << std::endl;
    }
}

std::string SystemInfo::repr() const {
    std::ostringstream ss;
    ss << "SystemInfo: Model=" << model 
       << ", Hostname=" << hostname 
       << ", Firmware=" << firmware_version 
       << ", Uptime=" << uptime
       << ", Valid=" << (valid ? "true" : "false");
    return ss.str();
}

/*************************************************************
 * WirelessInfo Implementation
 *************************************************************/
WirelessInfo::WirelessInfo() : noise_level(-999), valid(false) {}

void WirelessInfo::fromString(const std::string& info_str) {
    try {
        // Parse iw info output
        std::istringstream iss(info_str);
        std::string line;
        while (std::getline(iss, line)) {
            if (line.find("Interface") != std::string::npos) {
                size_t pos = line.find("Interface ");
                if (pos != std::string::npos) {
                    interface_name = line.substr(pos + 10);
                }
            } else if (line.find("type") != std::string::npos) {
                size_t pos = line.find("type ");
                if (pos != std::string::npos) {
                    mode = line.substr(pos + 5);
                }
            } else if (line.find("channel") != std::string::npos) {
                size_t pos = line.find("channel ");
                if (pos != std::string::npos) {
                    channel = line.substr(pos + 8);
                }
            }
        }
        valid = true;
    } catch (const std::exception& e) {
        valid = false;
        std::cerr << "Error parsing WirelessInfo: " << e.what() << std::endl;
    }
}

void WirelessInfo::fromJson(const json& j) {
    try {
        interface_name = j.value("phy", "");
        mode = j.value("mode", "");
        ssid = j.value("ssid", "");
        channel = std::to_string(j.value("channel", 0));
        frequency = std::to_string(j.value("frequency", 0));
        tx_power = std::to_string(j.value("txpower", 0));
        
        // Handle encryption info
        if (j.contains("encryption") && j["encryption"].is_object()) {
            const auto& enc = j["encryption"];
            if (enc.value("enabled", false)) {
                std::string enc_str = "";
                if (enc.contains("wpa") && enc["wpa"].is_array()) {
                    enc_str += "WPA";
                    for (const auto& wpa_ver : enc["wpa"]) {
                        enc_str += std::to_string(wpa_ver.get<int>());
                    }
                }
                if (enc.contains("authentication") && enc["authentication"].is_array()) {
                    if (!enc_str.empty()) enc_str += "/";
                    enc_str += enc["authentication"][0].get<std::string>();
                }
                encryption = enc_str;
            } else {
                encryption = "none";
            }
        }
        
        // Get noise level - this is the key addition
        noise_level = j.value("noise", -999.0);
        
        valid = true;
    } catch (const std::exception& e) {
        valid = false;
        std::cerr << "Error parsing WirelessInfo: " << e.what() << std::endl;
    }
}

std::string WirelessInfo::repr() const {
    std::ostringstream ss;
    ss << "WirelessInfo: Interface=" << interface_name
       << ", Mode=" << mode
       << ", Channel=" << channel
       << ", Noise=" << noise_level << " dBm"
       << ", Valid=" << (valid ? "true" : "false");
    return ss.str();
}

/*************************************************************
 * WiFiStatus Implementation
 *************************************************************/
WiFiStatus::RadioConfig::RadioConfig() : enabled(false) {}

WiFiStatus::WiFiStatus() : valid(false) {}

void WiFiStatus::fromJson(const json& j) {
    try {
        if (j.contains("radio0") && j["radio0"].is_object()) {
            const auto& r0 = j["radio0"];
            if (r0.contains("config")) {
                radio0.channel = r0["config"].value("channel", "");
                radio0.enabled = r0["config"].value("disabled", false) == false;
            }
            if (r0.contains("interfaces") && r0["interfaces"].is_array() && 
                !r0["interfaces"].empty()) {
                radio0.mode = r0["interfaces"][0]["config"].value("mode", "");
                radio0.ssid = r0["interfaces"][0]["config"].value("mesh_id", "");
            }
        }

        if (j.contains("radio1") && j["radio1"].is_object()) {
            const auto& r1 = j["radio1"];
            if (r1.contains("config")) {
                radio1.channel = r1["config"].value("channel", "");
                radio1.enabled = r1["config"].value("disabled", false) == false;
            }
            if (r1.contains("interfaces") && r1["interfaces"].is_array() && 
                !r1["interfaces"].empty()) {
                radio1.mode = r1["interfaces"][0]["config"].value("mode", "");
                radio1.ssid = r1["interfaces"][0]["config"].value("ssid", "");
            }
        }
        valid = true;
    } catch (const std::exception& e) {
        valid = false;
        std::cerr << "Error parsing WiFiStatus: " << e.what() << std::endl;
    }
}

std::string WiFiStatus::repr() const {
    std::ostringstream ss;
    ss << "WiFiStatus: Radio0(Mesh)=[Ch:" << radio0.channel 
       << ", Mode:" << radio0.mode << ", SSID:" << radio0.ssid << "], "
       << "Radio1(AP)=[Ch:" << radio1.channel 
       << ", Mode:" << radio1.mode << ", SSID:" << radio1.ssid << "], "
       << "Valid=" << (valid ? "true" : "false");
    return ss.str();
}

/*************************************************************
 * LinkStatus Implementation
 *************************************************************/
LinkStatus::StationInfo::StationInfo() : rssi(0), snr(0), packet_loss_ratio(0), tx_bytes(0), 
                   tx_packets(0), tx_retries(0), tx_failed(0), inactive_ms(0), mcs(0),
                   mesh_tq(0), mesh_last_seen_ms(0) {}

LinkStatus::LinkStatus() : noise_level(0.0), operating_channel(0), 
               operating_frequency(0), channel_width(-1), activity_percent(-1.0), valid(false) {}

void LinkStatus::fromJson(const json& j) {
    try {
        link_state = j.value("link_state", "");
        noise_level = j.value("noise_level", 0.0);
        last_update = std::chrono::system_clock::now();
        valid = true;
    } catch (const std::exception& e) {
        valid = false;
        std::cerr << "Error parsing LinkStatus: " << e.what() << std::endl;
    }
}

std::string LinkStatus::repr() const {
    std::ostringstream ss;
    ss << "LinkStatus: State=" << link_state
       << ", Noise=" << noise_level << "dBm"
       << ", Stations=" << stations.size()
       << ", Valid=" << (valid ? "true" : "false");
    return ss.str();
}

/*************************************************************
 * HTTPResponse Implementation
 *************************************************************/
HTTPResponse::HTTPResponse() : response_code(0) {}

/*************************************************************
 * CalibrationStatus Implementation
 *************************************************************/
CalibrationStatus::CalibrationStatus() : mcu_calibrated(false), backup_available(false), valid(false) {}

void CalibrationStatus::fromJson(const json& j) {
    try {
        mcu_calibrated = j.value("mcu_calibrated", false);
        backup_available = j.value("backup_available", false);
        calibration_date = j.value("calibration_date", "");
        calibration_version = j.value("calibration_version", "");
        status_message = j.value("status_message", "");
        valid = true;
    } catch (const std::exception& e) {
        valid = false;
        std::cerr << "Error parsing CalibrationStatus: " << e.what() << std::endl;
    }
}

std::string CalibrationStatus::repr() const {
    std::ostringstream ss;
    ss << "CalibrationStatus: MCU=" << (mcu_calibrated ? "true" : "false")
       << ", Backup=" << (backup_available ? "true" : "false")
       << ", Date=" << calibration_date
       << ", Valid=" << (valid ? "true" : "false");
    return ss.str();
}

/*************************************************************
 * SystemStatus Implementation
 *************************************************************/
SystemStatus::SystemStatus() : cpu_load(0.0), memory_usage_percent(0.0), 
                disk_usage_percent(0.0), temperature_celsius(0.0), valid(false) {}

void SystemStatus::fromJson(const json& j) {
    try {
        cpu_load = j.value("cpu_load", 0.0);
        memory_usage_percent = j.value("memory_usage_percent", 0.0);
        disk_usage_percent = j.value("disk_usage_percent", 0.0);
        temperature_celsius = j.value("temperature_celsius", 0.0);
        uptime = j.value("uptime", "");
        valid = true;
    } catch (const std::exception& e) {
        valid = false;
        std::cerr << "Error parsing SystemStatus: " << e.what() << std::endl;
    }
}

std::string SystemStatus::repr() const {
    std::ostringstream ss;
    ss << "SystemStatus: CPU=" << cpu_load << "%"
       << ", Memory=" << memory_usage_percent << "%"
       << ", Disk=" << disk_usage_percent << "%"
       << ", Temp=" << temperature_celsius << "°C"
       << ", Valid=" << (valid ? "true" : "false");
    return ss.str();
}

/*************************************************************
 * AssociationInfo Implementation
 *************************************************************/
AssociationInfo::PeerInfo::PeerInfo() : signal_strength(0.0), tx_rate(0.0), rx_rate(0.0) {}

AssociationInfo::AssociationInfo() : total_connections(0), valid(false) {}

void AssociationInfo::fromJson(const json& j) {
    try {
        mesh_peers.clear();
        ap_clients.clear();
        
        if (j.contains("mesh_peers") && j["mesh_peers"].is_array()) {
            for (const auto& peer : j["mesh_peers"]) {
                PeerInfo info;
                info.mac_address = peer.value("mac_address", "");
                info.ip_address = peer.value("ip_address", "");
                info.signal_strength = peer.value("signal_strength", 0.0);
                info.tx_rate = peer.value("tx_rate", 0.0);
                info.rx_rate = peer.value("rx_rate", 0.0);
                info.connection_time = peer.value("connection_time", "");
                mesh_peers.push_back(info);
            }
        }

        if (j.contains("ap_clients") && j["ap_clients"].is_array()) {
            for (const auto& client : j["ap_clients"]) {
                PeerInfo info;
                info.mac_address = client.value("mac_address", "");
                info.ip_address = client.value("ip_address", "");
                info.signal_strength = client.value("signal_strength", 0.0);
                info.tx_rate = client.value("tx_rate", 0.0);
                info.rx_rate = client.value("rx_rate", 0.0);
                info.connection_time = client.value("connection_time", "");
                ap_clients.push_back(info);
            }
        }

        total_connections = mesh_peers.size() + ap_clients.size();
        valid = true;
    } catch (const std::exception& e) {
        valid = false;
        std::cerr << "Error parsing AssociationInfo: " << e.what() << std::endl;
    }
}

std::string AssociationInfo::repr() const {
    std::ostringstream ss;
    ss << "AssociationInfo: Mesh=" << mesh_peers.size()
       << ", AP=" << ap_clients.size()
       << ", Total=" << total_connections
       << ", Valid=" << (valid ? "true" : "false");
    return ss.str();
}

/*************************************************************
 * TrafficStats Implementation
 *************************************************************/
TrafficStats::InterfaceStats::InterfaceStats() : bytes_sent(0), bytes_received(0), packets_sent(0), 
                      packets_received(0), errors(0), drops(0) {}

TrafficStats::TrafficStats() : total_bytes_sent(0), total_bytes_received(0), valid(false) {}

void TrafficStats::fromJson(const json& j) {
    try {
        interfaces.clear();
        total_bytes_sent = 0;
        total_bytes_received = 0;

        if (j.contains("interfaces") && j["interfaces"].is_array()) {
            for (const auto& iface : j["interfaces"]) {
                InterfaceStats stats;
                stats.interface_name = iface.value("name", "");
                stats.bytes_sent = iface.value("bytes_sent", 0ULL);
                stats.bytes_received = iface.value("bytes_received", 0ULL);
                stats.packets_sent = iface.value("packets_sent", 0ULL);
                stats.packets_received = iface.value("packets_received", 0ULL);
                stats.errors = iface.value("errors", 0ULL);
                stats.drops = iface.value("drops", 0ULL);
                
                interfaces.push_back(stats);
                total_bytes_sent += stats.bytes_sent;
                total_bytes_received += stats.bytes_received;
            }
        }

        last_update = std::chrono::system_clock::now();
        valid = true;
    } catch (const std::exception& e) {
        valid = false;
        std::cerr << "Error parsing TrafficStats: " << e.what() << std::endl;
    }
}

std::string TrafficStats::repr() const {
    std::ostringstream ss;
    ss << "TrafficStats: Interfaces=" << interfaces.size()
       << ", TotalTX=" << total_bytes_sent << " bytes"
       << ", TotalRX=" << total_bytes_received << " bytes"
       << ", Valid=" << (valid ? "true" : "false");
    return ss.str();
}

/*************************************************************
 * GPSData Implementation
 *************************************************************/
GPSData::GPSData() : latitude(0.0), longitude(0.0), altitude(0.0),
            hdop(99.99), vdop(99.99), pdop(99.99),
            speed(0.0), heading(0.0), climb(0.0),
            ept(0.0), mode(0), satellites_used(0), satellites_visible(0),
            eph(0.0), epv(0.0), epc(0.0), eps(0.0), epd(0.0),
            valid(false) {}

void GPSData::fromJson(const json& j) {
    try {
        // Parse gpspipe JSON output (TPV - Time Position Velocity object)
        if (j.contains("class") && j["class"] == "TPV") {
            // Position data
            if (j.contains("lat")) latitude = j["lat"].get<double>();
            if (j.contains("lon")) longitude = j["lon"].get<double>();
            if (j.contains("alt")) altitude = j["alt"].get<double>();
            
            // Motion data
            if (j.contains("speed")) speed = j["speed"].get<double>();
            if (j.contains("track")) heading = j["track"].get<double>();
            if (j.contains("climb")) climb = j["climb"].get<double>();
            
            // Time
            if (j.contains("time")) time = j["time"].get<std::string>();
            if (j.contains("ept")) ept = j["ept"].get<double>();
            
            // Mode/Status
            if (j.contains("mode")) {
                mode = j["mode"].get<int>();
                switch(mode) {
                    case 0:
                    case 1: status = "NO_FIX"; break;
                    case 2: status = "2D_FIX"; break;
                    case 3: status = "3D_FIX"; break;
                    default: status = "UNKNOWN"; break;
                }
            }
            
            // Error estimates
            if (j.contains("epx") && j.contains("epy")) {
                // Calculate horizontal error from X and Y components
                double epx = j["epx"].get<double>();
                double epy = j["epy"].get<double>();
                eph = std::sqrt(epx * epx + epy * epy);
            }
            if (j.contains("epv")) epv = j["epv"].get<double>();
            if (j.contains("epc")) epc = j["epc"].get<double>();
            if (j.contains("eps")) eps = j["eps"].get<double>();
            if (j.contains("epd")) epd = j["epd"].get<double>();
            
            // Device info
            if (j.contains("device")) device = j["device"].get<std::string>();
            
            valid = (mode >= 2);  // Valid if we have at least a 2D fix
        }
        // Parse SKY object for satellite information
        else if (j.contains("class") && j["class"] == "SKY") {
            if (j.contains("satellites") && j["satellites"].is_array()) {
                satellites_visible = j["satellites"].size();
                satellites_used = 0;
                for (const auto& sat : j["satellites"]) {
                    if (sat.contains("used") && sat["used"].get<bool>()) {
                        satellites_used++;
                    }
                }
            }
            
            // DOP values from SKY object
            if (j.contains("hdop")) hdop = j["hdop"].get<double>();
            if (j.contains("vdop")) vdop = j["vdop"].get<double>();
            if (j.contains("pdop")) pdop = j["pdop"].get<double>();
        }
        
        last_update = std::chrono::system_clock::now();
    } catch (const std::exception& e) {
        valid = false;
        std::cerr << "Error parsing GPSData: " << e.what() << std::endl;
    }
}

void GPSData::fromGpspipeOutput(const std::string& gpspipe_data) {
    try {
        std::istringstream iss(gpspipe_data);
        std::string line;
        
        // gpspipe outputs one JSON object per line
        while (std::getline(iss, line)) {
            if (line.empty()) continue;
            
            try {
                json j = json::parse(line);
                fromJson(j);  // This will update with TPV or SKY data
            } catch (const json::parse_error&) {
                // Skip malformed lines
                continue;
            }
        }
    } catch (const std::exception& e) {
        valid = false;
        std::cerr << "Error parsing gpspipe output: " << e.what() << std::endl;
    }
}

std::string GPSData::repr() const {
    std::ostringstream ss;
    ss << "GPSData: Lat=" << std::fixed << std::setprecision(6) << latitude
       << ", Lon=" << longitude
       << ", Alt=" << std::setprecision(1) << altitude << "m"
       << ", Status=" << status
       << ", Sats=" << satellites_used << "/" << satellites_visible
       << ", HDOP=" << std::setprecision(2) << hdop
       << ", Speed=" << std::setprecision(1) << speed << "m/s"
       << ", Heading=" << std::setprecision(1) << heading << "°"
       << ", Valid=" << (valid ? "true" : "false");
    return ss.str();
}

/*************************************************************
 * DoodleRadioDriver Implementation
 *************************************************************/
DoodleRadioDriver::DoodleRadioDriver() 
    : m_request_id(0), m_connected(false), m_curl(nullptr),
      m_connection_timeout(5), m_request_timeout(10) {
    curl_global_init(CURL_GLOBAL_DEFAULT);
    m_curl = curl_easy_init();
}

DoodleRadioDriver::~DoodleRadioDriver() {
    disconnect();
    if (m_curl) {
        curl_easy_cleanup(m_curl);
    }
    curl_global_cleanup();
}

bool DoodleRadioDriver::initialize(const std::string& ip, const std::string& username, 
                                  const std::string& password) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    m_ip = ip;
    m_base_url = "https://" + ip + "/ubus";
    m_username = username;
    m_password = password;
    m_session_token.clear();
    m_request_id = 0;
    m_connected = false;
    
    if (!m_curl) {
        setLastError("Failed to initialize CURL");
        return false;
    }
    
    return true;
}

bool DoodleRadioDriver::login() {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    json params = json::array({
        "00000000000000000000000000000000",
        "session",
        "login",
        json::object({
            {"username", m_username},
            {"password", m_password}
        })
    });
    
    HTTPResponse response = makeJsonRpcRequest("call", params);
    if (response.response_code != 200) {
        setLastError("Login request failed with HTTP " + std::to_string(response.response_code));
        return false;
    }
    
    json result;
    if (!parseJsonRpcResponse(response, result)) {
        setLastError("Failed to parse login response");
        return false;
    }
    
    if (result.contains("result") && result["result"].is_array() && 
        result["result"].size() > 1 && result["result"][1].contains("ubus_rpc_session")) {
        m_session_token = result["result"][1]["ubus_rpc_session"];
        m_connected = true;
        return true;
    }
    
    setLastError("Login failed - invalid credentials or response format");
    return false;
}

void DoodleRadioDriver::disconnect() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_session_token.clear();
    m_connected = false;
}

void DoodleRadioDriver::setTimeouts(long connection_timeout_seconds, long request_timeout_seconds) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_connection_timeout = connection_timeout_seconds;
    m_request_timeout = request_timeout_seconds;
}

// Getter methods
SystemInfo DoodleRadioDriver::getSystemInfo() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_system_info;
}

WirelessInfo DoodleRadioDriver::getWirelessInfo() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_wireless_info;
}

WiFiStatus DoodleRadioDriver::getWiFiStatus() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_wifi_status;
}

LinkStatus DoodleRadioDriver::getLinkStatus() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_link_status;
}

CalibrationStatus DoodleRadioDriver::getCalibrationStatus() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_calibration_status;
}

SystemStatus DoodleRadioDriver::getSystemStatus() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_system_status;
}

AssociationInfo DoodleRadioDriver::getAssociationInfo() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_association_info;
}

TrafficStats DoodleRadioDriver::getTrafficStats() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_traffic_stats;
}

GPSData DoodleRadioDriver::getGPSData() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_gps_data;
}

std::string DoodleRadioDriver::getLastError() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_last_error;
}

bool DoodleRadioDriver::isConnected() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_connected;
}

std::string DoodleRadioDriver::repr() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::ostringstream ss;
    ss << "DoodleRadioDriver: IP=" << m_ip
       << ", Connected=" << (m_connected ? "true" : "false")
       << ", Session=" << (!m_session_token.empty() ? "active" : "inactive");
    return ss.str();
}

// Private helper methods
size_t DoodleRadioDriver::WriteCallback(void* contents, size_t size, size_t nmemb, 
                                       HTTPResponse* response) {
    size_t total_size = size * nmemb;
    response->data.append(static_cast<char*>(contents), total_size);
    return total_size;
}

HTTPResponse DoodleRadioDriver::makeJsonRpcRequest(const std::string& method, 
                                                  const json& params) {
    HTTPResponse response;
    
    if (!m_curl) {
        response.response_code = 0;
        return response;
    }
    
    json payload = createJsonRpcPayload(method, params);
    std::string json_string = payload.dump();
    
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    
    curl_easy_setopt(m_curl, CURLOPT_URL, m_base_url.c_str());
    curl_easy_setopt(m_curl, CURLOPT_POSTFIELDS, json_string.c_str());
    curl_easy_setopt(m_curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(m_curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(m_curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(m_curl, CURLOPT_SSL_VERIFYPEER, 0L);
    curl_easy_setopt(m_curl, CURLOPT_SSL_VERIFYHOST, 0L);
    curl_easy_setopt(m_curl, CURLOPT_TIMEOUT, m_request_timeout);
    curl_easy_setopt(m_curl, CURLOPT_CONNECTTIMEOUT, m_connection_timeout);
    
    CURLcode res = curl_easy_perform(m_curl);
    curl_easy_getinfo(m_curl, CURLINFO_RESPONSE_CODE, &response.response_code);
    
    curl_slist_free_all(headers);
    
    if (res != CURLE_OK) {
        setLastError("CURL error: " + std::string(curl_easy_strerror(res)));
        response.response_code = 0;
    }
    
    return response;
}

bool DoodleRadioDriver::parseJsonRpcResponse(const HTTPResponse& response, json& result) {
    try {
        result = json::parse(response.data);
        return true;
    } catch (const std::exception& e) {
        setLastError("JSON parse error: " + std::string(e.what()));
        return false;
    }
}

void DoodleRadioDriver::setLastError(const std::string& error) {
    m_last_error = error;
    std::cerr << "DoodleRadioDriver Error: " << error << std::endl;
}

json DoodleRadioDriver::createJsonRpcPayload(const std::string& method, const json& params) {
    return json::object({
        {"jsonrpc", "2.0"},
        {"id", ++m_request_id},
        {"method", method},
        {"params", params}
    });
}

// Refresh methods implementation
bool DoodleRadioDriver::refreshSystemInfo() {
    if (!m_connected) {
        setLastError("Not connected to radio");
        return false;
    }
    
    json params = json::array({m_session_token, "system", "board", json::object()});
    HTTPResponse response = makeJsonRpcRequest("call", params);
    
    if (response.response_code != 200) {
        setLastError("System info request failed");
        return false;
    }
    
    json result;
    if (!parseJsonRpcResponse(response, result)) {
        return false;
    }
    
    if (result.contains("result") && result["result"].is_array() && 
        result["result"].size() > 1) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_system_info.fromJson(result["result"][1]);
        return m_system_info.valid;
    }
    
    setLastError("Invalid system info response format");
    return false;
}

bool DoodleRadioDriver::refreshWirelessInfo() {
    if (!m_connected) {
        setLastError("Not connected to radio");
        return false;
    }
    
    json params = json::array({
        m_session_token,
        "iwinfo", 
        "info", 
        json::object({{"device", "wlan0"}})
    });
    
    HTTPResponse response = makeJsonRpcRequest("call", params);
    if (response.response_code != 200) {
        setLastError("Wireless info request failed");
        return false;
    }
    
    json result;
    if (!parseJsonRpcResponse(response, result)) {
        return false;
    }
    
    if (result.contains("result") && result["result"].is_array() && 
        result["result"].size() > 1) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_wireless_info.fromJson(result["result"][1]);
        return m_wireless_info.valid;
    }
    
    setLastError("Invalid wireless info response format");
    return false;
}

bool DoodleRadioDriver::refreshWiFiStatus() {
    if (!m_connected) {
        setLastError("Not connected to radio");
        return false;
    }
    
    json params = json::array({
        m_session_token,
        "file",
        "exec",
        json::object({
            {"command", "wifi"},
            {"params", json::array({"status"})}
        })
    });
    
    HTTPResponse response = makeJsonRpcRequest("call", params);
    if (response.response_code != 200) {
        setLastError("WiFi status request failed");
        return false;
    }
    
    json result;
    if (!parseJsonRpcResponse(response, result)) {
        return false;
    }
    
    if (result.contains("result") && result["result"].is_array() && 
        result["result"].size() > 1 && result["result"][1].contains("stdout")) {
        try {
            std::string wifi_json_str = result["result"][1]["stdout"];
            json wifi_data = json::parse(wifi_json_str);
            std::lock_guard<std::mutex> lock(m_mutex);
            m_wifi_status.fromJson(wifi_data);
            return m_wifi_status.valid;
        } catch (const std::exception& e) {
            setLastError("Failed to parse WiFi status JSON");
            return false;
        }
    }
    
    setLastError("Invalid WiFi status response format");
    return false;
}

bool DoodleRadioDriver::refreshLinkStatus() {
    if (!m_connected) {
        setLastError("Not connected to radio");
        return false;
    }
    
    // Auto-refresh wireless info if we don't have valid noise data
    if (!m_wireless_info.valid || m_wireless_info.noise_level == -999) {
        // Temporarily unlock to call refreshWirelessInfo (which has its own lock)
        json iwinfo_params = json::array({
            m_session_token,
            "iwinfo", 
            "info", 
            json::object({{"device", "wlan0"}})
        });
        
        HTTPResponse iwinfo_response = makeJsonRpcRequest("call", iwinfo_params);
        if (iwinfo_response.response_code == 200) {
            json iwinfo_result;
            if (parseJsonRpcResponse(iwinfo_response, iwinfo_result)) {
                if (iwinfo_result.contains("result") && iwinfo_result["result"].is_array() && 
                    iwinfo_result["result"].size() > 1) {
                    // Update just the noise level from iwinfo without full wireless refresh
                    const auto& iwinfo_data = iwinfo_result["result"][1];
                    if (iwinfo_data.contains("noise") && iwinfo_data["noise"].is_number()) {
                        m_wireless_info.noise_level = iwinfo_data["noise"].get<double>();
                        m_wireless_info.valid = true; // Mark as having some valid data
                    }
                }
            }
        }
    }
    
    std::lock_guard<std::mutex> lock(m_mutex);
    
    // Try multiple possible locations for link status
    std::vector<std::string> status_paths = {
        "/tmp/linkstate_current.json",
        "/tmp/longtermlog/status.json"
    };
    
    for (const auto& path : status_paths) {
        json params = json::array({
            m_session_token,
            "file",
            "exec",
            json::object({
                {"command", "cat"},
                {"params", json::array({path})}
            })
        });
        
        HTTPResponse response = makeJsonRpcRequest("call", params);
        if (response.response_code == 200) {
            json result;
            if (parseJsonRpcResponse(response, result)) {
                if (result.contains("result") && result["result"].is_array() && 
                    result["result"].size() > 1 && result["result"][1].contains("stdout")) {
                    try {
                        std::string status_json = result["result"][1]["stdout"];
                        json status_data = json::parse(status_json);
                        
                        // Check if data is wrapped in "linkstate" object
                        json link_data = status_data;
                        if (status_data.contains("linkstate")) {
                            link_data = status_data["linkstate"];
                        }
                        
                        // Use cached noise level from WirelessInfo if available, otherwise try linkstate data
                        if (m_wireless_info.valid && m_wireless_info.noise_level != -999) {
                            m_link_status.noise_level = m_wireless_info.noise_level;
                        } else if (link_data.contains("noise")) {
                            // Fallback to linkstate data
                            if (link_data["noise"].is_string()) {
                                m_link_status.noise_level = std::stod(link_data["noise"].get<std::string>());
                            } else if (link_data["noise"].is_number()) {
                                double noise_val = link_data["noise"].get<double>();
                                // Only use linkstate noise if it's a reasonable value (not 0)
                                m_link_status.noise_level = (noise_val != 0.0) ? noise_val : -999;
                            }
                        } else {
                            m_link_status.noise_level = -999; // Invalid value indicates not available
                        }
                        
                        // Extract operating channel, frequency, and additional fields
                        if (link_data.contains("oper_chan")) {
                            m_link_status.operating_channel = link_data["oper_chan"].get<int>();
                        }
                        if (link_data.contains("oper_freq")) {
                            m_link_status.operating_frequency = link_data["oper_freq"].get<int>();
                        }
                        if (link_data.contains("chan_width")) {
                            // chan_width can be string or number, handle both
                            if (link_data["chan_width"].is_string()) {
                                std::string width_str = link_data["chan_width"].get<std::string>();
                                m_link_status.channel_width = std::stoi(width_str);
                            } else {
                                m_link_status.channel_width = link_data["chan_width"].get<int>();
                            }
                        }
                        if (link_data.contains("activity")) {
                            m_link_status.activity_percent = link_data["activity"].get<double>();
                        }
                        
                        // Clear existing stations
                        m_link_status.stations.clear();
                        
                        // Parse per-station data
                        if (link_data.contains("sta_stats") && link_data["sta_stats"].is_array()) {
                            auto sta_stats = link_data["sta_stats"];
                            
                            // Also get mesh stats for additional info
                            std::map<std::string, json> mesh_info_map;
                            if (link_data.contains("mesh_stats") && link_data["mesh_stats"].is_array()) {
                                for (const auto& mesh : link_data["mesh_stats"]) {
                                    if (mesh.contains("orig_address")) {
                                        mesh_info_map[mesh["orig_address"]] = mesh;
                                    }
                                }
                            }
                            
                            for (const auto& sta : sta_stats) {
                                LinkStatus::StationInfo station;
                                
                                // Basic info
                                station.mac_address = sta.value("mac", "unknown");
                                station.rssi = sta.value("rssi", -90.0);
                                // Note: Despite the variable name, pl_ratio from radio is actually 
                                // a percentage value (0.0-100.0), not a ratio (0.0-1.0)
                                station.packet_loss_ratio = sta.value("pl_ratio", 0.0);
                                station.tx_bytes = sta.value("tx_bytes", 0);
                                station.tx_packets = sta.value("tx_packets", 0);
                                station.tx_retries = sta.value("tx_retries", 0);
                                station.tx_failed = sta.value("tx_failed", 0);
                                station.inactive_ms = sta.value("inactive", 0);
                                station.mcs = sta.value("mcs", 0);
                                
                                // Per-antenna RSSI
                                if (sta.contains("rssi_ant") && sta["rssi_ant"].is_array()) {
                                    for (const auto& ant_rssi : sta["rssi_ant"]) {
                                        station.rssi_ant.push_back(ant_rssi.get<double>());
                                    }
                                }
                                
                                // Calculate SNR only if we have valid noise data
                                if (m_link_status.noise_level != -999) {
                                    station.snr = station.rssi - m_link_status.noise_level;
                                } else {
                                    station.snr = -999; // Invalid value indicates not available
                                }
                                
                                // Add mesh info if available
                                auto mesh_it = mesh_info_map.find(station.mac_address);
                                if (mesh_it != mesh_info_map.end()) {
                                    const auto& mesh = mesh_it->second;
                                    station.mesh_hop_status = mesh.value("hop_status", "");
                                    station.mesh_tq = mesh.value("tq", 0);
                                    station.mesh_last_seen_ms = mesh.value("last_seen_msecs", 0);
                                }
                                
                                m_link_status.stations.push_back(station);
                            }
                            
                            if (!m_link_status.stations.empty()) {
                                m_link_status.link_state = "connected";
                            } else {
                                m_link_status.link_state = "no_peers";
                            }
                        }
                        
                        m_link_status.valid = true;
                        return true; // Successfully parsed from this path
                    } catch (const std::exception& e) {
                        // Try next path if this one fails
                        continue;
                    }
                }
            }
        }
    }
    
    // Fallback with clear sentinel values if status.json doesn't exist or fails
    m_link_status.link_state = "UNAVAILABLE";
    m_link_status.noise_level = -999; // Sentinel value indicates not available
    m_link_status.stations.clear();
    m_link_status.operating_channel = -1; // Clearly invalid
    m_link_status.operating_frequency = -1; // Clearly invalid
    m_link_status.channel_width = -1; // Clearly invalid
    m_link_status.activity_percent = -1.0; // Clearly invalid
    m_link_status.valid = false; // Mark as invalid when no real data available
    
    return true;
}

bool DoodleRadioDriver::refreshCalibrationStatus() {
    if (!m_connected) {
        setLastError("Not connected to radio");
        return false;
    }
    
    std::lock_guard<std::mutex> lock(m_mutex);
    
    // Since dl-pical-check doesn't exist on this radio, set clear sentinel values
    m_calibration_status.mcu_calibrated = false; // Don't assume calibrated
    m_calibration_status.backup_available = false;
    m_calibration_status.calibration_date = "UNAVAILABLE";
    m_calibration_status.calibration_version = "UNAVAILABLE";
    m_calibration_status.status_message = "UNAVAILABLE: Calibration status not supported on this radio firmware";
    m_calibration_status.valid = false; // Mark as invalid since no real data
    
    return true; // Function succeeds but data is marked invalid
}

bool DoodleRadioDriver::refreshSystemStatus() {
    if (!m_connected) {
        setLastError("Not connected to radio");
        return false;
    }
    
    std::lock_guard<std::mutex> lock(m_mutex);
    
    // Use system/info API to get system status information
    json params = json::array({m_session_token, "system", "info", json::object()});
    HTTPResponse response = makeJsonRpcRequest("call", params);
    
    if (response.response_code != 200) {
        setLastError("System status request failed");
        return false;
    }
    
    json result;
    if (!parseJsonRpcResponse(response, result)) {
        return false;
    }
    
    if (result.contains("result") && result["result"].is_array() && 
        result["result"].size() > 1) {
        json system_data = result["result"][1];
        
        // Parse system information from the API response
        if (system_data.contains("load")) {
            auto load_array = system_data["load"];
            if (load_array.is_array() && load_array.size() > 0) {
                // Load values are scaled by 65536 in this API
                double raw_load = load_array[0];
                double actual_load = raw_load / 65536.0;
                // Convert to percentage (load of 1.0 = 100% CPU)
                m_system_status.cpu_load = actual_load * 100.0;
            }
        }
        
        if (system_data.contains("memory")) {
            auto memory = system_data["memory"];
            if (memory.contains("total") && memory.contains("free")) {
                long long total = memory["total"];
                long long free = memory["free"];
                if (total > 0) {
                    m_system_status.memory_usage_percent = ((double)(total - free) / total) * 100.0;
                }
            }
        }
        
        // Set sentinel values for unavailable data (clearly invalid values)
        m_system_status.disk_usage_percent = -1.0; // Clearly invalid - not available in this API
        m_system_status.temperature_celsius = -999.0; // Clearly invalid - not available in this API
        
        if (system_data.contains("uptime")) {
            double uptime_seconds = system_data["uptime"];
            int days = uptime_seconds / 86400;
            int hours = (uptime_seconds - days * 86400) / 3600;
            int minutes = (uptime_seconds - days * 86400 - hours * 3600) / 60;
            
            std::ostringstream uptime_ss;
            if (days > 0) uptime_ss << days << " days, ";
            if (hours > 0) uptime_ss << hours << " hours, ";
            uptime_ss << minutes << " minutes";
            m_system_status.uptime = uptime_ss.str();
        }
        
        m_system_status.valid = true;
        return true;
    }
    
    setLastError("Invalid system status response format");
    return false;
}

bool DoodleRadioDriver::refreshAssociationInfo() {
    if (!m_connected) {
        setLastError("Not connected to radio");
        return false;
    }
    
    std::lock_guard<std::mutex> lock(m_mutex);
    m_association_info.mesh_peers.clear();
    m_association_info.ap_clients.clear();
    
    // Use the working iwinfo API for wlan0 (mesh interface)
    json params = json::array({m_session_token, "iwinfo", "assoclist", json::object({{"device", "wlan0"}})});
    HTTPResponse response = makeJsonRpcRequest("call", params);
    
    if (response.response_code == 200) {
        json result;
        if (parseJsonRpcResponse(response, result)) {
            if (result.contains("result") && result["result"].is_array() && 
                result["result"].size() > 1 && result["result"][1].contains("results")) {
                
                json results = result["result"][1]["results"];
                if (results.is_array()) {
                    for (const auto& peer : results) {
                        AssociationInfo::PeerInfo peer_info;
                        peer_info.mac_address = peer.value("mac", "unknown");
                        peer_info.signal_strength = peer.value("signal", -90);
                        m_association_info.mesh_peers.push_back(peer_info);
                    }
                }
            }
        }
    }
    
    // wlan1 (AP) access is denied, so we can't get AP clients
    // Keep ap_clients empty
    
    m_association_info.total_connections = m_association_info.mesh_peers.size() + m_association_info.ap_clients.size();
    m_association_info.valid = true;
    
    return true;
}

bool DoodleRadioDriver::refreshTrafficStats() {
    if (!m_connected) {
        setLastError("Not connected to radio");
        return false;
    }
    
    std::lock_guard<std::mutex> lock(m_mutex);
    m_traffic_stats.interfaces.clear();
    m_traffic_stats.total_bytes_sent = 0;
    m_traffic_stats.total_bytes_received = 0;
    
    // Get traffic stats from iw wlan0 info (this works)
    json params = json::array({
        m_session_token,
        "file",
        "exec",
        json::object({
            {"command", "iw"},
            {"params", json::array({"wlan0", "info"})}
        })
    });
    
    HTTPResponse response = makeJsonRpcRequest("call", params);
    if (response.response_code == 200) {
        json result;
        if (parseJsonRpcResponse(response, result)) {
            if (result.contains("result") && result["result"].is_array() && 
                result["result"].size() > 1 && result["result"][1].contains("stdout")) {
                
                std::string iw_output = result["result"][1]["stdout"];
                TrafficStats::InterfaceStats wlan0_stats;
                wlan0_stats.interface_name = "wlan0";
                
                // Initialize to 0
                wlan0_stats.bytes_sent = 0;
                wlan0_stats.bytes_received = 0;
                wlan0_stats.packets_sent = 0;
                wlan0_stats.packets_received = 0;
                wlan0_stats.errors = 0;
                wlan0_stats.drops = 0;
                
                // Parse tx-bytes and tx-packets from iw output
                // The multicast TXQ section has format:
                // qsz-byt qsz-pkt flows drops marks overlmt hashcol tx-bytes tx-packets
                // 0       0       705   0     0     0       0       76218    705
                
                std::istringstream iss(iw_output);
                std::string line;
                bool in_multicast_section = false;
                
                while (std::getline(iss, line)) {
                    // Look for the multicast TXQ section
                    if (line.find("multicast TXQ:") != std::string::npos) {
                        in_multicast_section = true;
                        continue;
                    }
                    
                    // Skip the header line with column names
                    if (in_multicast_section && line.find("tx-bytes") != std::string::npos) {
                        continue;
                    }
                    
                    // Parse the data line with actual values
                    if (in_multicast_section && line.find_first_not_of(" \t") != std::string::npos) {
                        std::istringstream data_stream(line);
                        std::vector<std::string> tokens;
                        std::string token;
                        
                        // Collect all tokens
                        while (data_stream >> token) {
                            tokens.push_back(token);
                        }
                        
                        // tx-bytes is at index 7, tx-packets at index 8 (0-based)
                        if (tokens.size() >= 9) {
                            try {
                                wlan0_stats.bytes_sent = std::stoull(tokens[7]);
                                wlan0_stats.packets_sent = std::stoull(tokens[8]);
                            } catch (const std::exception&) {
                                // Keep defaults if parsing fails
                            }
                        }
                        break; // Done parsing
                    }
                }
                
                m_traffic_stats.interfaces.push_back(wlan0_stats);
                m_traffic_stats.total_bytes_sent += wlan0_stats.bytes_sent;
                m_traffic_stats.total_bytes_received += wlan0_stats.bytes_received;
            }
        }
    }
    
    // Don't add wlan1 interface - access denied, so no fake data
    // Applications should check interfaces vector size to see what's available
    m_traffic_stats.last_update = std::chrono::system_clock::now();
    m_traffic_stats.valid = true;
    
    return true;
}

bool DoodleRadioDriver::refreshGPSData() {
    if (!m_connected) {
        setLastError("Not connected to radio");
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    // Use gpspipe to get GPS data in JSON format
    // -w flag outputs JSON, -n 10 gets 10 samples to ensure we get both TPV and SKY objects
    json params = json::array({m_session_token,
                               "file",
                               "exec",
                               json::object({{"command", "gpspipe"},
                                             {"params", json::array({"-w", "-n", "10"})}})});

    HTTPResponse response = makeJsonRpcRequest("call", params);
    if (response.response_code != 200) {
        setLastError("GPS data request failed");
        return false;
    }

    json result;
    if (!parseJsonRpcResponse(response, result)) {
        return false;
    }

    if (result.contains("result") && result["result"].is_array() &&
        result["result"].size() > 1 && result["result"][1].contains("stdout")) {

        std::string gpspipe_output = result["result"][1]["stdout"];

        // Reset GPS data before parsing new data
        m_gps_data = GPSData();

        // Parse the output
        m_gps_data.fromGpspipeOutput(gpspipe_output);

        // If we didn't get a valid fix from TPV, but we have satellite info,
        // mark it as having some data
        if (!m_gps_data.valid && m_gps_data.satellites_visible > 0) {
            // We have satellite visibility but no position fix
            m_gps_data.status = "NO_FIX";
            m_gps_data.mode = 1;
        }

        return true;
    }

    setLastError("Invalid GPS data response format");
    return false;
}

bool DoodleRadioDriver::refreshAllData() {
    bool success = true;
    
    success &= refreshSystemInfo();
    success &= refreshWirelessInfo();
    success &= refreshWiFiStatus();
    success &= refreshLinkStatus();
    success &= refreshCalibrationStatus();
    success &= refreshSystemStatus();
    success &= refreshAssociationInfo();
    success &= refreshTrafficStats();
    success &= refreshGPSData();
    
    return success;
}

bool DoodleRadioDriver::executeCommand(const std::string& command, 
                                      const std::vector<std::string>& args) {
    if (!m_connected) {
        setLastError("Not connected to radio");
        return false;
    }
    
    json params = json::array({
        m_session_token,
        "file",
        "exec",
        json::object({
            {"command", command},
            {"params", args}
        })
    });
    
    HTTPResponse response = makeJsonRpcRequest("call", params);
    return response.response_code == 200;
}

std::string DoodleRadioDriver::executeCommandWithOutput(const std::string &command,
                                                        const std::vector<std::string> &args) {
    if (!m_connected) {
        setLastError("Not connected to radio");
        return "";
    }

    json params = json::array({m_session_token,
                               "file",
                               "exec",
                               json::object({{"command", command},
                                             {"params", args}})});

    HTTPResponse response = makeJsonRpcRequest("call", params);
    if (response.response_code != 200) {
        setLastError("Command execution failed with HTTP " + std::to_string(response.response_code));
        return "";
    }

    json result;
    if (!parseJsonRpcResponse(response, result)) {
        return "";
    }

    // Extract stdout from the response
    if (result.contains("result") && result["result"].is_array() &&
        result["result"].size() > 1 && result["result"][1].contains("stdout")) {
        return result["result"][1]["stdout"].get<std::string>();
    }

    setLastError("No output in command response");
    return "";
}