
/*************************************************************
      Name: 
      Orgn: MIT, Cambridge MA
      File: iDoodleLabs/DoodleLabs.cpp
   Last Ed:  2025-07-21
     Brief:
        Doodle Labs radio interface. Polls the radio JSON HTTP
        API for link metrics (RSSI, SNR, throughput, peer state)
        and publishes them to the MOOSDB.
*************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "GeomUtils.h"
#include "ACTable.h"
#include "DoodleLabs.h"
#include <sstream>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <algorithm>

using namespace std;

//---------------------------------------------------------
// Constructor()

DoodleLabs::DoodleLabs()
{
  m_debug = false;
  m_radio_connected = false;
  m_last_poll_time = 0;
  m_poll_interval = 1.0; // Default 1Hz
  m_gps_poll_interval = 10.0;
  m_last_gps_update = 0.0;
  m_disconnect_timeout_ms = 10000; // Default 10 seconds
  m_wearable_device = false;
  m_wearable_name = "wearable";  // Default wearable name
  m_username = "user";
  m_password = "DoodleSmartRadio";
  
  // Reconnection parameters
  m_reconnect_interval = 5.0; // Try reconnecting every 5 seconds
  m_last_reconnect_attempt = 0;
  m_reconnect_attempts = 0;
  m_max_reconnect_attempts = -1; // -1 means unlimited attempts
  m_connection_timeout = 5;
  m_request_timeout = 10;

  // Auto-discovery defaults
  m_auto_discover_peers = false;
  m_mac_lookup_file = "";
  m_monitor_all_peers = false;
  
  // Initialize known stations tracking
  m_previous_known_stations = "";

  wearable_marker.set_label("Wearable POSE");
  wearable_marker.set_type("gateway");
  wearable_marker.set_color("primary_color", "mediumseagreen");
  wearable_marker.set_width(3);
  wearable_marker.set_edge_color("black");
  wearable_marker.set_transparency(0.3);
  m_wear_x = 0;
  m_wear_y = 0;
}

//---------------------------------------------------------
// Destructor

DoodleLabs::~DoodleLabs()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool DoodleLabs::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    if (key == "FOO")
      cout << "great!";

    else if (key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: dbg_print()
bool DoodleLabs::dbg_print(const char *format, ...)
{
  if (m_debug == true)
  {
    va_list args;
    va_start(args, format);
    m_debug_stream = fopen(m_fname, "a");
    if (m_debug_stream != nullptr)
    {
      vfprintf(m_debug_stream, format, args);
      fclose(m_debug_stream);
      return true;
    }
    else
    {
      reportRunWarning("Debug mode is enabled and file could not be opened\n");
      return false;
    }
  }
  return false;
}

//---------------------------------------------------------
// Procedure: GeodesySetup()

bool DoodleLabs::GeodesySetup()
{
  double LatOrigin = 0.0;
  double LonOrigin = 0.0;

  // Get Latitude Origin from .MOOS Mission File
  bool latOK = m_MissionReader.GetValue("LatOrigin", LatOrigin);
  if (!latOK)
  {
    reportConfigWarning("Latitude origin missing in MOOS file.");
    return (false);
  }

  // Get Longitude Origin from .MOOS Mission File
  bool lonOK = m_MissionReader.GetValue("LongOrigin", LonOrigin);
  if (!lonOK)
  {
    reportConfigWarning("Longitude origin missing in MOOS file.");
    return (false);
  }

  // Initialise CMOOSGeodesy object
  bool geoOK = m_geodesy.Initialise(LatOrigin, LonOrigin);
  if (!geoOK)
  {
    reportConfigWarning("CMOOSGeodesy::Initialise() failed. Invalid origin.");
    return (false);
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool DoodleLabs::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool DoodleLabs::Iterate()
{
  AppCastingMOOSApp::Iterate();
  
  double current_time = MOOSTime();
  
  // If not connected, attempt reconnection
  if(!m_radio_connected && m_radio_driver) {
    if((current_time - m_last_reconnect_attempt) >= m_reconnect_interval) {
      if(m_max_reconnect_attempts < 0 || m_reconnect_attempts < m_max_reconnect_attempts) {
        reportEvent("Attempting to reconnect to radio (attempt " + 
                   intToString(m_reconnect_attempts + 1) + ")");
        if(attemptReconnection()) {
          m_radio_connected = true;
          m_reconnect_attempts = 0;
          reportEvent("Successfully reconnected to radio");
          Notify("DOODLE_RADIO_CONNECTED", 1.0);
        } else {
          m_reconnect_attempts++;
          m_last_reconnect_attempt = current_time;
        }
      }
    }
  }
  
  // Check if it's time to poll the radio
  if(m_radio_connected && (current_time - m_last_poll_time) >= m_poll_interval) {
    m_last_poll_time = current_time;
    
    // Refresh all radio data
    try {
      m_radio_driver->refreshWirelessInfo();
      m_radio_driver->refreshLinkStatus();

      if (m_wearable_device && ((current_time - m_last_gps_update) >= m_gps_poll_interval))
      {
        m_last_gps_update = current_time;
        m_radio_driver->refreshGPSData();
        displaywearablepose();
      }

      // Publish the link status data
      publishLinkStatus();
      
    } catch(const std::exception& e) {
      reportRunWarning("Error polling radio: " + string(e.what()));
      m_radio_connected = false;
      m_last_reconnect_attempt = current_time;
      Notify("DOODLE_RADIO_CONNECTED", 0.0);
    }
  }
  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool DoodleLabs::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());
  m_app_name = GetAppName();
  

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "radio_ip") {
      m_radio_ip = value;
      handled = true;
    }
    else if(param == "username") {
      m_username = value;
      handled = true;
    }
    else if(param == "password") {
      m_password = value;
      handled = true;
    }
    else if(param == "monitor_peers") {
      parseMonitorPeers(value);
      handled = true;
    }
    else if (param == "auto_discover")
    {
      handled = setBooleanOnString(m_auto_discover_peers, value);
    }
    else if (param == "mac_lookup_file")
    {
      m_mac_lookup_file = value;
      if (!m_mac_lookup_file.empty())
      {
        if (!loadMACLookupTable(m_mac_lookup_file))
        {
          reportConfigWarning("Failed to load MAC lookup table from: " + m_mac_lookup_file);
        }
      }
      handled = true;
    }
    else if (param == "monitor_all")
    {
      handled = setBooleanOnString(m_monitor_all_peers, value);
    }
    else if (param == "wearable")
    {
      handled = setBooleanOnString(m_wearable_device, value);
    }
    else if (param == "wearable_name")
    {
      m_wearable_name = value;
      handled = true;
    }
    else if(param == "poll_interval") {
      m_poll_interval = stod(value.c_str());
      if(m_poll_interval <= 0) m_poll_interval = 1.0;
      handled = true;
    }
    else if(param == "reconnect_interval") {
      m_reconnect_interval = stod(value.c_str());
      if(m_reconnect_interval <= 0) m_reconnect_interval = 5.0;
      handled = true;
    }
    else if(param == "max_reconnect_attempts") {
      m_max_reconnect_attempts = stoi(value.c_str());
      handled = true;
    }
    else if(param == "connection_timeout") {
      m_connection_timeout = atol(value.c_str());
      if(m_connection_timeout <= 0) m_connection_timeout = 5;
      handled = true;
    }
    else if(param == "request_timeout") {
      m_request_timeout = atol(value.c_str());
      if(m_request_timeout <= 0) m_request_timeout = 10;
      handled = true;
    }
    else if (param == "disconnect_timeout")
    {
      m_disconnect_timeout_ms = stoi(value.c_str()) * 1000; // Convert seconds to milliseconds
      if (m_disconnect_timeout_ms <= 0)
        m_disconnect_timeout_ms = 10000; // Default 10 seconds
      handled = true;
    }
    else if (param == "debug")
      {
        m_debug = (value == tolower("true")) ? true : false;
        if (m_debug)
        {
          time_t rawtime;
          struct tm *timeinfo;
          memset(m_fname, '\0', m_fname_buff_size);
          time(&rawtime);
          timeinfo = localtime(&rawtime);
          char fmt[m_fname_buff_size];
          memset(fmt, '\0', m_fname_buff_size);
          strftime(fmt, m_fname_buff_size, "%F_%T", timeinfo);
          snprintf(m_fname, m_fname_buff_size, "DBG_%s_%s_DATA.dbg",
                   m_app_name.c_str(), fmt);
        }
        handled = true;
      }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  if (m_wearable_device)
  {
    if (!GeodesySetup())
    {
      reportConfigWarning("Geodesy setup failed.");
      return false;
    }
  }

  // Initialize Doodle Radio Driver
  if(!m_radio_ip.empty()) {
    m_radio_driver.reset(new DoodleRadioDriver());
    m_radio_driver->setTimeouts(m_connection_timeout, m_request_timeout);
    if(m_radio_driver->initialize(m_radio_ip, m_username, m_password)) {
      if(m_radio_driver->login()) {
        m_radio_connected = true;
        reportEvent("Successfully connected to Doodle Radio at " + m_radio_ip);
      } else {
        reportConfigWarning("Failed to login to Doodle Radio at " + m_radio_ip);
      }
    }
    else
    {
      reportConfigWarning("Failed to initialize Doodle Radio driver for " + m_radio_ip);
    }
  } else {
    reportConfigWarning("RADIO_IP not specified - iDoodleLabs will not connect to radio");
  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void DoodleLabs::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // Register("FOOBAR", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool DoodleLabs::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "Doodle Labs Radio Monitor                   " << endl;
  m_msgs << "============================================" << endl;
  
  m_msgs << "Radio IP:     " << m_radio_ip << endl;
  m_msgs << "Connected:    " << (m_radio_connected ? "Yes" : "No") << endl;
  m_msgs << "Poll Rate:    " << (1.0/m_poll_interval) << " Hz" << endl;
  
  if(!m_radio_connected && m_radio_driver) {
    m_msgs << "Reconnect:    Attempt " << m_reconnect_attempts;
    if(m_max_reconnect_attempts > 0) {
      m_msgs << " / " << m_max_reconnect_attempts;
    }
    m_msgs << " (every " << m_reconnect_interval << "s)" << endl;
  }

  m_msgs << "Timeouts:     Connection=" << m_connection_timeout << "s, Request=" << m_request_timeout << "s, Disconnect=" << (m_disconnect_timeout_ms / 1000.0) << "s" << endl;
  m_msgs << endl;
  
  if(m_radio_connected && m_radio_driver) {
    // Show wireless info
    WirelessInfo wirelessInfo = m_radio_driver->getWirelessInfo();
    m_msgs << "Noise Floor:  " << wirelessInfo.noise_level << " dBm" << endl;
    m_msgs << "Channel:      " << wirelessInfo.channel << endl;
    m_msgs << "Frequency:    " << wirelessInfo.frequency << " MHz" << endl;
    m_msgs << endl;

    // Display Wearable GPS
    if (m_wearable_device)
    {
      m_msgs << "============================================" << endl;
      m_msgs << "Wearable GPS Data                          " << endl;
      m_msgs << "============================================" << endl;

      if (wearable_GPS.valid)
      {
        m_msgs << "Status:       " << wearable_GPS.status << endl;
        m_msgs << "Latitude:     " << doubleToString(wearable_GPS.latitude, 6) << "°" << endl;
        m_msgs << "Longitude:    " << doubleToString(wearable_GPS.longitude, 6) << "°" << endl;
        m_msgs << "Altitude:     " << doubleToString(wearable_GPS.altitude, 1) << " m" << endl;
        m_msgs << "Satellites:   " << wearable_GPS.satellites_used << " used / " << wearable_GPS.satellites_visible << " visible" << endl;
        m_msgs << "HDOP:         " << doubleToString(wearable_GPS.hdop, 2) << endl;
        m_msgs << "Speed:        " << doubleToString(wearable_GPS.speed, 1) << " m/s" << endl;
        m_msgs << "Heading:      " << doubleToString(wearable_GPS.heading, 1) << "°" << endl;
        if (!wearable_GPS.time.empty())
        {
          m_msgs << "GPS Time:     " << wearable_GPS.time << endl;
        }
      }
      else if (wearable_GPS.satellites_visible > 0)
      {
        m_msgs << "Status:       Acquiring fix..." << endl;
        m_msgs << "Satellites:   " << wearable_GPS.satellites_visible << " visible" << endl;
      }
      else
      {
        m_msgs << "Status:       No GPS data" << endl;
      }
      m_msgs << endl;
    }

    // Show monitored peers table
    m_msgs << "============================================" << endl;
    m_msgs << "Peer Connection Status                        " << endl;
    m_msgs << "============================================" << endl;

    ACTable actab(7);
    actab << "Vehicle | MAC | RSSI | SNR | TQ | Connected | Hop";
    actab.addHeaderLines();
    
    auto linkStatus = m_radio_driver->getLinkStatus();

    for (const std::pair<std::string, std::string> &peer : m_mac_to_vehicle)
    {
      string vehicle = peer.second;
      string mac = peer.first;

      // Find this peer in the link status
      bool found = false;
      for (const LinkStatus::StationInfo &station : linkStatus.stations)
      {
        if(normalizeMAC(station.mac_address) == mac) {
          found = true;
          
          // Use the same stale detection logic as publishLinkStatus
          bool is_connected = !isStationStale(station);

          if(is_connected) {
            // Peer is connected - show real data
            double snr = station.rssi - wirelessInfo.noise_level;
            actab << vehicle
                  << station.mac_address
                  << station.rssi
                  << doubleToString(snr, 1)
                  << station.mesh_tq
                  << "Yes"
                  << station.mesh_hop_status;
          } else {
            // Peer is disconnected/stale - show ---- for data values
            actab << vehicle
                  << station.mac_address
                  << "----"
                  << "----"
                  << "----"
                  << "No"
                  << "----";
          }
          break;
        }
      }

      if(!found) {
        actab << vehicle << mac << "----" << "----" << "----" << "No" << "----";
      }
    }

    m_msgs << actab.getFormattedString();
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: parseMonitorPeers()
void DoodleLabs::parseMonitorPeers(const string& peer_string)
{
  // Parse format: "alpha:00:30:18:0C:4A:2B,bravo:00:30:18:0C:4A:2C"
  vector<string> peers = parseString(peer_string, ',');
  
  for(const string& peer : peers) {
    string peer_copy = peer;
    string vehicle_name = toupper(biteStringX(peer_copy, ':'));
    string mac = peer_copy;
    
    if(!vehicle_name.empty() && !mac.empty()) {
      mac = normalizeMAC(mac);
      m_mac_to_vehicle[mac] = vehicle_name;
      m_vehicle_to_mac[vehicle_name] = mac;
      reportEvent("Monitoring peer: " + vehicle_name + " [" + mac + "]");
    }
  }
}

//---------------------------------------------------------
// Procedure: normalizeMAC()
string DoodleLabs::normalizeMAC(const string& mac)
{
  string normalized = mac;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](unsigned char c)
                 { return std::toupper(c); });
  return normalized;
}

//---------------------------------------------------------
// Procedure: isStationStale()
bool DoodleLabs::isStationStale(const LinkStatus::StationInfo &station)
{
  return (station.inactive_ms > m_disconnect_timeout_ms ||
          station.mesh_last_seen_ms > m_disconnect_timeout_ms);
}

//---------------------------------------------------------
// Procedure: publishLinkStatus()
void DoodleLabs::publishLinkStatus()
{
  if(!m_radio_driver) return;

  // Fetch once
  WirelessInfo wirelessInfo = m_radio_driver->getWirelessInfo();
  LinkStatus linkStatus = m_radio_driver->getLinkStatus();

  if (m_auto_discover_peers)
    autoDiscoverPeers(linkStatus);

  // Index stations by normalized MAC once
  std::unordered_map<std::string, const LinkStatus::StationInfo *> by_mac;
  by_mac.reserve(linkStatus.stations.size());
  for (const LinkStatus::StationInfo &s : linkStatus.stations)
  {
    by_mac.emplace(normalizeMAC(s.mac_address), &s);
  }

  int monitored_peer_count = 0;

  // Single pass over monitored peers
  for (const std::pair<const std::string, std::string> &mac_vehicle : m_mac_to_vehicle)
  {
    std::unordered_map<std::string, const LinkStatus::StationInfo *>::iterator it = by_mac.find(mac_vehicle.first);
    if (it == by_mac.end())
    {
      // Not found => disconnected
      Notify("DOODLE_" + mac_vehicle.second + "_CONNECTED", 0.0);
      continue;
    }

    const LinkStatus::StationInfo &station = *it->second;

    if (isStationStale(station))
    {
      Notify("DOODLE_" + mac_vehicle.second + "_CONNECTED", 0.0);
      continue;
    }

    // Connected path:
    monitored_peer_count++;
    
    // Add to all known stations set (never removed once added)
    m_all_known_stations.insert(mac_vehicle.second);

    // Publish minimal set (see change-threshold optimization below)
    Notify("DOODLE_" + mac_vehicle.second + "_CONNECTED", 1.0);
    Notify("DOODLE_" + mac_vehicle.second + "_RSSI", station.rssi);
    if (station.rssi_ant.size() > 0)
      Notify("DOODLE_" + mac_vehicle.second + "_RSSI_ANT0", station.rssi_ant[0]);
    if (station.rssi_ant.size() > 1)
      Notify("DOODLE_" + mac_vehicle.second + "_RSSI_ANT1", station.rssi_ant[1]);

    const double snr = station.rssi - wirelessInfo.noise_level;
    Notify("DOODLE_" + mac_vehicle.second + "_SNR", snr);

    Notify("DOODLE_" + mac_vehicle.second + "_TQ", (double)station.mesh_tq);

    double hop_count = 0.0;
    if (station.mesh_hop_status == "direct")
      hop_count = 1.0;
    else
    {
      size_t pos = station.mesh_hop_status.find(' ');
      if (pos != std::string::npos)
      {
        hop_count = std::stod(station.mesh_hop_status.substr(0, pos));
      }
    }
Notify("DOODLE_" + mac_vehicle.second + "_HOP_COUNT", hop_count);
  }

  Notify("DOODLE_NUM_PEERS", monitored_peer_count);
  
  // Build comma-separated list from the set
  std::string known_stations_list;
  for (const std::string& station : m_all_known_stations)
  {
    if (!known_stations_list.empty())
      known_stations_list += ",";
    known_stations_list += station;
  }
  
  // Only notify if the known stations list has changed
  if (known_stations_list != m_previous_known_stations)
  {
    Notify("DOODLE_KNOWN_STATIONS", known_stations_list);
    m_previous_known_stations = known_stations_list;
  }
  
  if (wirelessInfo.noise_level != 0)
  {
    Notify("DOODLE_NOISE_FLOOR", wirelessInfo.noise_level);
  }
  Notify("DOODLE_CONGESTION", linkStatus.activity_percent);
}

void DoodleLabs::displaywearablepose()
{
  // Get wearable GPS data
  if (!m_radio_driver)
    return;
  wearable_GPS = m_radio_driver->getGPSData();

  // Only publish data if GPS is valid
  if (wearable_GPS.valid)
  {
    // Publish minimal GPS position data
    Notify("WEARABLE_GPS_LAT", wearable_GPS.latitude);
    Notify("WEARABLE_GPS_LON", wearable_GPS.longitude);

    // Create marker
    // geodesic conversion for NAV_X and NAV_Y
    // Convert Data if we have position data
    if (wearable_GPS.latitude != 0.0 && wearable_GPS.longitude != 0.0)
    {
      double x, y;
      if (m_geodesy.LatLong2LocalGrid(wearable_GPS.latitude, wearable_GPS.longitude, y, x))
      {
        m_wear_x = x;
        m_wear_y = y;
        wearable_marker.set_vx(m_wear_x);
        wearable_marker.set_vy(m_wear_y);

        Notify("VIEW_MARKER", wearable_marker.get_spec());

        // Publish NODE_REPORT for wearable with configured name
        string node_report = "NAME=" + m_wearable_name + ",TYPE=WEARABLE,";
        node_report += "X=" + doubleToString(m_wear_x) + ",";
        node_report += "Y=" + doubleToString(m_wear_y) + ",";
        node_report += "LAT=" + doubleToString(wearable_GPS.latitude) + ",";
        node_report += "LON=" + doubleToString(wearable_GPS.longitude) + ",";
        node_report += "SPD=0,HDG=0,DEP=0,";
        node_report += "MODE=ACTIVE,";
        node_report += "TIME=" + doubleToString(MOOSTime());

        // Publish as NODE_REPORT_[NAME] following convention
        string report_var = "NODE_REPORT_" + toupper(m_wearable_name);
        Notify(report_var, node_report);
      }
    }
  }
}

//---------------------------------------------------------
// Procedure: attemptReconnection()
bool DoodleLabs::attemptReconnection()
{
  if(!m_radio_driver || m_radio_ip.empty()) {
    return false;
  }
  
  // Try to disconnect cleanly first
  m_radio_driver->disconnect();
  
  // Re-initialize and login
  if(m_radio_driver->initialize(m_radio_ip, m_username, m_password)) {
    if(m_radio_driver->login()) {
      return true;
    }
  }
  
  return false;
}

//---------------------------------------------------------
// Procedure: loadMACLookupTable()
bool DoodleLabs::loadMACLookupTable(const string &filename)
{
  FILE *fp = fopen(filename.c_str(), "r");
  if (!fp)
  {
    return false;
  }

  char line[256];
  while (fgets(line, sizeof(line), fp))
  {
    string str_line(line);

    // Remove newline and whitespace manually
    str_line = stripBlankEnds(str_line);

    // Skip empty lines and comments
    if (str_line.empty() || str_line[0] == '#' || str_line[0] == '/')
    {
      continue;
    }

    // Parse format: VEHICLE_NAME:MAC_ADDRESS
    size_t colon_pos = str_line.find(':');
    if (colon_pos != string::npos)
    {
      string vehicle_name = str_line.substr(0, colon_pos);
      string mac_raw = str_line.substr(colon_pos + 1);

      // Remove any remaining whitespace/newlines from both parts
      vehicle_name = stripBlankEnds(vehicle_name);
      mac_raw = stripBlankEnds(mac_raw);

      // Convert to uppercase and normalize
      vehicle_name = toupper(vehicle_name);
      string mac = normalizeMAC(mac_raw);

      if (!vehicle_name.empty() && !mac.empty())
      {
        // Only add to lookup table, not to monitored peers
        m_mac_lookup_table[mac] = vehicle_name;
        reportEvent("Loaded MAC lookup: " + vehicle_name + " [" + mac + "]");
      }
    }
  }

  fclose(fp);
  reportEvent("Loaded " + intToString(m_mac_lookup_table.size()) + " MAC lookups from " + filename);

  // Debug: print loaded lookup table
  dbg_print("MAC Lookup Table contents (%zu entries):\n", m_mac_lookup_table.size());
  for (const std::pair<const std::string, std::string> &entry : m_mac_lookup_table)
  {
    dbg_print("  '%s' -> '%s'\n", entry.first.c_str(), entry.second.c_str());
  }

  return true;
}

//---------------------------------------------------------
// Procedure: generateVehicleNameFromMAC()
string DoodleLabs::generateVehicleNameFromMAC(const string &mac)
{
  // Generate a simple vehicle name from the last 2 octets of MAC address
  // Example: 00:30:1A:3C:40:97 -> RADIO_4097
  string normalized = normalizeMAC(mac);

  // Extract last 2 octets
  size_t last_colon = normalized.rfind(':');
  if (last_colon != string::npos && last_colon >= 3)
  {
    string last_two = normalized.substr(last_colon - 2);
    // Remove colons
    last_two.erase(remove(last_two.begin(), last_two.end(), ':'), last_two.end());
    return "RADIO_" + last_two;
  }

  // Fallback: use full MAC with underscores
  string name = "RADIO_" + normalized;
  replace(name.begin(), name.end(), ':', '_');
  return name;
}

//---------------------------------------------------------
// Procedure: autoDiscoverPeers()
void DoodleLabs::autoDiscoverPeers(const LinkStatus &linkStatus)
{
  for (const LinkStatus::StationInfo &station : linkStatus.stations)
  {
    string mac = normalizeMAC(station.mac_address);
    dbg_print("Processing station MAC: original='%s' normalized='%s'\n",
              station.mac_address.c_str(), mac.c_str());

    // Check if we already monitor this peer
    std::unordered_map<std::string, std::string>::iterator it = m_mac_to_vehicle.find(mac);
    if (it == m_mac_to_vehicle.end())
    {
      // New peer discovered - not currently monitored
      string vehicle_name;

      // First check the lookup table for a known name
      std::unordered_map<std::string, std::string>::iterator lookup_it = m_mac_lookup_table.find(mac);
      if (lookup_it != m_mac_lookup_table.end())
      {
        vehicle_name = lookup_it->second;
        dbg_print("Found MAC '%s' in lookup table as '%s'\n", mac.c_str(), vehicle_name.c_str());
      }
      // If not in lookup table and monitor_all is enabled, generate a name
      else
      {
        dbg_print("MAC '%s' NOT found in lookup table (table has %zu entries)\n",
                  mac.c_str(), m_mac_lookup_table.size());
        if (m_monitor_all_peers)
        {
          vehicle_name = generateVehicleNameFromMAC(mac);
          dbg_print("Generated name for unknown MAC '%s': '%s'\n", mac.c_str(), vehicle_name.c_str());
        }
        else
        {
          dbg_print("MONITOR_ALL=false, skipping MAC '%s'\n", mac.c_str());
        }
      }

      // If we have a name (either from lookup or generated), add to monitored peers
      if (!vehicle_name.empty())
      {
        m_mac_to_vehicle[mac] = vehicle_name;
        m_vehicle_to_mac[vehicle_name] = mac;
        reportEvent("Auto-discovered peer: " + vehicle_name + " [" + mac + "]");
      }
    }
  }
}
