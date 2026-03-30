/*************************************************************
      Name: 
      Orgn: MIT, Cambridge MA
      File: iDoodleLabs/DoodleLabs.h
   Last Ed:  2025-07-21
     Brief:
        Lorem ipsum dolor sit amet, consectetur adipiscing 
        elit, sed do eiusmod tempor incididunt ut labore et 
        dolore magna aliqua. Ut enim ad minim veniam, quis 
        nostrud exercitation ullamco laboris nisi ut aliquip 
        ex ea commodo consequat.
*************************************************************/

#ifndef DoodleLabs_HEADER
#define DoodleLabs_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <string>
#include <map>
#include <set>
#include <memory>
#include "DoodleRadioDriver.h"
#include "XYMarker.h"
#include <cstdarg> //va_list, va_start, va_end
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"

// Forward declaration
class DoodleRadioDriver;

class DoodleLabs : public AppCastingMOOSApp
{
 public:
   DoodleLabs();
   ~DoodleLabs();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   bool GeodesySetup();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   bool dbg_print(const char *format, ...);

 private: // Configuration variables
   CMOOSGeodesy m_geodesy;
   double m_wear_x;
   double m_wear_y;
   bool m_debug;
   FILE *m_debug_stream;
   static const uint16_t m_fname_buff_size = 256;
   std::string m_app_name;
   char m_fname[m_fname_buff_size];

   // Doodle Radio configuration
   std::string m_radio_ip;
   std::string m_username;
   std::string m_password;
   std::unordered_map<std::string, std::string> m_mac_to_vehicle;   // MAC -> Vehicle name (monitored peers)
   std::unordered_map<std::string, std::string> m_vehicle_to_mac;   // Vehicle name -> MAC (monitored peers)
   std::unordered_map<std::string, std::string> m_mac_lookup_table; // MAC -> Vehicle name (lookup table for naming)
   double m_poll_interval;
   int m_disconnect_timeout_ms; // Timeout in milliseconds to consider peer disconnected

   // Auto-discovery configuration
   bool m_auto_discover_peers;    // Enable auto-discovery of peers
   std::string m_mac_lookup_file; // Path to MAC address lookup table file
   bool m_monitor_all_peers;      // Monitor all discovered peers, not just those in lookup table

 private: // State variables
  std::unique_ptr<DoodleRadioDriver> m_radio_driver;
  bool m_radio_connected;
  double m_last_poll_time;
  bool m_wearable_device;
  std::string m_wearable_name;
  GPSData wearable_GPS;
  XYMarker wearable_marker;
  double m_gps_poll_interval;
  double m_last_gps_update;
  std::string m_previous_known_stations;  // Track previous known stations list
  std::set<std::string> m_all_known_stations;  // Set of all stations ever seen

  // Reconnection parameters
  double m_reconnect_interval;
  double m_last_reconnect_attempt;
  int m_reconnect_attempts;
  int m_max_reconnect_attempts;
  long m_connection_timeout;
  long m_request_timeout;

  // Helper methods
  void parseMonitorPeers(const std::string& peer_string);
  std::string normalizeMAC(const std::string& mac);
  bool isStationStale(const LinkStatus::StationInfo& station);
  void publishLinkStatus();
  void displaywearablepose();
  bool attemptReconnection();
  bool loadMACLookupTable(const std::string &filename);
  std::string generateVehicleNameFromMAC(const std::string &mac);
  void autoDiscoverPeers(const LinkStatus &linkStatus);
};

#endif 
