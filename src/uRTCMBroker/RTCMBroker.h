/*************************************************************
      Name: Jeremy Wenger
      Orgn: MIT, Cambridge MA
      File: uRTCMBroker/RTCMBroker.h
   Last Ed: 2026-03-17
     Brief:
        NTRIP client that connects to an RTCM caster and
        distributes binary RTCM corrections to vehicle
        communities via MOOS binary messages. Designed to
        run on the shoreside community.
*************************************************************/

#ifndef RTCMBroker_HEADER
#define RTCMBroker_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <string>
#include <cstdarg>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

class RTCMBroker : public AppCastingMOOSApp
{
 public:
   RTCMBroker();
   ~RTCMBroker();

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
   bool buildReport();

 protected:
   void registerVariables();
   bool dbg_print(const char *format, ...);

 private: // NTRIP methods
   bool ntripConnect();
   void ntripDisconnect();
   void ntripThreadFunc();
   std::string buildGGA();

 private: // Configuration variables
   bool m_debug;
   FILE *m_debug_stream;
   static const uint16_t m_fname_buff_size = 256;
   std::string m_app_name;
   char m_fname[m_fname_buff_size];

   // NTRIP config
   std::string m_ntrip_host;
   int         m_ntrip_port;
   std::string m_ntrip_mountpoint;
   std::string m_ntrip_user;
   std::string m_ntrip_password;
   bool        m_send_gga;
   double      m_gga_lat;
   double      m_gga_lon;
   double      m_gga_alt;
   double      m_gga_interval;       // seconds between GGA sends
   double      m_reconnect_interval;  // seconds between reconnect attempts

 private: // State variables
   // NTRIP connection
   int  m_ntrip_sock;
   bool m_ntrip_connected;

   // NTRIP thread
   std::thread       m_ntrip_thread;
   std::atomic<bool> m_thread_running;
   std::mutex        m_data_mutex;

   // Buffered RTCM data for publishing in Iterate()
   std::vector<std::vector<unsigned char>> m_rtcm_queue;

   // Stats
   unsigned long m_bytes_received;
   unsigned int  m_msgs_published;
   unsigned int  m_connect_attempts;
   double        m_last_data_time;
   double        m_last_gga_time;
   std::string   m_connection_status;
};

#endif
