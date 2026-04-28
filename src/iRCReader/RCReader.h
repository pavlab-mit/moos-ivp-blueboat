/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: iRCReader/RCReader.h
   Last Ed:  2025-03-20
     Brief:
        MOOS app that reads SBUS data from a serial-connected RC
        receiver and publishes per-channel values and connection
        status to the MOOSDB (RC_CH1..RC_CH16, RC_CONNECTED, etc.).
*************************************************************/

#ifndef RCReader_HEADER
#define RCReader_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <string>
#include <cstdarg> //va_list, va_start, va_end
#include "sbus_handler.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

class RCReader : public AppCastingMOOSApp
{
 public:
   RCReader();
   ~RCReader();

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

 private: // Configuration variables

  bool m_debug;
  FILE *m_debug_stream;
  static const uint16_t m_fname_buff_size = 256;
  std::string m_app_name;
  char m_fname[m_fname_buff_size];

 private: // State variables
  // SBUS handler and thread
  SbusHandler m_sbus;
  std::thread m_sbus_thread;
  std::atomic<bool> m_running;
  std::mutex m_mutex;

  // Connection parameters.
  // m_last_update_time is retained for diagnostics in buildReport();
  // it no longer gates the connection decision (SbusHandler owns that).
  double m_last_update_time;
  bool m_rc_connected;
  bool m_frame_valid;     // mirror of m_sbus.isFrameValid()

  // Channel data
  uint16_t m_channels[SBUS_NUM_CHANNELS];
  double m_scaled_channels[SBUS_NUM_CHANNELS];

  // SBUS flags
  bool m_channel_17;
  bool m_channel_18;
  bool m_frame_lost;
  bool m_failsafe;

  // Helper functions
  void SbusThreadFunction();
  double ScaleJoystick(uint16_t value);
  int MapSwitch(uint16_t value, int num_states);
};

#endif
