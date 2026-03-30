#ifndef FrontSeatBroker_HEADER
#define FrontSeatBroker_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "BrokerV2Codec.hpp"
#include "networkbroker.hpp"
#include "networkclient.hpp"

#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>

class FrontSeatBroker : public AppCastingMOOSApp
{
public:
   FrontSeatBroker();
   ~FrontSeatBroker();

protected:
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

protected:
   bool buildReport();

protected:
   void registerVariables();
   bool dbg_print(const char *format, ...);
  bool configureSocketEndpoints();
  void stopRxThread();
  void receiveLoop();
  void drainInboundFrames();
  void sendOutboundFrame();
  void resetDropBaselines();
  void postDropDeltasToMoos();

private:
  bool m_debug;
  FILE *m_debug_stream;
  static const uint16_t m_fname_buff_size = 256;
  std::string m_app_name;
  char m_fname[m_fname_buff_size];

  std::string m_listen_ip;
  int m_listen_port;
  std::string m_tx_ip;
  int m_tx_port;

  std::set<std::string> m_tx_vars;
  broker_v2::NameMap m_in_map;

  std::unique_ptr<SocketClient> m_rx_client;
  std::unique_ptr<SocketBroker> m_tx_broker;
  std::thread m_rx_thread;
  std::atomic<bool> m_rx_thread_running;

  std::mutex m_rx_mutex;
  std::deque<broker_v2::FieldMap> m_rx_frames;

  broker_v2::FieldMap m_latest_tx_fields;

  unsigned int m_rx_frames_received;
  unsigned int m_rx_frames_parsed;
  unsigned int m_rx_parse_errors;
  unsigned int m_tx_frames_sent;
  unsigned int m_tx_send_errors;

  // Per-variable message counts for packet drop detection
  unsigned int m_tx_var_count;      // Total variables I have sent
  unsigned int m_rx_var_count;      // Total variables I have received
  unsigned int m_remote_tx_count;   // What remote reports sending to me
  unsigned int m_remote_rx_count;   // What remote reports receiving from me

  // Session-baseline counters (reset when backseat startup ID changes)
  std::string m_last_backseat_startup_id;
  unsigned int m_base_tx_var_count;
  unsigned int m_base_rx_var_count;
  unsigned int m_base_remote_tx_count;
  unsigned int m_base_remote_rx_count;
  bool m_drop_baseline_valid;

  // Optional MOOS posting for per-session drop deltas
  bool m_post_drop_deltas_to_moos;
  std::string m_outbound_drop_var;
  std::string m_inbound_drop_var;
  int m_last_posted_outbound_drop;
  int m_last_posted_inbound_drop;
};

#endif 
