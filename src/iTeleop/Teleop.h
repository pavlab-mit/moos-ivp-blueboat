/*************************************************************
      Name:
      Orgn: MIT, Cambridge MA
      File: iTeleop/Teleop.h
   Last Ed:  2026-07-15
     Brief:
        Front-seat teleop endpoint. Listens on UDP for a shore
        GUI, holds a single-client session with a deadman, and
        publishes TELEOP_ACTIVE / TELEOP_THRUST_L / TELEOP_THRUST_R
        for the navigator interface's teleop arbitration branch.
*************************************************************/

#ifndef Teleop_HEADER
#define Teleop_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "BrokerV2Codec.hpp"
#include "command_envelope.h"

#include <netinet/in.h>

#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

class Teleop : public AppCastingMOOSApp
{
public:
   Teleop();
   ~Teleop();

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
   bool configureSocket();
   void stopRxThread();
   void receiveLoop();
   void drainInboundFrames();
   void handleFrame(const broker_v2::FieldMap &fields, const sockaddr_in &src);
   void claimSession(const broker_v2::FieldMap &fields, const sockaddr_in &src);
   void applyCommand(const broker_v2::FieldMap &fields);
   void deactivateSession(const std::string &reason, bool warn);
   void sendAck(const sockaddr_in &dest, const std::string &ack_type, double seq);
   void publishTeleopState();
   void publishTeleopCmd();
   bool ownerMatches(const broker_v2::FieldMap &fields, const sockaddr_in &src) const;
   std::string fusedMode() const;
   static std::string addrToString(const sockaddr_in &addr);

private:
  bool m_debug;
  FILE *m_debug_stream;
  static const uint16_t m_fname_buff_size = 256;
  std::string m_app_name;
  char m_fname[m_fname_buff_size];

  // Configuration
  std::string m_listen_ip;
  int m_listen_port;
  double m_gui_deadman_timeout;  // seconds without an owner frame -> release
  double m_max_thrust;           // clamp on inbound thrust magnitude
  double m_ack_heartbeat_hz;     // idle ack rate while a session is active

  // Socket / receive thread
  int m_sockfd;
  std::thread m_rx_thread;
  std::atomic<bool> m_rx_thread_running;

  struct InboundFrame {
    broker_v2::FieldMap fields;
    sockaddr_in src;
  };
  std::mutex m_rx_mutex;
  std::deque<InboundFrame> m_rx_frames;

  // Session state (single owner at a time)
  bool m_session_active;
  sockaddr_in m_owner_addr;
  std::string m_owner_sid;
  std::string m_owner_client;
  double m_owner_last_seq;
  double m_last_owner_frame_time;
  double m_last_ack_time;

  // Latest teleop command.
  //
  // The GUI speaks TANK (THRUST_L/THRUST_R). The command contract
  // speaks SEMANTIC surge/yaw (invariant 2). Both are held: the
  // tank pair for the legacy TELEOP_THRUST_L/R publications that
  // the current Navigator still consumes, and the semantic pair
  // for TELEOP_CMD.
  //
  // A GUI that sends SURGE/YAW directly is preferred and its
  // values are used verbatim. Otherwise they are derived from the
  // tank pair -- see publishTeleopCmd() for why that conversion
  // is an approximation and not a round trip.
  double m_cmd_thrust_left;
  double m_cmd_thrust_right;
  double m_cmd_surge;
  double m_cmd_yaw;
  bool m_cmd_is_semantic;   // GUI sent SURGE/YAW rather than tank
  bool m_estop;

  // Command contract identity. The epoch is minted once per
  // launch; the sequence advances on every publication so the
  // arbiter's lease can tell a live session from a stuck one.
  std::string m_cmd_epoch;
  uint64_t    m_cmd_seq;
  std::string m_teleop_cmd_var;

  // Cached vehicle telemetry for acks (from OnNewMail)
  double m_applied_thrust_left;   // NVGR_THRUST_LEFT_WIRE
  double m_applied_thrust_right;  // NVGR_THRUST_RIGHT_WIRE
  bool m_rc_mode;                 // NVGR_RC_MODE
  bool m_rc_deadman_active;       // NVGR_RC_DEADMAN_ACTIVE
  double m_rolling_voltage;       // NVGR_ROLLING_VOLTAGE

  // Counters for appcasting
  unsigned int m_rx_frames_received;
  unsigned int m_rx_parse_errors;
  unsigned int m_frames_applied;
  unsigned int m_busy_rejects;
  unsigned int m_stale_seq_drops;
  unsigned int m_acks_sent;
  unsigned int m_ack_send_errors;
  unsigned int m_sessions_started;
  unsigned int m_deadman_trips;
};

#endif
