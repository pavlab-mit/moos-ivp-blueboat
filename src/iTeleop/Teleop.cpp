/*************************************************************
      Name:
      Orgn: MIT, Cambridge MA
      File: iTeleop/Teleop.cpp
   Last Ed:  2026-07-15
     Brief:
        Front-seat teleop endpoint. See Teleop.h.
*************************************************************/

#include "Teleop.h"

#include "ACTable.h"
#include "MBUtils.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <ctime>

using namespace std;

Teleop::Teleop()
    : m_debug(false),
      m_debug_stream(nullptr),
      m_app_name(""),
      m_listen_ip("0.0.0.0"),
      m_listen_port(9310),
      m_gui_deadman_timeout(1.0),
      m_max_thrust(100.0),
      m_ack_heartbeat_hz(5.0),
      m_sockfd(-1),
      m_rx_thread_running(false),
      m_session_active(false),
      m_owner_sid(""),
      m_owner_client(""),
      m_owner_last_seq(0),
      m_last_owner_frame_time(0),
      m_last_ack_time(0),
      m_cmd_thrust_left(0),
      m_cmd_thrust_right(0),
      m_estop(false),
      m_applied_thrust_left(0),
      m_applied_thrust_right(0),
      m_rc_mode(false),
      m_rc_deadman_active(false),
      m_rolling_voltage(0),
      m_rx_frames_received(0),
      m_rx_parse_errors(0),
      m_frames_applied(0),
      m_busy_rejects(0),
      m_stale_seq_drops(0),
      m_acks_sent(0),
      m_ack_send_errors(0),
      m_sessions_started(0),
      m_deadman_trips(0) {
  memset(m_fname, 0, m_fname_buff_size);
  memset(&m_owner_addr, 0, sizeof(m_owner_addr));
}

Teleop::~Teleop() { stopRxThread(); }

//---------------------------------------------------------
// Procedure: OnNewMail()

bool Teleop::OnNewMail(MOOSMSG_LIST &NewMail) {
  AppCastingMOOSApp::OnNewMail(NewMail);

  for (MOOSMSG_LIST::iterator p = NewMail.begin(); p != NewMail.end(); ++p) {
    CMOOSMsg &msg = *p;
    const string key = msg.GetKey();

    if (key == "NVGR_THRUST_LEFT_WIRE")
      m_applied_thrust_left = msg.GetDouble();
    else if (key == "NVGR_THRUST_RIGHT_WIRE")
      m_applied_thrust_right = msg.GetDouble();
    else if (key == "NVGR_RC_MODE")
      m_rc_mode = (msg.GetString() == "true");
    else if (key == "NVGR_RC_DEADMAN_ACTIVE")
      m_rc_deadman_active = (msg.GetString() == "true");
    else if (key == "NVGR_ROLLING_VOLTAGE")
      m_rolling_voltage = msg.GetDouble();
    else if (key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }

  return true;
}

//---------------------------------------------------------
// Procedure: dbg_print()

bool Teleop::dbg_print(const char *format, ...) {
  if (!m_debug) {
    return false;
  }

  va_list args;
  va_start(args, format);
  m_debug_stream = fopen(m_fname, "a");
  if (m_debug_stream != nullptr) {
    vfprintf(m_debug_stream, format, args);
    fclose(m_debug_stream);
    va_end(args);
    return true;
  }
  va_end(args);
  reportRunWarning("Debug mode is enabled and file could not be opened");
  return false;
}

//---------------------------------------------------------
// Procedure: configureSocket()
//            Raw POSIX UDP socket rather than lib_nb_utils
//            SocketClient: readSentence() discards the sender
//            address, and acks must go back to the GUI's source
//            addr/port. Precedent: pBB_Status status socket.

bool Teleop::configureSocket() {
  m_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (m_sockfd < 0) {
    reportConfigWarning(string("Failed to create socket: ") + strerror(errno));
    return false;
  }

  int reuse = 1;
  setsockopt(m_sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(m_listen_port));
  if (m_listen_ip.empty() || m_listen_ip == "0.0.0.0") {
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
  } else if (inet_pton(AF_INET, m_listen_ip.c_str(), &addr.sin_addr) != 1) {
    reportConfigWarning("Invalid listen_ip: " + m_listen_ip);
    ::close(m_sockfd);
    m_sockfd = -1;
    return false;
  }

  if (::bind(m_sockfd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    reportConfigWarning("Failed to bind " + m_listen_ip + ":" +
                        intToString(m_listen_port) + ": " + strerror(errno));
    ::close(m_sockfd);
    m_sockfd = -1;
    return false;
  }

  m_rx_thread_running = true;
  m_rx_thread = thread(&Teleop::receiveLoop, this);
  return true;
}

//---------------------------------------------------------
// Procedure: stopRxThread()

void Teleop::stopRxThread() {
  if (m_rx_thread_running) {
    m_rx_thread_running = false;
    if (m_sockfd >= 0) {
      // shutdown + close unblocks the recvfrom in receiveLoop
      ::shutdown(m_sockfd, SHUT_RDWR);
      ::close(m_sockfd);
      m_sockfd = -1;
    }
  }
  if (m_rx_thread.joinable()) {
    m_rx_thread.join();
  }
}

//---------------------------------------------------------
// Procedure: receiveLoop()

void Teleop::receiveLoop() {
  while (m_rx_thread_running) {
    char buf[2048];
    sockaddr_in src;
    socklen_t src_len = sizeof(src);
    memset(&src, 0, sizeof(src));

    const ssize_t got = recvfrom(m_sockfd, buf, sizeof(buf) - 1, 0,
                                 reinterpret_cast<sockaddr *>(&src), &src_len);
    if (!m_rx_thread_running) {
      break;
    }
    if (got <= 0) {
      continue;
    }
    buf[got] = '\0';

    m_rx_frames_received++;

    broker_v2::FieldMap fields;
    string error;
    if (!broker_v2::parseFrame(string(buf), fields, error)) {
      m_rx_parse_errors++;
      dbg_print("RX parse error: %s, raw=[%s]\n", error.c_str(), buf);
      continue;
    }

    lock_guard<mutex> guard(m_rx_mutex);
    m_rx_frames.push_back(InboundFrame{fields, src});
  }
}

//---------------------------------------------------------
// Procedure: drainInboundFrames()

void Teleop::drainInboundFrames() {
  deque<InboundFrame> local_frames;
  {
    lock_guard<mutex> guard(m_rx_mutex);
    local_frames.swap(m_rx_frames);
  }

  for (const auto &frame : local_frames) {
    handleFrame(frame.fields, frame.src);
  }
}

//---------------------------------------------------------
// Procedure: handleFrame()
//            Session state machine. Ownership is keyed on
//            (src_ip, src_port, SID); a non-owner gets a BUSY
//            ack and is otherwise ignored.

void Teleop::handleFrame(const broker_v2::FieldMap &fields, const sockaddr_in &src) {
  string cmd = "CMD";
  auto p = fields.find("TOP_CMD");
  if ((p != fields.end()) && !p->second.is_double) {
    cmd = p->second.sval;
  }

  double seq = 0;
  p = fields.find("SEQ");
  if ((p != fields.end()) && p->second.is_double) {
    seq = p->second.dval;
  }

  // No active session: CONNECT claims it. A bare CMD also claims
  // it so a GUI restarted after a crash regains control without a
  // handshake round-trip.
  if (!m_session_active) {
    if ((cmd == "CONNECT") || (cmd == "CMD")) {
      claimSession(fields, src);
      if (cmd == "CMD") {
        applyCommand(fields);
      }
      sendAck(src, "OK", seq);
    } else {
      sendAck(src, "BYE", seq);
    }
    return;
  }

  // Active session, frame from someone else: reject.
  if (!ownerMatches(fields, src)) {
    m_busy_rejects++;
    dbg_print("BUSY reject from %s\n", addrToString(src).c_str());
    sendAck(src, "BUSY", seq);
    return;
  }

  // Owner frame. Guard against reordered/replayed datagrams.
  if (seq <= m_owner_last_seq) {
    m_stale_seq_drops++;
    return;
  }
  m_owner_last_seq = seq;
  m_last_owner_frame_time = MOOSTime();

  if (cmd == "DISCONNECT") {
    sendAck(src, "BYE", seq);
    deactivateSession("owner disconnected", false);
    return;
  }

  applyCommand(fields);
  sendAck(src, "OK", seq);
}

//---------------------------------------------------------
// Procedure: claimSession()

void Teleop::claimSession(const broker_v2::FieldMap &fields, const sockaddr_in &src) {
  m_session_active = true;
  m_owner_addr = src;
  m_owner_sid = "";
  m_owner_client = "";
  m_owner_last_seq = 0;
  m_cmd_thrust_left = 0;
  m_cmd_thrust_right = 0;
  m_estop = false;
  m_last_owner_frame_time = MOOSTime();
  m_sessions_started++;

  auto p = fields.find("SID");
  if ((p != fields.end()) && !p->second.is_double) {
    m_owner_sid = p->second.sval;
  }
  p = fields.find("CLIENT");
  if ((p != fields.end()) && !p->second.is_double) {
    m_owner_client = p->second.sval;
  }
  p = fields.find("SEQ");
  if ((p != fields.end()) && p->second.is_double) {
    m_owner_last_seq = p->second.dval;
  }

  reportEvent("Teleop session started by " + m_owner_client + " (" +
              addrToString(src) + ")");
  Notify("TELEOP_CLIENT", m_owner_client + " (" + addrToString(src) + ")");
  dbg_print("Session claimed by %s sid=%s\n", addrToString(src).c_str(),
            m_owner_sid.c_str());
}

//---------------------------------------------------------
// Procedure: applyCommand()
//            Thrust is in the wire convention (matches
//            DESIRED_THRUST_L/R); per-thruster inversion is
//            applied downstream by the navigator interface.

void Teleop::applyCommand(const broker_v2::FieldMap &fields) {
  auto p = fields.find("THRUST_L");
  if ((p != fields.end()) && p->second.is_double) {
    m_cmd_thrust_left = p->second.dval;
  }
  p = fields.find("THRUST_R");
  if ((p != fields.end()) && p->second.is_double) {
    m_cmd_thrust_right = p->second.dval;
  }
  p = fields.find("ESTOP");
  if ((p != fields.end()) && p->second.is_double) {
    const bool estop = (p->second.dval != 0);
    if (estop != m_estop) {
      reportEvent(estop ? "Teleop E-STOP latched" : "Teleop E-STOP released");
    }
    m_estop = estop;
  }

  const double limit = (m_max_thrust < 100.0) ? m_max_thrust : 100.0;
  m_cmd_thrust_left = vclip(m_cmd_thrust_left, -limit, limit);
  m_cmd_thrust_right = vclip(m_cmd_thrust_right, -limit, limit);

  m_frames_applied++;
}

//---------------------------------------------------------
// Procedure: deactivateSession()

void Teleop::deactivateSession(const string &reason, bool warn) {
  m_session_active = false;
  m_cmd_thrust_left = 0;
  m_cmd_thrust_right = 0;
  m_estop = false;

  // Publish the safe state immediately rather than waiting for the
  // next iterate: the navigator interface gates on TELEOP_ACTIVE.
  Notify("TELEOP_ACTIVE", "false");
  Notify("TELEOP_THRUST_L", 0.0);
  Notify("TELEOP_THRUST_R", 0.0);
  Notify("TELEOP_CLIENT", "none");

  if (warn) {
    reportRunWarning("Teleop session ended: " + reason);
  } else {
    reportEvent("Teleop session ended: " + reason);
  }
  dbg_print("Session ended: %s\n", reason.c_str());
}

//---------------------------------------------------------
// Procedure: sendAck()

void Teleop::sendAck(const sockaddr_in &dest, const string &ack_type, double seq) {
  if (m_sockfd < 0) {
    return;
  }

  broker_v2::FieldMap out;
  broker_v2::FieldValue fv;

  fv.is_double = false;
  fv.sval = ack_type;
  out["TOP_ACK"] = fv;

  fv.sval = fusedMode();
  out["MODE"] = fv;

  fv.sval = m_host_community;
  out["VNAME"] = fv;

  if (ack_type == "BUSY") {
    fv.sval = m_owner_client;
    out["OWNER"] = fv;
  }

  fv.is_double = true;
  fv.dval = seq;
  out["SEQ"] = fv;
  fv.dval = m_applied_thrust_left;
  out["THR_L"] = fv;
  fv.dval = m_applied_thrust_right;
  out["THR_R"] = fv;
  fv.dval = m_estop ? 1.0 : 0.0;
  out["ESTOP"] = fv;
  fv.dval = m_rolling_voltage;
  out["VOLT"] = fv;
  fv.dval = m_rc_deadman_active ? 1.0 : 0.0;
  out["RC_DEADMAN"] = fv;

  const string frame = broker_v2::serializeFrame(out);
  const ssize_t sent =
      sendto(m_sockfd, frame.c_str(), frame.size(), 0,
             reinterpret_cast<const sockaddr *>(&dest), sizeof(dest));
  if (sent < 0) {
    m_ack_send_errors++;
    dbg_print("Ack send error: %s\n", strerror(errno));
    return;
  }
  m_acks_sent++;
  m_last_ack_time = MOOSTime();
}

//---------------------------------------------------------
// Procedure: publishTeleopState()
//            While a session is active, TELEOP_ACTIVE and the
//            thrust values are re-published every iterate. The
//            navigator interface treats stale teleop mail as a
//            second deadman layer, so a hung iTeleop fails safe.

void Teleop::publishTeleopState() {
  if (!m_session_active) {
    return;
  }
  Notify("TELEOP_ACTIVE", "true");
  Notify("TELEOP_THRUST_L", m_estop ? 0.0 : m_cmd_thrust_left);
  Notify("TELEOP_THRUST_R", m_estop ? 0.0 : m_cmd_thrust_right);
}

//---------------------------------------------------------
// Procedure: ownerMatches()

bool Teleop::ownerMatches(const broker_v2::FieldMap &fields,
                          const sockaddr_in &src) const {
  if ((src.sin_addr.s_addr != m_owner_addr.sin_addr.s_addr) ||
      (src.sin_port != m_owner_addr.sin_port)) {
    return false;
  }
  string sid = "";
  auto p = fields.find("SID");
  if ((p != fields.end()) && !p->second.is_double) {
    sid = p->second.sval;
  }
  return sid == m_owner_sid;
}

//---------------------------------------------------------
// Procedure: fusedMode()
//            RC mode (physical transmitter) outranks teleop by
//            design in the navigator interface; reflect that in
//            what the GUI is told.

string Teleop::fusedMode() const {
  if (m_rc_mode) {
    return "RC";
  }
  if (m_session_active) {
    return "TELEOP";
  }
  return "AUTO";
}

//---------------------------------------------------------
// Procedure: addrToString()

string Teleop::addrToString(const sockaddr_in &addr) {
  char ip[INET_ADDRSTRLEN];
  memset(ip, 0, sizeof(ip));
  inet_ntop(AF_INET, &addr.sin_addr, ip, sizeof(ip));
  return string(ip) + ":" + intToString(ntohs(addr.sin_port));
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool Teleop::OnConnectToServer() {
  registerVariables();
  return true;
}

//---------------------------------------------------------
// Procedure: Iterate()

bool Teleop::Iterate() {
  AppCastingMOOSApp::Iterate();

  drainInboundFrames();

  // GUI deadman: an active session with no owner frame within the
  // timeout is released and the vehicle returns to autonomy.
  if (m_session_active &&
      ((MOOSTime() - m_last_owner_frame_time) > m_gui_deadman_timeout)) {
    m_deadman_trips++;
    deactivateSession("GUI deadman tripped (no command for " +
                          doubleToString(m_gui_deadman_timeout, 1) + "s)",
                      true);
  }

  publishTeleopState();

  // Idle heartbeat ack so the GUI can track link health even when
  // its own frames (and their per-frame acks) are being lost.
  if (m_session_active && (m_ack_heartbeat_hz > 0) &&
      ((MOOSTime() - m_last_ack_time) > (1.0 / m_ack_heartbeat_hz))) {
    sendAck(m_owner_addr, "OK", m_owner_last_seq);
  }

  AppCastingMOOSApp::PostReport();
  return true;
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool Teleop::OnStartUp() {
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    reportConfigWarning("No config block found for " + GetAppName());
  }
  m_app_name = GetAppName();

  for (STRING_LIST::iterator p = sParams.begin(); p != sParams.end(); ++p) {
    const string orig = *p;
    string line = *p;
    const string param = tolower(biteStringX(line, '='));
    const string value = stripBlankEnds(line);

    bool handled = false;

    if (param == "listen_ip") {
      m_listen_ip = value;
      handled = true;
    } else if (param == "listen_port") {
      m_listen_port = atoi(value.c_str());
      handled = true;
    } else if (param == "gui_deadman_timeout") {
      m_gui_deadman_timeout = atof(value.c_str());
      if (m_gui_deadman_timeout < 0.1) {
        m_gui_deadman_timeout = 0.1;
      }
      handled = true;
    } else if (param == "max_thrust") {
      m_max_thrust = atof(value.c_str());
      handled = true;
    } else if (param == "ack_heartbeat_hz") {
      m_ack_heartbeat_hz = atof(value.c_str());
      handled = true;
    } else if (param == "debug") {
      bool parsed_bool = false;
      if (broker_v2::parseBool(value, parsed_bool)) {
        m_debug = parsed_bool;
      }
      if (m_debug) {
        time_t rawtime;
        struct tm *timeinfo;
        memset(m_fname, 0, m_fname_buff_size);
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        char fmt[m_fname_buff_size];
        memset(fmt, 0, m_fname_buff_size);
        strftime(fmt, m_fname_buff_size, "%F_%H-%M-%S", timeinfo);
        snprintf(m_fname, m_fname_buff_size, "DBG_%s_%s_DATA.dbg",
                 m_app_name.c_str(), fmt);
      }
      handled = true;
    }

    if (!handled) {
      reportUnhandledConfigWarning(orig);
    }
  }

  if (!configureSocket()) {
    return false;
  }

  // Make the idle state explicit on startup so a rebooted vehicle
  // never comes up with a stale TELEOP_ACTIVE from a prior session.
  Notify("TELEOP_ACTIVE", "false");
  Notify("TELEOP_THRUST_L", 0.0);
  Notify("TELEOP_THRUST_R", 0.0);
  Notify("TELEOP_CLIENT", "none");

  registerVariables();
  return true;
}

//---------------------------------------------------------
// Procedure: registerVariables()

void Teleop::registerVariables() {
  AppCastingMOOSApp::RegisterVariables();
  Register("NVGR_THRUST_LEFT_WIRE", 0);
  Register("NVGR_THRUST_RIGHT_WIRE", 0);
  Register("NVGR_RC_MODE", 0);
  Register("NVGR_RC_DEADMAN_ACTIVE", 0);
  Register("NVGR_ROLLING_VOLTAGE", 0);
}

//---------------------------------------------------------
// Procedure: buildReport()

bool Teleop::buildReport() {
  m_msgs << "Teleop Endpoint" << endl;
  m_msgs << "Listen: " << m_listen_ip << ":" << m_listen_port << endl;
  m_msgs << "GUI deadman timeout: " << m_gui_deadman_timeout << "s" << endl;
  m_msgs << endl;

  ACTable actab(2);
  actab << "Session | Value";
  actab.addHeaderLines();
  actab << "Active:" << (m_session_active ? "true" : "false");
  actab << "Owner:" << (m_session_active
                            ? (m_owner_client + " (" + addrToString(m_owner_addr) + ")")
                            : "none");
  actab << "Owner SID:" << (m_session_active ? m_owner_sid : "-");
  actab << "Last SEQ:" << m_owner_last_seq;
  double frame_age = m_session_active ? (MOOSTime() - m_last_owner_frame_time) : 0;
  actab << "Owner Frame Age (sec):" << frame_age;
  actab << "E-STOP Latched:" << (m_estop ? "true" : "false");
  actab << "Cmd Thrust L:" << m_cmd_thrust_left;
  actab << "Cmd Thrust R:" << m_cmd_thrust_right;
  actab << "Fused Mode:" << fusedMode();
  m_msgs << actab.getFormattedString();
  m_msgs << endl << endl;

  ACTable actab2(2);
  actab2 << "Counters | Value";
  actab2.addHeaderLines();
  actab2 << "RX frames:" << m_rx_frames_received;
  actab2 << "RX parse errors:" << m_rx_parse_errors;
  actab2 << "Frames applied:" << m_frames_applied;
  actab2 << "Busy rejects:" << m_busy_rejects;
  actab2 << "Stale SEQ drops:" << m_stale_seq_drops;
  actab2 << "Acks sent:" << m_acks_sent;
  actab2 << "Ack send errors:" << m_ack_send_errors;
  actab2 << "Sessions started:" << m_sessions_started;
  actab2 << "Deadman trips:" << m_deadman_trips;
  m_msgs << actab2.getFormattedString();
  return true;
}
