#include "FrontSeatBroker.h"

#include "ACTable.h"
#include "MBUtils.h"

#include <cstdio>
#include <cstring>
#include <ctime>
#include <algorithm>

using namespace std;

FrontSeatBroker::FrontSeatBroker()
    : m_debug(false),
      m_debug_stream(nullptr),
      m_app_name(""),
      m_listen_ip("0.0.0.0"),
      m_listen_port(9201),
      m_tx_ip("127.0.0.1"),
      m_tx_port(9200),
      m_rx_thread_running(false),
      m_rx_frames_received(0),
      m_rx_frames_parsed(0),
      m_rx_parse_errors(0),
      m_tx_frames_sent(0),
      m_tx_send_errors(0),
      m_tx_var_count(0),
      m_rx_var_count(0),
      m_remote_tx_count(0),
      m_remote_rx_count(0),
      m_last_backseat_startup_id(""),
      m_base_tx_var_count(0),
      m_base_rx_var_count(0),
      m_base_remote_tx_count(0),
      m_base_remote_rx_count(0),
      m_drop_baseline_valid(false),
      m_post_drop_deltas_to_moos(false),
      m_outbound_drop_var("FS_BROKER_OUTBOUND_DROPPED"),
      m_inbound_drop_var("FS_BROKER_INBOUND_DROPPED"),
      m_last_posted_outbound_drop(-1),
      m_last_posted_inbound_drop(-1) {
  memset(m_fname, 0, m_fname_buff_size);
}

FrontSeatBroker::~FrontSeatBroker() { stopRxThread(); }

bool FrontSeatBroker::OnNewMail(MOOSMSG_LIST &NewMail) {
  AppCastingMOOSApp::OnNewMail(NewMail);

  for (MOOSMSG_LIST::iterator p = NewMail.begin(); p != NewMail.end(); ++p) {
    CMOOSMsg &msg = *p;
    const string key = msg.GetKey();

    if (key == "APPCAST_REQ") {
      continue;
    }
    if (m_tx_vars.find(key) == m_tx_vars.end()) {
      continue;
    }

    broker_v2::FieldValue fv;
    if (msg.IsDouble()) {
      fv.is_double = true;
      fv.dval = msg.GetDouble();
      m_latest_tx_fields[key] = fv;
      continue;
    }
    if (msg.IsString()) {
      fv.is_double = false;
      fv.sval = msg.GetString();
      m_latest_tx_fields[key] = fv;
      continue;
    }
  }

  return true;
}

bool FrontSeatBroker::dbg_print(const char *format, ...) {
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

bool FrontSeatBroker::configureSocketEndpoints() {
  m_rx_client.reset(new SocketClient(m_listen_ip, m_listen_port));
  const string rx_error = m_rx_client->open();
  if (!rx_error.empty()) {
    reportConfigWarning("Failed to open receive socket: " + rx_error);
    return false;
  }

  m_tx_broker.reset(new SocketBroker(m_tx_ip, m_tx_port));
  const string tx_error = m_tx_broker->open();
  if (!tx_error.empty()) {
    reportConfigWarning("Failed to open transmit socket: " + tx_error);
    return false;
  }

  m_rx_thread_running = true;
  m_rx_thread = thread(&FrontSeatBroker::receiveLoop, this);
  return true;
}

void FrontSeatBroker::stopRxThread() {
  if (m_rx_thread_running) {
    m_rx_thread_running = false;
    if (m_rx_client) {
      m_rx_client->close();
    }
  }
  if (m_rx_thread.joinable()) {
    m_rx_thread.join();
  }
}

void FrontSeatBroker::receiveLoop() {
  while (m_rx_thread_running) {
    string sentence;
    const int got = m_rx_client->readSentence(sentence, "<", ">");
    if (!m_rx_thread_running) {
      break;
    }
    if (got < 0 || sentence.empty()) {
      continue;
    }

    m_rx_frames_received++;

    broker_v2::FieldMap fields;
    string error;
    if (!broker_v2::parseFrame(sentence, fields, error)) {
      m_rx_parse_errors++;
      dbg_print("RX parse error: %s, raw=[%s]\n", error.c_str(), sentence.c_str());
      continue;
    }

    m_rx_frames_parsed++;
    lock_guard<mutex> guard(m_rx_mutex);
    m_rx_frames.push_back(fields);
    // No queue limit - never discard frames. Caller must drain via Iterate().
    // Memory bounded by kernel buffer (1MB) + processing rate.
  }
}

void FrontSeatBroker::drainInboundFrames() {
  deque<broker_v2::FieldMap> local_frames;
  {
    lock_guard<mutex> guard(m_rx_mutex);
    local_frames.swap(m_rx_frames);
  }

  for (const auto &frame : local_frames) {
    unsigned int var_count = 0;
    for (const auto &kv : frame) {
      // Skip all broker count fields
      if (kv.first == "BS_BROKER_TX_COUNT") {
        if (kv.second.is_double)
          m_remote_tx_count = static_cast<unsigned int>(kv.second.dval);
        continue;
      }
      if (kv.first == "BS_BROKER_RX_COUNT") {
        if (kv.second.is_double)
          m_remote_rx_count = static_cast<unsigned int>(kv.second.dval);
        continue;
      }
      if (kv.first == "BS_BROKER_STARTUP_ID") {
        if (!kv.second.is_double) {
          const string startup_id = kv.second.sval;
          if (!startup_id.empty() && startup_id != m_last_backseat_startup_id) {
            m_last_backseat_startup_id = startup_id;
            resetDropBaselines();
          }
        }
        continue;
      }

      var_count++;
      const string mapped_name = broker_v2::remapName(kv.first, m_in_map);
      if (kv.second.is_double)
        Notify(mapped_name, kv.second.dval);
      else
        Notify(mapped_name, kv.second.sval);
    }
    m_rx_var_count += var_count;
  }
}

void FrontSeatBroker::resetDropBaselines() {
  m_base_tx_var_count = m_tx_var_count;
  m_base_rx_var_count = m_rx_var_count;
  m_base_remote_tx_count = m_remote_tx_count;
  m_base_remote_rx_count = m_remote_rx_count;
  m_drop_baseline_valid = true;
  m_last_posted_outbound_drop = -1;
  m_last_posted_inbound_drop = -1;
}

void FrontSeatBroker::sendOutboundFrame() {
  if (!m_tx_broker) {
    return;
  }

  broker_v2::FieldMap outbound;
  for (const auto &name : m_tx_vars) {
    auto p = m_latest_tx_fields.find(name);
    if (p != m_latest_tx_fields.end()) {
      outbound[name] = p->second;
    }
  }

  if (outbound.empty()) {
    return;
  }

  // Count variables being sent (before adding meta fields)
  m_tx_var_count += outbound.size();

  // Inject broker counts for packet drop detection
  broker_v2::FieldValue tx_count_fv;
  tx_count_fv.is_double = true;
  tx_count_fv.dval = static_cast<double>(m_tx_var_count);
  outbound["FS_BROKER_TX_COUNT"] = tx_count_fv;

  broker_v2::FieldValue rx_count_fv;
  rx_count_fv.is_double = true;
  rx_count_fv.dval = static_cast<double>(m_rx_var_count);
  outbound["FS_BROKER_RX_COUNT"] = rx_count_fv;

  const string frame = broker_v2::serializeFrame(outbound);
  const string error = m_tx_broker->send(frame);
  if (!error.empty()) {
    m_tx_send_errors++;
    dbg_print("TX send error: %s\n", error.c_str());
    return;
  }
  m_tx_frames_sent++;
}

bool FrontSeatBroker::OnConnectToServer() {
  registerVariables();
  return true;
}

bool FrontSeatBroker::Iterate() {
  AppCastingMOOSApp::Iterate();
  drainInboundFrames();
  sendOutboundFrame();
  postDropDeltasToMoos();
  AppCastingMOOSApp::PostReport();
  return true;
}

bool FrontSeatBroker::OnStartUp() {
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
    } else if (param == "tx_ip") {
      m_tx_ip = value;
      handled = true;
    } else if (param == "tx_port") {
      m_tx_port = atoi(value.c_str());
      handled = true;
    } else if (param == "tx_vars") {
      const set<string> parsed = broker_v2::parseNameSet(value);
      m_tx_vars.insert(parsed.begin(), parsed.end());
      handled = true;
    } else if (param == "in_map") {
      const broker_v2::NameMap parsed = broker_v2::parseNameMap(value);
      m_in_map.insert(parsed.begin(), parsed.end());
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
        snprintf(m_fname, m_fname_buff_size, "DBG_%s_%s_DATA.dbg", m_app_name.c_str(),
                 fmt);
      }
      handled = true;
    } else if (param == "post_drop_deltas_to_moos") {
      bool parsed_bool = false;
      if (broker_v2::parseBool(value, parsed_bool)) {
        m_post_drop_deltas_to_moos = parsed_bool;
      }
      handled = true;
    } else if (param == "outbound_drop_var") {
      if (!value.empty()) {
        m_outbound_drop_var = value;
      }
      handled = true;
    } else if (param == "inbound_drop_var") {
      if (!value.empty()) {
        m_inbound_drop_var = value;
      }
      handled = true;
    }

    if (!handled) {
      reportUnhandledConfigWarning(orig);
    }
  }

  if (!configureSocketEndpoints()) {
    return false;
  }

  registerVariables();
  return true;
}

void FrontSeatBroker::postDropDeltasToMoos() {
  if (!m_post_drop_deltas_to_moos || !m_drop_baseline_valid) {
    return;
  }

  const int tx_delta = static_cast<int>(m_tx_var_count) - static_cast<int>(m_base_tx_var_count);
  const int remote_rx_delta =
      static_cast<int>(m_remote_rx_count) - static_cast<int>(m_base_remote_rx_count);
  const int remote_tx_delta =
      static_cast<int>(m_remote_tx_count) - static_cast<int>(m_base_remote_tx_count);
  const int rx_delta = static_cast<int>(m_rx_var_count) - static_cast<int>(m_base_rx_var_count);

  const int outbound_dropped = max(0, tx_delta - remote_rx_delta);
  const int inbound_dropped = max(0, remote_tx_delta - rx_delta);

  if (outbound_dropped != m_last_posted_outbound_drop) {
    Notify(m_outbound_drop_var, static_cast<double>(outbound_dropped));
    m_last_posted_outbound_drop = outbound_dropped;
  }
  if (inbound_dropped != m_last_posted_inbound_drop) {
    Notify(m_inbound_drop_var, static_cast<double>(inbound_dropped));
    m_last_posted_inbound_drop = inbound_dropped;
  }
}

void FrontSeatBroker::registerVariables() {
  AppCastingMOOSApp::RegisterVariables();
  for (const auto &name : m_tx_vars) {
    Register(name, 0);
  }
}

bool FrontSeatBroker::buildReport() {
  m_msgs << "FrontSeat Broker v2" << endl;
  m_msgs << "Listen: " << m_listen_ip << ":" << m_listen_port << endl;
  m_msgs << "Transmit: " << m_tx_ip << ":" << m_tx_port << endl;
  m_msgs << "tx_vars count: " << m_tx_vars.size() << endl;
  m_msgs << "in_map count: " << m_in_map.size() << endl;
  m_msgs << endl;

  m_msgs << "Frame Statistics:" << endl;
  m_msgs << "  RX frames: " << m_rx_frames_received
         << " (parsed: " << m_rx_frames_parsed
         << ", errors: " << m_rx_parse_errors << ")" << endl;
  m_msgs << "  TX frames: " << m_tx_frames_sent
         << " (errors: " << m_tx_send_errors << ")" << endl;
  m_msgs << endl;

  m_msgs << "Variable Counts (Packet Drop Detection):" << endl;
  const int lifetime_outbound_dropped =
      static_cast<int>(m_tx_var_count) - static_cast<int>(m_remote_rx_count);
  const int lifetime_inbound_dropped =
      static_cast<int>(m_remote_tx_count) - static_cast<int>(m_rx_var_count);
  m_msgs << "  Lifetime outbound dropped=" << lifetime_outbound_dropped
         << " (sent=" << m_tx_var_count
         << ", remote_rcvd=" << m_remote_rx_count << ")" << endl;
  m_msgs << "  Lifetime inbound  dropped=" << lifetime_inbound_dropped
         << " (remote_sent=" << m_remote_tx_count
         << ", rcvd=" << m_rx_var_count << ")" << endl;

  if (m_drop_baseline_valid) {
    const int tx_delta =
        static_cast<int>(m_tx_var_count) - static_cast<int>(m_base_tx_var_count);
    const int remote_rx_delta =
        static_cast<int>(m_remote_rx_count) - static_cast<int>(m_base_remote_rx_count);
    const int remote_tx_delta =
        static_cast<int>(m_remote_tx_count) - static_cast<int>(m_base_remote_tx_count);
    const int rx_delta =
        static_cast<int>(m_rx_var_count) - static_cast<int>(m_base_rx_var_count);
    const int session_outbound_dropped = max(0, tx_delta - remote_rx_delta);
    const int session_inbound_dropped = max(0, remote_tx_delta - rx_delta);
    m_msgs << "  Session baseline id=" << m_last_backseat_startup_id << endl;
    m_msgs << "  Session outbound dropped=" << session_outbound_dropped
           << " (sent_delta=" << tx_delta
           << ", remote_rcvd_delta=" << remote_rx_delta << ")" << endl;
    m_msgs << "  Session inbound  dropped=" << session_inbound_dropped
           << " (remote_sent_delta=" << remote_tx_delta
           << ", rcvd_delta=" << rx_delta << ")" << endl;
  } else {
    m_msgs << "  Session baseline id=<waiting for BS_BROKER_STARTUP_ID>" << endl;
  }
  m_msgs << "  post_drop_deltas_to_moos=" << (m_post_drop_deltas_to_moos ? "true" : "false");
  if (m_post_drop_deltas_to_moos) {
    m_msgs << " (" << m_outbound_drop_var << ", " << m_inbound_drop_var << ")";
  }
  m_msgs << endl;
  m_msgs << endl;

  ACTable actab(2);
  actab << "in_map_source | in_map_dest";
  actab.addHeaderLines();
  for (const auto &kv : m_in_map) {
    actab << kv.first << kv.second;
  }
  m_msgs << actab.getFormattedString();
  return true;
}
