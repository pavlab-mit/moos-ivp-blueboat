/*************************************************************
      Name: Jeremy Wenger
      Orgn: MIT, Cambridge MA
      File: uRTCMBroker/RTCMBroker.cpp
   Last Ed: 2026-03-17
     Brief:
        NTRIP client that connects to an RTCM caster and
        distributes binary RTCM corrections to vehicle
        communities via MOOS binary messages. Designed to
        run on the shoreside community.
*************************************************************/

#include <iterator>
#include <cstring>
#include <cmath>
#include <cerrno>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>

#include "MBUtils.h"
#include "ACTable.h"
#include "RTCMBroker.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

RTCMBroker::RTCMBroker()
{
  m_debug = false;
  m_debug_stream = nullptr;
  memset(m_fname, '\0', m_fname_buff_size);

  // NTRIP config defaults
  m_ntrip_host = "";
  m_ntrip_port = 2101;
  m_ntrip_mountpoint = "";
  m_ntrip_user = "";
  m_ntrip_password = "";
  m_send_gga = true;
  m_gga_lat = 0.0;
  m_gga_lon = 0.0;
  m_gga_alt = 0.0;
  m_gga_interval = 10.0;
  m_reconnect_interval = 5.0;

  // State
  m_ntrip_sock = -1;
  m_ntrip_connected = false;
  m_thread_running = false;

  m_bytes_received = 0;
  m_msgs_published = 0;
  m_connect_attempts = 0;
  m_last_data_time = 0;
  m_last_gga_time = 0;
  m_connection_status = "Not configured";
}

//---------------------------------------------------------
// Destructor

RTCMBroker::~RTCMBroker()
{
  m_thread_running = false;
  if (m_ntrip_thread.joinable())
    m_ntrip_thread.join();
  ntripDisconnect();
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool RTCMBroker::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

    if (key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }

  return true;
}

//---------------------------------------------------------
// Procedure: dbg_print()

bool RTCMBroker::dbg_print(const char *format, ...)
{
  if (m_debug) {
    va_list args;
    va_start(args, format);
    m_debug_stream = fopen(m_fname, "a");
    if (m_debug_stream != nullptr) {
      vfprintf(m_debug_stream, format, args);
      fclose(m_debug_stream);
      va_end(args);
      return true;
    } else {
      va_end(args);
      reportRunWarning("Debug mode is enabled and file could not be opened");
      return false;
    }
  }
  return false;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool RTCMBroker::OnConnectToServer()
{
  registerVariables();
  return true;
}

//---------------------------------------------------------
// Procedure: Iterate()

bool RTCMBroker::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Publish any queued RTCM data as binary MOOS messages
  {
    std::lock_guard<std::mutex> guard(m_data_mutex);
    for (const auto &chunk : m_rtcm_queue) {
      Notify("RTCM_DATA", (void *)chunk.data(),
             static_cast<unsigned int>(chunk.size()));
      m_msgs_published++;
    }
    m_rtcm_queue.clear();
  }

  // Publish status string for debugging / uXMS
  string status = "connected=" + string(m_ntrip_connected ? "true" : "false") +
                  ",bytes_rx=" + to_string(m_bytes_received) +
                  ",msgs_pub=" + to_string(m_msgs_published) +
                  ",host=" + m_ntrip_host +
                  ",mount=" + m_ntrip_mountpoint;
  Notify("RTCM_STATUS", status);

  AppCastingMOOSApp::PostReport();
  return true;
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool RTCMBroker::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());
  m_app_name = GetAppName();

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++) {
    string orig = *p;
    string line = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if (param == "ntrip_host") {
      m_ntrip_host = value;
      handled = true;
    }
    else if (param == "ntrip_port") {
      m_ntrip_port = stoi(value);
      handled = true;
    }
    else if (param == "ntrip_mountpoint") {
      m_ntrip_mountpoint = value;
      handled = true;
    }
    else if (param == "ntrip_user") {
      m_ntrip_user = value;
      handled = true;
    }
    else if (param == "ntrip_password") {
      m_ntrip_password = value;
      handled = true;
    }
    else if (param == "send_gga") {
      m_send_gga = (tolower(value) == "true");
      handled = true;
    }
    else if (param == "gga_lat") {
      m_gga_lat = stod(value);
      handled = true;
    }
    else if (param == "gga_lon") {
      m_gga_lon = stod(value);
      handled = true;
    }
    else if (param == "gga_alt") {
      m_gga_alt = stod(value);
      handled = true;
    }
    else if (param == "gga_interval") {
      m_gga_interval = stod(value);
      handled = true;
    }
    else if (param == "reconnect_interval") {
      m_reconnect_interval = stod(value);
      handled = true;
    }
    else if (param == "debug") {
      m_debug = (tolower(value) == "true");
      if (m_debug) {
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

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  // Try to get GGA position from LatOrigin/LongOrigin if not explicitly set
  if (m_gga_lat == 0.0 && m_gga_lon == 0.0) {
    double lat_origin = 0.0, lon_origin = 0.0;
    if (m_MissionReader.GetValue("LatOrigin", lat_origin) &&
        m_MissionReader.GetValue("LongOrigin", lon_origin)) {
      m_gga_lat = lat_origin;
      m_gga_lon = lon_origin;
      dbg_print("Using LatOrigin/LongOrigin for GGA: %.6f, %.6f\n",
                m_gga_lat, m_gga_lon);
    }
  }

  // Validate config
  if (m_ntrip_host.empty()) {
    reportConfigWarning("ntrip_host is not configured");
    m_connection_status = "Not configured - no host";
    registerVariables();
    return true;
  }
  if (m_ntrip_mountpoint.empty()) {
    reportConfigWarning("ntrip_mountpoint is not configured");
    m_connection_status = "Not configured - no mountpoint";
    registerVariables();
    return true;
  }
  if (m_send_gga && m_gga_lat == 0.0 && m_gga_lon == 0.0) {
    reportConfigWarning("send_gga is true but no GGA position configured "
                        "and no LatOrigin/LongOrigin in mission file");
  }

  // Launch NTRIP thread
  m_thread_running = true;
  m_ntrip_thread = std::thread(&RTCMBroker::ntripThreadFunc, this);
  m_connection_status = "Connecting...";

  registerVariables();
  return true;
}

//---------------------------------------------------------
// Procedure: registerVariables()

void RTCMBroker::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
}

//---------------------------------------------------------
// Procedure: ntripConnect()

bool RTCMBroker::ntripConnect()
{
  m_connect_attempts++;

  // Resolve hostname
  struct addrinfo hints, *result;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  string port_str = to_string(m_ntrip_port);
  int rv = getaddrinfo(m_ntrip_host.c_str(), port_str.c_str(), &hints, &result);
  if (rv != 0) {
    m_connection_status = "DNS failed: " + string(gai_strerror(rv));
    dbg_print("getaddrinfo failed: %s\n", gai_strerror(rv));
    return false;
  }

  m_ntrip_sock = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
  if (m_ntrip_sock < 0) {
    m_connection_status = "Socket creation failed";
    freeaddrinfo(result);
    return false;
  }

  // Set connect timeout via non-blocking + select
  fcntl(m_ntrip_sock, F_SETFL, O_NONBLOCK);

  int conn_rv = connect(m_ntrip_sock, result->ai_addr, result->ai_addrlen);
  freeaddrinfo(result);

  if (conn_rv < 0 && errno != EINPROGRESS) {
    m_connection_status = "Connect failed: " + string(strerror(errno));
    close(m_ntrip_sock);
    m_ntrip_sock = -1;
    return false;
  }

  // Wait for connection with 10s timeout
  fd_set wfds;
  FD_ZERO(&wfds);
  FD_SET(m_ntrip_sock, &wfds);
  struct timeval tv;
  tv.tv_sec = 10;
  tv.tv_usec = 0;

  rv = select(m_ntrip_sock + 1, NULL, &wfds, NULL, &tv);
  if (rv <= 0) {
    m_connection_status = "Connect timeout";
    close(m_ntrip_sock);
    m_ntrip_sock = -1;
    return false;
  }

  // Check for connect error
  int so_error;
  socklen_t len = sizeof(so_error);
  getsockopt(m_ntrip_sock, SOL_SOCKET, SO_ERROR, &so_error, &len);
  if (so_error != 0) {
    m_connection_status = "Connect error: " + string(strerror(so_error));
    close(m_ntrip_sock);
    m_ntrip_sock = -1;
    return false;
  }

  // Set back to blocking with recv timeout
  int flags = fcntl(m_ntrip_sock, F_GETFL);
  fcntl(m_ntrip_sock, F_SETFL, flags & ~O_NONBLOCK);

  struct timeval recv_tv;
  recv_tv.tv_sec = 5;
  recv_tv.tv_usec = 0;
  setsockopt(m_ntrip_sock, SOL_SOCKET, SO_RCVTIMEO, &recv_tv, sizeof(recv_tv));

  // Build NTRIP HTTP request
  // Base64 encode credentials
  string credentials = m_ntrip_user + ":" + m_ntrip_password;
  static const char b64chars[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  string auth_b64;
  int val = 0, valb = -6;
  for (unsigned char c : credentials) {
    val = (val << 8) + c;
    valb += 8;
    while (valb >= 0) {
      auth_b64.push_back(b64chars[(val >> valb) & 0x3F]);
      valb -= 6;
    }
  }
  if (valb > -6)
    auth_b64.push_back(b64chars[((val << 8) >> (valb + 8)) & 0x3F]);
  while (auth_b64.size() % 4)
    auth_b64.push_back('=');

  string request = "GET /" + m_ntrip_mountpoint + " HTTP/1.1\r\n"
                   "Host: " + m_ntrip_host + "\r\n"
                   "Ntrip-Version: Ntrip/2.0\r\n"
                   "User-Agent: uRTCMBroker/1.0\r\n"
                   "Authorization: Basic " + auth_b64 + "\r\n";

  if (m_send_gga && (m_gga_lat != 0.0 || m_gga_lon != 0.0)) {
    string gga = buildGGA();
    if (!gga.empty()) {
      request += "Ntrip-GGA: " + gga;
    }
  }

  request += "\r\n";

  // Send request
  ssize_t sent = send(m_ntrip_sock, request.c_str(), request.size(), 0);
  if (sent < 0) {
    m_connection_status = "Send request failed";
    close(m_ntrip_sock);
    m_ntrip_sock = -1;
    return false;
  }

  // Read HTTP response
  char resp_buf[4096];
  ssize_t resp_len = recv(m_ntrip_sock, resp_buf, sizeof(resp_buf) - 1, 0);
  if (resp_len <= 0) {
    m_connection_status = "No response from caster";
    close(m_ntrip_sock);
    m_ntrip_sock = -1;
    return false;
  }
  resp_buf[resp_len] = '\0';

  string response(resp_buf);
  string first_line = response.substr(0, response.find("\r\n"));

  if (first_line.find("200") != string::npos) {
    m_ntrip_connected = true;
    m_connection_status = "Connected to " + m_ntrip_host + "/" + m_ntrip_mountpoint;
    m_last_gga_time = MOOSTime();
    dbg_print("NTRIP connected: %s\n", first_line.c_str());

    // The response may contain RTCM data after the headers — check for it
    // Find end of HTTP headers
    size_t header_end = response.find("\r\n\r\n");
    if (header_end != string::npos && (header_end + 4) < (size_t)resp_len) {
      size_t data_offset = header_end + 4;
      size_t data_len = resp_len - data_offset;
      if (data_len > 0) {
        std::vector<unsigned char> initial_data(
            resp_buf + data_offset, resp_buf + data_offset + data_len);
        std::lock_guard<std::mutex> guard(m_data_mutex);
        m_rtcm_queue.push_back(std::move(initial_data));
        m_bytes_received += data_len;
        m_last_data_time = MOOSTime();
      }
    }

    return true;
  } else {
    m_connection_status = "Rejected: " + first_line;
    dbg_print("NTRIP rejected: %s\n", first_line.c_str());
    close(m_ntrip_sock);
    m_ntrip_sock = -1;
    return false;
  }
}

//---------------------------------------------------------
// Procedure: ntripDisconnect()

void RTCMBroker::ntripDisconnect()
{
  m_ntrip_connected = false;
  if (m_ntrip_sock >= 0) {
    close(m_ntrip_sock);
    m_ntrip_sock = -1;
  }
}

//---------------------------------------------------------
// Procedure: ntripThreadFunc()

void RTCMBroker::ntripThreadFunc()
{
  while (m_thread_running) {
    // Connect if not connected
    if (!m_ntrip_connected) {
      if (!ntripConnect()) {
        dbg_print("NTRIP connect failed, retrying in %.0fs\n", m_reconnect_interval);
        // Sleep in small increments so we can exit quickly
        double sleep_end = MOOSTime() + m_reconnect_interval;
        while (m_thread_running && MOOSTime() < sleep_end) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        continue;
      }
    }

    // Read RTCM data
    unsigned char buf[4096];
    ssize_t n = recv(m_ntrip_sock, buf, sizeof(buf), 0);

    if (n > 0) {
      std::vector<unsigned char> chunk(buf, buf + n);
      {
        std::lock_guard<std::mutex> guard(m_data_mutex);
        m_rtcm_queue.push_back(std::move(chunk));
      }
      m_bytes_received += static_cast<unsigned long>(n);
      m_last_data_time = MOOSTime();

    } else if (n == 0) {
      // Connection closed by server
      m_connection_status = "Disconnected by caster";
      ntripDisconnect();
      continue;

    } else {
      // n < 0
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // Recv timeout — this is normal, just no data yet
      } else {
        m_connection_status = "Recv error: " + string(strerror(errno));
        ntripDisconnect();
        continue;
      }
    }

    // Periodically send GGA to caster for VRS
    if (m_send_gga && m_ntrip_connected) {
      double now = MOOSTime();
      if (now - m_last_gga_time >= m_gga_interval) {
        string gga = buildGGA();
        if (!gga.empty()) {
          ssize_t sent = send(m_ntrip_sock, gga.c_str(), gga.size(), 0);
          if (sent < 0) {
            m_connection_status = "GGA send failed, reconnecting";
            ntripDisconnect();
            continue;
          }
          dbg_print("Sent GGA: %s\n", gga.c_str());
        }
        m_last_gga_time = now;
      }
    }
  }
}

//---------------------------------------------------------
// Procedure: buildGGA()
//   Synthesize a $GPGGA sentence from configured position

std::string RTCMBroker::buildGGA()
{
  if (m_gga_lat == 0.0 && m_gga_lon == 0.0)
    return "";

  // UTC time
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  struct tm utc_tm;
  gmtime_r(&time_t_now, &utc_tm);

  char utc_str[16];
  snprintf(utc_str, sizeof(utc_str), "%02d%02d%02d.00",
           utc_tm.tm_hour, utc_tm.tm_min, utc_tm.tm_sec);

  // Convert decimal degrees to NMEA DDmm.mmmmmmm format
  double lat_abs = fabs(m_gga_lat);
  int lat_deg = static_cast<int>(lat_abs);
  double lat_min = (lat_abs - lat_deg) * 60.0;
  char lat_str[20];
  snprintf(lat_str, sizeof(lat_str), "%02d%010.7f", lat_deg, lat_min);
  char lat_ns = (m_gga_lat >= 0) ? 'N' : 'S';

  double lon_abs = fabs(m_gga_lon);
  int lon_deg = static_cast<int>(lon_abs);
  double lon_min = (lon_abs - lon_deg) * 60.0;
  char lon_str[20];
  snprintf(lon_str, sizeof(lon_str), "%03d%010.7f", lon_deg, lon_min);
  char lon_ew = (m_gga_lon >= 0) ? 'E' : 'W';

  // Fix quality = 1 (GPS fix), 8 sats, 1.0 HDOP
  char body[256];
  snprintf(body, sizeof(body),
           "GPGGA,%s,%s,%c,%s,%c,1,08,1.0,%.3f,M,0.000,M,,",
           utc_str, lat_str, lat_ns, lon_str, lon_ew, m_gga_alt);

  // NMEA checksum: XOR of all chars between $ and *
  unsigned char cksum = 0;
  for (const char *cp = body; *cp; cp++)
    cksum ^= static_cast<unsigned char>(*cp);

  char sentence[300];
  snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", body, cksum);

  return string(sentence);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool RTCMBroker::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "uRTCMBroker - NTRIP RTCM Corrections" << endl;
  m_msgs << "============================================" << endl;
  m_msgs << endl;

  // Connection info
  ACTable conntab(2);
  conntab << "Connection | Value";
  conntab.addHeaderLines();
  conntab << "Status" << m_connection_status;
  conntab << "Host" << (m_ntrip_host.empty() ? "N/A" : m_ntrip_host);
  conntab << "Port" << to_string(m_ntrip_port);
  conntab << "Mountpoint" << (m_ntrip_mountpoint.empty() ? "N/A" : m_ntrip_mountpoint);
  conntab << "Connect Attempts" << to_string(m_connect_attempts);
  m_msgs << conntab.getFormattedString() << endl << endl;

  // GGA config
  ACTable ggatab(2);
  ggatab << "GGA Config | Value";
  ggatab.addHeaderLines();
  ggatab << "Send GGA" << (m_send_gga ? "YES" : "NO");
  ggatab << "GGA Lat" << doubleToString(m_gga_lat, 6);
  ggatab << "GGA Lon" << doubleToString(m_gga_lon, 6);
  ggatab << "GGA Alt" << doubleToString(m_gga_alt, 1);
  ggatab << "GGA Interval" << doubleToString(m_gga_interval, 0) + "s";
  m_msgs << ggatab.getFormattedString() << endl << endl;

  // Stats
  ACTable stattab(2);
  stattab << "Stats | Value";
  stattab.addHeaderLines();
  stattab << "Bytes Received" << to_string(m_bytes_received);
  stattab << "Msgs Published" << to_string(m_msgs_published);

  double data_age = (m_last_data_time > 0) ? MOOSTime() - m_last_data_time : -1;
  if (data_age >= 0)
    stattab << "Last Data Age" << doubleToString(data_age, 1) + "s";
  else
    stattab << "Last Data Age" << "N/A";

  m_msgs << stattab.getFormattedString() << endl;

  return true;
}
