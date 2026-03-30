
/*************************************************************
      Name: Jeremy Wenger
      Orgn: MIT, Cambridge MA
      File: iUnicoreGPS/UnicoreGPS.cpp
   Last Ed: 2026-03-04
     Brief:
        MOOS interface for Unicore UM982 dual-antenna GNSS
        receiver. Reads position, velocity, heading, and DOP
        data via serial and publishes to MOOSDB with
        configurable publish groups.
*************************************************************/

#include "MBUtils.h"
#include "ACTable.h"
#include "UnicoreGPS.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

UnicoreGPS::UnicoreGPS()
{
  m_debug = false;
  m_debug_stream = nullptr;
  memset(m_fname, '\0', m_fname_buff_size);

  m_port_name = "/dev/ttyUSB0";
  m_baud_rate = 460800;
  m_nav_suffix = "";
  m_enable_geodesic = true;
  m_geodesy_initialized = false;

  m_publish_position = true;
  m_publish_velocity = true;
  m_publish_heading = true;
  m_publish_dops = true;
  m_publish_status = true;
  m_publish_time = false;
  m_publish_state = true;
  m_publish_dto = false;

  m_parser = nullptr;
  m_thread_running = false;
  m_new_data_available = false;
  m_data_stale = false;
  m_last_gps_data_moos_time = 0.0;
  m_stale_data_timeout_sec = 2.0;
  m_stale_warn_repeat_sec = 5.0;
  m_last_stale_warn_moos_time = 0.0;
  m_quality_degraded = false;
  m_quality_warn_repeat_sec = 10.0;
  m_last_quality_warn_moos_time = 0.0;

  m_nav_x = 0.0;
  m_nav_y = 0.0;
  m_nav_lat = 0.0;
  m_nav_lon = 0.0;
  m_nav_alt = 0.0;
  m_h_acc = 0.0;
  m_v_acc = 0.0;
  m_fix_type = "NONE";
  m_num_sats = 0;
  m_gps_lock = false;

  m_nav_speed = 0.0;
  m_vel_n = 0.0;
  m_vel_e = 0.0;
  m_vel_d = 0.0;
  m_track_over_ground = 0.0;

  m_gps_heading = 0.0;
  m_gps_heading_acc = 0.0;
  m_gps_baseline = 0.0;
  m_gps_pitch = 0.0;
  m_carr_soln = "NONE";
  m_heading_valid = false;

  m_hdop = 0.0f;
  m_vdop = 0.0f;
  m_pdop = 0.0f;
  m_gdop = 0.0f;

  m_hdg_hdop = 0.0f;
  m_hdg_vdop = 0.0f;
  m_hdg_pdop = 0.0f;
  m_hdg_gdop = 0.0f;

  m_corr_healthy = false;
  m_rtk_calc_status = 0;

  m_corr_age = 0.0f;
  m_sol_age = 0.0f;
  m_base_id = "";

  m_rtcm_bytes_written = 0;
  m_rtcm_msgs_received = 0;

  m_epoch_time = 0.0;

  m_bestnava_count = 0;
  m_uniheadinga_count = 0;
  m_bestvela_count = 0;
  m_psrdopa_count = 0;
  m_stadopa_count = 0;
  m_stadopha_count = 0;
  m_rtkstatusa_count = 0;
}

//---------------------------------------------------------
// Destructor

UnicoreGPS::~UnicoreGPS()
{
  m_thread_running = false;
  if (m_reader_thread.joinable())
    m_reader_thread.join();

  if (m_parser != nullptr) {
    m_parser->disconnect();
    delete m_parser;
    m_parser = nullptr;
  }
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool UnicoreGPS::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

    if (key == "RTCM_DATA") {
      if (msg.IsBinary()) {
        std::vector<unsigned char> rtcm_data;
        msg.GetBinaryData(rtcm_data);
        if (!rtcm_data.empty() && m_parser && m_parser->isConnected()) {
          std::lock_guard<std::mutex> guard(m_data_mutex);
          if (m_parser->writeRawData(rtcm_data.data(), rtcm_data.size())) {
            m_rtcm_bytes_written += rtcm_data.size();
            m_rtcm_msgs_received++;
          } else {
            reportRunWarning("Failed to write RTCM data to serial port");
          }
        }
      } else {
        reportRunWarning("RTCM_DATA received but not binary format");
      }
    }
    else if (key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }

  return true;
}

//---------------------------------------------------------
// Procedure: dbg_print()

bool UnicoreGPS::dbg_print(const char *format, ...)
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

bool UnicoreGPS::OnConnectToServer()
{
  registerVariables();
  return true;
}

//---------------------------------------------------------
// Procedure: Iterate()

bool UnicoreGPS::Iterate()
{
  AppCastingMOOSApp::Iterate();

  std::lock_guard<std::mutex> guard(m_data_mutex);
  const double now = MOOSTime();

  if (m_last_gps_data_moos_time > 0.0) {
    const double data_age = now - m_last_gps_data_moos_time;
    const bool currently_stale = (data_age > m_stale_data_timeout_sec);

    if (currently_stale && !m_data_stale) {
      reportRunWarning("No new GPS serial data for " +
                       doubleToString(data_age, 2) + "s (timeout=" +
                       doubleToString(m_stale_data_timeout_sec, 2) + "s)");
      dbg_print("WARN: No new GPS serial data for %.2fs (timeout=%.2fs)\n",
                data_age, m_stale_data_timeout_sec);
      m_data_stale = true;
      m_last_stale_warn_moos_time = now;
    } else if (currently_stale &&
               (now - m_last_stale_warn_moos_time) >= m_stale_warn_repeat_sec) {
      reportRunWarning("Still no GPS serial data (age=" +
                       doubleToString(data_age, 2) + "s)");
      dbg_print("WARN: Still no GPS serial data (age=%.2fs)\n", data_age);
      m_last_stale_warn_moos_time = now;
    } else if (!currently_stale && m_data_stale) {
      reportEvent("GPS serial data stream recovered");
      dbg_print("INFO: GPS serial data stream recovered\n");
      m_data_stale = false;
    }
  }

  // Debug-only quality watchdog: logs transitions and periodic reminders when
  // position/heading quality degrades during mission.
  if (m_last_gps_data_moos_time > 0.0) {
    const bool position_quality_good = m_gps_lock && (m_fix_type != "NONE");
    const bool heading_quality_good = m_heading_valid;
    const bool quality_degraded = !(position_quality_good && heading_quality_good);

    if (quality_degraded && !m_quality_degraded) {
      dbg_print(
          "WARN: GPS quality degraded: pos_ok=%s fix=%s lock=%s sats=%u "
          "h_acc=%.3f v_acc=%.3f | hdg_ok=%s carr_soln=%s hdg_acc=%.3f "
          "corr_healthy=%s rtk_calc=%u\n",
          position_quality_good ? "true" : "false",
          m_fix_type.c_str(),
          m_gps_lock ? "true" : "false",
          static_cast<unsigned>(m_num_sats),
          m_h_acc, m_v_acc,
          heading_quality_good ? "true" : "false",
          m_carr_soln.c_str(),
          m_gps_heading_acc,
          m_corr_healthy ? "true" : "false",
          static_cast<unsigned>(m_rtk_calc_status));
      m_quality_degraded = true;
      m_last_quality_warn_moos_time = now;
    } else if (quality_degraded &&
               (now - m_last_quality_warn_moos_time) >= m_quality_warn_repeat_sec) {
      dbg_print(
          "WARN: GPS quality still degraded: pos_ok=%s fix=%s lock=%s sats=%u "
          "| hdg_ok=%s carr_soln=%s hdg_acc=%.3f corr_healthy=%s rtk_calc=%u\n",
          position_quality_good ? "true" : "false",
          m_fix_type.c_str(),
          m_gps_lock ? "true" : "false",
          static_cast<unsigned>(m_num_sats),
          heading_quality_good ? "true" : "false",
          m_carr_soln.c_str(),
          m_gps_heading_acc,
          m_corr_healthy ? "true" : "false",
          static_cast<unsigned>(m_rtk_calc_status));
      m_last_quality_warn_moos_time = now;
    } else if (!quality_degraded && m_quality_degraded) {
      dbg_print(
          "INFO: GPS quality recovered: pos_ok=true fix=%s lock=%s sats=%u "
          "| hdg_ok=true carr_soln=%s hdg_acc=%.3f corr_healthy=%s rtk_calc=%u\n",
          m_fix_type.c_str(),
          m_gps_lock ? "true" : "false",
          static_cast<unsigned>(m_num_sats),
          m_carr_soln.c_str(),
          m_gps_heading_acc,
          m_corr_healthy ? "true" : "false",
          static_cast<unsigned>(m_rtk_calc_status));
      m_quality_degraded = false;
    }
  }

  if (m_new_data_available) {

    // GROUP: position
    if (m_publish_position) {
      if (m_enable_geodesic && m_geodesy_initialized) {
        Notify("NAV_X" + m_nav_suffix, m_nav_x);
        Notify("NAV_Y" + m_nav_suffix, m_nav_y);
      }
      Notify("NAV_X_GPS", m_nav_x);
      Notify("NAV_Y_GPS", m_nav_y);
      Notify("NAV_LAT", m_nav_lat);
      Notify("NAV_LONG", m_nav_lon);
    }

    // GROUP: velocity
    if (m_publish_velocity) {
      Notify("NAV_SPEED", m_nav_speed);
      Notify("VEL_N", m_vel_n);
      Notify("VEL_E", m_vel_e);
      Notify("VEL_D", m_vel_d);
      Notify("NAV_COG", m_track_over_ground);
    }

    // GROUP: heading (dual-antenna)
    if (m_publish_heading) {
      Notify("GPS_HEADING", m_gps_heading);
      Notify("GPS_HEADING_ACC", m_gps_heading_acc);
      Notify("GPS_BASELINE", m_gps_baseline);
      Notify("GPS_PITCH", m_gps_pitch);
      Notify("GPS_CARR_SOLN", m_carr_soln);
      Notify("GPS_HEADING_VALID", m_heading_valid ? "true" : "false");
    }

    // GROUP: dops (position geometry)
    if (m_publish_dops) {
      Notify("HDOP", static_cast<double>(m_hdop));
      Notify("VDOP", static_cast<double>(m_vdop));
      Notify("PDOP", static_cast<double>(m_pdop));
      Notify("GDOP", static_cast<double>(m_gdop));
      // Heading geometry DOPs (from STADOPHA)
      Notify("HDG_HDOP", static_cast<double>(m_hdg_hdop));
      Notify("HDG_PDOP", static_cast<double>(m_hdg_pdop));
    }

    // GROUP: status
    if (m_publish_status) {
      Notify("GPS_HAS_LOCK", m_gps_lock ? "true" : "false");
      Notify("GPS_FIX_TYPE", m_fix_type);
      Notify("GPS_NUM_SATS", static_cast<double>(m_num_sats));
      Notify("GPS_CORR_HEALTHY", m_corr_healthy ? "true" : "false");
      Notify("GPS_RTK_CALC_STATUS", static_cast<double>(m_rtk_calc_status));
      Notify("GPS_CORR_AGE", static_cast<double>(m_corr_age));
      Notify("GPS_SOL_AGE", static_cast<double>(m_sol_age));
      Notify("GPS_BASE_ID", m_base_id.empty() ? "NONE" : m_base_id);
    }

    // GROUP: time
    if (m_publish_time) {
      Notify("GPS_EPOCH_TIME", m_epoch_time);
    }

    // GROUP: state
    if (m_publish_state) {
      Notify("GPS_STATE", buildStateString());
    }

    // GROUP: dto
    if (m_publish_dto) {
      Notify("GNSS_POSITION", m_position_dto.to_moos_string());
      Notify("GNSS_HEADING", m_heading_dto.to_moos_string());
      Notify("GNSS_VELOCITY", m_velocity_dto.to_moos_string());
      Notify("GNSS_DOPS", m_dops_dto.to_moos_string());
      Notify("GNSS_STATUS", m_status_dto.to_moos_string());
      Notify("GNSS_RTK_STATUS", m_rtk_status_dto.to_moos_string());
    }

    m_new_data_available = false;
  }

  AppCastingMOOSApp::PostReport();
  return true;
}

//---------------------------------------------------------
// Procedure: buildStateString()

string UnicoreGPS::buildStateString()
{
  char buf[2048];
  snprintf(buf, sizeof(buf),
           "MOOSTime=%.6f,NAV_LAT=%.9f,NAV_LONG=%.9f,NAV_ALT=%.3f,"
           "NAV_X=%.3f,NAV_Y=%.3f,NAV_SPEED=%.3f,"
           "VEL_N=%.3f,VEL_E=%.3f,VEL_D=%.3f,"
           "NAV_COG=%.2f,"
           "GPS_HEADING=%.3f,GPS_HEADING_ACC=%.3f,GPS_BASELINE=%.4f,"
           "GPS_PITCH=%.3f,GPS_CARR_SOLN=%s,GPS_HEADING_VALID=%s,"
           "HDOP=%.2f,VDOP=%.2f,PDOP=%.2f,GDOP=%.2f,"
           "FIX_TYPE=%s,NUM_SATS=%d,"
           "H_ACC=%.3f,V_ACC=%.3f,"
           "GPS_LOCK=%s,"
           "CORR_HEALTHY=%s,CORR_AGE=%.1f,BASE_ID=%s",
           MOOSTime(),
           m_nav_lat, m_nav_lon, m_nav_alt,
           m_nav_x, m_nav_y, m_nav_speed,
           m_vel_n, m_vel_e, m_vel_d,
           m_track_over_ground,
           m_gps_heading, m_gps_heading_acc, m_gps_baseline,
           m_gps_pitch, m_carr_soln.c_str(),
           m_heading_valid ? "true" : "false",
           m_hdop, m_vdop, m_pdop, m_gdop,
           m_fix_type.c_str(), static_cast<int>(m_num_sats),
           m_h_acc, m_v_acc,
           m_gps_lock ? "true" : "false",
           m_corr_healthy ? "true" : "false",
           m_corr_age, m_base_id.c_str());
  return string(buf);
}

//---------------------------------------------------------
// Procedure: GeodesySetup()

bool UnicoreGPS::GeodesySetup()
{
  double LatOrigin = 0.0;
  double LonOrigin = 0.0;

  bool latOK = m_MissionReader.GetValue("LatOrigin", LatOrigin);
  if (!latOK) {
    reportConfigWarning("Latitude origin missing in MOOS file.");
    return false;
  }

  bool lonOK = m_MissionReader.GetValue("LongOrigin", LonOrigin);
  if (!lonOK) {
    reportConfigWarning("Longitude origin missing in MOOS file.");
    return false;
  }

  bool geoOK = m_geodesy.Initialise(LatOrigin, LonOrigin);
  if (!geoOK) {
    reportConfigWarning("CMOOSGeodesy::Initialise() failed. Invalid origin.");
    return false;
  }

  return true;
}

//---------------------------------------------------------
// Procedure: initializeGPS()

bool UnicoreGPS::initializeGPS()
{
  m_parser = new UnicoreParser(m_port_name, m_baud_rate);

  int max_retries = 5;
  for (int i = 0; i < max_retries; i++) {
    if (m_parser->connect()) {
      dbg_print("Connected to Unicore GPS on %s at %d baud\n",
                m_port_name.c_str(), m_baud_rate);
      return true;
    }
    reportRunWarning("GPS connect attempt " + to_string(i + 1) +
                     " of " + to_string(max_retries) + " failed.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  reportRunWarning("Failed to connect to Unicore GPS after " +
                   to_string(max_retries) + " attempts.");
  return false;
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool UnicoreGPS::OnStartUp()
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
    if (param == "port") {
      m_port_name = value;
      handled = true;
    }
    else if (param == "baud_rate") {
      m_baud_rate = stoi(value);
      handled = true;
    }
    else if (param == "nav_suffix") {
      m_nav_suffix = value;
      handled = true;
    }
    else if (param == "enable_geodesic") {
      m_enable_geodesic = (tolower(value) == "true");
      handled = true;
    }
    else if (param == "publish_position") {
      m_publish_position = (tolower(value) == "true");
      handled = true;
    }
    else if (param == "publish_velocity") {
      m_publish_velocity = (tolower(value) == "true");
      handled = true;
    }
    else if (param == "publish_heading") {
      m_publish_heading = (tolower(value) == "true");
      handled = true;
    }
    else if (param == "publish_dops") {
      m_publish_dops = (tolower(value) == "true");
      handled = true;
    }
    else if (param == "publish_status") {
      m_publish_status = (tolower(value) == "true");
      handled = true;
    }
    else if (param == "publish_time") {
      m_publish_time = (tolower(value) == "true");
      handled = true;
    }
    else if (param == "publish_state") {
      m_publish_state = (tolower(value) == "true");
      handled = true;
    }
    else if (param == "publish_dto") {
      m_publish_dto = (tolower(value) == "true");
      handled = true;
    }
    else if (param == "stale_data_timeout_sec" ||
             param == "gps_data_timeout_sec") {
      m_stale_data_timeout_sec = stod(value);
      if (m_stale_data_timeout_sec < 0.1)
        m_stale_data_timeout_sec = 0.1;
      handled = true;
    }
    else if (param == "stale_warn_repeat_sec") {
      m_stale_warn_repeat_sec = stod(value);
      if (m_stale_warn_repeat_sec < 0.1)
        m_stale_warn_repeat_sec = 0.1;
      handled = true;
    }
    else if (param == "quality_warn_repeat_sec") {
      m_quality_warn_repeat_sec = stod(value);
      if (m_quality_warn_repeat_sec < 0.1)
        m_quality_warn_repeat_sec = 0.1;
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

  if (m_enable_geodesic) {
    if (!GeodesySetup()) {
      reportConfigWarning("Geodesy setup failed. Local coordinates disabled.");
      m_enable_geodesic = false;
    } else {
      m_geodesy_initialized = true;
    }
  }

  if (!initializeGPS()) {
    reportConfigWarning("Failed to initialize GPS.");
    return false;
  }

  // Start stale-data timer at app startup so complete data loss is detected.
  m_last_gps_data_moos_time = MOOSTime();
  m_data_stale = false;
  m_last_stale_warn_moos_time = 0.0;
  m_quality_degraded = false;
  m_last_quality_warn_moos_time = 0.0;

  m_thread_running = true;
  m_reader_thread = std::thread(&UnicoreGPS::gpsReaderThread, this);

  registerVariables();
  return true;
}

//---------------------------------------------------------
// Procedure: gpsReaderThread()

void UnicoreGPS::gpsReaderThread()
{
  while (m_thread_running) {
    try {
      if (!m_parser || !m_parser->isConnected())
        break;

      auto parsed = m_parser->update();

      // Check if any messages were parsed
      bool any_new = false;
      for (const auto &kv : parsed) {
        if (kv.second) {
          any_new = true;
          break;
        }
      }

      if (!any_new) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }

      std::lock_guard<std::mutex> guard(m_data_mutex);

      // BESTNAVA - position
      if (parsed.count(UnicoreMessageType::BESTNAVA) &&
          parsed.at(UnicoreMessageType::BESTNAVA)) {
        const auto &nav = m_parser->getBestNavA();
        m_nav_lat = nav.lat;
        m_nav_lon = nav.lon;
        m_nav_alt = nav.height;
        m_h_acc = nav.hAcc;
        m_v_acc = nav.vAcc;
        m_fix_type = nav.fixType;
        m_num_sats = nav.numUsed;
        m_gps_lock = m_parser->hasFix();
        m_epoch_time = nav.epochTime;
        m_corr_age = nav.diffAge;
        m_sol_age = nav.solAge;
        m_base_id = nav.baseId;

        // Geodesic conversion
        if (m_enable_geodesic && m_geodesy_initialized &&
            m_nav_lat != 0.0 && m_nav_lon != 0.0) {
          double x, y;
          if (m_geodesy.LatLong2LocalGrid(m_nav_lat, m_nav_lon, y, x)) {
            m_nav_x = x;
            m_nav_y = y;
          }
        }

        // Populate position DTO
        m_position_dto.timestamp_epoch = MOOSTime();
        m_position_dto.latitude = nav.lat;
        m_position_dto.longitude = nav.lon;
        m_position_dto.altitude = nav.height;
        m_position_dto.undulation = nav.undulation;
        m_position_dto.h_acc = nav.hAcc;
        m_position_dto.v_acc = nav.vAcc;
        m_position_dto.lat_sigma = nav.latSigma;
        m_position_dto.lon_sigma = nav.lonSigma;
        m_position_dto.hgt_sigma = nav.hgtSigma;
        m_position_dto.fix_type = nav.fixType;
        m_position_dto.sol_status = nav.solStatus;
        m_position_dto.num_tracked = nav.numTracked;
        m_position_dto.num_used = nav.numUsed;
        m_position_dto.nav_x = m_nav_x;
        m_position_dto.nav_y = m_nav_y;
        m_position_dto.has_local_coords =
            (m_enable_geodesic && m_geodesy_initialized);
        m_position_dto.epoch_time = nav.epochTime;

        m_bestnava_count++;
      }

      // UNIHEADINGA - dual-antenna heading
      if (parsed.count(UnicoreMessageType::UNIHEADINGA) &&
          parsed.at(UnicoreMessageType::UNIHEADINGA)) {
        const auto &hdg = m_parser->getUniHeadingA();
        m_gps_heading = hdg.heading;
        m_gps_heading_acc = hdg.headingAcc;
        m_gps_baseline = hdg.baseline;
        m_gps_pitch = hdg.pitch;
        m_carr_soln = hdg.carrSoln;
        m_heading_valid = hdg.headingValid;

        // Populate heading DTO
        m_heading_dto.timestamp_epoch = MOOSTime();
        m_heading_dto.heading = hdg.heading;
        m_heading_dto.pitch = hdg.pitch;
        m_heading_dto.baseline = hdg.baseline;
        m_heading_dto.heading_sigma = hdg.headingSigma;
        m_heading_dto.pitch_sigma = hdg.pitchSigma;
        m_heading_dto.carr_soln = hdg.carrSoln;
        m_heading_dto.heading_valid = hdg.headingValid;
        m_heading_dto.sol_status = hdg.solStatus;
        m_heading_dto.num_tracked = hdg.numTracked;
        m_heading_dto.num_used = hdg.numUsed;
        m_heading_dto.epoch_time = hdg.epochTime;

        m_uniheadinga_count++;
      }

      // BESTVELA - velocity
      if (parsed.count(UnicoreMessageType::BESTVELA) &&
          parsed.at(UnicoreMessageType::BESTVELA)) {
        const auto &vel = m_parser->getBestVelA();
        m_nav_speed = vel.horSpd;
        m_vel_n = vel.velN;
        m_vel_e = vel.velE;
        m_vel_d = -vel.vertSpd; // Convert up-positive to down-positive
        m_track_over_ground = vel.trkGnd;

        // Populate velocity DTO
        m_velocity_dto.timestamp_epoch = MOOSTime();
        m_velocity_dto.ground_speed = vel.horSpd;
        m_velocity_dto.track_over_ground = vel.trkGnd;
        m_velocity_dto.vert_speed = vel.vertSpd;
        m_velocity_dto.vel_n = vel.velN;
        m_velocity_dto.vel_e = vel.velE;
        m_velocity_dto.sol_status = vel.solStatus;
        m_velocity_dto.latency = vel.latency;
        m_velocity_dto.epoch_time = vel.epochTime;

        m_bestvela_count++;
      }

      // PSRDOPA - DOPs (fallback if STADOPA not available)
      if (parsed.count(UnicoreMessageType::PSRDOPA) &&
          parsed.at(UnicoreMessageType::PSRDOPA)) {
        const auto &dop = m_parser->getPsrDopA();
        m_hdop = dop.hdop;
        m_vdop = dop.vdop;
        m_pdop = dop.pdop;
        m_gdop = dop.gdop;

        // Populate DOPs DTO
        m_dops_dto.timestamp_epoch = MOOSTime();
        m_dops_dto.gdop = dop.gdop;
        m_dops_dto.pdop = dop.pdop;
        m_dops_dto.hdop = dop.hdop;
        m_dops_dto.vdop = dop.vdop;
        m_dops_dto.htdop = dop.htdop;
        m_dops_dto.tdop = dop.tdop;
        m_dops_dto.epoch_time = dop.epochTime;

        m_psrdopa_count++;
      }

      // STADOPA - solution-aware DOPs (preferred over PSRDOPA)
      if (parsed.count(UnicoreMessageType::STADOPA) &&
          parsed.at(UnicoreMessageType::STADOPA)) {
        const auto &dop = m_parser->getStaDopA();
        m_hdop = dop.hdop;
        m_vdop = dop.vdop;
        m_pdop = dop.pdop;
        m_gdop = dop.gdop;

        // Overwrite DOPs DTO with solution-aware values
        m_dops_dto.timestamp_epoch = MOOSTime();
        m_dops_dto.gdop = dop.gdop;
        m_dops_dto.pdop = dop.pdop;
        m_dops_dto.hdop = dop.hdop;
        m_dops_dto.vdop = dop.vdop;
        m_dops_dto.htdop = dop.htdop;
        m_dops_dto.tdop = dop.tdop;
        m_dops_dto.epoch_time = dop.epochTime;

        m_stadopa_count++;
      }

      // STADOPHA - heading-specific DOPs
      if (parsed.count(UnicoreMessageType::STADOPHA) &&
          parsed.at(UnicoreMessageType::STADOPHA)) {
        const auto &dop = m_parser->getStaDophA();
        m_hdg_hdop = dop.hdop;
        m_hdg_vdop = dop.vdop;
        m_hdg_pdop = dop.pdop;
        m_hdg_gdop = dop.gdop;

        m_stadopha_count++;
      }

      // RTKSTATUSA - RTK solution status (Unicore Ref Manual 7.5.84)
      if (parsed.count(UnicoreMessageType::RTKSTATUSA) &&
          parsed.at(UnicoreMessageType::RTKSTATUSA)) {
        const auto &rtk = m_parser->getRtkStatusA();
        m_corr_healthy = rtk.corrHealthy;
        m_rtk_calc_status = rtk.calcStatus;

        // Populate RTK status DTO
        m_rtk_status_dto.timestamp_epoch = MOOSTime();
        m_rtk_status_dto.pos_type = rtk.posType;
        m_rtk_status_dto.calc_status = rtk.calcStatus;
        m_rtk_status_dto.ion_detected = rtk.ionDetected;
        m_rtk_status_dto.dual_rtk_flag = rtk.dualRtkFlag;
        m_rtk_status_dto.adr_number = rtk.adrNumber;
        m_rtk_status_dto.corr_healthy = rtk.corrHealthy;
        m_rtk_status_dto.epoch_time = rtk.epochTime;

        m_rtkstatusa_count++;
      }

      // Update composite status DTO
      m_status_dto.timestamp_epoch = MOOSTime();
      m_status_dto.gps_lock = m_gps_lock;
      m_status_dto.fix_type = m_fix_type;
      m_status_dto.carr_soln = m_carr_soln;
      m_status_dto.num_sats = m_num_sats;
      m_status_dto.heading_valid = m_heading_valid;
      m_status_dto.h_acc = m_h_acc;
      m_status_dto.v_acc = m_v_acc;
      m_status_dto.hdop = m_hdop;
      m_status_dto.diff_age = m_corr_age;
      m_status_dto.sol_age = m_sol_age;
      m_status_dto.base_id = m_base_id;
      m_status_dto.epoch_time = m_epoch_time;

      m_last_gps_data_moos_time = MOOSTime();
      m_new_data_available = true;

    } catch (const std::exception &e) {
      dbg_print("Exception in reader thread: %s\n", e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

//---------------------------------------------------------
// Procedure: registerVariables()

void UnicoreGPS::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("RTCM_DATA", 0);
}

//---------------------------------------------------------
// Procedure: buildReport()

bool UnicoreGPS::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "iUnicoreGPS (UM982) Status" << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Port: " << m_port_name << " @ " << m_baud_rate << endl;
  m_msgs << "Suffix: \"" << m_nav_suffix << "\"" << endl;
  m_msgs << "GPS Lock: " << (m_gps_lock ? "YES" : "NO") << endl;
  if (m_last_gps_data_moos_time > 0.0) {
    const double data_age = MOOSTime() - m_last_gps_data_moos_time;
    m_msgs << "Serial Data Age: " << doubleToString(data_age, 2)
           << " sec (timeout=" << doubleToString(m_stale_data_timeout_sec, 2)
           << " sec, stale=" << (m_data_stale ? "YES" : "NO") << ")"
           << endl;
  } else {
    m_msgs << "Serial Data Age: N/A (no messages received yet)" << endl;
  }
  m_msgs << endl;

  // Position
  ACTable postab(2);
  postab << "Position | Value";
  postab.addHeaderLines();
  postab << "NAV_LAT" << doubleToString(m_nav_lat, 7);
  postab << "NAV_LONG" << doubleToString(m_nav_lon, 7);
  postab << "NAV_ALT" << doubleToString(m_nav_alt, 2);
  postab << "NAV_X" << doubleToString(m_nav_x, 2);
  postab << "NAV_Y" << doubleToString(m_nav_y, 2);
  postab << "H_ACC" << doubleToString(m_h_acc, 3);
  postab << "V_ACC" << doubleToString(m_v_acc, 3);
  postab << "FIX_TYPE" << m_fix_type;
  postab << "NUM_SATS" << intToString(static_cast<int>(m_num_sats));
  m_msgs << postab.getFormattedString() << endl << endl;

  // Velocity
  ACTable veltab(2);
  veltab << "Velocity | Value";
  veltab.addHeaderLines();
  veltab << "NAV_SPEED" << doubleToString(m_nav_speed, 3);
  veltab << "VEL_N" << doubleToString(m_vel_n, 3);
  veltab << "VEL_E" << doubleToString(m_vel_e, 3);
  veltab << "VEL_D" << doubleToString(m_vel_d, 3);
  veltab << "NAV_COG" << doubleToString(m_track_over_ground, 1);
  m_msgs << veltab.getFormattedString() << endl << endl;

  // Heading (dual-antenna)
  ACTable hdgtab(2);
  hdgtab << "Heading (Dual-Ant) | Value";
  hdgtab.addHeaderLines();
  hdgtab << "GPS_HEADING" << doubleToString(m_gps_heading, 2);
  hdgtab << "GPS_HEADING_ACC" << doubleToString(m_gps_heading_acc, 3);
  hdgtab << "GPS_BASELINE" << doubleToString(m_gps_baseline, 4);
  hdgtab << "GPS_PITCH" << doubleToString(m_gps_pitch, 2);
  hdgtab << "GPS_CARR_SOLN" << m_carr_soln;
  hdgtab << "HEADING_VALID" << (m_heading_valid ? "true" : "false");
  m_msgs << hdgtab.getFormattedString() << endl << endl;

  // DOPs
  ACTable doptab(2);
  doptab << "DOPs | Value";
  doptab.addHeaderLines();
  doptab << "HDOP" << doubleToString(m_hdop, 2);
  doptab << "VDOP" << doubleToString(m_vdop, 2);
  doptab << "PDOP" << doubleToString(m_pdop, 2);
  doptab << "GDOP" << doubleToString(m_gdop, 2);
  doptab << "DOP_SRC" << (m_stadopa_count > 0 ? "STADOPA" : "PSRDOPA");
  m_msgs << doptab.getFormattedString() << endl << endl;

  // Heading DOPs
  ACTable hdgdoptab(2);
  hdgdoptab << "Heading DOPs | Value";
  hdgdoptab.addHeaderLines();
  hdgdoptab << "HDG_HDOP" << doubleToString(m_hdg_hdop, 2);
  hdgdoptab << "HDG_VDOP" << doubleToString(m_hdg_vdop, 2);
  hdgdoptab << "HDG_PDOP" << doubleToString(m_hdg_pdop, 2);
  hdgdoptab << "HDG_GDOP" << doubleToString(m_hdg_gdop, 2);
  m_msgs << hdgdoptab.getFormattedString() << endl << endl;

  // RTK Status
  ACTable rtktab(2);
  rtktab << "RTK Status | Value";
  rtktab.addHeaderLines();
  rtktab << "CORR_HEALTHY" << (m_corr_healthy ? "YES" : "NO");
  rtktab << "CORR_AGE" << doubleToString(m_corr_age, 1);
  rtktab << "BASE_ID" << (m_base_id.empty() ? "N/A" : m_base_id);
  rtktab << "RTCM_MSGS_RX" << uintToString(m_rtcm_msgs_received);
  rtktab << "RTCM_BYTES_TX" << uintToString(m_rtcm_bytes_written);
  m_msgs << rtktab.getFormattedString() << endl << endl;

  // Message counts
  ACTable msgtab(2);
  msgtab << "Message | Count";
  msgtab.addHeaderLines();
  msgtab << "BESTNAVA" << intToString(m_bestnava_count);
  msgtab << "UNIHEADINGA" << intToString(m_uniheadinga_count);
  msgtab << "BESTVELA" << intToString(m_bestvela_count);
  msgtab << "PSRDOPA" << intToString(m_psrdopa_count);
  msgtab << "STADOPA" << intToString(m_stadopa_count);
  msgtab << "STADOPHA" << intToString(m_stadopha_count);
  msgtab << "RTKSTATUSA" << intToString(m_rtkstatusa_count);
  m_msgs << msgtab.getFormattedString() << endl << endl;

  // Publish groups
  m_msgs << "Publish Groups:" << endl;
  m_msgs << "  position=" << (m_publish_position ? "ON" : "OFF")
         << "  velocity=" << (m_publish_velocity ? "ON" : "OFF")
         << "  heading=" << (m_publish_heading ? "ON" : "OFF") << endl;
  m_msgs << "  dops=" << (m_publish_dops ? "ON" : "OFF")
         << "  status=" << (m_publish_status ? "ON" : "OFF")
         << "  time=" << (m_publish_time ? "ON" : "OFF") << endl;
  m_msgs << "  state=" << (m_publish_state ? "ON" : "OFF")
         << "  dto=" << (m_publish_dto ? "ON" : "OFF") << endl;

  return true;
}
