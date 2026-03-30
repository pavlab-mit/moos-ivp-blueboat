/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: pBB_DGPS_EKF/BB_DGPS_EKF.cpp
   Last Ed: 2026-03-25
     Brief:
        Extended Kalman Filter for BlueBat navigation using
        dual-antenna GPS (DGPS) heading and level-frame gyroscope.

        Fuses GPS_STATE measurements (position, heading, speed, COG)
        with gyroscope for smooth heading estimation.
*************************************************************/

#include <iterator>
#include <ctime>
#include <cstring>
#include <cmath>
#include "MBUtils.h"
#include "ACTable.h"
#include "XYVector.h"
#include "BB_DGPS_EKF.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

BB_DGPS_EKF::BB_DGPS_EKF()
  : m_debug(false),
    m_debug_stream(nullptr),
    m_input_gps_state("GPS_STATE"),
    m_input_gyro_z("GYRO_LVL_Z"),
    m_input_thrust_l("DESIRED_THRUST_L"),
    m_input_thrust_r("DESIRED_THRUST_R"),
    m_output_nav_x("NAV_X"),
    m_output_nav_y("NAV_Y"),
    m_output_nav_lat("NAV_LAT"),
    m_output_nav_long("NAV_LONG"),
    m_output_nav_heading("NAV_HEADING"),
    m_output_nav_speed("NAV_SPEED"),
    m_output_nav_cog("NAV_COG"),
    m_output_nav_state("NAV_STATE"),
    m_speed_threshold(0.3),
    m_data_freshness_threshold(5.0),
    m_drift_decay_period(20.0),
    m_drift_thrust_threshold(1.0),
    m_gps_stale_threshold(5.0),
    m_enable_manual_override(true),
    m_enable_drift_seglist(true),
    m_drift_vector_scale(10.0),
    m_latest_gyro_z(0.0),
    m_latest_thrust_l(0.0),
    m_latest_thrust_r(0.0),
    m_thrust_off_start_time(0.0),
    m_thrusts_below_threshold(false),
    m_last_gps_time(0.0),
    m_last_gyro_time(0.0),
    m_last_iterate_time(0.0),
    m_gps_ready(false),
    m_gyro_ready(false),
    m_gps_valid(false),
    m_nav_published(false),
    m_gps_update_count(0),
    m_iterate_count(0),
    m_geodesy_initialized(false)
{
  memset(m_fname, '\0', m_fname_buff_size);
}

//---------------------------------------------------------
// Destructor

BB_DGPS_EKF::~BB_DGPS_EKF()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool BB_DGPS_EKF::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();
    double mtime = msg.GetTime();

    if (key == m_input_gps_state) {
      string sval = msg.GetString();
      if (parseGPSState(sval, m_latest_gps)) {
        m_last_gps_time = mtime;
        m_gps_ready = true;
        m_gps_update_count++;

        dbg_print("[%.3f] GPS_STATE: x=%.2f y=%.2f spd=%.2f hdg=%.1f cog=%.1f h_acc=%.2f hdop=%.2f hdg_valid=%d\n",
                  mtime, m_latest_gps.nav_x, m_latest_gps.nav_y,
                  m_latest_gps.speed, m_latest_gps.heading, m_latest_gps.cog,
                  m_latest_gps.h_acc, m_latest_gps.hdop, m_latest_gps.heading_valid);
      }
    }
    else if (key == m_input_gyro_z) {
      m_latest_gyro_z = msg.GetDouble();
      m_last_gyro_time = mtime;
      m_gyro_ready = true;

      dbg_print("[%.3f] GYRO_Z: %.4f rad/s\n", mtime, m_latest_gyro_z);
    }
    else if (key == m_input_thrust_l) {
      m_latest_thrust_l = msg.GetDouble();
    }
    else if (key == m_input_thrust_r) {
      m_latest_thrust_r = msg.GetDouble();
    }
    else if (key != "APPCAST_REQ") {
      reportRunWarning("Unhandled Mail: " + key);
    }
  }

  return true;
}

//---------------------------------------------------------
// Procedure: parseGPSState()
// Parse the GPS_STATE string from iUnicoreGPS

bool BB_DGPS_EKF::parseGPSState(const string &sval, BB_DGPS_EKF_Model::GPSMeasurement &gps)
{
  // Reset to defaults
  gps = BB_DGPS_EKF_Model::GPSMeasurement();

  vector<string> pairs = parseString(sval, ',');
  for (const string &pair_raw : pairs) {
    string pair = stripBlankEnds(pair_raw);
    string key = biteStringX(pair, '=');
    string value = stripBlankEnds(pair);

    if (key == "MOOSTime") {
      gps.timestamp = stod(value);
    }
    else if (key == "NAV_LAT") {
      gps.nav_lat = stod(value);
    }
    else if (key == "NAV_LONG") {
      gps.nav_lon = stod(value);
    }
    else if (key == "NAV_SPEED") {
      gps.speed = stod(value);
    }
    else if (key == "NAV_COG") {
      gps.cog = stod(value);
    }
    else if (key == "GPS_HEADING") {
      gps.heading = stod(value);
    }
    else if (key == "GPS_HEADING_ACC") {
      gps.heading_acc = stod(value);
    }
    else if (key == "GPS_HEADING_VALID") {
      gps.heading_valid = (value == "true");
    }
    else if (key == "HDOP") {
      gps.hdop = stod(value);
    }
    else if (key == "H_ACC") {
      gps.h_acc = stod(value);
    }
    else if (key == "GPS_LOCK") {
      gps.gps_lock = (value == "true");
    }
    else if (key == "FIX_TYPE") {
      gps.fix_type = value;
    }
  }

  // Convert lat/lon to local x/y using geodesy
  if (m_geodesy_initialized && gps.nav_lat != 0.0 && gps.nav_lon != 0.0) {
    double x, y;
    if (m_geodesy.LatLong2LocalGrid(gps.nav_lat, gps.nav_lon, y, x)) {
      gps.nav_x = x;
      gps.nav_y = y;
    }
  }

  return gps.isValid();
}

//---------------------------------------------------------
// Procedure: dbg_print()

bool BB_DGPS_EKF::dbg_print(const char *format, ...)
{
  if (!m_debug)
    return false;

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

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool BB_DGPS_EKF::OnConnectToServer()
{
  registerVariables();
  return true;
}

//---------------------------------------------------------
// Procedure: Iterate()

bool BB_DGPS_EKF::Iterate()
{
  AppCastingMOOSApp::Iterate();

  double current_time = MOOSTime();
  m_iterate_count++;

  // Compute dt since last iteration
  double dt = 0.05;  // Default 20Hz
  if (m_last_iterate_time > 0) {
    dt = current_time - m_last_iterate_time;
    dt = max(0.001, min(dt, 1.0));  // Clamp to reasonable range
  }
  m_last_iterate_time = current_time;

  // Check data freshness
  bool gps_fresh = (current_time - m_last_gps_time) < m_data_freshness_threshold;
  bool gyro_fresh = (current_time - m_last_gyro_time) < m_data_freshness_threshold;

  // Check GPS validity for manual override
  // GPS is valid if: fresh AND has lock AND fix type is valid
  bool gps_stale = (current_time - m_last_gps_time) > m_gps_stale_threshold;
  bool gps_currently_valid = !gps_stale && m_latest_gps.isValid();

  // Handle GPS validity state change and MOOS_MANUAL_OVERRIDE
  if (m_enable_manual_override) {
    if (gps_currently_valid && !m_gps_valid) {
      // Transitioning from invalid to valid
      Notify("MOOS_MANUAL_OVERRIDE", "false");
      dbg_print("[%.3f] GPS valid, MOOS_MANUAL_OVERRIDE=false\n", current_time);
    }
    else if (!gps_currently_valid && m_gps_valid) {
      // Transitioning from valid to invalid
      Notify("MOOS_MANUAL_OVERRIDE", "true");
      dbg_print("[%.3f] GPS invalid (stale=%d, lock=%d, fix=%s), MOOS_MANUAL_OVERRIDE=true\n",
                current_time, gps_stale, m_latest_gps.gps_lock, m_latest_gps.fix_type.c_str());
    }
    else if (!gps_currently_valid && !m_gps_valid && !m_nav_published) {
      // Still waiting for first valid GPS - keep override true
      Notify("MOOS_MANUAL_OVERRIDE", "true");
    }
  }
  m_gps_valid = gps_currently_valid;

  // Initialize filter on first valid GPS
  if (!m_ekf.isInitialized() && gps_fresh && m_latest_gps.isValid()) {
    m_ekf.initialize(m_latest_gps);
    dbg_print("[%.3f] EKF initialized: x=%.2f y=%.2f phi=%.1f gamma=%.1f s=%.2f\n",
              current_time, m_ekf.getX(), m_ekf.getY(),
              m_ekf.getHeadingCompass(), m_ekf.getCOGCompass(), m_ekf.getSpeed());
    Notify("EKF_STATUS", "INITIALIZED");
  }

  if (!m_ekf.isInitialized()) {
    Notify("EKF_STATUS", "WAITING_FOR_GPS");
    AppCastingMOOSApp::PostReport();
    return true;
  }

  // Prediction step using gyro (or zero if no fresh gyro)
  double gyro_z = gyro_fresh ? m_latest_gyro_z : 0.0;
  m_ekf.predict(dt, gyro_z);

  // Measurement update with GPS (pass speed threshold for COG gating)
  if (gps_fresh && m_latest_gps.isValid()) {
    m_ekf.updateGPS(m_latest_gps, m_speed_threshold);
  }

  // Check filter health
  if (!m_ekf.isHealthy()) {
    reportRunWarning("EKF health check failed - resetting");
    Notify("EKF_STATUS", "UNHEALTHY");

    // Re-initialize on next valid GPS
    m_ekf = BB_DGPS_EKF_Model();
    m_nav_published = false;  // Reset nav published flag
    AppCastingMOOSApp::PostReport();
    return true;
  }

  // Publish outputs only when GPS is valid
  if (m_gps_valid) {
    double nav_x = m_ekf.getX();
    double nav_y = m_ekf.getY();
    double nav_heading = m_ekf.getHeadingCompass();
    double nav_speed = m_ekf.getSpeed();
    double nav_cog = m_ekf.getCOGCompass();
    double nav_lat = 0.0, nav_lon = 0.0;

    Notify(m_output_nav_x, nav_x);
    Notify(m_output_nav_y, nav_y);
    Notify(m_output_nav_heading, nav_heading);
    Notify(m_output_nav_speed, nav_speed);
    Notify(m_output_nav_cog, nav_cog);

    // Convert filtered x/y back to lat/lon via reverse geodesy
    if (m_geodesy_initialized) {
      if (m_geodesy.LocalGrid2LatLong(nav_x, nav_y, nav_lat, nav_lon)) {
        Notify(m_output_nav_lat, nav_lat);
        Notify(m_output_nav_long, nav_lon);
      }
    }

    // Build and publish NAV_STATE bundle (similar to NODE_REPORT format)
    char buf[1024];
    snprintf(buf, sizeof(buf),
             "MOOSTime=%.6f,NAV_X=%.3f,NAV_Y=%.3f,NAV_LAT=%.9f,NAV_LONG=%.9f,"
             "NAV_HEADING=%.2f,NAV_SPEED=%.3f,NAV_COG=%.2f,"
             "STD_X=%.3f,STD_Y=%.3f,STD_HEADING=%.2f,STD_SPEED=%.3f,"
             "GPS_LOCK=%s,FIX_TYPE=%s,H_ACC=%.3f,HDOP=%.2f,"
             "HEADING_VALID=%s,GPS_HEADING=%.2f,GPS_HEADING_ACC=%.2f",
             current_time, nav_x, nav_y, nav_lat, nav_lon,
             nav_heading, nav_speed, nav_cog,
             m_ekf.getStdX(), m_ekf.getStdY(),
             m_ekf.getStdPhi() * 180.0 / M_PI, m_ekf.getStdSpeed(),
             m_latest_gps.gps_lock ? "true" : "false",
             m_latest_gps.fix_type.c_str(),
             m_latest_gps.h_acc, m_latest_gps.hdop,
             m_latest_gps.heading_valid ? "true" : "false",
             m_latest_gps.heading, m_latest_gps.heading_acc);
    Notify(m_output_nav_state, string(buf));

    m_nav_published = true;
  }

  // Publish diagnostics
  Notify("EKF_STATUS", m_gps_valid ? "RUNNING" : "GPS_INVALID");
  Notify("EKF_STD_X", m_ekf.getStdX());
  Notify("EKF_STD_Y", m_ekf.getStdY());
  Notify("EKF_STD_HEADING", m_ekf.getStdPhi() * 180.0 / M_PI);
  Notify("EKF_INTEGRATED_DIST", m_ekf.integratedDistance());
  Notify("EKF_GPS_UPDATE_COUNT", static_cast<double>(m_gps_update_count));

  //----------------------------------------------------------
  // Drift Estimator Logic
  //----------------------------------------------------------
  // Check if thrusters are below threshold (use absolute value for bidirectional thrust)
  bool thrusts_low = (fabs(m_latest_thrust_l) < m_drift_thrust_threshold) &&
                     (fabs(m_latest_thrust_r) < m_drift_thrust_threshold);

  if (thrusts_low) {
    if (!m_thrusts_below_threshold) {
      // Just transitioned to low thrust - start timer
      m_thrust_off_start_time = current_time;
      m_thrusts_below_threshold = true;
      dbg_print("[%.3f] Thrusts below threshold (|L|=%.1f, |R|=%.1f < %.1f), starting drift timer\n",
                current_time, fabs(m_latest_thrust_l), fabs(m_latest_thrust_r), m_drift_thrust_threshold);
    }

    // Check if we've been drifting long enough
    double drift_time = current_time - m_thrust_off_start_time;
    if (drift_time >= m_drift_decay_period) {
      // Start drift estimation if not already active
      if (!m_drift_estimator.isActive()) {
        m_drift_estimator.startDrifting();
        dbg_print("[%.3f] Drift decay period met (%.1fs), starting drift estimation\n",
                  current_time, drift_time);
      }

      // Run drift estimator prediction
      m_drift_estimator.predict(dt);

      // Update drift estimator with velocity from main EKF
      // Use COG and speed from main filter
      double speed = m_ekf.getSpeed();
      double cog = m_ekf.getCOG();  // Already in Cartesian radians
      m_drift_estimator.updateVelocity(speed, cog);
    }
  } else {
    // Thrusters engaged - stop drift estimation
    if (m_thrusts_below_threshold) {
      dbg_print("[%.3f] Thrusters engaged (|L|=%.1f, |R|=%.1f), stopping drift estimation\n",
                current_time, fabs(m_latest_thrust_l), fabs(m_latest_thrust_r));
    }
    m_thrusts_below_threshold = false;
    m_drift_estimator.stopDrifting();
  }

  // Publish drift estimator outputs
  Notify("DRIFT_ACTIVE", m_drift_estimator.isActive() ? 1.0 : 0.0);
  if (m_drift_estimator.isInitialized()) {
    Notify("DRIFT_VX", m_drift_estimator.getDriftVx());
    Notify("DRIFT_VY", m_drift_estimator.getDriftVy());
    Notify("DRIFT_SPEED", m_drift_estimator.getDriftSpeed());
    Notify("DRIFT_DIR", m_drift_estimator.getDriftDirectionCompass());
    Notify("DRIFT_STD_VX", m_drift_estimator.getStdVx());
    Notify("DRIFT_STD_VY", m_drift_estimator.getStdVy());
    Notify("DRIFT_SAMPLES", static_cast<double>(m_drift_estimator.getSampleCount()));

    // Publish drift vector visualization (VIEW_VECTOR for pMarineViewer)
    if (m_enable_drift_seglist && m_drift_estimator.getDriftSpeed() > 0.01) {
      double vx = m_drift_estimator.getDriftVx();
      double vy = m_drift_estimator.getDriftVy();

      // Create XYVector for visualization
      XYVector drift_vec;
      drift_vec.setPosition(m_ekf.getX(), m_ekf.getY());
      drift_vec.setVectorXY(vx * m_drift_vector_scale, vy * m_drift_vector_scale);

      // Label shows drift velocity components
      string label = "drift[" + doubleToString(vx, 3) + "," + doubleToString(vy, 3) + "]";
      drift_vec.set_label(label);

      string spec = drift_vec.get_spec();
      spec += ",head_size=5,edge_color=cyan,vertex_color=cyan";
      Notify("VIEW_VECTOR", spec);
    }
  }

  dbg_print("[%.3f] EKF state: x=%.2f y=%.2f phi=%.1f gamma=%.1f s=%.2f | std_x=%.2f std_y=%.2f\n",
            current_time, m_ekf.getX(), m_ekf.getY(),
            m_ekf.getHeadingCompass(), m_ekf.getCOGCompass(), m_ekf.getSpeed(),
            m_ekf.getStdX(), m_ekf.getStdY());

  AppCastingMOOSApp::PostReport();
  return true;
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool BB_DGPS_EKF::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  m_app_name = GetAppName();

  // Get noise config reference
  BB_DGPS_EKF_Model::NoiseConfig &noise = m_ekf.noiseConfig();

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++) {
    string orig = *p;
    string line = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;

    // Input/output variable names
    if (param == "input_gps_state") {
      m_input_gps_state = value;
      handled = true;
    }
    else if (param == "input_gyro_z") {
      m_input_gyro_z = value;
      handled = true;
    }
    else if (param == "output_nav_x") {
      m_output_nav_x = value;
      handled = true;
    }
    else if (param == "output_nav_y") {
      m_output_nav_y = value;
      handled = true;
    }
    else if (param == "output_nav_lat") {
      m_output_nav_lat = value;
      handled = true;
    }
    else if (param == "output_nav_long") {
      m_output_nav_long = value;
      handled = true;
    }
    else if (param == "output_nav_heading") {
      m_output_nav_heading = value;
      handled = true;
    }
    else if (param == "output_nav_speed") {
      m_output_nav_speed = value;
      handled = true;
    }
    else if (param == "output_nav_cog") {
      m_output_nav_cog = value;
      handled = true;
    }
    else if (param == "output_nav_state") {
      m_output_nav_state = value;
      handled = true;
    }

    // Process noise parameters
    else if (param == "sigma_x") {
      noise.sigma_x = stod(value);
      handled = true;
    }
    else if (param == "sigma_y") {
      noise.sigma_y = stod(value);
      handled = true;
    }
    else if (param == "sigma_phi" || param == "sigma_heading") {
      noise.sigma_phi = stod(value);
      handled = true;
    }
    else if (param == "sigma_gamma" || param == "sigma_cog") {
      noise.sigma_gamma = stod(value);
      handled = true;
    }
    else if (param == "sigma_s" || param == "sigma_speed") {
      noise.sigma_s = stod(value);
      handled = true;
    }

    // Measurement noise parameters
    else if (param == "sigma_gps_pos") {
      noise.sigma_gps_pos = stod(value);
      handled = true;
    }
    else if (param == "sigma_gps_hdg") {
      noise.sigma_gps_hdg = stod(value) * M_PI / 180.0;  // Convert degrees to radians
      handled = true;
    }
    else if (param == "sigma_gps_cog") {
      noise.sigma_gps_cog = stod(value) * M_PI / 180.0;  // Convert degrees to radians
      handled = true;
    }
    else if (param == "sigma_gps_spd") {
      noise.sigma_gps_spd = stod(value);
      handled = true;
    }

    // Thresholds
    else if (param == "speed_threshold") {
      m_speed_threshold = stod(value);
      handled = true;
    }
    else if (param == "data_freshness_threshold") {
      m_data_freshness_threshold = stod(value);
      handled = true;
    }

    // Drift estimator configuration
    else if (param == "drift_decay_period") {
      m_drift_decay_period = stod(value);
      handled = true;
    }
    else if (param == "drift_thrust_threshold") {
      m_drift_thrust_threshold = stod(value);
      handled = true;
    }
    else if (param == "input_thrust_l") {
      m_input_thrust_l = value;
      handled = true;
    }
    else if (param == "input_thrust_r") {
      m_input_thrust_r = value;
      handled = true;
    }

    // GPS validity / manual override configuration
    else if (param == "gps_stale_threshold") {
      m_gps_stale_threshold = stod(value);
      handled = true;
    }
    else if (param == "enable_manual_override") {
      m_enable_manual_override = (tolower(value) == "true");
      handled = true;
    }

    // Drift visualization configuration
    else if (param == "enable_drift_seglist") {
      m_enable_drift_seglist = (tolower(value) == "true");
      handled = true;
    }
    else if (param == "drift_vector_scale") {
      m_drift_vector_scale = stod(value);
      handled = true;
    }

    // Drift estimator noise parameters
    else if (param == "drift_sigma_vx") {
      m_drift_estimator.noiseConfig().sigma_vx = stod(value);
      handled = true;
    }
    else if (param == "drift_sigma_vy") {
      m_drift_estimator.noiseConfig().sigma_vy = stod(value);
      handled = true;
    }
    else if (param == "drift_sigma_meas") {
      m_drift_estimator.noiseConfig().sigma_meas = stod(value);
      handled = true;
    }

    // Debug
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

  // Initialize geodesy from mission file LatOrigin/LongOrigin
  double lat_origin = 0.0;
  double lon_origin = 0.0;
  if (m_MissionReader.GetValue("LatOrigin", lat_origin) &&
      m_MissionReader.GetValue("LongOrigin", lon_origin)) {
    if (m_geodesy.Initialise(lat_origin, lon_origin)) {
      m_geodesy_initialized = true;
      dbg_print("Geodesy initialized: LatOrigin=%.7f, LongOrigin=%.7f\n",
                lat_origin, lon_origin);
    } else {
      reportConfigWarning("Geodesy initialization failed");
    }
  } else {
    reportConfigWarning("LatOrigin/LongOrigin not found - local coordinates unavailable");
  }

  registerVariables();
  return true;
}

//---------------------------------------------------------
// Procedure: registerVariables()

void BB_DGPS_EKF::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register(m_input_gps_state, 0);
  Register(m_input_gyro_z, 0);
  Register(m_input_thrust_l, 0);
  Register(m_input_thrust_r, 0);
}

//---------------------------------------------------------
// Procedure: buildReport()

bool BB_DGPS_EKF::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "pBB_DGPS_EKF - DGPS Navigation EKF          " << endl;
  m_msgs << "============================================" << endl;

  double current_time = MOOSTime();

  // Status
  string status = m_ekf.isInitialized() ? "RUNNING" : "WAITING_FOR_GPS";
  if (m_ekf.isInitialized() && !m_ekf.isHealthy())
    status = "UNHEALTHY";
  else if (m_ekf.isInitialized() && !m_gps_valid)
    status = "GPS_INVALID";

  m_msgs << "Status: " << status << endl;
  m_msgs << "GPS Valid: " << (m_gps_valid ? "YES" : "NO") << endl;
  m_msgs << "NAV Published: " << (m_nav_published ? "YES" : "NO") << endl;
  m_msgs << "GPS Updates: " << m_gps_update_count << endl;
  m_msgs << "Iterate Count: " << m_iterate_count << endl;
  m_msgs << endl;

  // State table
  ACTable state_tab(4);
  state_tab << "State | Value | Std Dev | Units";
  state_tab.addHeaderLines();

  if (m_ekf.isInitialized()) {
    state_tab << "X" << doubleToString(m_ekf.getX(), 2)
              << doubleToString(m_ekf.getStdX(), 2) << "m";
    state_tab << "Y" << doubleToString(m_ekf.getY(), 2)
              << doubleToString(m_ekf.getStdY(), 2) << "m";
    state_tab << "Heading (phi)" << doubleToString(m_ekf.getHeadingCompass(), 1)
              << doubleToString(m_ekf.getStdPhi() * 180.0 / M_PI, 1) << "deg";
    state_tab << "COG (gamma)" << doubleToString(m_ekf.getCOGCompass(), 1)
              << doubleToString(m_ekf.getStdGamma() * 180.0 / M_PI, 1) << "deg";
    state_tab << "Speed" << doubleToString(m_ekf.getSpeed(), 2)
              << doubleToString(m_ekf.getStdSpeed(), 2) << "m/s";
  } else {
    state_tab << "X" << "N/A" << "N/A" << "m";
    state_tab << "Y" << "N/A" << "N/A" << "m";
    state_tab << "Heading" << "N/A" << "N/A" << "deg";
    state_tab << "COG" << "N/A" << "N/A" << "deg";
    state_tab << "Speed" << "N/A" << "N/A" << "m/s";
  }
  m_msgs << state_tab.getFormattedString() << endl;

  // Sensor freshness
  m_msgs << "Sensor Freshness:" << endl;
  m_msgs << "  GPS: " << doubleToString(current_time - m_last_gps_time, 1) << "s ago";
  if ((current_time - m_last_gps_time) > m_data_freshness_threshold)
    m_msgs << " [STALE]";
  m_msgs << endl;

  m_msgs << "  Gyro: " << doubleToString(current_time - m_last_gyro_time, 1) << "s ago";
  if ((current_time - m_last_gyro_time) > m_data_freshness_threshold)
    m_msgs << " [STALE]";
  m_msgs << endl;

  // Latest GPS data
  m_msgs << endl << "Latest GPS Measurement:" << endl;
  m_msgs << "  Lat/Lon: (" << doubleToString(m_latest_gps.nav_lat, 7) << ", "
         << doubleToString(m_latest_gps.nav_lon, 7) << ") deg" << endl;
  m_msgs << "  Local XY: (" << doubleToString(m_latest_gps.nav_x, 2) << ", "
         << doubleToString(m_latest_gps.nav_y, 2) << ") m";
  m_msgs << (m_geodesy_initialized ? "" : " [NO GEODESY]") << endl;
  m_msgs << "  Speed: " << doubleToString(m_latest_gps.speed, 2) << " m/s" << endl;
  m_msgs << "  COG: " << doubleToString(m_latest_gps.cog, 1) << " deg" << endl;
  m_msgs << "  Heading: " << doubleToString(m_latest_gps.heading, 1) << " deg";
  m_msgs << (m_latest_gps.heading_valid ? " [VALID]" : " [INVALID]") << endl;
  m_msgs << "  H_ACC: " << doubleToString(m_latest_gps.h_acc, 2) << " m" << endl;
  m_msgs << "  HDOP: " << doubleToString(m_latest_gps.hdop, 2) << endl;
  m_msgs << "  Fix: " << m_latest_gps.fix_type << endl;

  // Configuration
  m_msgs << endl << "Configuration:" << endl;
  m_msgs << "  Input GPS: " << m_input_gps_state << endl;
  m_msgs << "  Input Gyro: " << m_input_gyro_z << endl;
  m_msgs << "  Speed Threshold: " << m_speed_threshold << " m/s" << endl;
  m_msgs << "  Freshness Threshold: " << m_data_freshness_threshold << " s" << endl;
  m_msgs << "  GPS Stale Threshold: " << m_gps_stale_threshold << " s" << endl;
  m_msgs << "  Manual Override: " << (m_enable_manual_override ? "enabled" : "disabled") << endl;

  // Metrics
  if (m_ekf.isInitialized()) {
    m_msgs << endl << "Metrics:" << endl;
    m_msgs << "  Integrated Distance: " << doubleToString(m_ekf.integratedDistance(), 1)
           << " m" << endl;
  }

  // Drift Estimator section
  m_msgs << endl << "============================================" << endl;
  m_msgs << "Drift Estimator" << endl;
  m_msgs << "============================================" << endl;

  m_msgs << "Thrust Status:" << endl;
  m_msgs << "  Thrust L: " << doubleToString(m_latest_thrust_l, 1) << " (|" << doubleToString(fabs(m_latest_thrust_l), 1) << "|)" << endl;
  m_msgs << "  Thrust R: " << doubleToString(m_latest_thrust_r, 1) << " (|" << doubleToString(fabs(m_latest_thrust_r), 1) << "|)" << endl;
  m_msgs << "  Threshold: " << doubleToString(m_drift_thrust_threshold, 1) << " (absolute)" << endl;
  m_msgs << "  Below threshold: " << (m_thrusts_below_threshold ? "YES" : "NO") << endl;

  if (m_thrusts_below_threshold) {
    double drift_time = current_time - m_thrust_off_start_time;
    m_msgs << "  Time below threshold: " << doubleToString(drift_time, 1) << "s";
    m_msgs << " / " << doubleToString(m_drift_decay_period, 1) << "s" << endl;
  }

  m_msgs << endl << "Drift Estimator Status:" << endl;
  m_msgs << "  Active: " << (m_drift_estimator.isActive() ? "YES" : "NO") << endl;
  m_msgs << "  Initialized: " << (m_drift_estimator.isInitialized() ? "YES" : "NO") << endl;
  m_msgs << "  Samples: " << m_drift_estimator.getSampleCount() << endl;

  if (m_drift_estimator.isInitialized()) {
    ACTable drift_tab(4);
    drift_tab << "State | Value | Std Dev | Units";
    drift_tab.addHeaderLines();
    drift_tab << "Drift Vx" << doubleToString(m_drift_estimator.getDriftVx(), 3)
              << doubleToString(m_drift_estimator.getStdVx(), 3) << "m/s";
    drift_tab << "Drift Vy" << doubleToString(m_drift_estimator.getDriftVy(), 3)
              << doubleToString(m_drift_estimator.getStdVy(), 3) << "m/s";
    drift_tab << "Drift Speed" << doubleToString(m_drift_estimator.getDriftSpeed(), 3)
              << "-" << "m/s";
    drift_tab << "Drift Dir" << doubleToString(m_drift_estimator.getDriftDirectionCompass(), 1)
              << "-" << "deg";
    m_msgs << drift_tab.getFormattedString() << endl;
  }

  m_msgs << endl << "Drift Config:" << endl;
  m_msgs << "  Decay Period: " << doubleToString(m_drift_decay_period, 1) << " s" << endl;
  m_msgs << "  Thrust Threshold: " << doubleToString(m_drift_thrust_threshold, 1) << " (absolute value)" << endl;
  m_msgs << "  Process Noise (sigma_vx): " << doubleToString(m_drift_estimator.noiseConfig().sigma_vx, 4) << endl;
  m_msgs << "  Meas Noise (sigma_meas): " << doubleToString(m_drift_estimator.noiseConfig().sigma_meas, 2) << endl;
  m_msgs << "  Drift Seglist: " << (m_enable_drift_seglist ? "enabled" : "disabled") << endl;
  m_msgs << "  Vector Scale: " << doubleToString(m_drift_vector_scale, 1) << " m per m/s" << endl;

  return true;
}
