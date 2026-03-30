
/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: pBB_EKF/BB_EKF.cpp
   Last Ed:  2025-11-25
     Brief:
        Lorem ipsum dolor sit amet, consectetur adipiscing 
        elit, sed do eiusmod tempor incididunt ut labore et 
        dolore magna aliqua. Ut enim ad minim veniam, quis 
        nostrud exercitation ullamco laboris nisi ut aliquip 
        ex ea commodo consequat.
*************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "BB_EKF.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;

//---------------------------------------------------------
// Constructor()

BB_EKF::BB_EKF()
{
  // Initialize configuration parameters
  m_speed_threshold = 0.5;
  m_dt_nominal = 0.1;
  m_last_update_time = -1;
  
  // Process variance defaults
  m_pos_process_variance = 0.05;
  m_speed_process_variance = 0.01;
  m_heading_process_variance = 0.001;
  m_bias_process_variance = 0.0001;
  
  // Measurement standard deviation defaults
  m_gps_pos_stddev = 1.0;      // meters
  m_gps_speed_stddev = 0.5;    // m/s
  m_mag_meas_stddev = 1.0;     // degrees
  m_bias_meas_stddev = 1.0;    // degrees
  
  // Default variable name mappings
  m_input_gps_x = "NAV_X_GPS";
  m_input_gps_y = "NAV_Y_GPS";
  m_input_gps_speed = "NAV_SPEED_GPS";
  m_input_gps_heading = "NAV_HEADING_GPS";
  m_input_mag_heading = "NAV_HEADING_MAG";
  m_input_gyro_z = "GYRO_Z_LVL";
  
  m_output_nav_x = "NAV_X";
  m_output_nav_y = "NAV_Y";
  m_output_nav_speed = "NAV_SPEED";
  m_output_nav_heading = "NAV_HEADING";
  
  m_last_mag_heading_cart = 0.0;
}

//---------------------------------------------------------
// Destructor

BB_EKF::~BB_EKF()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool BB_EKF::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    double mtime = msg.GetTime();

    // GNSS Position
    if(key == m_input_gps_x) {
      m_nav_pos_gnss.x = msg.GetDouble();
      m_nav_pos_gnss.x_new = true;
      m_nav_pos_gnss.timestamp = mtime;
    }
    else if(key == m_input_gps_y) {
      m_nav_pos_gnss.y = msg.GetDouble();
      m_nav_pos_gnss.y_new = true;
      m_nav_pos_gnss.timestamp = mtime;
    }
    // GNSS Speed
    else if(key == m_input_gps_speed) {
      m_nav_speed_gnss.speed = msg.GetDouble();
      m_nav_speed_gnss.ready_flag = true;
      m_nav_speed_gnss.timestamp = mtime;
    }
    // GNSS Heading (Course over Ground - compass convention)
    else if(key == m_input_gps_heading) {
      double cog_deg = msg.GetDouble();
      // Convert from compass (0°=North,CW) to Cartesian (0=East,CCW)
      m_nav_heading_gnss.heading = wrapAngle(M_PI/2.0 - cog_deg * M_PI / 180.0);
      m_nav_heading_gnss.ready_flag = true;
      m_nav_heading_gnss.timestamp = mtime;
    }
    // Magnetic Heading (compass convention)
    else if(key == m_input_mag_heading) {
      double mag_deg = msg.GetDouble();
      // Convert from compass (0°=North,CW) to Cartesian (0=East,CCW)
      m_mag_heading.heading = wrapAngle(M_PI/2.0 - mag_deg * M_PI / 180.0);
      m_mag_heading.ready_flag = true;
      m_mag_heading.timestamp = mtime;
    }
    // Gyro Z (NED frame - positive is right turn)
    else if(key == m_input_gyro_z) {
      m_gyro_z.gz_lvl = msg.GetDouble();
      // NED to Cartesian: negate because NED positive is CW, Cartesian positive is CCW
      m_gyro_z.gz_lvl = -m_gyro_z.gz_lvl;
      m_gyro_z.ready_flag = true;
      m_gyro_z.timestamp = mtime;
    }
    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }
	
  return(true);
}

//---------------------------------------------------------
// Procedure: wrapAngle()
double BB_EKF::wrapAngle(double angle)
{
  return atan2(sin(angle), cos(angle));
}

// Procedure: dbg_print()
bool BB_EKF::dbg_print(const char *format, ...)
{
  if (m_debug == true)
  {
    va_list args;
    va_start(args, format);
    m_debug_stream = fopen(m_fname, "a");
    if (m_debug_stream != nullptr)
    {
      vfprintf(m_debug_stream, format, args);
      fclose(m_debug_stream);
      return true;
    }
    else
    {
      reportRunWarning("Debug mode is enabled and file could not be opened\n");
      return false;
    }
  }
  return false;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool BB_EKF::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool BB_EKF::Iterate()
{
  AppCastingMOOSApp::Iterate();
  
  double current_time = MOOSTime();
  
  // Initialize EKF if we have first complete GPS fix
  if (!m_ekf.isInitialized()) {
    if (m_nav_pos_gnss.ready() && m_nav_speed_gnss.ready() && 
        m_nav_heading_gnss.ready()) {
      
      // Use GPS heading for initial true heading
      // If we have mag heading available, initialize with it, otherwise use GPS heading for both
      double init_mag_heading = m_mag_heading.ready() ? m_mag_heading.heading : m_nav_heading_gnss.heading;
      
      m_ekf.initialize(m_nav_pos_gnss.x, m_nav_pos_gnss.y, 
                       m_nav_speed_gnss.speed,
                       m_nav_heading_gnss.heading,  // True heading from GPS
                       init_mag_heading);           // Mag heading (or GPS if mag not ready)
      
      m_last_update_time = current_time;
      m_last_mag_heading_cart = init_mag_heading;
      
      // Reset all measurement flags
      m_nav_pos_gnss.reset();
      m_nav_speed_gnss.reset();
      m_nav_heading_gnss.reset();
      m_mag_heading.reset();
      m_gyro_z.reset();
      
      Notify("EKF_STATUS", "INITIALIZED");
    }
    AppCastingMOOSApp::PostReport();
    return(true);
  }
  
  // Compute dt - clamp large gaps but don't replace with nominal
  double dt = current_time - m_last_update_time;
  if (dt <= 0) {
    dt = 0.01;  // Minimum reasonable dt
  } else if (dt > 1.0) {
    dt = 1.0;   // Clamp maximum dt to prevent covariance explosion
  }
  
  // ============= PREDICTION STEP =============
  // Always predict with gyro if available
  double gyro_z = 0.0;
  if (m_gyro_z.ready()) {
    gyro_z = m_gyro_z.gz_lvl; // Already converted to Cartesian in OnNewMail
  }
  
  m_ekf.predict(dt, gyro_z);
  
  // ============= UPDATE STEPS =============
  // 1. Position Update (only use fresh data within 5 seconds)
  if (m_nav_pos_gnss.ready() && (current_time - m_nav_pos_gnss.timestamp) < 5.0) {
    m_ekf.updatePosition(m_nav_pos_gnss.x, m_nav_pos_gnss.y);
    m_nav_pos_gnss.reset();
  }
  
  // 2. Speed Update (only use fresh data within 5 seconds)
  if (m_nav_speed_gnss.ready() && (current_time - m_nav_speed_gnss.timestamp) < 5.0) {
    m_ekf.updateSpeed(m_nav_speed_gnss.speed);
    m_nav_speed_gnss.reset();
  }
  
  // 3. Magnetic Heading Update (only when moving fast enough)
  bool mag_available = m_mag_heading.ready() && (current_time - m_mag_heading.timestamp) < 5.0;
  if (mag_available) {
    m_last_mag_heading_cart = m_mag_heading.heading;  // Always store for output
    
    // Only update phi with mag when moving fast (GNSS+gyro drives phi when slow)
    if (m_ekf.getSpeed() > m_speed_threshold) {
      m_ekf.updateMagHeading(m_mag_heading.heading);
    }
    
    // 4. Bias Update (only when moving and have fresh GNSS heading)
    if (m_nav_heading_gnss.ready() && (current_time - m_nav_heading_gnss.timestamp) < 5.0 && m_ekf.getSpeed() > m_speed_threshold) {
      double gps_heading = m_nav_heading_gnss.heading;
      double mag_heading = m_mag_heading.heading;
      
      // Check for reverse motion using configurable threshold
      double heading_diff = wrapAngle(gps_heading - mag_heading);
      double thresh_rad = m_reverse_heading_error_thresh * M_PI / 180.0;
      bool reverse_motion_detected = (fabs(heading_diff) > thresh_rad);
      
      if (reverse_motion_detected) {
        gps_heading = wrapAngle(gps_heading + M_PI);  // Add 180 degrees
        Notify("EKF_REVERSE_MOTION", 1.0);
      } else {
        Notify("EKF_REVERSE_MOTION", 0.0);
      }
      
      m_ekf.updateBias(gps_heading, mag_heading);
    }
    
    m_mag_heading.reset();
  }
  
  // Reset stale data flags
  if (m_nav_heading_gnss.ready()) {
    m_nav_heading_gnss.reset();
  }
  
  if (m_gyro_z.ready()) {
    m_gyro_z.reset();
  }
  
  // Reset stale data that wasn't used
  if (m_nav_pos_gnss.ready() && (current_time - m_nav_pos_gnss.timestamp) >= 5.0) {
    m_nav_pos_gnss.reset();
  }
  if (m_nav_speed_gnss.ready() && (current_time - m_nav_speed_gnss.timestamp) >= 5.0) {
    m_nav_speed_gnss.reset();
  }
  if (m_mag_heading.ready() && (current_time - m_mag_heading.timestamp) >= 5.0) {
    m_mag_heading.reset();
  }
  
  // ============= PUBLISH OUTPUTS =============
  // Position
  Notify(m_output_nav_x, m_ekf.getX());
  Notify(m_output_nav_y, m_ekf.getY());
  
  // Speed
  Notify(m_output_nav_speed, m_ekf.getSpeed());
  
  // Heading - use smart output that switches based on speed
  // Use last known mag heading for low-speed operation
  double output_heading_cart = m_ekf.getOutputHeading(m_last_mag_heading_cart);
  // Convert back to compass convention (0°=North, CW)
  double output_heading_compass = wrapAngle(M_PI/2.0 - output_heading_cart) * 180.0 / M_PI;
  if (output_heading_compass < 0) {
    output_heading_compass += 360.0;
  }
  Notify(m_output_nav_heading, output_heading_compass);
  
  // Debug outputs
  Notify("EKF_BIAS_DEG", m_ekf.getBias() * 180.0 / M_PI);
  Notify("EKF_SPEED_THRESHOLD_ACTIVE", m_ekf.getSpeed() <= m_speed_threshold ? 1.0 : 0.0);
  
  // Check filter health
  if (!m_ekf.isHealthy()) {
    reportRunWarning("EKF unhealthy - check covariance");
    Notify("EKF_STATUS", "UNHEALTHY");
  }
  
  m_last_update_time = current_time;
  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool BB_EKF::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());
  m_app_name = GetAppName();
  

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    
    // Threshold parameters
    if(param == "speed_threshold") {
      m_speed_threshold = atof(value.c_str());
      handled = true;
    }
    else if(param == "reverse_heading_error_thresh") {
      m_reverse_heading_error_thresh = atof(value.c_str());
      handled = true;
    }
    else if(param == "dt_nominal") {
      m_dt_nominal = atof(value.c_str());
      handled = true;
    }
    // Process variance parameters
    else if(param == "pos_process_variance") {
      m_pos_process_variance = atof(value.c_str());
      handled = true;
    }
    else if(param == "speed_process_variance") {
      m_speed_process_variance = atof(value.c_str());
      handled = true;
    }
    else if(param == "heading_process_variance") {
      m_heading_process_variance = atof(value.c_str());
      handled = true;
    }
    else if(param == "bias_process_variance") {
      m_bias_process_variance = atof(value.c_str());
      handled = true;
    }
    // Measurement standard deviation parameters
    else if(param == "gps_pos_stddev") {
      m_gps_pos_stddev = atof(value.c_str());
      handled = true;
    }
    else if(param == "gps_speed_stddev") {
      m_gps_speed_stddev = atof(value.c_str());
      handled = true;
    }
    else if(param == "mag_meas_stddev") {
      m_mag_meas_stddev = atof(value.c_str());
      handled = true;
    }
    else if(param == "bias_meas_stddev") {
      m_bias_meas_stddev = atof(value.c_str());
      handled = true;
    }
    // Input variable mappings
    else if(param == "input_gps_x") {
      m_input_gps_x = value;
      handled = true;
    }
    else if(param == "input_gps_y") {
      m_input_gps_y = value;
      handled = true;
    }
    else if(param == "input_gps_speed") {
      m_input_gps_speed = value;
      handled = true;
    }
    else if(param == "input_gps_heading") {
      m_input_gps_heading = value;
      handled = true;
    }
    else if(param == "input_mag_heading") {
      m_input_mag_heading = value;
      handled = true;
    }
    else if(param == "input_gyro_z") {
      m_input_gyro_z = value;
      handled = true;
    }
    // Output variable mappings
    else if(param == "output_nav_x") {
      m_output_nav_x = value;
      handled = true;
    }
    else if(param == "output_nav_y") {
      m_output_nav_y = value;
      handled = true;
    }
    else if(param == "output_nav_speed") {
      m_output_nav_speed = value;
      handled = true;
    }
    else if(param == "output_nav_heading") {
      m_output_nav_heading = value;
      handled = true;
    }
    else if (param == "debug")
      {
        m_debug = (value == tolower("true")) ? true : false;
        if (m_debug)
        {
          time_t rawtime;
          struct tm *timeinfo;
          memset(m_fname, m_fname_buff_size, '\0');
          time(&rawtime);
          timeinfo = localtime(&rawtime);
          char fmt[m_fname_buff_size];
          memset(fmt, m_fname_buff_size, '\0');
          strftime(fmt, m_fname_buff_size, "%F_%T", timeinfo);
          snprintf(m_fname, m_fname_buff_size, "DBG_%s_%s_DATA.dbg",
                   m_app_name.c_str(), fmt);
        }
        handled = true;
      }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  // Configure the EKF with loaded parameters
  m_ekf.setProcessNoise(m_pos_process_variance, m_speed_process_variance, 
                        m_heading_process_variance, m_bias_process_variance);
  m_ekf.setMeasurementNoise(m_gps_pos_stddev, m_gps_speed_stddev, 
                            m_mag_meas_stddev * M_PI / 180.0, 
                            m_bias_meas_stddev * M_PI / 180.0);
  m_ekf.setSpeedThreshold(m_speed_threshold);
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void BB_EKF::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  
  // Input variables
  Register(m_input_gps_x, 0);
  Register(m_input_gps_y, 0);
  Register(m_input_gps_speed, 0);
  Register(m_input_gps_heading, 0);
  Register(m_input_mag_heading, 0);
  Register(m_input_gyro_z, 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool BB_EKF::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "BB_EKF Status Report                        " << endl;
  m_msgs << "============================================" << endl;

  m_msgs << "\nFilter Status: " << (m_ekf.isInitialized() ? "INITIALIZED" : "WAITING FOR DATA") << endl;
  if (m_ekf.isInitialized()) {
    m_msgs << "Filter Health: " << (m_ekf.isHealthy() ? "HEALTHY" : "UNHEALTHY") << endl;
  }
  
  m_msgs << "\nState Estimate:" << endl;
  ACTable state_tab(4);
  state_tab << "Variable | Value | Units | Description";
  state_tab.addHeaderLines();
  
  if (m_ekf.isInitialized()) {
    state_tab << "X" << doubleToStringX(m_ekf.getX(), 2) << "m" << "Position East";
    state_tab << "Y" << doubleToStringX(m_ekf.getY(), 2) << "m" << "Position North";
    state_tab << "Speed" << doubleToStringX(m_ekf.getSpeed(), 2) << "m/s" << "Speed over ground";
    
    // Convert heading to compass for display
    double hdg_cart = m_ekf.getHeading();
    double hdg_compass = wrapAngle(M_PI/2.0 - hdg_cart) * 180.0 / M_PI;
    if (hdg_compass < 0) hdg_compass += 360.0;
    state_tab << "Heading" << doubleToStringX(hdg_compass, 1) << "deg" << "True heading (compass)";
    
    state_tab << "Bias" << doubleToStringX(m_ekf.getBias() * 180.0 / M_PI, 1) << "deg" << "Mag-GPS heading bias";
  } else {
    state_tab << "--" << "--" << "--" << "Waiting for initialization";
  }
  m_msgs << state_tab.getFormattedString();
  
  m_msgs << "\nDetailed Sensor Status:" << endl;
  double current_time = MOOSTime();
  
  ACTable sensor_tab(4);
  sensor_tab << "Sensor | Ready | Last Value | Age(s)";
  sensor_tab.addHeaderLines();
  
  sensor_tab << "GPS Position" << (m_nav_pos_gnss.ready() ? "YES" : "NO") 
             << "(" + doubleToStringX(m_nav_pos_gnss.x, 1) + ", " + doubleToStringX(m_nav_pos_gnss.y, 1) + ")"
             << doubleToStringX(current_time - m_nav_pos_gnss.timestamp, 2);
  sensor_tab << "GPS Speed" << (m_nav_speed_gnss.ready() ? "YES" : "NO") 
             << doubleToStringX(m_nav_speed_gnss.speed, 2) + " m/s"
             << doubleToStringX(current_time - m_nav_speed_gnss.timestamp, 2);
  sensor_tab << "GPS Heading" << (m_nav_heading_gnss.ready() ? "YES" : "NO") 
             << doubleToStringX(wrapAngle(M_PI/2.0 - m_nav_heading_gnss.heading) * 180.0 / M_PI, 1) + " deg"
             << doubleToStringX(current_time - m_nav_heading_gnss.timestamp, 2);
  sensor_tab << "Mag Heading" << (m_mag_heading.ready() ? "YES" : "NO") 
             << doubleToStringX(wrapAngle(M_PI/2.0 - m_mag_heading.heading) * 180.0 / M_PI, 1) + " deg"
             << doubleToStringX(current_time - m_mag_heading.timestamp, 2);
  sensor_tab << "Gyro Z" << (m_gyro_z.ready() ? "YES" : "NO") 
             << doubleToStringX(m_gyro_z.gz_lvl, 4) + " rad/s"
             << doubleToStringX(current_time - m_gyro_z.timestamp, 2);
  
  m_msgs << sensor_tab.getFormattedString();
  
  m_msgs << "\nEKF Debug Info:" << endl;
  ACTable debug_tab(2);
  debug_tab << "Parameter | Value";
  debug_tab.addHeaderLines();
  
  if (m_ekf.isInitialized()) {
    double bias_deg = m_ekf.getBias() * 180.0 / M_PI;
    debug_tab << "Bias (deg)" << doubleToStringX(bias_deg, 1);
    debug_tab << "Bias Constrained" << (fabs(bias_deg) >= 29.9 ? "YES" : "NO");
    debug_tab << "Speed Threshold" << (m_ekf.getSpeed() <= m_speed_threshold ? "ACTIVE" : "INACTIVE");
    
    // Check for reverse motion detection if we have both headings
    if (m_nav_heading_gnss.ready() && m_mag_heading.ready()) {
      double heading_diff = wrapAngle(m_nav_heading_gnss.heading - m_mag_heading.heading);
      bool reverse_detected = (fabs(heading_diff) > M_PI/2.0);
      debug_tab << "Reverse Motion" << (reverse_detected ? "DETECTED" : "NORMAL");
      debug_tab << "Heading Diff (deg)" << doubleToStringX(heading_diff * 180.0 / M_PI, 1);
    } else {
      debug_tab << "Reverse Motion" << "NO DATA";
      debug_tab << "Heading Diff (deg)" << "NO DATA";
    }
    
    // Covariance health check
    double max_var = 0.0;
    for (int i = 0; i < 5; i++) {
      double var = m_ekf.getCovariance()(i, i);
      if (var > max_var) max_var = var;
    }
    debug_tab << "Max Covariance" << doubleToStringX(max_var, 4);
    debug_tab << "Covariance Healthy" << (max_var < 100.0 ? "YES" : "NO");
  } else {
    debug_tab << "EKF Status" << "NOT INITIALIZED";
  }
  
  m_msgs << debug_tab.getFormattedString();
  
  m_msgs << "\nConfiguration:" << endl;
  m_msgs << "  Speed Threshold: " << m_speed_threshold << " m/s" << endl;
  m_msgs << "  Max Bias Constraint: ±30 degrees" << endl;
  if (m_ekf.isInitialized()) {
    m_msgs << "  Using Mag+Bias: " << (m_ekf.getSpeed() <= m_speed_threshold ? "YES" : "NO") << endl;
    m_msgs << "  Bias Updates Active: " << (m_ekf.getSpeed() > m_speed_threshold ? "YES" : "NO") << endl;
  }

  return(true);
}




