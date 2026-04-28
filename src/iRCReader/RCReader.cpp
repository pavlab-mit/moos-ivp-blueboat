/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: iRCReader/RCReader.cpp
   Last Ed: 2025-03-20
     Brief:
        MOOS App for reading and publishing RC controller data
        from SBUS protocol. Maps channels to simplified states
        and detects controller connectivity.
*************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "RCReader.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

RCReader::RCReader()
{
  m_running = true;
  m_debug = false;
  m_last_update_time = 0;

  // Initialize channel values
  for (int i = 0; i < SBUS_NUM_CHANNELS; i++) {
    m_channels[i] = 0;
    m_scaled_channels[i] = 0;
  }

  // Set initial connection status
  m_rc_connected = false;
  m_frame_valid  = false;
}

//---------------------------------------------------------
// Destructor

RCReader::~RCReader()
{
  // Signal thread to stop and wait for it to join
  m_running = false;
  if (m_sbus_thread.joinable()) {
    m_sbus_thread.join();
    dbg_print("SBUS thread joined on shutdown\n");
  }
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool RCReader::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

     if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }

   return(true);
}

//---------------------------------------------------------
// Procedure: dbg_print()
bool RCReader::dbg_print(const char *format, ...)
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
      va_end(args);
      return true;
    }
    else
    {
      reportRunWarning("Debug mode is enabled and file could not be opened\n");
      va_end(args);
      return false;
    }
  }
  return false;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool RCReader::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Thread function for reading SBUS data
void RCReader::SbusThreadFunction()
{
  while (m_running) {
    // Process available SBUS data
    bool new_frame = m_sbus.update();

    if (new_frame) {
      // Lock mutex to update shared data
      std::lock_guard<std::mutex> lock(m_mutex);

      // Update last update time
      m_last_update_time = MOOSTime();

      // Update channel values
      for (int i = 0; i < SBUS_NUM_CHANNELS; i++) {
        m_channels[i] = m_sbus.getChannel(i);
      }

      // Update flags
      m_channel_17 = m_sbus.getChannel17();
      m_channel_18 = m_sbus.getChannel18();
      m_frame_lost = m_sbus.isFrameLost();
      m_failsafe = m_sbus.isFailsafe();

      dbg_print("New SBUS frame received\n");
    }

    // Small delay to reduce CPU usage
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

//---------------------------------------------------------
// Scale joystick value from SBUS range to -100 to 100
double RCReader::ScaleJoystick(uint16_t value)
{
  // Ensure value is in valid range
  if (value < SBUS_MIN_VALUE) value = SBUS_MIN_VALUE;
  if (value > SBUS_MAX_VALUE) value = SBUS_MAX_VALUE;

  // Calculate center point and range
  double center = (SBUS_MAX_VALUE + SBUS_MIN_VALUE) / 2.0;
  double range = (SBUS_MAX_VALUE - SBUS_MIN_VALUE) / 2.0;

  // Scale to -100 to 100
  return ((value - center) / range) * 100.0;
}

//---------------------------------------------------------
// Map switch value to discrete states (1 to num_states)
int RCReader::MapSwitch(uint16_t value, int num_states)
{
  // Ensure value is in valid range
  if (value < SBUS_MIN_VALUE) value = SBUS_MIN_VALUE;
  if (value > SBUS_MAX_VALUE) value = SBUS_MAX_VALUE;

  // Calculate state range
  double range = (SBUS_MAX_VALUE - SBUS_MIN_VALUE) / num_states;

  // Map to state (1 to num_states)
  for (int i = 0; i < num_states; i++) {
    if (value <= SBUS_MIN_VALUE + range * (i + 1)) {
      return i + 1;
    }
  }

  return num_states;
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool RCReader::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Query SBUS state directly. SbusHandler is the single owner of
  // the connection decision. Two flags with distinct semantics:
  //   m_frame_valid  - per-frame, instantaneous (safety gate)
  //   m_rc_connected - debounced (mode/UI/logging)
  // Downstream consumers can subscribe to either depending on
  // whether they want sharp per-frame response or debounced state.
  m_frame_valid  = m_sbus.isFrameValid();
  m_rc_connected = m_sbus.isControllerConnected();

  Notify("RC_FRAME_VALID", m_frame_valid  ? "true" : "false");
  Notify("RC_CONNECTED",   m_rc_connected ? "true" : "false");

  if (m_rc_connected)
  {
    // Lock mutex to safely access shared data
    std::lock_guard<std::mutex> lock(m_mutex);

    // Process and publish channel data

    // Joystick channels (1-4) mapped to -100 to 100
    for (int i = 0; i < 4; i++) {
      m_scaled_channels[i] = ScaleJoystick(m_channels[i]);
      Notify("RC_CH" + intToString(i+1), m_scaled_channels[i]);
    }

    // Switch channels mapped to discrete states
    // Ch5: Three state switch
    m_scaled_channels[4] = MapSwitch(m_channels[4], 3);
    Notify("RC_CH5", m_scaled_channels[4]);

    // Ch6: Two state switch
    m_scaled_channels[5] = MapSwitch(m_channels[5], 2);
    Notify("RC_CH6", m_scaled_channels[5]);

    // Ch7: Two state switch
    m_scaled_channels[6] = MapSwitch(m_channels[6], 2);
    Notify("RC_CH7", m_scaled_channels[6]);

    // Ch8: Three state switch
    m_scaled_channels[7] = MapSwitch(m_channels[7], 3);
    Notify("RC_CH8", m_scaled_channels[7]);

    // Ch9: Two state switch
    m_scaled_channels[8] = MapSwitch(m_channels[8], 2);
    Notify("RC_CH9", m_scaled_channels[8]);

    // Remaining channels as raw values
    for (int i = 9; i < SBUS_NUM_CHANNELS; i++) {
      m_scaled_channels[i] = m_channels[i];
      Notify("RC_CH" + intToString(i+1), m_channels[i]);
    }

    // Publish flags
    Notify("RC_CH17", m_channel_17 ? "true" : "false");
    Notify("RC_CH18", m_channel_18 ? "true" : "false");
    Notify("RC_FRAME_LOST", m_frame_lost ? "true" : "false");
    Notify("RC_FAILSAFE", m_failsafe ? "true" : "false");
  }
  else
  {
    // === Disconnected fallback: publish safe defaults for ALL
    // === channels.
    //
    // Joystick channels CH1-4 -> 0 (zero thrust commanded).
    // Switch channels   CH5-9 -> 1 (lowest position).
    //   * iBBNavigatorInterface treats CH6 == 2 as "RC mode";
    //     publishing 1 here forces m_rc_mode -> false on
    //     disconnect, which routes thrust through the
    //     autonomy/MOOS path (or thrust_command_timeout) rather
    //     than the RC mixer. This is the desired fail-safe for
    //     scenarios 1 and 2.
    //   * Any consumer that needs to LATCH operator state across
    //     dropouts (e.g. a future mode arbiter that should keep
    //     a vehicle in AUTONOMY across an intentional RC dropout)
    //     MUST gate its reads on RC_CONNECTED instead of trusting
    //     RC_CHx as fresh operator input. The values below are
    //     "operator unavailable" tokens, not commanded positions.
    // Aux/raw channels CH10-16 -> SBUS_MID_VALUE (no command bias).
    //
    // The RC_DEADMAN watchdog in iBBNavigatorInterface refreshes
    // its freshness timestamp ONLY on RC_CONNECTED=true (post-fix
    // to BBNavigatorInterface.cpp), so these RC_CH* publishes do
    // NOT defeat the watchdog.
    Notify("RC_CH1", 0.0);
    Notify("RC_CH2", 0.0);
    Notify("RC_CH3", 0.0);
    Notify("RC_CH4", 0.0);
    Notify("RC_CH5", 1.0);
    Notify("RC_CH6", 1.0);
    Notify("RC_CH7", 1.0);
    Notify("RC_CH8", 1.0);
    Notify("RC_CH9", 1.0);
    for (int i = 9; i < SBUS_NUM_CHANNELS; i++) {
      Notify("RC_CH" + intToString(i+1), (double)SBUS_MID_VALUE);
    }
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool RCReader::OnStartUp()
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
    if (param == "debug")
    {
      m_debug = (tolower(value) == "true") ? true : false;
      if (m_debug)
      {
        time_t rawtime;
        struct tm *timeinfo;
        memset(m_fname, 0, m_fname_buff_size);
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        char fmt[m_fname_buff_size];
        memset(fmt, 0, m_fname_buff_size);
        strftime(fmt, m_fname_buff_size, "%F_%T", timeinfo);
        snprintf(m_fname, m_fname_buff_size, "DBG_%s_%s_DATA.dbg",
                m_app_name.c_str(), fmt);
      }
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  // Initialize SBUS handler
  if (!m_sbus.initialize()) {
    reportRunWarning("Failed to initialize SBUS handler");
  } else {
    dbg_print("SBUS handler initialized\n");
  }

  // Start SBUS processing thread
  m_sbus_thread = std::thread(&RCReader::SbusThreadFunction, this);

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void RCReader::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // No variables to register for now
}


//------------------------------------------------------------
// Procedure: buildReport()

bool RCReader::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "RC Controller Status" << endl;
  m_msgs << "============================================" << endl;

  m_msgs << "Frame Valid (per-frame):    " << (m_frame_valid  ? "YES" : "NO")  << endl;
  m_msgs << "RC Connected (debounced):   " << (m_rc_connected ? "YES" : "NO")  << endl << endl;
  m_msgs << "Failsafe Active: "             << (m_failsafe     ? "YES" : "NO")  << endl;
  m_msgs << "Time Since Last Frame: "       << doubleToStringX(m_sbus.getTimeSinceLastFrame() / 1000.0, 1) << " ms" << endl;
  m_msgs << "Consecutive Frame Losses:   " << m_sbus.getConsecutiveFrameLosses() << endl;
  m_msgs << "Consecutive Good Frames:    " << m_sbus.getConsecutiveGoodFrames() << endl;

  ACTable actab(4);
  actab << "Channel | Description | Raw Value | Mapped Value";
  actab.addHeaderLines();

  // Lock mutex to safely access shared data
  std::lock_guard<std::mutex> lock(m_mutex);

  // Add joystick channels
  actab << "1" << "Right Stick L/R" << m_channels[0] << doubleToStringX(m_scaled_channels[0], 1) + "%";
  actab << "2" << "Right Stick F/B" << m_channels[1] << doubleToStringX(m_scaled_channels[1], 1) + "%";
  actab << "3" << "Left Stick F/B" << m_channels[2] << doubleToStringX(m_scaled_channels[2], 1) + "%";
  actab << "4" << "Left Stick L/R" << m_channels[3] << doubleToStringX(m_scaled_channels[3], 1) + "%";

  // Add switch channels
  actab << "5" << "Top Left Switch (3-pos)" << m_channels[4] << intToString((int)m_scaled_channels[4]);
  actab << "6" << "Top Left Small SW (2-pos)" << m_channels[5] << intToString((int)m_scaled_channels[5]);
  actab << "7" << "Top Left Big SW (2-pos)" << m_channels[6] << intToString((int)m_scaled_channels[6]);
  actab << "8" << "Top Right Big SW (3-pos)" << m_channels[7] << intToString((int)m_scaled_channels[7]);
  actab << "9" << "Top Right Small SW (2-pos)" << m_channels[8] << intToString((int)m_scaled_channels[8]);

  // Add remaining channels
  for (int i = 9; i < SBUS_NUM_CHANNELS; i++) {
    actab << intToString(i+1) << "Channel " + intToString(i+1) << m_channels[i] << m_channels[i];
  }

  m_msgs << actab.getFormattedString();

  // Add flags
  m_msgs << endl << "Flags:" << endl;
  m_msgs << "Channel 17: " << (m_channel_17 ? "ON" : "OFF") << endl;
  m_msgs << "Channel 18: " << (m_channel_18 ? "ON" : "OFF") << endl;
  m_msgs << "Frame Lost: " << (m_frame_lost ? "YES" : "NO") << endl;
  m_msgs << "Failsafe: " << (m_failsafe ? "YES" : "NO") << endl;

  return(true);
}
