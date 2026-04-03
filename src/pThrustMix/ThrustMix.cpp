/************************************************************/
/*    NAME: J. Wenger                                       */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ThrustMix.cpp                                   */
/*    DATE: Oct 19, 2025                                    */
/*    Brief: Differential thrust mixer for BlueBoat        */
/************************************************************/

#include <iterator>
#include <cmath>
#include <algorithm>
#include "MBUtils.h"
#include "ACTable.h"
#include "ThrustMix.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

ThrustMix::ThrustMix()
{
  // Default configuration values
  m_k_inner_base = 2.0;
  m_k_outer_base = 1.0;

  // Speed-dependent scaling (disabled by default)
  m_enable_speed_scaling = false;

  // Yaw rate feedback (disabled by default)
  m_enable_yaw_feedback = false;
  m_k_yaw_correction = 0.0;
  m_max_yaw_rate = 25.0;
  m_k_heading_to_rate = 1.0;

  // Default variable names
  m_desired_thrust_var = "DESIRED_THRUST";
  m_desired_rudder_var = "DESIRED_RUDDER";
  m_feedback_speed_var = "NAV_SPEED";
  m_feedback_yaw_var = "NAV_YAW_RATE";
  m_feedback_heading_var = "NAV_HEADING";
  m_desired_heading_var = "DESIRED_HEADING";
  m_desired_thrust_l_var = "DESIRED_THRUST_L";
  m_desired_thrust_r_var = "DESIRED_THRUST_R";

  // Initialize state variables
  m_desired_thrust = 0.0;
  m_desired_rudder = 0.0;
  m_nav_speed = 0.0;
  m_nav_yaw_rate = 0.0;
  m_nav_heading = 0.0;
  m_desired_heading = 0.0;

  m_mixer = 0.0;
  m_k_inner = m_k_inner_base;
  m_k_outer = m_k_outer_base;
  m_speed_gain_factor = 1.0;
  m_thrust_l = 0.0;
  m_thrust_r = 0.0;

  // Flags
  m_have_thrust = false;
  m_have_rudder = false;
  m_have_speed = false;
  m_have_yaw_rate = false;
  m_have_heading = false;
  m_have_desired_heading = false;
}

//---------------------------------------------------------
// Destructor

ThrustMix::~ThrustMix()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool ThrustMix::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

    if (key == m_desired_thrust_var)
    {
      if (msg.IsDouble())
      {
        m_desired_thrust = msg.GetDouble();
        m_have_thrust = true;
      }
      else
      {
        reportRunWarning(m_desired_thrust_var + " is not a double!");
      }
    }
    else if (key == m_desired_rudder_var)
    {
      if (msg.IsDouble())
      {
        m_desired_rudder = msg.GetDouble();
        m_have_rudder = true;
      }
      else
      {
        reportRunWarning(m_desired_rudder_var + " is not a double!");
      }
    }
    else if (key == m_feedback_speed_var)
    {
      if (msg.IsDouble())
      {
        m_nav_speed = msg.GetDouble();
        m_have_speed = true;
      }
      else
      {
        reportRunWarning(m_feedback_speed_var + " is not a double!");
      }
    }
    else if (key == m_feedback_yaw_var)
    {
      if (msg.IsDouble())
      {
        m_nav_yaw_rate = msg.GetDouble();
        // Convert from rad/s to deg/s
        m_nav_yaw_rate = m_nav_yaw_rate * (180.0 / M_PI);
        m_have_yaw_rate = true;
      }
      else
      {
        reportRunWarning(m_feedback_yaw_var + " is not a double!");
      }
    }
    else if (key == m_feedback_heading_var)
    {
      if (msg.IsDouble())
      {
        m_nav_heading = msg.GetDouble();
        m_have_heading = true;
      }
      else
      {
        reportRunWarning(m_feedback_heading_var + " is not a double!");
      }
    }
    else if (key == m_desired_heading_var)
    {
      if (msg.IsDouble())
      {
        m_desired_heading = msg.GetDouble();
        m_have_desired_heading = true;
      }
      else
      {
        reportRunWarning(m_desired_heading_var + " is not a double!");
      }
    }
    else if (key != "APPCAST_REQ") // handled by AppCastingMOOSApp
    {
      reportRunWarning("Unhandled Mail: " + key);
    }
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool ThrustMix::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool ThrustMix::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Only compute if we have the required inputs
  if (!m_have_thrust || !m_have_rudder)
  {
    AppCastingMOOSApp::PostReport();
    return (true);
  }

  // Step 1: Normalize rudder command to [-1, 1]
  m_mixer = m_desired_rudder / 100.0;

  // Step 2: Optional yaw rate feedback correction
  if (m_enable_yaw_feedback && m_have_yaw_rate && m_have_heading && m_have_desired_heading)
  {
    // Compute heading error [-180, 180]
    double heading_error = m_desired_heading - m_nav_heading;
    heading_error = fmod(heading_error + 180.0, 360.0);
    if (heading_error < 0)
      heading_error += 360.0;
    heading_error -= 180.0;

    // Compute desired yaw rate
    double desired_yaw_rate = m_k_heading_to_rate * heading_error;

    // Compute rate error
    double rate_error = desired_yaw_rate - m_nav_yaw_rate;

    // Add correction to mixer
    if (m_max_yaw_rate > 0.0)
    {
      m_mixer += (rate_error / m_max_yaw_rate) * m_k_yaw_correction;
    }
  }

  // Clamp mixer to [-1, 1]
  m_mixer = max(-1.0, min(1.0, m_mixer));

  // Step 3: Speed-dependent gain scaling
  if (m_enable_speed_scaling && m_have_speed)
  {
    m_speed_gain_factor = getSpeedGainFactor(m_nav_speed);
    m_k_inner = m_k_inner_base * m_speed_gain_factor;
    m_k_outer = m_k_outer_base * m_speed_gain_factor;
  }
  else
  {
    m_speed_gain_factor = 1.0;
    m_k_inner = m_k_inner_base;
    m_k_outer = m_k_outer_base;
  }

  // Step 4: Asymmetric differential mixing

  /*
  if (m_mixer > 0.0)
  {
    // Right turn: left thruster increases, right thruster decreases more
    m_thrust_l = m_desired_thrust * (1.0 + m_k_outer * m_mixer);
    m_thrust_r = m_desired_thrust * (1.0 - m_k_inner * m_mixer);
  }
  else if (m_mixer < 0.0)
  {
    // Left turn: left thruster decreases more, right thruster increases
    m_thrust_l = m_desired_thrust * (1.0 + m_k_inner * m_mixer);
    m_thrust_r = m_desired_thrust * (1.0 - m_k_outer * m_mixer);
  }
  else
  {
    // Straight
    m_thrust_l = m_desired_thrust;
    m_thrust_r = m_desired_thrust;
  }

  */

  double turn_left;
  double turn_right;

  if (m_mixer > 0.0)
  {
    // left outer, right inner
    turn_left = m_k_outer * abs(m_mixer);
    turn_right = -m_k_inner * abs(m_mixer);
  }
  else if (m_mixer < 0.0)
  {
    // left inner, right outer
    turn_left = -m_k_inner * abs(m_mixer);
    turn_right = m_k_outer * abs(m_mixer);
  }

  // additive scaling
  m_thrust_l = m_desired_thrust + (turn_left * 100.0);
  m_thrust_r = m_desired_thrust + (turn_right * 100.0);

  // shift back within bounds maintaining difference
  if (m_thrust_l > 100)
  {
    double over_left = m_thrust_l - 100;
    m_thrust_l -= over_left;
    m_thrust_r -= over_left;
  }
  else if (m_thrust_r > 100)
  {
    double over_right = m_thrust_r - 100;
    m_thrust_l -= over_right;
    m_thrust_r -= over_right;
  }
  else if (m_thrust_l < -100)
  {
    double under_left = m_thrust_l + 100;
    m_thrust_l -= under_left;
    m_thrust_r -= under_left;
  }
  else if (m_thrust_r < -100)
  {
    double under_right = m_thrust_r + 100;
    m_thrust_l -= under_right;
    m_thrust_r -= under_right;
  }

  // Step 5: Clamp outputs to motor limits [-100, 100]
  m_thrust_l = max(-100.0, min(100.0, m_thrust_l));
  m_thrust_r = max(-100.0, min(100.0, m_thrust_r));

  // Step 6: Publish outputs
  Notify(m_desired_thrust_l_var, m_thrust_l);
  Notify(m_desired_thrust_r_var, m_thrust_r);

  AppCastingMOOSApp::PostReport();
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool ThrustMix::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++)
  {
    string orig = *p;
    string line = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;

    // Basic mixing gains
    if (param == "k_inner_base")
    {
      m_k_inner_base = atof(value.c_str());
      handled = true;
    }
    else if (param == "k_outer_base")
    {
      m_k_outer_base = atof(value.c_str());
      handled = true;
    }
    // Speed-dependent scaling
    else if (param == "enable_speed_scaling")
    {
      m_enable_speed_scaling = (tolower(value) == "true");
      handled = true;
    }
    else if (param == "speed_gain_points")
    {
      handled = parseSpeedGainPoints(value);
      if (!handled)
        reportConfigWarning("Failed to parse speed_gain_points: " + value);
    }
    // Yaw rate feedback
    else if (param == "enable_yaw_feedback")
    {
      m_enable_yaw_feedback = (tolower(value) == "true");
      handled = true;
    }
    else if (param == "k_yaw_correction")
    {
      m_k_yaw_correction = atof(value.c_str());
      handled = true;
    }
    else if (param == "max_yaw_rate")
    {
      m_max_yaw_rate = atof(value.c_str());
      handled = true;
    }
    else if (param == "k_heading_to_rate")
    {
      m_k_heading_to_rate = atof(value.c_str());
      handled = true;
    }
    // Variable name customization
    else if (param == "desired_thrust_var")
    {
      m_desired_thrust_var = value;
      handled = true;
    }
    else if (param == "desired_rudder_var")
    {
      m_desired_rudder_var = value;
      handled = true;
    }
    else if (param == "feedback_speed_var")
    {
      m_feedback_speed_var = value;
      handled = true;
    }
    else if (param == "feedback_yaw_var")
    {
      m_feedback_yaw_var = value;
      handled = true;
    }
    else if (param == "feedback_heading_var")
    {
      m_feedback_heading_var = value;
      handled = true;
    }
    else if (param == "desired_heading_var")
    {
      m_desired_heading_var = value;
      handled = true;
    }
    else if (param == "desired_thrust_l_var")
    {
      m_desired_thrust_l_var = value;
      handled = true;
    }
    else if (param == "desired_thrust_r_var")
    {
      m_desired_thrust_r_var = value;
      handled = true;
    }

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void ThrustMix::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();

  Register(m_desired_thrust_var, 0);
  Register(m_desired_rudder_var, 0);
  Register(m_feedback_speed_var, 0);

  if (m_enable_yaw_feedback)
  {
    Register(m_feedback_yaw_var, 0);
    Register(m_feedback_heading_var, 0);
    Register(m_desired_heading_var, 0);
  }
}

//------------------------------------------------------------
// Procedure: buildReport()

bool ThrustMix::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "pThrustMix Status Report" << endl;
  m_msgs << "============================================" << endl;

  // Configuration section
  m_msgs << endl << "Configuration:" << endl;
  m_msgs << "  k_inner_base:         " << doubleToString(m_k_inner_base, 2) << endl;
  m_msgs << "  k_outer_base:         " << doubleToString(m_k_outer_base, 2) << endl;
  m_msgs << "  Speed scaling:        " << boolToString(m_enable_speed_scaling) << endl;
  m_msgs << "  Yaw feedback:         " << boolToString(m_enable_yaw_feedback) << endl;

  // Inputs section
  ACTable intab(2);
  intab << "Input" << "Value";
  intab.addHeaderLines();
  intab << "DESIRED_THRUST" << doubleToString(m_desired_thrust, 2);
  intab << "DESIRED_RUDDER" << doubleToString(m_desired_rudder, 2);
  intab << "NAV_SPEED" << doubleToString(m_nav_speed, 2);
  if (m_enable_yaw_feedback)
  {
    intab << "NAV_YAW_RATE" << doubleToString(m_nav_yaw_rate, 2);
    intab << "NAV_HEADING" << doubleToString(m_nav_heading, 2);
    intab << "DESIRED_HEADING" << doubleToString(m_desired_heading, 2);
  }

  m_msgs << endl << intab.getFormattedString();

  // Computed values section
  ACTable comptab(2);
  comptab << "Computed" << "Value";
  comptab.addHeaderLines();
  comptab << "Mixer (normalized)" << doubleToString(m_mixer, 3);
  comptab << "Speed gain factor" << doubleToString(m_speed_gain_factor, 2);
  comptab << "k_inner (active)" << doubleToString(m_k_inner, 2);
  comptab << "k_outer (active)" << doubleToString(m_k_outer, 2);

  m_msgs << endl << comptab.getFormattedString();

  // Outputs section
  ACTable outtab(2);
  outtab << "Output" << "Value";
  outtab.addHeaderLines();
  outtab << "DESIRED_THRUST_L" << doubleToString(m_thrust_l, 2);
  outtab << "DESIRED_THRUST_R" << doubleToString(m_thrust_r, 2);

  m_msgs << endl << outtab.getFormattedString();

  // Status flags
  m_msgs << endl << "Input Status:" << endl;
  m_msgs << "  Have thrust:   " << boolToString(m_have_thrust) << endl;
  m_msgs << "  Have rudder:   " << boolToString(m_have_rudder) << endl;
  m_msgs << "  Have speed:    " << boolToString(m_have_speed) << endl;
  if (m_enable_yaw_feedback)
  {
    m_msgs << "  Have yaw rate: " << boolToString(m_have_yaw_rate) << endl;
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: getSpeedGainFactor()
//   Purpose: Piecewise constant lookup of gain factor based on speed

double ThrustMix::getSpeedGainFactor(double speed)
{
  if (m_speed_gain_table.empty())
    return 1.0;

  // Find the highest speed point that is <= current speed
  double gain = m_speed_gain_table[0].second; // default to first entry

  for (size_t i = 0; i < m_speed_gain_table.size(); i++)
  {
    if (speed >= m_speed_gain_table[i].first)
    {
      gain = m_speed_gain_table[i].second;
    }
    else
    {
      break; // Table should be sorted, so we can stop here
    }
  }

  return gain;
}

//---------------------------------------------------------
// Procedure: parseSpeedGainPoints()
//   Purpose: Parse speed,gain pairs from config string
//   Example: "0.0,1.0, 1.0,1.0, 2.0,1.5"

bool ThrustMix::parseSpeedGainPoints(const std::string& config_str)
{
  m_speed_gain_table.clear();

  // Split by commas
  vector<string> tokens = parseString(config_str, ',');

  // Should have even number of tokens (speed, gain pairs)
  if (tokens.size() % 2 != 0)
  {
    reportConfigWarning("speed_gain_points must have even number of values (speed,gain pairs)");
    return false;
  }

  for (size_t i = 0; i < tokens.size(); i += 2)
  {
    double speed = atof(tokens[i].c_str());
    double gain = atof(tokens[i + 1].c_str());
    m_speed_gain_table.push_back(make_pair(speed, gain));
  }

  // Sort by speed (should already be sorted, but ensure it)
  sort(m_speed_gain_table.begin(), m_speed_gain_table.end());

  return true;
}
