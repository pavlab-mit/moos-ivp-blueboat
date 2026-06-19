/************************************************************/
/*    NAME: Karan Mahesh                                    */
/*    ORGN: MIT / Project Greece                            */
/*    FILE: BBPIDEngine.cpp                                 */
/*    DATE: 2026/06/19                                      */
/************************************************************/

#include <cmath>
#include "BBPIDEngine.h"
#include "AngleUtils.h"   // angle180()

using namespace std;

//---------------------------------------------------------
// Constructor

BBPIDEngine::BBPIDEngine()
{
  m_max_thrust     = 100.0;
  m_max_rudder     = 100.0;
  m_max_yawrate    = 25.0;    // deg/s
  m_yawrate_scale  = 1.0;     // assume feedback already deg/s; flip sign here
  m_rudder_polarity= 1.0;
  m_allow_reverse  = false;

  m_speed_error    = 0.0;
  m_heading_error  = 0.0;
  m_yawrate_error  = 0.0;
  m_desired_yawrate= 0.0;
  m_meas_yawrate   = 0.0;
  m_out_thrust     = 0.0;
  m_out_rudder     = 0.0;

  // Sensible defaults; expect these to be overwritten from the .moos block.
  // NOTE: ScalarPID::SetGains argument order is (Kp, Kd, Ki).
  m_speed_pid.SetGains(20.0, 0.0, 5.0);
  m_speed_pid.SetLimits(50.0, m_max_thrust);
  m_speed_pid.SetName("bbpid_speed");

  m_heading_pid.SetGains(1.0, 0.0, 0.0);   // P-only by default (hdg->rate)
  m_heading_pid.SetLimits(m_max_yawrate, m_max_yawrate);
  m_heading_pid.SetName("bbpid_heading");

  m_yawrate_pid.SetGains(2.0, 0.1, 0.0);   // rate->rudder
  m_yawrate_pid.SetLimits(50.0, m_max_rudder);
  m_yawrate_pid.SetName("bbpid_yawrate");
}

//---------------------------------------------------------
// Gain / limit setters (note Kp,Kd,Ki order into ScalarPID)

void BBPIDEngine::setSpeedGains(double kp, double ki, double kd)
{ m_speed_pid.SetGains(kp, kd, ki); }

void BBPIDEngine::setHeadingGains(double kp, double ki, double kd)
{ m_heading_pid.SetGains(kp, kd, ki); }

void BBPIDEngine::setYawRateGains(double kp, double ki, double kd)
{ m_yawrate_pid.SetGains(kp, kd, ki); }

void BBPIDEngine::setSpeedLimits(double il, double ol)
{ m_speed_pid.SetLimits(il, ol); m_max_thrust = ol; }

void BBPIDEngine::setHeadingLimits(double il, double ol)
{ m_heading_pid.SetLimits(il, ol); m_max_yawrate = ol; }

void BBPIDEngine::setYawRateLimits(double il, double ol)
{ m_yawrate_pid.SetLimits(il, ol); m_max_rudder = ol; }

//---------------------------------------------------------
// update(): one full control tick

void BBPIDEngine::update(double curr_time,
                         double desired_speed,   double nav_speed,
                         double desired_heading, double nav_heading,
                         double nav_yawrate_raw,
                         double& out_thrust, double& out_rudder)
{
  // ---------- Speed loop (true positional PID) ----------
  m_speed_error = desired_speed - nav_speed;

  double thrust = 0.0;
  m_speed_pid.Run(m_speed_error, curr_time, thrust);

  if(!m_allow_reverse && thrust < 0.0)
    thrust = 0.0;
  if(thrust >  m_max_thrust) thrust =  m_max_thrust;
  if(thrust < -m_max_thrust) thrust = -m_max_thrust;

  // ---------- Yaw cascade ----------
  // Outer: heading error -> desired yaw rate (deg/s)
  m_heading_error = angle180(desired_heading - nav_heading);

  double des_rate = 0.0;
  m_heading_pid.Run(m_heading_error, curr_time, des_rate);

  // Clamp commanded turn rate. This is the speed-schedule hook:
  // shrink m_max_yawrate as speed rises to respect the deep hull.
  if(des_rate >  m_max_yawrate) des_rate =  m_max_yawrate;
  if(des_rate < -m_max_yawrate) des_rate = -m_max_yawrate;
  m_desired_yawrate = des_rate;

  // Inner: yaw-rate error -> rudder
  m_meas_yawrate  = nav_yawrate_raw * m_yawrate_scale;
  m_yawrate_error = des_rate - m_meas_yawrate;

  double rudder = 0.0;
  m_yawrate_pid.Run(m_yawrate_error, curr_time, rudder);

  rudder *= m_rudder_polarity;
  if(rudder >  m_max_rudder) rudder =  m_max_rudder;
  if(rudder < -m_max_rudder) rudder = -m_max_rudder;

  m_out_thrust = thrust;
  m_out_rudder = rudder;
  out_thrust   = thrust;
  out_rudder   = rudder;
}
