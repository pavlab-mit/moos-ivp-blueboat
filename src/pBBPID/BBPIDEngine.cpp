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
  m_speed_ilim     = 50.0;
  m_yawrate_ilim   = 50.0;
  m_yawrate_scale  = 1.0;     // assume feedback already deg/s; flip sign here
  m_rudder_polarity= 1.0;
  m_allow_reverse  = false;

  m_schedule_enabled = false;
  m_sched_speed      = 0.0;

  m_yawrate_derive    = false;
  m_yr_lpf_alpha      = 0.5;
  m_prev_heading      = 0.0;
  m_prev_heading_time = 0.0;
  m_have_prev_heading = false;

  // Feedforward (identified model); disabled until configured.
  m_ff_enable       = false;
  m_ff_speed_enable = true;
  m_ff_yaw_enable   = true;
  m_ff_c0 = m_ff_cv = m_ff_crr = 0.0;
  m_ff_d0 = m_ff_dr = m_ff_dvr = 0.0;
  m_ff_rudder_scale = 1.0;
  m_ff_thrust = m_ff_rudder = 0.0;

  m_des_yawrate_tau   = 0.0;     // desired-yaw-rate LPF time const (0 = off)
  m_des_yawrate_filt  = 0.0;
  m_des_lpf_prev_time = 0.0;
  m_have_des_lpf      = false;

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
{ m_speed_pid.SetLimits(il, ol); m_max_thrust = ol; m_speed_ilim = il; }

void BBPIDEngine::setHeadingLimits(double il, double ol)
{ m_heading_pid.SetLimits(il, ol); m_max_yawrate = ol; }

void BBPIDEngine::setYawRateLimits(double il, double ol)
{ m_yawrate_pid.SetLimits(il, ol); m_max_rudder = ol; m_yawrate_ilim = il; }

//---------------------------------------------------------
// resetIntegrators(): clear the accumulated integral (and derivative
// history) of all three PID loops by rebuilding them with the current
// gains + limits. Use after a big retune or windup to start clean.

static void rebuildPID(ScalarPID& pid, const std::string& name,
                       double ilim, double olim)
{
  double kp = pid.getKP(), kd = pid.getKD(), ki = pid.getKI();
  pid = ScalarPID();                 // clears m_dfeSum, diff history, iter count
  pid.SetGains(kp, kd, ki);          // ScalarPID order is (Kp, Kd, Ki)
  pid.SetLimits(ilim, olim);
  pid.SetName(name);
}

void BBPIDEngine::resetIntegrators()
{
  rebuildPID(m_speed_pid,   "bbpid_speed",   m_speed_ilim,   m_max_thrust);
  rebuildPID(m_heading_pid, "bbpid_heading", m_max_yawrate,  m_max_yawrate);
  rebuildPID(m_yawrate_pid, "bbpid_yawrate", m_yawrate_ilim, m_max_rudder);
}

//---------------------------------------------------------
// addSchedulePoint(): insert a breakpoint, keeping the table
// sorted ascending by speed. A repeated speed overwrites the prior row.

void BBPIDEngine::addSchedulePoint(double speed, double kp, double ki,
                                   double kd, double max_yawrate)
{
  SchedPoint pt;
  pt.speed = speed; pt.kp = kp; pt.ki = ki; pt.kd = kd;
  pt.max_yawrate = max_yawrate;

  size_t i = 0;
  while(i < m_schedule.size() && m_schedule[i].speed < speed)
    i++;
  if(i < m_schedule.size() && m_schedule[i].speed == speed)
    m_schedule[i] = pt;                       // overwrite duplicate
  else
    m_schedule.insert(m_schedule.begin() + i, pt);
}

//---------------------------------------------------------
// applySchedule(): linearly interpolate the yaw-rate gains and the
// turn-rate cap for the current speed, then push them into the loop.

void BBPIDEngine::applySchedule(double speed)
{
  if(m_schedule.empty())
    return;

  m_sched_speed = speed;

  double kp, ki, kd, mxy;

  if(speed <= m_schedule.front().speed) {
    // Below the first breakpoint: hold the first row (no extrapolation).
    const SchedPoint& p = m_schedule.front();
    kp = p.kp; ki = p.ki; kd = p.kd; mxy = p.max_yawrate;
  }
  else if(speed >= m_schedule.back().speed) {
    // Above the last breakpoint: hold the last row.
    const SchedPoint& p = m_schedule.back();
    kp = p.kp; ki = p.ki; kd = p.kd; mxy = p.max_yawrate;
  }
  else {
    // Find the bracketing pair [lo, hi] and lerp each field.
    size_t i = 1;
    while(i < m_schedule.size() && m_schedule[i].speed < speed)
      i++;
    const SchedPoint& lo = m_schedule[i-1];
    const SchedPoint& hi = m_schedule[i];

    double span = hi.speed - lo.speed;
    double t = (span > 0.0) ? (speed - lo.speed) / span : 0.0;

    kp  = lo.kp          + t * (hi.kp          - lo.kp);
    ki  = lo.ki          + t * (hi.ki          - lo.ki);
    kd  = lo.kd          + t * (hi.kd          - lo.kd);
    mxy = lo.max_yawrate + t * (hi.max_yawrate - lo.max_yawrate);
  }

  m_yawrate_pid.SetGains(kp, kd, ki);   // ScalarPID order is (Kp,Kd,Ki)
  setHeadingLimits(mxy, mxy);           // updates outer-loop limit + m_max_yawrate
}

//---------------------------------------------------------
// computeMeasYawRate(): update m_meas_yawrate from the configured source
// (external gyro scaled, or derived from heading: r ~= dpsi/dt, angle-wrapped
// and low-pass filtered). Safe to call every iterate even when the controller
// is idle, so the yaw-rate scope stays live.

double BBPIDEngine::computeMeasYawRate(double curr_time, double nav_heading,
                                       double nav_yawrate_raw)
{
  if(m_yawrate_derive) {
    if(m_have_prev_heading) {
      double dt = curr_time - m_prev_heading_time;
      if(dt > 1e-6) {
        double raw_rate = angle180(nav_heading - m_prev_heading)
                          * (M_PI / 180.0) / dt;                       // rad/s
        m_meas_yawrate  = m_yr_lpf_alpha * raw_rate +
                          (1.0 - m_yr_lpf_alpha) * m_meas_yawrate;
      }
    }
    m_prev_heading      = nav_heading;
    m_prev_heading_time = curr_time;
    m_have_prev_heading = true;
  }
  else {
    m_meas_yawrate = nav_yawrate_raw * m_yawrate_scale;
  }
  return m_meas_yawrate;
}

//---------------------------------------------------------
// update(): one full control tick

void BBPIDEngine::update(double curr_time,
                         double desired_speed,   double nav_speed,
                         double desired_heading, double nav_heading,
                         double nav_yawrate_raw,
                         double& out_thrust, double& out_rudder)
{
  // ---------- Gain scheduling ----------
  // Retune the inner yaw-rate loop + turn-rate cap for the current
  // (measured) speed before closing the yaw loops.
  if(m_schedule_enabled)
    applySchedule(nav_speed);

  // ---------- Outer heading loop: heading error -> desired yaw rate ----------
  // Computed first so the feedforward (turning-drag term) can use des_rate.
  // Units: heading arrives in degrees (MOOS); convert the error to RADIANS so
  // the whole yaw-rate path (des_rate, FF, inner loop) is in rad and rad/s.
  m_heading_error = angle180(desired_heading - nav_heading) * (M_PI / 180.0);

  double des_rate = 0.0;
  m_heading_pid.Run(m_heading_error, curr_time, des_rate);
  if(des_rate >  m_max_yawrate) des_rate =  m_max_yawrate;
  if(des_rate < -m_max_yawrate) des_rate = -m_max_yawrate;

  // Low-pass the desired yaw rate (dt-aware first order, time const tau).
  // Heading-loop / sensor noise here is amplified ~dr-fold in the yaw FF
  // (ff_diff = d0 + dr*des_rate), so smoothing it cleans the rudder FF and
  // the inner-loop setpoint. tau=0 disables.
  if(m_des_yawrate_tau > 0.0) {
    if(m_have_des_lpf) {
      double dt = curr_time - m_des_lpf_prev_time;
      if(dt > 0.0) {
        double alpha = dt / (m_des_yawrate_tau + dt);
        m_des_yawrate_filt += alpha * (des_rate - m_des_yawrate_filt);
      }
    }
    else {
      m_des_yawrate_filt = des_rate;
      m_have_des_lpf = true;
    }
    m_des_lpf_prev_time = curr_time;
    des_rate = m_des_yawrate_filt;
  }
  m_desired_yawrate = des_rate;

  // ---------- Feedforward (identified from field data) ----------
  // Static thrust split needed to hold the DESIRED (v*, r*):
  //   common c = c0 + cv*v* + crr*r*^2      -> adds to DESIRED_THRUST
  //   diff   d = d0 + dr*r* + dvr*v* * r*   -> adds to DESIRED_RUDDER (scaled)
  // The PIDs then only correct the residual.
  m_ff_thrust = 0.0;
  m_ff_rudder = 0.0;
  if(m_ff_enable) {
    if(m_ff_speed_enable)
      m_ff_thrust = m_ff_c0 + m_ff_cv * desired_speed
                  + m_ff_crr * des_rate * des_rate;
    if(m_ff_yaw_enable) {
      double ff_diff = m_ff_d0 + m_ff_dr * des_rate
                     + m_ff_dvr * desired_speed * des_rate;
      m_ff_rudder = m_ff_rudder_scale * ff_diff;
    }
  }

  // ---------- Speed loop (true positional PID) + speed FF ----------
  m_speed_error = desired_speed - nav_speed;

  double thrust = 0.0;
  m_speed_pid.Run(m_speed_error, curr_time, thrust);
  thrust += m_ff_thrust;

  if(!m_allow_reverse && thrust < 0.0)
    thrust = 0.0;
  if(thrust >  m_max_thrust) thrust =  m_max_thrust;
  if(thrust < -m_max_thrust) thrust = -m_max_thrust;

  // ---------- Inner yaw loop: yaw-rate error -> rudder ----------
  computeMeasYawRate(curr_time, nav_heading, nav_yawrate_raw);
  m_yawrate_error = des_rate - m_meas_yawrate;

  double rudder = 0.0;
  m_yawrate_pid.Run(m_yawrate_error, curr_time, rudder);

  // rudder_polarity reverses the ENTIRE command (PID + feedforward) to match
  // the mixer/hardware convention -- set rudder_polarity = -1 to flip.
  rudder = (rudder + m_ff_rudder) * m_rudder_polarity;
  if(rudder >  m_max_rudder) rudder =  m_max_rudder;
  if(rudder < -m_max_rudder) rudder = -m_max_rudder;

  m_out_thrust = thrust;
  m_out_rudder = rudder;
  out_thrust   = thrust;
  out_rudder   = rudder;
}
