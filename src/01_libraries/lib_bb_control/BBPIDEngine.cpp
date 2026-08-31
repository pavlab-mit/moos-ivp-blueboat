/************************************************************/
/*    NAME: Karan Mahesh                                    */
/*    ORGN: MIT / Project Greece                            */
/*    FILE: BBPIDEngine.cpp                                 */
/*    DATE: 2026/06/19                                      */
/************************************************************/

#include <cmath>
#include "BBPIDEngine.h"

using namespace std;

//---------------------------------------------------------
// angle180: verbatim lib_geometry/AngleUtils.cpp semantics,
// replicated locally so lib_bb_control stays free of the
// geometry->mbutil static-link chain. Wraps into (-180, 180].

static double angle180(double degval)
{
  while(degval > 180)
    degval -= 360.0;
  while(degval <= -180)
    degval += 360.0;
  return(degval);
}

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
  m_ff_c0 = m_ff_cv = m_ff_crr = m_ff_cvv = 0.0;
  m_ff_d0 = m_ff_dr = m_ff_dvr = 0.0;
  m_ff_rudder_scale = 1.0;
  m_ff_thrust = m_ff_rudder = 0.0;

  m_des_yawrate_tau   = 0.0;     // desired-yaw-rate FF LPF time const (0 = off)
  m_des_yawrate_filt  = 0.0;
  m_des_lpf_prev_time = 0.0;
  m_have_des_lpf      = false;
  m_prev_des_heading  = 0.0;
  m_des_chg_time      = 0.0;
  m_des_yawrate_raw   = 0.0;

  // Yaw-priority speed governor: off until configured.
  m_yaw_priority_gain = 0.0;
  m_yaw_priority_knee = 0.6;
  m_min_speed_frac    = 0.4;
  m_derate_tau        = 0.5;
  m_speed_derate      = 1.0;
  m_speed_derate_filt = 1.0;
  m_gov_speed_cmd     = 0.0;
  m_gov_prev_time     = 0.0;
  m_have_gov_time     = false;

  m_speed_error    = 0.0;
  m_heading_error  = 0.0;
  m_yawrate_error  = 0.0;
  m_desired_yawrate= 0.0;
  m_meas_yawrate   = 0.0;
  m_out_thrust     = 0.0;
  m_out_rudder     = 0.0;

  // Integration lifecycle: legacy behavior by default (no dt cap, always
  // integrating, no reported rails) so a parity replay against the old
  // engine is clean; the mission plug switches the fixes on.
  m_prev_update_time = 0.0;
  m_have_update_time = false;
  m_max_dt           = 0.0;
  m_integrate_enable = true;
  m_antiwindup       = false;
  m_ext_sat_surge = m_ext_sat_yaw = 0;
  m_rail_thrust   = m_rail_rudder = 0;

  m_ff_hold_time      = 0.0;   // 0 = legacy hold-forever
  m_ff_step_limit_deg = 0.0;   // 0 = no step limit

  // Sensible defaults; expect these to be overwritten from the .moos block.
  m_speed_pid.set_gains(20.0, 5.0, 0.0);
  m_speed_pid.set_limits(50.0, m_max_thrust);

  m_heading_pid.set_gains(1.0, 0.0, 0.0);  // P-only by default (hdg->rate)
  m_heading_pid.set_limits(m_max_yawrate, m_max_yawrate);

  m_yawrate_pid.set_gains(2.0, 0.0, 0.1);  // rate->rudder
  m_yawrate_pid.set_limits(50.0, m_max_rudder);
}

//---------------------------------------------------------
// Gain / limit setters (bb::Pid takes the natural Kp,Ki,Kd order)

void BBPIDEngine::setSpeedGains(double kp, double ki, double kd)
{ m_speed_pid.set_gains(kp, ki, kd); }

void BBPIDEngine::setHeadingGains(double kp, double ki, double kd)
{ m_heading_pid.set_gains(kp, ki, kd); }

void BBPIDEngine::setYawRateGains(double kp, double ki, double kd)
{ m_yawrate_pid.set_gains(kp, ki, kd); }

void BBPIDEngine::setSpeedLimits(double il, double ol)
{ m_speed_pid.set_limits(il, ol); m_max_thrust = ol; m_speed_ilim = il; }

void BBPIDEngine::setHeadingLimits(double il, double ol)
{ m_heading_pid.set_limits(il, ol); m_max_yawrate = ol; }

void BBPIDEngine::setYawRateLimits(double il, double ol)
{ m_yawrate_pid.set_limits(il, ol); m_max_rudder = ol; m_yawrate_ilim = il; }

void BBPIDEngine::setMaxDt(double sec)
{
  m_max_dt = sec;
  m_speed_pid.set_max_dt(sec);
  m_heading_pid.set_max_dt(sec);
  m_yawrate_pid.set_max_dt(sec);
}

void BBPIDEngine::setIntegrateEnable(bool on)
{
  m_integrate_enable = on;
  m_speed_pid.set_integrate_enable(on);
  m_heading_pid.set_integrate_enable(on);
  m_yawrate_pid.set_integrate_enable(on);
}

void BBPIDEngine::setAntiWindup(bool on)
{
  m_antiwindup = on;
  m_speed_pid.set_antiwindup_enable(on);
  m_heading_pid.set_antiwindup_enable(on);
  m_yawrate_pid.set_antiwindup_enable(on);
  if(!on) {
    m_rail_thrust = m_rail_rudder = 0;
    m_speed_pid.set_external_saturation(0);
    m_heading_pid.set_external_saturation(0);
    m_yawrate_pid.set_external_saturation(0);
  }
}

//---------------------------------------------------------
// resetIntegrators(): clear the accumulated integral (and derivative
// history) of all three PID loops, plus the engine's dt clock and
// observed rails. Use on command resume or after a big retune. Gains,
// limits and the lifecycle configuration are untouched.

void BBPIDEngine::resetIntegrators()
{
  m_speed_pid.reset();
  m_heading_pid.reset();
  m_yawrate_pid.reset();
  m_have_update_time = false;
  m_rail_thrust = m_rail_rudder = 0;
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

  m_yawrate_pid.set_gains(kp, ki, kd);
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
    m_meas_yawrate = nav_yawrate_raw * m_yawrate_scale * -1.0; // flip sign to match yaw rate convention
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
  // ---------- One dt for all three loops ----------
  // Measured here from consecutive calls; each bb::Pid additionally
  // clamps it to max_dt, so a command gap earns one bounded integral
  // step instead of the whole gap's worth. First call: dt = 0 (no
  // integration, same as ScalarPID's first iteration).
  double dt = 0.0;
  if(m_have_update_time && curr_time > m_prev_update_time)
    dt = curr_time - m_prev_update_time;
  m_prev_update_time = curr_time;
  m_have_update_time = true;

  // ---------- Downstream rails (anti-windup family) ----------
  // A loop must not wind into a rail the water never sees past. Two
  // sources, OR'd: the app-reported mixer saturation (contract frame),
  // and last tick's post-FF output clamps observed right here. Yaw
  // rails map through rudder_polarity into the loop frame (the loops
  // run pre-polarity); polarity is +/-1 so multiplication is the map.
  if(m_antiwindup) {
    int surge_rail = (m_ext_sat_surge != 0) ? m_ext_sat_surge : m_rail_thrust;
    int yaw_rail_cmd = (m_ext_sat_yaw != 0) ? m_ext_sat_yaw : m_rail_rudder;
    int yaw_rail = (m_rudder_polarity < 0) ? -yaw_rail_cmd : yaw_rail_cmd;
    m_speed_pid.set_external_saturation(surge_rail);
    m_yawrate_pid.set_external_saturation(yaw_rail);
    // The outer loop feeds the same actuator chain: a rate command that
    // the rudder cannot serve must not wind the heading integral either.
    m_heading_pid.set_external_saturation(yaw_rail);
  }

  // ---------- Gain scheduling ----------
  // Retune the inner yaw-rate loop + turn-rate cap for the current
  // (measured) speed before closing the yaw loops.
  if(m_schedule_enabled)
    applySchedule(nav_speed);

  // ---------- Outer heading loop (2-DOF): desired yaw rate ----------
  // des_rate = reference feedforward + feedback, computed first so the
  // turning-drag FF (ff_diff = d0 + dr*des_rate + ...) can use it.
  // Units: heading arrives in degrees (MOOS); the whole yaw-rate path
  // (des_rate, FF, inner loop) is in rad and rad/s.
  m_heading_error = angle180(desired_heading - nav_heading) * (M_PI / 180.0);

  // Feedback: heading-error PID -> corrective yaw rate.
  double fb_rate = m_heading_pid.step(m_heading_error, dt);

  // Feedforward: LPF of the numerical derivative of the DESIRED heading
  // (rad/s) -- the turn rate implied by a moving command. Depends only on
  // the trajectory, not the gains, so the yaw FF/rudder stays alive even
  // with the heading loop zeroed. Mechanics follow pTrajectTranslate: the
  // signed heading step (desired_now - desired_prev) is taken via the
  // unit-vector cross/dot method (robust across the +/-180 wrap), and the
  // impulsive raw derivative of a step command is smoothed with a
  // first-order LPF, alpha = 1 - exp(-dt/tau). tau=0 passes it through.
  double ff_rate = 0.0;
  if(m_have_des_lpf) {
    // ---- Derivative: recompute ONLY when the command actually changes ----
    // DESIRED_HEADING is a staircase at the helm's tick rate, which may be
    // far slower than ours (helm AppTick 1 vs our 16 => 1 change per 16
    // iterates). Differencing on OUR clock yields dpsi=0 on most ticks and a
    // spike of (helm_period/our_period)x the true rate on the rest, which the
    // LPF turns into a ~3x sawtooth that clips against max_yawrate. So
    // difference against the last CHANGED value over the time since that
    // change, and HOLD the result in between.
    if(desired_heading != m_prev_des_heading) {
      double dt_cmd = curr_time - m_des_chg_time;
      if(dt_cmd > 0.0) {
        // signed(desired_now - desired_prev) in radians, a=prev, b=now:
        //   cross = a x b = -sin(now-prev), dot = a . b = cos(now-prev)
        double a1 = sin(m_prev_des_heading * (M_PI / 180.0));
        double a2 = cos(m_prev_des_heading * (M_PI / 180.0));
        double b1 = sin(desired_heading    * (M_PI / 180.0));
        double b2 = cos(desired_heading    * (M_PI / 180.0));
        double cross = a1 * b2 - a2 * b1;
        double dot   = a1 * b1 + a2 * b2;
        if(dot >  1.0) dot =  1.0;          // clip for acos safety
        if(dot < -1.0) dot = -1.0;
        double dpsi = acos(dot) * ((cross >= 0) ? -1.0 : 1.0);  // rad, signed
        // Repositioning guard: a step beyond ff_step_limit is not a
        // trajectory to differentiate. At an exact 180 the cross term
        // is numeric noise, so the FF's SIGN is a coin flip -- on the
        // 27 Aug block it landed against the railed feedback and the
        // boat pivoted at 2 deg/s instead of 25. Publish no FF; the
        // heading loop turns the boat.
        if(m_ff_step_limit_deg > 0.0 &&
           fabs(dpsi) > m_ff_step_limit_deg * (M_PI / 180.0))
          m_des_yawrate_raw = 0.0;
        else
          m_des_yawrate_raw = dpsi / dt_cmd;                    // rad/s
      }
      m_prev_des_heading = desired_heading;
      m_des_chg_time     = curr_time;
    }
    else if(m_ff_hold_time > 0.0 &&
            (curr_time - m_des_chg_time) > m_ff_hold_time) {
      // The command has SETTLED: the derivative a helm staircase
      // implies is over, and holding it forces a standing error of
      // ff/kp (6.4 deg measured on the block). Zero the raw value;
      // the LPF below decays the published FF smoothly.
      m_des_yawrate_raw = 0.0;
    }

    // ---- LPF: steps every iterate, so alpha uses OUR tick dt ----
    double dt = curr_time - m_des_lpf_prev_time;
    if(dt > 0.0) {
      if(m_des_yawrate_tau > 0.0) {
        double alpha = 1.0 - exp(-dt / m_des_yawrate_tau);
        m_des_yawrate_filt += alpha * (m_des_yawrate_raw - m_des_yawrate_filt);
      }
      else
        m_des_yawrate_filt = m_des_yawrate_raw;
    }
    ff_rate = m_des_yawrate_filt;       // dt<=0: hold last filtered value

    // Cap the reference FF at the turn-rate budget. The 2-DOF sum is
    // clamped below regardless; capping the FF ITSELF keeps feedback
    // authoritative inside the clamp instead of buried under an
    // impulsive derivative (when ff exceeds the cap and fb opposes it,
    // min(ff,cap)+fb != min(ff+fb,cap)). Gated with the repositioning
    // guard -- same pathology family, and all-defaults must stay
    // verbatim legacy (a 27 Aug helm-log replay caught this cap, then
    // unconditional, diverging during an end-of-run command jump).
    if(m_ff_step_limit_deg > 0.0) {
      if(ff_rate >  m_max_yawrate) ff_rate =  m_max_yawrate;
      if(ff_rate < -m_max_yawrate) ff_rate = -m_max_yawrate;
    }
  }
  else {
    // First sample: no derivative yet, but seed the change clock so the first
    // real change divides by its own interval and not by the MOOS uptime.
    m_have_des_lpf     = true;
    m_prev_des_heading = desired_heading;
    m_des_chg_time     = curr_time;
  }
  m_des_lpf_prev_time = curr_time;

  // 2-DOF setpoint: reference feedforward + feedback correction, then clamp
  // to the turn-rate cap regardless of source.
  double des_rate = ff_rate + fb_rate;
  if(des_rate >  m_max_yawrate) des_rate =  m_max_yawrate;
  if(des_rate < -m_max_yawrate) des_rate = -m_max_yawrate;
  m_desired_yawrate = des_rate;

  // ---------- Yaw-priority speed governor ----------
  // Path curvature, speed and yaw rate are not independent: holding curvature
  // k at speed v demands r = k*v. Once the commanded rate approaches the
  // vehicle's turn ceiling, no amount of extra rudder tightens the turn (on
  // this hull the rate is flat above ~30% rudder) -- the only remaining
  // control is speed. So spend the speed setpoint to buy heading tracking:
  // derate in proportion to how much of the yaw budget the command is using.
  // The derate is low-passed so a momentary rate transient does not step the
  // throttle. gain = 0 leaves the setpoint untouched (legacy behavior).
  m_gov_speed_cmd = desired_speed;
  if(m_yaw_priority_gain > 0.0 && m_max_yawrate > 0.0) {
    double demand = fabs(des_rate) / m_max_yawrate;      // [0,1]
    m_speed_derate = 1.0;
    if(demand > m_yaw_priority_knee) {
      double span = 1.0 - m_yaw_priority_knee;
      double x = (span > 0.0) ? (demand - m_yaw_priority_knee) / span : 1.0;
      if(x > 1.0) x = 1.0;
      m_speed_derate = 1.0 - m_yaw_priority_gain * x;
    }
    if(m_speed_derate < m_min_speed_frac) m_speed_derate = m_min_speed_frac;
    if(m_speed_derate > 1.0)              m_speed_derate = 1.0;

    double dt_gov = m_have_gov_time ? (curr_time - m_gov_prev_time) : 0.0;
    if(m_derate_tau > 0.0 && dt_gov > 0.0) {
      double alpha = 1.0 - exp(-dt_gov / m_derate_tau);
      m_speed_derate_filt += alpha * (m_speed_derate - m_speed_derate_filt);
    }
    else if(!m_have_gov_time || m_derate_tau <= 0.0)
      m_speed_derate_filt = m_speed_derate;

    m_gov_speed_cmd = desired_speed * m_speed_derate_filt;
  }
  else {
    m_speed_derate      = 1.0;
    m_speed_derate_filt = 1.0;
  }
  m_gov_prev_time = curr_time;
  m_have_gov_time = true;

  // ---------- Feedforward (identified from field data) ----------
  // Static thrust split needed to hold the DESIRED (v*, r*):
  //   common c = c0 + cv*v* + crr*r*^2      -> adds to DESIRED_THRUST
  //   diff   d = d0 + dr*r* + dvr*v* * r*   -> adds to DESIRED_RUDDER (scaled)
  // The PIDs then only correct the residual.
  m_ff_thrust = 0.0;
  m_ff_rudder = 0.0;
  if(m_ff_enable) {
    // Both FF terms use the GOVERNED speed -- that is the speed the boat is
    // actually being asked to hold, so it is the right operating point for
    // the thrust trim and for the speed-dependent yaw gain.
    if(m_ff_speed_enable)
      m_ff_thrust = m_ff_c0 + m_ff_cv * m_gov_speed_cmd
                  + m_ff_cvv * m_gov_speed_cmd * fabs(m_gov_speed_cmd)
                  + m_ff_crr * des_rate * des_rate;
    if(m_ff_yaw_enable) {
      double ff_diff = m_ff_d0 + m_ff_dr * des_rate
                     + m_ff_dvr * m_gov_speed_cmd * des_rate;
      m_ff_rudder = m_ff_rudder_scale * ff_diff;
    }
  }

  // ---------- Speed loop (true positional PID) + speed FF ----------
  m_speed_error = m_gov_speed_cmd - nav_speed;

  double thrust = m_speed_pid.step(m_speed_error, dt);
  thrust += m_ff_thrust;

  // The PID's own clamp cannot see the FF added above, so observe the
  // POST-FF rails here and report them back next tick (the loop frame
  // and the thrust frame are the same sign). The allow_reverse floor
  // is a rail at zero from below: integrating further down is windup.
  m_rail_thrust = 0;
  if(!m_allow_reverse && thrust < 0.0) {
    thrust = 0.0;
    m_rail_thrust = -1;
  }
  if(thrust >=  m_max_thrust) { thrust =  m_max_thrust; m_rail_thrust =  1; }
  if(thrust <= -m_max_thrust) { thrust = -m_max_thrust; m_rail_thrust = -1; }

  // ---------- Inner yaw loop: yaw-rate error -> rudder ----------
  computeMeasYawRate(curr_time, nav_heading, nav_yawrate_raw);
  m_yawrate_error = des_rate - m_meas_yawrate;

  double rudder = m_yawrate_pid.step(m_yawrate_error, dt);

  // rudder_polarity reverses the ENTIRE command (PID + feedforward) to match
  // the mixer/hardware convention -- set rudder_polarity = -1 to flip.
  rudder = (rudder + m_ff_rudder) * m_rudder_polarity;
  m_rail_rudder = 0;
  if(rudder >=  m_max_rudder) { rudder =  m_max_rudder; m_rail_rudder =  1; }
  if(rudder <= -m_max_rudder) { rudder = -m_max_rudder; m_rail_rudder = -1; }

  m_out_thrust = thrust;
  m_out_rudder = rudder;
  out_thrust   = thrust;
  out_rudder   = rudder;
}
