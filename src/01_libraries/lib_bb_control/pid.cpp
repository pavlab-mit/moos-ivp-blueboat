/*************************************************************
 *  pid.cpp -- see pid.h for the design and parity contract.
 *
 *  Author: Jeremy Wenger
 *************************************************************/

#include "pid.h"

#include <cmath>

namespace bb {

//---------------------------------------------------------
Pid::Pid()
{
  m_kp = m_ki = m_kd = 0.0;
  m_i_lim   = 0.0;
  m_out_lim = 0.0;
  m_max_dt  = 0.0;
  m_integrate_enable = true;
  m_aw_enable = false;       // legacy: integral clamps only at i_lim
  m_ext_sat = 0;

  m_i = 0.0;
  m_prev_error = 0.0;
  m_d = 0.0;
  m_have_prev = false;
  m_out = 0.0;

  m_p = m_dterm = 0.0;
  m_saturated  = false;
  m_integrated = false;
}

//---------------------------------------------------------
void Pid::set_gains(double kp, double ki, double kd)
{
  m_kp = kp;
  m_ki = ki;
  m_kd = kd;
}

void Pid::set_limits(double integral_limit, double output_limit)
{
  m_i_lim   = integral_limit;
  m_out_lim = output_limit;
}

void Pid::set_max_dt(double max_dt_sec)
{
  m_max_dt = max_dt_sec;
}

void Pid::set_integrate_enable(bool on)
{
  m_integrate_enable = on;
}

void Pid::set_antiwindup_enable(bool on)
{
  m_aw_enable = on;
}

void Pid::set_external_saturation(int sign)
{
  m_ext_sat = (sign > 0) ? 1 : (sign < 0) ? -1 : 0;
}

//---------------------------------------------------------
void Pid::reset()
{
  m_i = 0.0;
  m_prev_error = 0.0;
  m_d = 0.0;
  m_have_prev = false;
  m_out = 0.0;
  m_p = m_dterm = 0.0;
  m_saturated  = false;
  m_integrated = false;
}

void Pid::reset_integral()
{
  m_i = 0.0;
}

//---------------------------------------------------------
double Pid::step(double error, double dt)
{
  return step_impl(error, dt, false, 0.0);
}

double Pid::step(double error, double d_error, double dt)
{
  return step_impl(error, dt, true, d_error);
}

//---------------------------------------------------------
// step_impl: ScalarPID's operation order -- integrate first,
// output includes this tick's increment, then clamp -- so that a
// clean tick is arithmetically identical to the class this
// replaces. The anti-windup is a ROLLBACK of the increment when
// the clamped output railed in the increment's direction: on
// clean segments nothing differs, at a rail the integral stops
// climbing instead of parking at its limit.

double Pid::step_impl(double error, double dt, bool have_d, double d_in)
{
  // Fail closed: never let a NaN walk into an integral or an
  // actuator command. Hold the last output (SlewLimiter policy).
  if (!std::isfinite(error) || !std::isfinite(dt) ||
      (have_d && !std::isfinite(d_in)))
    return m_out;

  if (dt < 0.0) dt = 0.0;
  if (m_max_dt > 0.0 && dt > m_max_dt) dt = m_max_dt;

  // ---- Derivative ----
  // Caller-supplied when given; else a single backward difference.
  // No sample to difference against (first call, or dt = 0): hold
  // the previous value rather than inventing a spike.
  double d = m_d;
  if (have_d)
    d = d_in;
  else if (m_have_prev && dt > 0.0)
    d = (error - m_prev_error) / dt;

  // ---- Integral (candidate) ----
  // The first step after construction or reset() establishes state
  // and integrates nothing -- ScalarPID's first-iteration behavior,
  // which the engine's resume-reset contract depends on.
  const double i_prev = m_i;
  m_integrated = false;
  if (m_ki > 0.0) {
    // Skip the increment when it would wind further into a rail
    // the caller reported downstream. sign(increment) = sign(e).
    const bool into_ext_rail = (m_ext_sat != 0) && (error * m_ext_sat > 0.0);
    if (m_integrate_enable && dt > 0.0 && m_have_prev && !into_ext_rail) {
      m_i += m_ki * error * dt;
      if (std::fabs(m_i) >= std::fabs(m_i_lim))
        m_i = (m_i >= 0.0) ? std::fabs(m_i_lim) : -std::fabs(m_i_lim);
      m_integrated = (m_i != i_prev);
    }
  }
  else {
    // ScalarPID semantics: no Ki, no memory.
    m_i = 0.0;
  }

  m_p     = m_kp * error;
  m_dterm = m_kd * d;
  double out = m_p + m_dterm + m_i;

  // ---- Output clamp + tracking anti-windup ----
  // When the clamped output railed in the increment's direction,
  // keep exactly as much of this tick's increment as the output can
  // use and refuse the excess: I tracks to (rail - P - D), bounded
  // by [previous, candidate] so it never runs backwards and never
  // overshoots what was actually accrued. The integral therefore
  // parks AT the rail's worth of authority instead of at its own
  // limit -- and comes off the rail on the first reversed tick.
  m_saturated = false;
  if (m_out_lim > 0.0 && std::fabs(out) >= m_out_lim) {
    const double rail = (out >= 0.0) ? 1.0 : -1.0;
    m_saturated = true;
    if (m_aw_enable && m_integrated && (error * rail > 0.0)) {
      double target = rail * m_out_lim - m_p - m_dterm;
      const double lo = (i_prev < m_i) ? i_prev : m_i;
      const double hi = (i_prev < m_i) ? m_i : i_prev;
      if (target < lo) target = lo;
      if (target > hi) target = hi;
      m_integrated = (target != i_prev);
      m_i = target;
      out = m_p + m_dterm + m_i;
    }
    if (std::fabs(out) >= m_out_lim)
      out = rail * m_out_lim;
    else
      m_saturated = false;   // adjustment brought it back inside
  }

  m_prev_error = error;
  m_have_prev  = true;
  m_d   = d;
  m_out = out;
  return out;
}

//---------------------------------------------------------
std::string Pid::validate() const
{
  if (!std::isfinite(m_kp) || !std::isfinite(m_ki) || !std::isfinite(m_kd))
    return "non-finite gain";
  if (!std::isfinite(m_i_lim) || m_i_lim < 0.0)
    return "integral_limit must be finite and >= 0";
  if (!std::isfinite(m_out_lim))
    return "output_limit must be finite";
  if (!std::isfinite(m_max_dt))
    return "max_dt must be finite";
  if (m_ki > 0.0 && m_i_lim == 0.0)
    return "Ki > 0 with integral_limit 0: the integral can never act";
  return "";
}

} // namespace bb
