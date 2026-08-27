#include "slew_limiter.h"

#include <cmath>

namespace bb {

SlewLimiter::SlewLimiter(double rate_pct_per_sec, double max_dt_sec)
  : m_rate(rate_pct_per_sec),
    m_max_dt(max_dt_sec),
    m_state(0.0),
    m_limited(false)
{
}

std::string SlewLimiter::validate() const
{
  if (!std::isfinite(m_rate))   return "slew rate is not finite";
  if (!std::isfinite(m_max_dt)) return "slew max_dt is not finite";
  if (m_max_dt <= 0.0)          return "slew max_dt must be > 0";
  return "";
}

double SlewLimiter::update(double request_pct, double dt_sec)
{
  m_limited = false;

  // Fail closed on garbage: hold the last good state rather than
  // letting a NaN reach the mixer (invariant 12).
  if (!std::isfinite(request_pct) || !std::isfinite(dt_sec))
    return m_state;

  // Non-positive dt earns no credit. Two calls in the same tick
  // must not be able to move the output twice.
  if (dt_sec <= 0.0)
    return m_state;

  // ArduPilot semantics: a non-positive rate disables limiting.
  if (m_rate <= 0.0) {
    m_state = request_pct;
    return m_state;
  }

  const double dt = (dt_sec > m_max_dt) ? m_max_dt : dt_sec;
  const double step_max = m_rate * dt;

  const double lo = m_state - step_max;
  const double hi = m_state + step_max;

  if (request_pct < lo) {
    m_state = lo;
    m_limited = true;
  } else if (request_pct > hi) {
    m_state = hi;
    m_limited = true;
  } else {
    m_state = request_pct;
  }

  return m_state;
}

void SlewLimiter::reset(double state_pct)
{
  m_state = std::isfinite(state_pct) ? state_pct : 0.0;
  m_limited = false;
}

} // namespace bb
