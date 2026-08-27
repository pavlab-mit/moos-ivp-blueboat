#include "esc_mapper.h"

#include <cmath>

namespace bb {

std::string EscChannelConfig::validate() const
{
  if (!std::isfinite(min_us) || !std::isfinite(trim_us) || !std::isfinite(max_us))
    return "ESC endpoints must all be finite";
  if (!(min_us < trim_us && trim_us < max_us))
    return "ESC endpoints must satisfy min < trim < max";
  if (min_us < 500.0 || max_us > 2500.0)
    return "ESC endpoints outside the plausible servo-PWM band [500, 2500] us";
  return "";
}

double esc_neutral_us(const EscChannelConfig& cfg)
{
  return cfg.trim_us;
}

EscOutput esc_map(double effort_pct, const EscChannelConfig& cfg)
{
  EscOutput out;
  out.clamped = false;

  if (!std::isfinite(effort_pct)) {
    out.physical_effort   = 0.0;
    out.electrical_effort = 0.0;
    out.pulse_us          = cfg.trim_us;
    out.clamped           = true;
    return out;
  }

  double physical = effort_pct;
  if (physical < -100.0) { physical = -100.0; out.clamped = true; }
  if (physical >  100.0) { physical =  100.0; out.clamped = true; }

  const double electrical = cfg.reversed ? -physical : physical;
  const double u = electrical * 0.01;   // [-1, 1]

  // Piecewise about trim: the two spans differ, so a single
  // symmetric scale would put full reverse in the wrong place.
  double pulse;
  if (u >= 0.0) pulse = cfg.trim_us + u * (cfg.max_us - cfg.trim_us);
  else          pulse = cfg.trim_us + u * (cfg.trim_us - cfg.min_us);

  out.physical_effort   = physical;
  out.electrical_effort = electrical;
  out.pulse_us          = pulse;
  return out;
}

} // namespace bb
