#include "mixer_stage.h"

#include <cmath>
#include <cstdio>

namespace bb {

std::string MixerConfig::validate() const
{
  if (mixer_model.empty())
    return "mixer_model is required and has no default -- name the equation "
           "that produces the motor pair";
  if (!std::isfinite(selected_cmd_timeout_sec) || selected_cmd_timeout_sec <= 0.0)
    return "selected_cmd_timeout_sec must be finite and > 0";
  if (!std::isfinite(slew_rate_pct_sec) || slew_rate_pct_sec < 0.0)
    return "slew_rate_pct_sec must be finite and >= 0";
  if (!std::isfinite(slew_max_dt_sec) || slew_max_dt_sec <= 0.0)
    return "slew_max_dt_sec must be finite and > 0";
  return skid.validate();
}

MixerStage::MixerStage(const MixerConfig& cfg, const std::string& mix_epoch)
  : m_cfg(cfg),
    m_epoch(mix_epoch),
    m_seq(0),
    m_slew(cfg.slew_rate_pct_sec, cfg.slew_max_dt_sec),
    m_previous_selected(CommandSource::NONE)
{
}

MixedCommand MixerStage::update(double now, double dt,
                                const DecisionMailbox& selected)
{
  MixedCommand m;
  m.mix_epoch   = m_epoch;
  m.mix_seq     = ++m_seq;
  m.mix_time    = now;
  m.mixer_model = m_cfg.mixer_model;
  m.input_age   = selected.age(now);

  const bool have  = selected.has_decision();
  const bool fresh = selected.fresh(now, m_cfg.selected_cmd_timeout_sec);

  // --- Input failure, or an upstream stop -------------------
  //
  // All three collapse to the same actuator outcome, but they get
  // DIFFERENT stop reasons, because they point at different
  // faults: no arbiter at all, an arbiter that has gone quiet, or
  // an arbiter that deliberately stopped. A log that cannot tell
  // those apart cannot be debugged.
  if (!have || !fresh || selected.snapshot().hard_stop) {
    m.hard_stop = true;

    if (!have)       m.stop_reason = StopReason::MIXER_INPUT_INVALID;
    else if (!fresh) m.stop_reason = StopReason::MIXER_INPUT_STALE;
    else             m.stop_reason = selected.snapshot().stop_reason;

    // Invariant 7: bypass the limiter and reset it, so resuming
    // starts from rest rather than from wherever the ramp had got
    // to when the fault hit.
    m_slew.reset(0.0);

    if (have) {
      const DecisionSnapshot& d = selected.snapshot();
      m.decision_epoch  = d.decision_epoch;
      m.decision_seq    = d.decision_seq;
      m.selected        = d.selected;
      m.source_producer = d.source_producer;
      m.source_epoch    = d.source_epoch;
      m.source_seq      = d.source_seq;
    }

    m_previous_selected = CommandSource::NONE;
    return m;   // left/right stay at zero
  }

  // --- Normal path ------------------------------------------
  const DecisionSnapshot& d = selected.snapshot();

  m.decision_epoch  = d.decision_epoch;
  m.decision_seq    = d.decision_seq;
  m.selected        = d.selected;
  m.source_producer = d.source_producer;
  m.source_epoch    = d.source_epoch;
  m.source_seq      = d.source_seq;
  m.hard_stop       = false;
  m.stop_reason     = StopReason::NONE;
  m.surge_in        = d.surge;
  m.yaw_in          = d.yaw;

  // Plan decision (c). A handoff is a change of who is driving,
  // not a fault, so by default the common-throttle state carries
  // and the new owner picks up from the current speed.
  m.handoff = (m_previous_selected != CommandSource::NONE) &&
              (d.selected != m_previous_selected);
  if (m.handoff && m_cfg.slew_reset_on_handoff)
    m_slew.reset(0.0);
  m_previous_selected = d.selected;

  m.surge_shaped = m_slew.update(d.surge, dt);
  m.slew_limited = m_slew.limited();

  // Yaw is NOT slewed. See the header.
  m.yaw_shaped = d.yaw;

  m.alloc        = skid_mix(m.surge_shaped, m.yaw_shaped, m_cfg.skid);
  m.left_effort  = m.alloc.left_effort;
  m.right_effort = m.alloc.right_effort;

  return m;
}

std::string serialize_mixed(const MixedCommand& m)
{
  char buf[768];
  std::snprintf(buf, sizeof(buf),
                "v=1,mixer_model=%s,mix_epoch=%s,mix_seq=%llu,mix_time=%.3f,"
                "decision_epoch=%s,decision_seq=%llu,"
                "selected=%s,hard_stop=%d,stop=%s,"
                "source_producer=%s,source_epoch=%s,source_seq=%llu,"
                "surge_in=%.3f,yaw_in=%.3f,"
                "surge_shaped=%.3f,yaw_shaped=%.3f,"
                "left_effort=%.3f,right_effort=%.3f,"
                "q=%.4f,f=%.4f,limits=%d%d%d%d",
                m.mixer_model.c_str(),
                m.mix_epoch.c_str(),
                (unsigned long long)m.mix_seq,
                m.mix_time,
                m.decision_epoch.empty() ? "-" : m.decision_epoch.c_str(),
                (unsigned long long)m.decision_seq,
                to_string(m.selected),
                m.hard_stop ? 1 : 0,
                to_string(m.stop_reason),
                m.source_producer.empty() ? "-" : m.source_producer.c_str(),
                m.source_epoch.empty() ? "-" : m.source_epoch.c_str(),
                (unsigned long long)m.source_seq,
                m.surge_in, m.yaw_in,
                m.surge_shaped, m.yaw_shaped,
                m.left_effort, m.right_effort,
                m.alloc.saturation_value, m.alloc.fair_scaler,
                m.alloc.limit_steer_left ? 1 : 0,
                m.alloc.limit_steer_right ? 1 : 0,
                m.alloc.limit_throttle_lower ? 1 : 0,
                m.alloc.limit_throttle_upper ? 1 : 0);
  return std::string(buf);
}

} // namespace bb
