#include "actuator_stage.h"
#include "wire_format.h"

#include <cmath>
#include <cstdio>

namespace bb {

std::string ActuatorConfig::validate() const
{
  if (!std::isfinite(mixed_cmd_timeout_sec) || mixed_cmd_timeout_sec <= 0.0)
    return "mixed_cmd_timeout_sec must be finite and > 0";
  if (!std::isfinite(actuator_frame_ttl_sec) || actuator_frame_ttl_sec <= 0.0)
    return "actuator_frame_ttl_sec must be finite and > 0";
  if (rc_deadman_enabled &&
      (!std::isfinite(rc_deadman_timeout_sec) || rc_deadman_timeout_sec <= 0.0))
    return "rc_deadman_timeout_sec must be finite and > 0 when the deadman is enabled";

  const std::string l = left.validate();
  if (!l.empty()) return "left ESC: " + l;
  const std::string r = right.validate();
  if (!r.empty()) return "right ESC: " + r;
  return "";
}

ActuatorStage::ActuatorStage(const ActuatorConfig& cfg)
  : m_cfg(cfg)
{
}

ActuatorFrame ActuatorStage::update(double now,
                                    const MixedMailbox& mixed,
                                    const NavigatorSafetyState& safety,
                                    double last_rc_good_time)
{
  ActuatorFrame f;
  f.committed_at = now;
  f.expires_at   = now + m_cfg.actuator_frame_ttl_sec;
  f.mixed_age    = mixed.age(now);

  // Copy lineage first, so even a stopped frame says what it was
  // stopping. A neutral frame with no lineage tells an incident
  // review nothing.
  if (mixed.has_mixed()) {
    const MixedSnapshot& m = mixed.snapshot();
    f.source_producer = m.source_producer;
    f.source_epoch    = m.source_epoch;
    f.source_seq      = m.source_seq;
    f.decision_epoch  = m.decision_epoch;
    f.decision_seq    = m.decision_seq;
    f.mix_epoch       = m.mix_epoch;
    f.mix_seq         = m.mix_seq;
    f.selected        = m.selected;
  }

  // --- LOCAL SAFETY FIRST -----------------------------------
  //
  // Evaluated before the command so a valid command arriving in
  // the same cycle cannot mask a kill or a fault. Ordered by
  // severity: the things that mean "this process must not drive
  // the motors at all" come before the things that mean "not
  // right now".
  f.local_stop = true;

  if (safety.shutdown_requested) {
    f.stop_reason = StopReason::SHUTDOWN;
    return f;
  }
  if (!safety.hardware_healthy) {
    f.stop_reason = StopReason::HARDWARE_FAULT;
    return f;
  }
  if (!safety.pwm_armed) {
    f.stop_reason = StopReason::PWM_DISARMED;
    return f;
  }

  // RC kill: the sideband that must survive a failure of
  // everything upstream (plan decision (b)).
  if (safety.rc_kill_asserted) {
    f.stop_reason = StopReason::RC_KILL;
    return f;
  }

  // RC deadman: opt-in blanket, any mode, no exceptions
  // (plan decision (d)). Note this is about the RC LINK, not
  // about who is driving -- with it enabled, an autonomous
  // mission stops when the handset goes away, which is exactly
  // what it is for.
  if (m_cfg.rc_deadman_enabled) {
    const bool never_seen = !std::isfinite(last_rc_good_time) ||
                            (last_rc_good_time <= 0.0);
    const double rc_age = never_seen ? 1.0e9 : (now - last_rc_good_time);
    if (safety.rc_link_lost || rc_age > m_cfg.rc_deadman_timeout_sec) {
      f.stop_reason = StopReason::RC_DEADMAN;
      return f;
    }
  }

  f.local_stop = false;

  // --- UPSTREAM ---------------------------------------------
  //
  // Three input failures, three reasons, for the same argument as
  // in the mixer: no mixer at all, a mixer that went quiet, and a
  // mixer that deliberately stopped are different faults.
  if (!mixed.has_mixed()) {
    f.stop_reason = StopReason::NAV_INPUT_INVALID;
    return f;
  }
  if (!mixed.fresh(now, m_cfg.mixed_cmd_timeout_sec)) {
    f.stop_reason = StopReason::NAV_INPUT_STALE;
    return f;
  }

  const MixedSnapshot& m = mixed.snapshot();
  if (m.hard_stop) {
    f.stop_reason = m.stop_reason;   // propagate, never overwrite
    return f;
  }

  // --- DRIVE -------------------------------------------------
  const EscOutput lo = esc_map(m.left_effort,  m_cfg.left);
  const EscOutput ro = esc_map(m.right_effort, m_cfg.right);

  f.neutral      = false;
  f.stop_reason  = StopReason::NONE;
  f.left_effort  = lo.physical_effort;
  f.right_effort = ro.physical_effort;
  f.left_pwm_us  = lo.pulse_us;
  f.right_pwm_us = ro.pulse_us;
  return f;
}

std::string serialize_actuator_trace(const ActuatorFrame& f)
{
  return wire::formatf(
                "v=1,t=%.3f,selected=%s,stop=%s,neutral=%d,local_stop=%d,"
                "source_producer=%s,source_epoch=%s,source_seq=%llu,"
                "decision_epoch=%s,decision_seq=%llu,"
                "mix_epoch=%s,mix_seq=%llu,mix_age=%.3f,"
                "left_effort=%.3f,right_effort=%.3f,"
                "left_pwm_us=%.1f,right_pwm_us=%.1f,expires_at=%.3f",
                f.committed_at,
                to_string(f.selected),
                to_string(f.stop_reason),
                f.neutral ? 1 : 0,
                f.local_stop ? 1 : 0,
                f.source_producer.empty() ? "-" : f.source_producer.c_str(),
                f.source_epoch.empty() ? "-" : f.source_epoch.c_str(),
                (unsigned long long)f.source_seq,
                f.decision_epoch.empty() ? "-" : f.decision_epoch.c_str(),
                (unsigned long long)f.decision_seq,
                f.mix_epoch.empty() ? "-" : f.mix_epoch.c_str(),
                (unsigned long long)f.mix_seq,
                f.mixed_age,
                f.left_effort, f.right_effort,
                f.left_pwm_us, f.right_pwm_us,
                f.expires_at);
}

} // namespace bb
