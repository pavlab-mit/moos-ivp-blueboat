#include "arm_sequencer.h"

#include <cmath>

namespace bb {

const char* to_string(ArmState s)
{
  switch (s) {
    case ArmState::DISARMED: return "DISARMED";
    case ArmState::ARMING:   return "ARMING";
    case ArmState::ARMED:    return "ARMED";
  }
  return "DISARMED";
}

const char* to_string(ArmAction a)
{
  switch (a) {
    case ArmAction::IDLE:         return "IDLE";
    case ArmAction::BEGIN_ARM:    return "BEGIN_ARM";
    case ArmAction::HOLD_NEUTRAL: return "HOLD_NEUTRAL";
    case ArmAction::DISARM:       return "DISARM";
    case ArmAction::DRIVE:        return "DRIVE";
  }
  return "IDLE";
}

std::string ArmSequencerConfig::validate() const
{
  if (!std::isfinite(arm_hold_sec) || arm_hold_sec < 0.0)
    return "arm_hold_sec must be finite and >= 0";
  return "";
}

ArmSequencer::ArmSequencer(const ArmSequencerConfig& cfg)
  : m_cfg(cfg),
    m_state(ArmState::DISARMED),
    m_hold_start(0.0),
    m_arm_cycles(0)
{
}

void ArmSequencer::force_disarmed()
{
  m_state = ArmState::DISARMED;
}

double ArmSequencer::hold_remaining(double now) const
{
  if (m_state != ArmState::ARMING) return 0.0;
  const double r = m_cfg.arm_hold_sec - (now - m_hold_start);
  return (r > 0.0) ? r : 0.0;
}

ArmAction ArmSequencer::update(double now, bool want_armed, bool hardware_ok)
{
  if (!std::isfinite(now)) {
    // A clock we cannot trust must not be able to complete an arm
    // hold. Hold position rather than advancing on a bad stamp.
    return (m_state == ArmState::ARMED) ? ArmAction::DRIVE
         : (m_state == ArmState::ARMING) ? ArmAction::HOLD_NEUTRAL
                                         : ArmAction::IDLE;
  }

  switch (m_state) {

  case ArmState::DISARMED:
    // hardware_ok gates ARMING only. Beginning an arm sequence on
    // a board that is not there would enable an output that
    // cannot be driven.
    if (want_armed && hardware_ok) {
      m_state = ArmState::ARMING;
      m_hold_start = m_cfg.skip_hold ? (now - m_cfg.arm_hold_sec) : now;
      return ArmAction::BEGIN_ARM;
    }
    return ArmAction::IDLE;

  case ArmState::ARMING:
    // A withdrawn request during the hold aborts through the SAME
    // disarm path as any other, so the channels are always left
    // latched off however the sequence ends. An abort that merely
    // returned to DISARMED would leave them running and arm the
    // RESTART trap for the next attempt.
    if (!want_armed) {
      m_state = ArmState::DISARMED;
      return ArmAction::DISARM;
    }
    // Hardware failing mid-hold is also a disarm, not a silent
    // return: the output is already enabled and must be cut.
    if (!hardware_ok) {
      m_state = ArmState::DISARMED;
      return ArmAction::DISARM;
    }
    if ((now - m_hold_start) >= m_cfg.arm_hold_sec) {
      m_state = ArmState::ARMED;
      ++m_arm_cycles;
      return ArmAction::HOLD_NEUTRAL;   // this cycle still holds
    }
    return ArmAction::HOLD_NEUTRAL;

  case ArmState::ARMED:
    if (!want_armed) {
      m_state = ArmState::DISARMED;
      return ArmAction::DISARM;
    }
    // NOTE hardware_ok is deliberately NOT a disarm trigger here.
    // A transient bus error must not cut the signal and cost a
    // 2 s re-arm mid-mission; ActuatorStage already holds the
    // props at neutral on HARDWARE_FAULT, which is the safe and
    // recoverable response. Only an explicit request disarms.
    return ArmAction::DRIVE;
  }

  return ArmAction::IDLE;
}

} // namespace bb
