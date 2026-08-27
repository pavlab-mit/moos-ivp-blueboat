/*************************************************************
 *  actuator_stage -- motor effort to ESC pulse, and the last
 *  safety word before the water.
 *
 *  The pure core of iBBNavigatorInterface's actuation path. The
 *  app around it owns threads, I2C and MOOS; every rule about
 *  WHEN the props may turn lives here, where it can be tested
 *  exhaustively without a boat.
 *
 *  This is the last stage in the chain, so it is the one that
 *  fails closed on everything upstream AND on everything local:
 *
 *      upstream   no mixer / stale mixer / mixer said stop
 *      sidebands  RC kill, RC deadman
 *      hardware   not armed, unhealthy, shutting down
 *
 *  TWO SIDEBANDS BYPASS THE COMMAND PATH ENTIRELY, and that is
 *  the point of putting them here rather than in the arbiter:
 *
 *    RC KILL (plan decision (b)) is enforced at the hardware
 *    boundary so it survives a failure of the arbiter, the mixer,
 *    or the contract between them. An arbiter that has crashed
 *    mid-decision cannot stop the boat; this can.
 *
 *    RC DEADMAN (plan decision (d)) is the opt-in blanket: if
 *    enabled, loss of the RC link safes the boat REGARDLESS of
 *    mode. It exists for running experimental code, so routing it
 *    through the very code under test would defeat it. Default
 *    off. This is NOT the source lease -- that lives in the
 *    arbiter and is never disableable.
 *
 *  ORDER MATTERS, BUT NOT FOR THE REASON IT LOOKS LIKE. Local
 *  safety is evaluated before the command. Either ordering stops
 *  the boat -- every branch returns early, so there is no
 *  ordering in which a fault gets driven through. What the order
 *  decides is WHICH REASON the log records when a local fault and
 *  an upstream fault are true in the same cycle: with the mixer
 *  quiet AND the kill asserted, local-first says RC_KILL, which
 *  is the operator's action, rather than NAV_INPUT_STALE, which
 *  is a downstream symptom of it. That is a G4 property, not a
 *  safety one, and it is worth exactly as much as an incident
 *  review is worth.
 *
 *  Author: Jeremy Wenger
 *************************************************************/

#ifndef BB_ACTUATOR_STAGE_HEADER
#define BB_ACTUATOR_STAGE_HEADER

#include "esc_mapper.h"
#include "mixer_stage.h"

#include <cstdint>
#include <string>

namespace bb {

// Local hardware and sideband state, owned by the Navigator.
// Deliberately separate from the arbiter's SafetyInputs: these
// are facts about THIS process's hardware, not policy about who
// may drive.
struct NavigatorSafetyState
{
  bool pwm_armed          = false;
  bool hardware_healthy   = true;
  bool shutdown_requested = false;

  // Sidebands. Both bypass the command path.
  bool rc_kill_asserted   = false;
  bool rc_link_lost       = true;   // pessimistic until told otherwise
};

struct ActuatorConfig
{
  double mixed_cmd_timeout_sec = 0.5;

  EscChannelConfig left;    // BlueBoat120: SERVO3, reversed
  EscChannelConfig right;   // BlueBoat120: SERVO1, not reversed

  // Opt-in blanket, default OFF (plan decision (d)).
  bool   rc_deadman_enabled     = false;
  double rc_deadman_timeout_sec = 2.0;

  // How long a committed frame stays valid to the PWM writer.
  // The writer neutralises past this, so a stalled or dead
  // Iterate() cannot leave the props running on an old command.
  double actuator_frame_ttl_sec = 0.25;

  std::string validate() const;
};

// One atomic actuator command. The PWM thread consumes this whole
// or not at all -- it must never read a half-updated left/right
// pair, which is what an unsynchronised pair of doubles allows.
struct ActuatorFrame
{
  // Full lineage, so any pulse in any log joins back to exactly
  // one stick movement or one controller output (invariant 10).
  std::string   source_producer;
  std::string   source_epoch;
  uint64_t      source_seq = 0;
  std::string   decision_epoch;
  uint64_t      decision_seq = 0;
  std::string   mix_epoch;
  uint64_t      mix_seq = 0;
  CommandSource selected = CommandSource::NONE;

  StopReason stop_reason = StopReason::STARTUP;
  bool       neutral     = true;

  double left_effort  = 0.0;   // physical, forward-positive
  double right_effort = 0.0;
  double left_pwm_us  = 0.0;   // electrical, post-reversal
  double right_pwm_us = 0.0;

  double committed_at = 0.0;
  double expires_at   = 0.0;

  double mixed_age = 0.0;

  // True when the stop came from a local sideband or hardware
  // rather than from upstream. Distinguishing them is the whole
  // reason this stage has its own reasons.
  bool local_stop = false;
};

class ActuatorStage
{
 public:
  explicit ActuatorStage(const ActuatorConfig& cfg);

  // One control cycle. `last_rc_good_time` is the local time of
  // the most recent good RC link observation, used only by the
  // opt-in deadman.
  ActuatorFrame update(double now,
                       const MixedMailbox& mixed,
                       const NavigatorSafetyState& safety,
                       double last_rc_good_time);

  const ActuatorConfig& config() const { return m_cfg; }

  // Neutral pulses, for the shutdown and disarm paths. Taken from
  // the same config the mapper uses so they cannot drift apart.
  double left_neutral_us()  const { return esc_neutral_us(m_cfg.left); }
  double right_neutral_us() const { return esc_neutral_us(m_cfg.right); }

 private:
  ActuatorConfig m_cfg;
};

// Serialise as NVGR_ACTUATOR_TRACE (design doc 11).
std::string serialize_actuator_trace(const ActuatorFrame& f);

} // namespace bb

#endif
