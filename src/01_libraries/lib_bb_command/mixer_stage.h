/*************************************************************
 *  mixer_stage -- surge/yaw to left/right motor effort.
 *
 *  The pure core of pThrustMix: slew limiting, ArduRover skid
 *  allocation, lineage propagation, and fail-closed handling of a
 *  bad input. The MOOS app around it does mail and publication.
 *
 *  It subscribes, conceptually, to ONE thing: the arbiter's
 *  decision. It cannot see RC, teleop, autonomy, or ALL_STOP, and
 *  that is deliberate -- a mixer that cannot observe the sources
 *  cannot accidentally implement a second authority policy. If
 *  you find yourself wanting to add a source input here, the
 *  thing you actually want belongs in the arbiter.
 *
 *  Two behaviours worth knowing before reading the code:
 *
 *  ONLY COMMON THROTTLE IS SLEWED. Yaw passes through unshaped.
 *  Slewing both sides would slew their difference, which is the
 *  turn command -- the exact defect the old per-side LPF had
 *  (plan section 2.6). This asymmetry is the point, not an
 *  oversight.
 *
 *  A HARD STOP BYPASSES THE LIMITER AND RESETS IT (invariant 7).
 *  A safety stop that ramps down over half a second is not a
 *  safety stop.
 *
 *  Author: Jeremy Wenger
 *************************************************************/

#ifndef BB_MIXER_STAGE_HEADER
#define BB_MIXER_STAGE_HEADER

#include "authority.h"
#include "skid_mixer.h"
#include "slew_limiter.h"

#include <cstdint>
#include <string>

namespace bb {

struct MixerConfig
{
  // REQUIRED, no default (plan decision 4). A log must never be
  // ambiguous about which equation produced a motor pair, so the
  // app refuses to start without an explicit model name.
  std::string mixer_model;

  double selected_cmd_timeout_sec = 0.5;

  double slew_rate_pct_sec = 200.0;   // MOT_SLEWRATE
  double slew_max_dt_sec   = 0.5;

  // Plan decision (c). Default false: carry the common-throttle
  // state across a source handoff, so an operator taking manual
  // control mid-transit does not get a drop to zero and a ramp
  // back up. Flip to true to make every handoff start from rest.
  bool slew_reset_on_handoff = false;

  SkidMixerParams skid;

  std::string validate() const;
};

struct MixedCommand
{
  std::string mix_epoch;
  uint64_t    mix_seq  = 0;
  double      mix_time = 0.0;
  std::string mixer_model;

  // Upstream lineage, copied unchanged (invariant 10).
  std::string   decision_epoch;
  uint64_t      decision_seq = 0;
  CommandSource selected     = CommandSource::NONE;
  std::string   source_producer;
  std::string   source_epoch;
  uint64_t      source_seq   = 0;

  bool       hard_stop   = true;
  StopReason stop_reason = StopReason::STARTUP;

  double surge_in     = 0.0;
  double yaw_in       = 0.0;
  double surge_shaped = 0.0;   // post-slew
  double yaw_shaped   = 0.0;   // == yaw_in; yaw is never slewed

  double left_effort  = 0.0;
  double right_effort = 0.0;

  AllocationResult alloc;

  double input_age = 0.0;
  bool   slew_limited = false;
  bool   handoff = false;      // selected source changed this cycle
};

class MixerStage
{
 public:
  MixerStage(const MixerConfig& cfg, const std::string& mix_epoch);

  // One control cycle. `dt` is the mixer's own cycle time; `now`
  // is local MOOSTime, used only for the input lease.
  MixedCommand update(double now, double dt, const DecisionMailbox& selected);

  const std::string& epoch() const { return m_epoch; }
  uint64_t           seq()   const { return m_seq; }
  const MixerConfig& config() const { return m_cfg; }
  double slew_state() const { return m_slew.state(); }

 private:
  MixerConfig   m_cfg;
  std::string   m_epoch;
  uint64_t      m_seq;
  SlewLimiter   m_slew;
  CommandSource m_previous_selected;
};

// Serialise as BB_MIXED_CMD (design doc 6.2).
std::string serialize_mixed(const MixedCommand& m);

} // namespace bb

#endif
