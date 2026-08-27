/*************************************************************
 *  esc_mapper -- physical motor effort to ESC pulse width.
 *
 *  THE ONLY PLACE electrical inversion is allowed to happen
 *  (invariant 9). Upstream, positive effort always means physical
 *  forward thrust on that side. Which way a given ESC has to be
 *  driven to produce that is a wiring fact, and it lives here,
 *  per channel, exactly as ArduPilot's SERVOx_REVERSED does.
 *
 *  A global sign multiplier somewhere in the middle of the
 *  pipeline is prohibited: that is how the old
 *  "-1 * value * invert" convention produced a boat whose turns
 *  were correct and whose throttle was reversed.
 *
 *  BlueBoat120 endpoints, from the Blue Robotics ArduRover 4.7
 *  parameter file:
 *      min 1100 / trim 1510 / max 1900 us
 *      right = SERVO1, not reversed
 *      left  = SERVO3, reversed
 *
 *  Note trim is 1510, not 1500. The 10 us offset is real and is
 *  what the ESC treats as stop; using 1500 is a small permanent
 *  idle bias.
 *
 *  Mapping is piecewise-linear about trim, because the forward
 *  and reverse spans are not equal (390 us up, 410 us down).
 *
 *  Author: Jeremy Wenger
 *************************************************************/

#ifndef BB_ESC_MAPPER_HEADER
#define BB_ESC_MAPPER_HEADER

#include <string>

namespace bb {

struct EscChannelConfig
{
  double min_us  = 1100.0;
  double trim_us = 1510.0;
  double max_us  = 1900.0;
  bool   reversed = false;

  // Must be called at startup. Configuration validation failing
  // is a startup failure, not a runtime warning: a boat with a
  // mis-ordered PWM range should refuse to run, not discover it
  // with props in the water.
  std::string validate() const;
};

struct EscOutput
{
  double physical_effort;    // input, clamped to [-100, 100]
  double electrical_effort;  // after channel reversal
  double pulse_us;           // pre-quantisation; caller rounds
  bool   clamped;            // input was outside [-100, 100]
};

// Map one channel. Non-finite effort maps to trim (stop) with
// clamped = true -- fail closed (invariant 12).
EscOutput esc_map(double effort_pct, const EscChannelConfig& cfg);

// The pulse this channel emits for a full stop. Used by the
// neutral/hard-stop paths so they cannot drift from the mapping.
double esc_neutral_us(const EscChannelConfig& cfg);

} // namespace bb

#endif
