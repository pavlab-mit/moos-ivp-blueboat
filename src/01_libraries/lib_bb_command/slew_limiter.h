/*************************************************************
 *  slew_limiter -- symmetric dt-based rate limit on COMMON
 *  throttle, applied BEFORE skid mixing.
 *
 *  This is ArduRover's MOT_SLEWRATE behaviour
 *  (AP_MotorsUGV::slew_limit_throttle, called at
 *  AP_MotorsUGV.cpp:338 -- before output_skid_steering at :344).
 *
 *  WHY COMMON THROTTLE ONLY. Filtering left and right
 *  independently also filters their DIFFERENCE, which is the yaw
 *  command -- so a turn is smoothed as much as an acceleration.
 *  ArduPilot slews only the common term and leaves steering
 *  untouched, which is a large part of why an ArduPilot boat
 *  feels crisp in a turn. The BlueBoat's previous behaviour
 *  (independent per-side LPF in iBBNavigatorInterface) is exactly
 *  the mistake this class exists to avoid. Do not "improve" it by
 *  adding a yaw filter here.
 *
 *  Author: Jeremy Wenger
 *************************************************************/

#ifndef BB_SLEW_LIMITER_HEADER
#define BB_SLEW_LIMITER_HEADER

#include <string>

namespace bb {

class SlewLimiter
{
 public:
  // rate_pct_per_sec: MOT_SLEWRATE. BlueBoat default 200, i.e.
  // zero to full forward in 0.5 s, full reverse to full forward
  // in 1.0 s. A rate <= 0 disables limiting (ArduPilot semantics).
  //
  // max_dt_sec bounds how much credit a single call may earn. It
  // has no ArduPilot equivalent and is deliberate: ArduPilot runs
  // a fixed-rate scheduler, we run a MOOS app that can be
  // descheduled. Without it, one long stall grants an unbounded
  // step and the limiter silently stops limiting at the worst
  // possible moment.
  explicit SlewLimiter(double rate_pct_per_sec = 200.0,
                       double max_dt_sec = 0.5);

  // Advance one control cycle. Returns the new limited value.
  // Non-finite request or dt leaves the state untouched and
  // returns the current state -- fail closed, never propagate NaN
  // into an actuator command.
  double update(double request_pct, double dt_sec);

  // Hard stop: bypass the limiter entirely and reset the state.
  // Invariant 7 -- a safety stop must not be slewed into.
  void reset(double state_pct = 0.0);

  double state() const { return m_state; }
  double rate()  const { return m_rate; }

  // True if the most recent update() was rate-limited rather than
  // reaching the request. Trace only.
  bool limited() const { return m_limited; }

  std::string validate() const;

 private:
  double m_rate;
  double m_max_dt;
  double m_state;
  bool   m_limited;
};

} // namespace bb

#endif
