/*************************************************************
 *  pid -- the one PID primitive for BlueBoat controllers.
 *
 *  Written to replace moos-ivp's ScalarPID inside our own
 *  controllers (pBBPID first), after the 2026-08-27 handling
 *  block exposed two lifecycle defects that ScalarPID cannot
 *  express fixes for:
 *
 *    1. It integrates Ki*e*dt with dt = time since its LAST
 *       call, so the first tick after a 43 s command gap
 *       integrated the whole gap in one step (surge resumed at
 *       -13.7 with +1.5 m/s commanded).
 *    2. It has no way to HOLD the integral while the output is
 *       not reaching the water: setting Ki=0 destroys the
 *       accumulated sum (ScalarPID.cpp `else { m_dfeSum = 0; }`)
 *       and calling with dt=0 freezes P and D along with I.
 *
 *  PARITY CONTRACT. On a clean tick -- integration enabled, no
 *  external saturation reported, dt under max_dt, output inside
 *  its limit -- the arithmetic below is EXACTLY ScalarPID's with
 *  Kd = 0 (every live BlueBoat gain set has Kd = 0):
 *
 *      I   += Ki * e * dt,  |I| clamped to integral_limit
 *      out  = Kp*e + Kd*d + I,  clamped to output_limit
 *
 *  Deliberate compatibility choices, so live tuning behaves
 *  identically: the integral state is kept in OUTPUT units (Ki
 *  folded into the sum -- a live Ki change does not rescale
 *  history), integral_limit is therefore in output units, and
 *  Ki <= 0 zeroes the accumulated integral. The one semantic
 *  departure: the derivative is a single backward difference
 *  (or caller-supplied), not ScalarPID's mean of the last nine
 *  samples -- dormant while Kd = 0, and preferable when not.
 *  test_pid_scalar_parity.cpp pins all of this against the real
 *  ScalarPID as an oracle.
 *
 *  Divergence from ScalarPID happens exactly where the fixes
 *  bite, and only there:
 *    - dt is caller-supplied and clamped to max_dt: a gap earns
 *      one bounded increment, not the whole gap's worth.
 *    - integrate_enable=false freezes I while P and D stay live
 *      (authority gate: integrate only while our command is
 *      actually driving the boat).
 *    - with anti-windup enabled, an increment that pushes into
 *      a rail -- this loop's own output clamp, or a DOWNSTREAM
 *      rail the caller reports (mixer saturation, post-
 *      feedforward clamp) -- commits only up to the rail's
 *      worth of authority (tracking anti-windup): the integral
 *      parks AT the rail, never climbs to its own limit behind
 *      it, and comes off the rail on the first reversed tick.
 *      Unwinding is always allowed. Anti-windup defaults OFF
 *      because the integral is memory: one refused increment
 *      carries divergence into later clean ticks, so "off =
 *      verbatim legacy, rails included" is what keeps a parity
 *      replay auditable.
 *
 *  The commit-only-what-reaches-the-water pattern follows
 *  pDiffThrustPID_v2 (Turrisi), which ran it on the water; the
 *  max_dt clamp follows SlewLimiter, for the same reason it has
 *  one: a MOOS app can be descheduled, and a stall must not
 *  grant an unbounded step.
 *
 *  NO MOOS, NO CLOCK, NO I/O. Time only enters as the caller's
 *  dt. Fail closed: a non-finite input holds the last output.
 *
 *  Author: Jeremy Wenger
 *************************************************************/

#ifndef BB_PID_HEADER
#define BB_PID_HEADER

#include <string>

namespace bb {

class Pid
{
 public:
  Pid();

  // Natural order (unlike ScalarPID::SetGains's Kp,Kd,Ki trap).
  void set_gains(double kp, double ki, double kd);

  // Both in OUTPUT units. integral_limit caps |I| (Ki is folded
  // into the sum); output_limit clamps the total symmetric.
  // output_limit <= 0 disables the output clamp (ScalarPID would
  // clamp everything to zero; nobody means that).
  void set_limits(double integral_limit, double output_limit);

  // Bound on the dt credited to a single step. <= 0 disables
  // (legacy behavior, kept for parity replay). 0.5 s is a sane
  // choice for a 16-50 Hz loop.
  void set_max_dt(double max_dt_sec);

  // false: freeze I (P and D stay live). The authority gate --
  // integrate only while authority == AUTONOMY && stop == NONE.
  void set_integrate_enable(bool on);

  // Tracking anti-windup at this loop's own output clamp (see the
  // header comment). Default OFF = ScalarPID's exact behavior at
  // rails too (integral climbs to its own limit behind the wall):
  // because the integral is memory, a single refused increment
  // would otherwise carry divergence into clean ticks and muddy a
  // parity replay. One rule for every fix: off = verbatim legacy.
  void set_antiwindup_enable(bool on);

  // Direction (-1/0/+1, in this loop's OUTPUT sign) of a
  // downstream rail the loop cannot see in its own output clamp:
  // mixer saturation, a post-feedforward clamp. Sticky until
  // changed. Increments pushing further that way are skipped.
  void set_external_saturation(int sign);

  void reset();            // I, derivative history, held output
  void reset_integral();   // I only

  // One control tick. dt in seconds, already measured by the
  // caller (this class has no clock). The two-argument form
  // differences the error for D; the three-argument form takes
  // d_error directly (pass -measured_rate for derivative-on-
  // measurement, immune to setpoint kick).
  double step(double error, double dt);
  double step(double error, double d_error, double dt);

  // --- Telemetry / test accessors ---
  double output()    const { return m_out; }
  double p_term()    const { return m_p; }
  double i_term()    const { return m_i; }
  double d_term()    const { return m_dterm; }
  bool   saturated() const { return m_saturated; }  // output clamp bit
  bool   integrated_last_step() const { return m_integrated; }

  double kp() const { return m_kp; }
  double ki() const { return m_ki; }
  double kd() const { return m_kd; }
  double integral_limit() const { return m_i_lim; }
  double output_limit()   const { return m_out_lim; }
  double max_dt()         const { return m_max_dt; }
  bool   integrate_enabled() const { return m_integrate_enable; }
  int    external_saturation() const { return m_ext_sat; }

  // "" if the configuration is sane, else a human-readable
  // reason (config-time rejection beats mid-mission surprise).
  std::string validate() const;

 private:
  double step_impl(double error, double dt, bool have_d, double d_in);

  // Configuration
  double m_kp, m_ki, m_kd;
  double m_i_lim;            // |I| cap, output units
  double m_out_lim;          // |out| cap; <= 0 = no clamp
  double m_max_dt;           // dt credit cap; <= 0 = no clamp
  bool   m_integrate_enable;
  bool   m_aw_enable;        // tracking anti-windup at the output clamp
  int    m_ext_sat;          // -1/0/+1 downstream rail direction

  // State
  double m_i;                // integral term, output units
  double m_prev_error;
  double m_d;                // last derivative (held across dt=0)
  bool   m_have_prev;
  double m_out;              // last output (held on bad input)

  // Trace of the last step
  double m_p, m_dterm;
  bool   m_saturated;
  bool   m_integrated;       // an increment was committed
};

} // namespace bb

#endif
