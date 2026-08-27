/*************************************************************
 *  arm_sequencer -- the ESC arm/disarm transition, as pure logic.
 *
 *  Extracted from iBBNavigatorInterface for one reason: this is
 *  the highest-risk code in the actuation path and it was the
 *  only part not reachable by a test. Its predecessor took three
 *  attempts and a PCA9685 datasheet trap to get right, and it
 *  panicked the ESCs on alternating launches the whole time
 *  (docs/rc_controllers.md 8.3).
 *
 *  It decides WHAT should happen; the caller performs it. That
 *  split is what makes it testable without a board:
 *
 *      ArmAction a = seq.update(now, want_armed, hardware_ok);
 *      switch (a) { case BEGIN_ARM: ...pwm_enable(true)... }
 *
 *  THE ORDERING THAT MATTERS, and why:
 *
 *    DISARM must leave the PCA9685's channels latched OFF, not
 *    merely raise OE. pwm_enable(false) alone stops the output
 *    pin while the channels keep RUNNING -- and the next arm's
 *    pwm_set_frequency() sleeps the chip, which on a running
 *    chip sets RESTART-pending, after which the outputs stay
 *    dead and the ESCs beep at an enabled line carrying no
 *    pulses. That is the alternating-launch panic. The caller
 *    MUST do an orderly all-off on DISARM; this class cannot
 *    enforce that, but the tests below pin the state machine
 *    that surrounds it.
 *
 *    ARMING must write neutral EVERY cycle, not sleep. An ESC
 *    arms on a stable neutral signal it can actually see; a
 *    blocking sleep in some other thread holds nothing.
 *
 *  Author: Jeremy Wenger
 *************************************************************/

#ifndef BB_ARM_SEQUENCER_HEADER
#define BB_ARM_SEQUENCER_HEADER

#include <cstdint>
#include <string>

namespace bb {

enum class ArmState { DISARMED, ARMING, ARMED };

enum class ArmAction {
  IDLE,          // disarmed and staying so; output is cut, write nothing
  BEGIN_ARM,     // set frequency, write neutral, enable output
  HOLD_NEUTRAL,  // write neutral this cycle (the arming hold)
  DISARM,        // write neutral, orderly all-off, cut output
  DRIVE          // armed and healthy; the caller may write commands
};

const char* to_string(ArmState s);
const char* to_string(ArmAction a);

struct ArmSequencerConfig
{
  // Stable-neutral duration the Basic ESC 500 needs.
  double arm_hold_sec = 2.0;

  // initialize_esc = false. Still transits ARMING for one cycle
  // so there is exactly ONE path to ARMED -- a second, shorter
  // path is how the two diverge later.
  bool skip_hold = false;

  std::string validate() const;
};

class ArmSequencer
{
 public:
  explicit ArmSequencer(const ArmSequencerConfig& cfg);

  // One cycle. `hardware_ok` gates arming only: a board that is
  // not there must never begin an arm sequence, but a board that
  // FAILS while armed still needs the disarm path to run.
  ArmAction update(double now, bool want_armed, bool hardware_ok);

  ArmState state()      const { return m_state; }
  bool     armed()      const { return m_state == ArmState::ARMED; }
  uint64_t arm_cycles() const { return m_arm_cycles; }
  double   hold_remaining(double now) const;

  // Force back to disarmed without emitting an action. For
  // shutdown paths where the caller has already cut the output.
  void force_disarmed();

 private:
  ArmSequencerConfig m_cfg;
  ArmState m_state;
  double   m_hold_start;
  uint64_t m_arm_cycles;
};

} // namespace bb

#endif
