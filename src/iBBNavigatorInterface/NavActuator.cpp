/*************************************************************
 *  NavActuator -- BB_MIXED_CMD to ESC pulses.
 *
 *  The half of iBBNavigatorInterface that can spin a propeller.
 *  Every rule about WHEN it may is in lib_bb_command's
 *  ActuatorStage, tested without hardware; what lives here is the
 *  hardware itself: the PWM writer thread, the ESC lifecycle, and
 *  the shutdown paths that must work when nothing else does.
 *
 *  Author: Jeremy Wenger
 *************************************************************/

#include "BBNavigatorInterface.h"

#include "MBUtils.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
#include <unistd.h>

using namespace std;

// The Navigator handle. One per process: both halves of this app
// share the I2C bus and the PCA9685 through it.
extern Navigator g_nav;

// Signal-handler state. A handler may run at any instant, on any
// thread, with locks held elsewhere -- so it may only touch
// lock-free values and call navigator_force_pwm_off(), which
// navigator-cpp provides for exactly this.
static std::atomic<bool> g_shutdown_done{false};
static std::atomic<bool> g_disarm_on_exit{false};

//---------------------------------------------------------
// Safe shutdown.
//
// Uses navigator-cpp main's pca9685 teardown rather than the
// hand-rolled "write neutral 50 times" loop this replaced. That
// loop existed because there was no orderly stop available;
// there is now, and it latches every channel full-off and reads
// back to confirm.

void safePwmShutdown()
{
  if (g_shutdown_done.exchange(true))
    return;

  if (g_disarm_on_exit.load()) {
    // Explicit signal-cut: OE high tri-states the outputs, the
    // ESCs stop on signal loss and beep until the next arm.
    std::string detail;
    navigator_force_pwm_off(&detail);
  } else {
    // Leave the chip parked at neutral with the signal present.
    // Pin state PERSISTS after exit, so this is the fleet safe
    // state: props stopped, ESCs still armed, no re-arm needed.
    g_nav.shutdown();
  }
}

// Set from OnStartUp. Kept next to the handler that reads it so
// the two cannot drift into different translation units.
void g_disarm_on_exit_set(bool v) { g_disarm_on_exit.store(v); }

void signalHandler(int signum)
{
  safePwmShutdown();
  _exit(signum == 0 ? 0 : 128 + signum);
}

//---------------------------------------------------------
// Frame exchange.
//
// Iterate() writes, the PWM thread reads. A mutex rather than two
// atomics because the pair must move together: a writer caught
// mid-update must never let the reader see a new left with an old
// right. The critical section is a struct copy.

void BBNavigatorInterface::commitFrame(const bb::ActuatorFrame &f)
{
  std::lock_guard<std::mutex> lock(m_frame_mutex);
  m_frame = f;
  m_have_frame = true;
}

bool BBNavigatorInterface::latestFrame(bb::ActuatorFrame &out) const
{
  std::lock_guard<std::mutex> lock(m_frame_mutex);
  if (!m_have_frame) return false;
  out = m_frame;
  return true;
}

//---------------------------------------------------------
// Pulse output.

void BBNavigatorInterface::writePulsePair(double left_us, double right_us)
{
  // As close together as the bus permits. There is no atomic
  // two-channel write on this chip, so the boat is briefly
  // asymmetric between these two calls; keeping them adjacent
  // with nothing in between is the whole mitigation.
  g_nav.pwm_set_pulse_us(m_left_thruster_pin,  static_cast<float>(left_us));
  g_nav.pwm_set_pulse_us(m_right_thruster_pin, static_cast<float>(right_us));
  m_pwm_writes.fetch_add(1);
}

void BBNavigatorInterface::writeNeutralPair()
{
  // Neutral comes from the same ESC config the mapper uses, so
  // the stop pulse and the commanded pulse can never disagree
  // about where zero is. The old code hardcoded 1500 while the
  // BlueBoat's trim is 1510 -- a permanent 10 us idle bias.
  writePulsePair(m_stage->left_neutral_us(), m_stage->right_neutral_us());
}

//---------------------------------------------------------
// The PWM writer thread.
//
// Its own watchdog. If Iterate() stalls, dies, or is descheduled
// past the frame's TTL, this neutralises rather than continuing
// to replay the last command. That is the difference between a
// hung app and a runaway boat.

void BBNavigatorInterface::pwmWriterThread()
{
  using clock = std::chrono::steady_clock;
  const auto period = std::chrono::duration<double>(1.0 / PWM_FREQ_HZ);
  auto next = clock::now();

  while (m_running.load()) {
    next += std::chrono::duration_cast<clock::duration>(period);

    const double now = MOOSTime();

    // The sequencer DECIDES; this thread PERFORMS. Keeping the
    // decision in lib_bb_command is what makes the arm/disarm
    // transition testable without a board -- it was the last part
    // of the actuation path no test could reach, and its
    // predecessor panicked the ESCs for months.
    const bb::ArmAction action =
        m_arm->update(now, m_arm_requested.load(), m_nav_init_ok);
    m_arm_state_pub.store((int)m_arm->state());
    m_esc_armed.store(m_arm->armed());

    switch (action) {

    case bb::ArmAction::IDLE:
      // Output is cut and staying cut. Writing pulses into a
      // tri-stated chip is pointless bus traffic.
      break;

    case bb::ArmAction::BEGIN_ARM:
      beginArmSequence();
      break;

    case bb::ArmAction::HOLD_NEUTRAL:
      writeNeutralPair();
      break;

    case bb::ArmAction::DISARM:
      performDisarm("sequencer");
      break;

    case bb::ArmAction::DRIVE:
      if (!m_thruster_enabled) { writeNeutralPair(); break; }
      {
        bb::ActuatorFrame f;
        if (!latestFrame(f) || f.neutral) {
          writeNeutralPair();
        } else if (now > f.expires_at) {
          // The frame outlived its TTL: whoever produced it is no
          // longer running. This thread's watchdog is the last
          // thing between a hung Iterate() and a boat still
          // driving on a stale command.
          writeNeutralPair();
          m_pwm_watchdog_count.fetch_add(1);
        } else {
          writePulsePair(f.left_pwm_us, f.right_pwm_us);
        }
      }
      break;
    }

    std::this_thread::sleep_until(next);
  }

  if (m_arm && m_arm->armed()) writeNeutralPair();
}

//---------------------------------------------------------
// Arm and disarm, called ONLY from the PWM thread.

void BBNavigatorInterface::beginArmSequence()
{
  // Frequency first, while the channels are still stopped. This
  // ordering is load-bearing: pca9685_set_frequency sleeps the
  // chip, and sleeping a chip whose channels are RUNNING sets the
  // RESTART-pending condition that panicked the ESCs on
  // alternating launches (rc_controllers.md 8.3). performDisarm()
  // below leaves the channels latched off precisely so this call
  // cannot spring that trap.
  std::string err = g_nav.pwm_set_frequency(static_cast<float>(PWM_FREQ_HZ));
  if (!err.empty()) {
    reportRunWarning("PWM frequency set failed: " + err);
    return;
  }

  // Neutral BEFORE enabling output, so the first pulse the ESCs
  // ever see is trim and not a stale register value.
  writeNeutralPair();

  err = g_nav.pwm_enable(true);
  if (!err.empty()) {
    reportRunWarning("PWM enable failed: " + err);
    return;
  }
  m_pwm_output_enabled.store(true);

  reportEvent("ESC arm sequence started");
}

void BBNavigatorInterface::performDisarm(const std::string &reason)
{
  // Drop the command frame first: nothing may be replayed between
  // the neutral write and the signal cut.
  {
    std::lock_guard<std::mutex> lock(m_frame_mutex);
    m_have_frame = false;
  }
  writeNeutralPair();

  // Orderly all-off with readback, then the signal cut.
  //
  // pwm_enable(false) alone raises OE but leaves the channels
  // RUNNING, which is what armed the RESTART trap for the next
  // set_frequency and made in-run re-arm panic the ESCs. The
  // orderly stop is why the NVGR_DISARM cycle bench-passed on
  // 2026-08-14, and it must stay on this path.
  std::string detail;
  const std::string err = navigator_force_pwm_off(&detail);
  if (!err.empty())
    reportRunWarning("orderly PWM stop failed: " + err);

  g_nav.pwm_enable(false);
  m_pwm_output_enabled.store(false);
  m_esc_armed.store(false);
  reportEvent("ESCs DISARMED (" + reason + ")");
}

//---------------------------------------------------------
// Safety snapshot.

bb::NavigatorSafetyState BBNavigatorInterface::snapshotSafety() const
{
  bb::NavigatorSafetyState s;
  s.pwm_armed          = m_pwm_output_enabled.load() && m_esc_armed.load();
  s.shutdown_requested = m_shutdown_requested.load();

  // is_pwm_ready() is navigator-cpp main's answer to "is the
  // PCA9685 actually usable right now". The previous version of
  // this app never asked -- it called init() in its constructor,
  // stored the error string, and drove the motors regardless of
  // what it said.
  s.hardware_healthy   = m_nav_init_ok && g_nav.is_pwm_ready();

  s.rc_kill_asserted   = m_rc_kill_asserted.load();
  s.rc_link_lost       = !m_rc_link_alive.load();
  return s;
}
