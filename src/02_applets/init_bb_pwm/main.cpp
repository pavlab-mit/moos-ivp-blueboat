/*************************************************************
      Name: Raymond Turrisi (orig.)
      Orgn: MIT, Cambridge MA
      File: init_bb_pwm/main.cpp
   Last Ed: 2026-05-14 (mikedef)
     Brief:
        BlueBoat ESC arming utility. Builds against navigator-lib
        0.0.6 (NAVOS v1) by default, and against navigator-lib
        0.1.2 (NAVOS v2) when IBBNAV_NAVOS_V2 is defined by the
        build system. Drives Ch14 / Ch16 through the standard
        max/min/neutral arm sequence.
*************************************************************/

#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <thread>

#include "bindings.h"

using std::cout;
using std::endl;

// ─── PWM pulse / frequency constants (shared between NAVOS versions) ──
static constexpr double PWM_FREQ_HZ   = 100.0;
static constexpr double PWM_MIN_US    = 800.0;
static constexpr double PWM_MAX_US    = 2200.0;
static constexpr double PWM_CENTER_US = 1500.0;

// ─── NAVOS version selection ──────────────────────────────────────────
//
// v1: PwmChannel is an enum class; pass PwmChannel::ChNN to the setter.
// v2: PwmChannel was removed; channels are plain 0-based uintptr_t
//     indices (Ch14 -> 13, Ch16 -> 15).
//
// CMake should define IBBNAV_NAVOS_V2=1 when building against
// navigator-lib 0.1.2 or newer. Otherwise we assume NAVOS v1.
#if defined(IBBNAV_NAVOS_V2) && IBBNAV_NAVOS_V2
  using PwmChannelType = uintptr_t;
  static constexpr PwmChannelType kPwmCh14 = 13;  // legacy Ch14 -> index 13
  static constexpr PwmChannelType kPwmCh16 = 15;  // legacy Ch16 -> index 15
#else
  using PwmChannelType = PwmChannel;
  static constexpr PwmChannelType kPwmCh14 = PwmChannel::Ch14;
  static constexpr PwmChannelType kPwmCh16 = PwmChannel::Ch16;
#endif

// Convert normalized command [-100,100] to a duty cycle in [0,1] and
// drive the PCA9685 via the navigator-lib duty-cycle setter. Both v1
// (0.0.6) and v2 (0.1.2) expose this function with the same name and
// the same float duty argument; only the channel type differs.
static void setPinPulseWidth(PwmChannelType pin_num, double target) {
  const double pulse_us_span = (PWM_MAX_US - PWM_MIN_US) / 2.0;
  double pulse_us = PWM_CENTER_US + (target / 100.0) * pulse_us_span;

  if (pulse_us < PWM_MIN_US) pulse_us = PWM_MIN_US;
  if (pulse_us > PWM_MAX_US) pulse_us = PWM_MAX_US;

  const double period_us = 1e6 / PWM_FREQ_HZ;
  double duty = pulse_us / period_us;
  if (duty < 0.0) duty = 0.0;
  if (duty > 1.0) duty = 1.0;

  set_pwm_channel_duty_cycle(pin_num, static_cast<float>(duty));
}

// Best-effort safe shutdown on signal: drive both ESCs neutral.
static void signalHandler(int /*signum*/) {
  setPinPulseWidth(kPwmCh14, 0);
  setPinPulseWidth(kPwmCh16, 0);
}

// Standard BlueRobotics-style ESC arm sequence: max -> min -> neutral.
static void initializeESC(PwmChannelType pin) {
  setPinPulseWidth(pin, 100);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  setPinPulseWidth(pin, -100);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  setPinPulseWidth(pin, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
}

static void showHelpAndExit() {
  cout << "BlueBoat ESC arming utility" << endl;
  cout << "  Arms ESCs on PWM channels 14 and 16." << endl;
#if defined(IBBNAV_NAVOS_V2) && IBBNAV_NAVOS_V2
  cout << "  Built for NAVOS v2 (navigator-lib 0.1.2)." << endl;
#else
  cout << "  Built for NAVOS v1 (navigator-lib 0.0.6)." << endl;
#endif
  std::exit(0);
}

int main(int ac, char* av[]) {
  for (int i = 1; i < ac; ++i) {
    const std::string a = av[i];
    if (a == "-h" || a == "--help") showHelpAndExit();
  }

  std::signal(SIGINT,  signalHandler);
  std::signal(SIGTERM, signalHandler);

  // navigator-lib 0.1.2 requires explicit hardware selection BEFORE
  // init(). v1 (0.0.6) doesn't expose these symbols at all, so the
  // entire prelude is gated out at preprocess time.
#if defined(IBBNAV_NAVOS_V2) && IBBNAV_NAVOS_V2
  set_navigator_version(NavigatorVersion::Version2);
  #if defined(IBBNAV_RASPBERRY_PI5) && IBBNAV_RASPBERRY_PI5
    set_raspberry_pi_version(Raspberry::Pi5);
  #else
    set_raspberry_pi_version(Raspberry::Pi4);
  #endif
#endif
  init();

  set_pwm_freq_hz(static_cast<float>(PWM_FREQ_HZ));
  set_pwm_enable(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  initializeESC(kPwmCh14);
  initializeESC(kPwmCh16);

  return 0;
}
