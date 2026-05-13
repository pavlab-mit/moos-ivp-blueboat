/*************************************************************
      Name: Raymond Turrisi (orig.) / Ported to navigator-lib 0.1.2
      Orgn: MIT, Cambridge MA
      File: init_bb_pwm/main.cpp
   Last Ed: 2026-05-13
     Brief:
        BlueBoat ESC arming utility, ported to navigator-lib
        0.1.2 (a.k.a. "v2"). Drives Ch14 / Ch16 through the
        standard max/min/neutral arm sequence.
*************************************************************/

#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <thread>

#include "bindings.h"

using std::cout;
using std::endl;

// PWM frequency and pulse range constants (full range, 100 Hz)
static constexpr double PWM_FREQ_HZ   = 100.0;
static constexpr double PWM_MIN_US    = 800.0;
static constexpr double PWM_MAX_US    = 2200.0;
static constexpr double PWM_CENTER_US = 1500.0;

// navigator-lib 0.1.2: PWM channels are 0-based indices.
// Legacy mapping: PwmChannel::Ch14 -> 13, PwmChannel::Ch16 -> 15.
static constexpr uintptr_t kPwmIndexCh14 = 13;
static constexpr uintptr_t kPwmIndexCh16 = 15;

// Convert normalized command [-100,100] to a duty cycle in [0,1]
// for set_pwm_channel_duty_cycle (matches v2 navigator interface).
static void setPinPulseWidth(uintptr_t pin_num, double target) {
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
  setPinPulseWidth(kPwmIndexCh14, 0);
  setPinPulseWidth(kPwmIndexCh16, 0);
}

// Standard BlueRobotics-style ESC arm sequence: max -> min -> neutral.
static void initializeESC(uintptr_t pin) {
  setPinPulseWidth(pin, 100);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  setPinPulseWidth(pin, -100);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  setPinPulseWidth(pin, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
}

static void showHelpAndExit() {
  cout << "BlueBoat ESC arming utility (navigator-lib 0.1.2)" << endl;
  cout << "  Arms ESCs on PWM channels 14 and 16."           << endl;
  std::exit(0);
}

int main(int ac, char* av[]) {
  for (int i = 1; i < ac; ++i) {
    const std::string a = av[i];
    if (a == "-h" || a == "--help") showHelpAndExit();
  }

  std::signal(SIGINT,  signalHandler);
  std::signal(SIGTERM, signalHandler);

  // navigator-lib 0.1.2 requires explicit hardware selection
  // BEFORE init(). Pi version is wired in via the CMake-set
  // IBBNAV_RASPBERRY_PI5 define, matching iBBNavigatorInterface_v2.
  set_navigator_version(NavigatorVersion::Version2);
#if defined(IBBNAV_RASPBERRY_PI5) && IBBNAV_RASPBERRY_PI5
  set_raspberry_pi_version(Raspberry::Pi5);
#else
  set_raspberry_pi_version(Raspberry::Pi4);
#endif
  init();

  set_pwm_freq_hz(static_cast<float>(PWM_FREQ_HZ));
  set_pwm_enable(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  initializeESC(kPwmIndexCh14);
  initializeESC(kPwmIndexCh16);

  return 0;
}
