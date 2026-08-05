/*************************************************************
      Name: Raymond Turrisi (orig.)
      Orgn: MIT, Cambridge MA
      File: init_bb_pwm/main.cpp
   Last Ed: 2026-07-24 (navigator-cpp port)
     Brief:
        BlueBoat ESC arming utility, built on navigator-cpp
        (runtime Navigator V1/V2 + Pi 4/5 detection). Arms the
        ESCs on Ch14 / Ch16 with a neutral hold by default, or
        the legacy max/min/neutral sweep with --sweep.

        WARNING: --sweep commands full throttle. Never run it
        against ESCs that are already powered and armed.
*************************************************************/

#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "nav_bindings.h"

using std::cout;
using std::endl;

// Pulse range matches the Basic ESC 500's documented 1100-1900 us
// input range, so a --sweep never sends the ESC an out-of-spec pulse.
static constexpr double PWM_FREQ_HZ   = 100.0;
static constexpr double PWM_MIN_US    = 1100.0;
static constexpr double PWM_MAX_US    = 1900.0;
static constexpr double PWM_CENTER_US = 1500.0;

// PWM channels are 0-based indices (legacy Ch14 -> 13, Ch16 -> 15).
static constexpr int kPwmCh14 = 13;
static constexpr int kPwmCh16 = 15;

static Navigator g_nav;

// Convert normalized command [-100,100] to a pulse width in us.
static void setPinPulseWidth(int pin_num, double target) {
  const double pulse_us_span = (PWM_MAX_US - PWM_MIN_US) / 2.0;
  double pulse_us = PWM_CENTER_US + (target / 100.0) * pulse_us_span;

  if (pulse_us < PWM_MIN_US) pulse_us = PWM_MIN_US;
  if (pulse_us > PWM_MAX_US) pulse_us = PWM_MAX_US;

  g_nav.pwm_set_pulse_us(pin_num, static_cast<float>(pulse_us));
}

// Best-effort safe shutdown on signal: drive both ESCs neutral.
static void signalHandler(int /*signum*/) {
  setPinPulseWidth(kPwmCh14, 0);
  setPinPulseWidth(kPwmCh16, 0);
}

// Legacy BlueRobotics-style ESC arm sequence: max -> min -> neutral.
static void sweepArm(int pin) {
  setPinPulseWidth(pin, 100);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  setPinPulseWidth(pin, -100);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  setPinPulseWidth(pin, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
}

static void showHelpAndExit() {
  cout << "BlueBoat ESC arming utility (navigator-cpp)" << endl;
  cout << "  Arms ESCs on PWM channels 14 and 16." << endl;
  cout << "Options:" << endl;
  cout << "  --sweep    Legacy max/min/neutral throttle sweep instead of" << endl;
  cout << "             the neutral hold. DANGEROUS on armed ESCs." << endl;
  cout << "  -h, --help Show this help message" << endl;
  std::exit(0);
}

int main(int ac, char* av[]) {
  bool sweep = false;
  for (int i = 1; i < ac; ++i) {
    const std::string a = av[i];
    if (a == "-h" || a == "--help") showHelpAndExit();
    else if (a == "--sweep") sweep = true;
  }

  std::signal(SIGINT,  signalHandler);
  std::signal(SIGTERM, signalHandler);

  std::string err = g_nav.init();
  if (!err.empty())
    std::cerr << "navigator init warnings: " << err << endl;

  g_nav.pwm_set_frequency(static_cast<float>(PWM_FREQ_HZ));
  // Neutral before enabling output so the first pulse is 1500us.
  setPinPulseWidth(kPwmCh14, 0);
  setPinPulseWidth(kPwmCh16, 0);
  g_nav.pwm_enable(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  if (sweep) {
    sweepArm(kPwmCh14);
    sweepArm(kPwmCh16);
  } else {
    // BlueRobotics Basic ESCs arm on a stable neutral signal.
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  return 0;
}
