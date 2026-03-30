/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: cal_fins/main.cpp
   Last Ed:  2024-03-25
     Brief:

*************************************************************/

#include "bindings.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <csignal>

using namespace std;

// PWM frequency and pulse range constants (full range, 100 Hz)
static constexpr double PWM_FREQ_HZ = 100.0;      // 100 Hz for smoother response
static constexpr double PWM_MIN_US = 800.0;       // Minimum pulse width (microseconds)
static constexpr double PWM_MAX_US = 2200.0;      // Maximum pulse width (microseconds)
static constexpr double PWM_CENTER_US = 1500.0;   // Neutral pulse width (microseconds)

// Convert normalized command [-100,100] to PCA9685 counts
void setPinPulseWidth(PwmChannel pin_num, double target)
{
  // Map normalized command [-100,100] to pulse range
  double pulse_us_span = (PWM_MAX_US - PWM_MIN_US) / 2.0;
  double pulse_us = PWM_CENTER_US + (target / 100.0) * pulse_us_span;

  // Clamp to allowable range
  if (pulse_us < PWM_MIN_US) pulse_us = PWM_MIN_US;
  if (pulse_us > PWM_MAX_US) pulse_us = PWM_MAX_US;

  // Convert microseconds to PCA9685 counts: counts = pulse_us * freq_hz * 4096 / 1e6
  double counts = pulse_us * PWM_FREQ_HZ * 4096.0 / 1e6;

  if (counts < 0.0) counts = 0.0;
  if (counts > 4095.0) counts = 4095.0;

  set_pwm_channel_value(pin_num, static_cast<uint16_t>(counts));
}

void signalHandler(int signum)
{
  // Set all servos to a neutral position before disabling PWM
  setPinPulseWidth(PwmChannel::Ch14, 0);
  setPinPulseWidth(PwmChannel::Ch16, 0);
}

void initializeESC(PwmChannel pin)
{
  // Set to Maximum Throttle
  setPinPulseWidth(pin, 100);                        // Assuming 100% corresponds to 2000 microseconds
  this_thread::sleep_for(chrono::milliseconds(500)); // Wait for 0.5 seconds

  // Set to Minimum Throttle
  setPinPulseWidth(pin, -100);                       // Assuming 0% corresponds to 1000 microseconds
  this_thread::sleep_for(chrono::milliseconds(500)); // Wait for 0.5 seconds

  // Optional: Set to Neutral Throttle (e.g., for ESCs that need this)
  setPinPulseWidth(pin, 0);                          // Assuming 50% corresponds to 1500 microseconds
  this_thread::sleep_for(chrono::milliseconds(250)); // Wait for 0.25 second
}

void showHelpAndExit()
{
  cout << "Sea Scout Fin Calibration Utility                                            " << endl;
  exit(0);
}

int main(int ac, char *av[])
{
  //todo: initialize LEDs here
  init();
  set_pwm_freq_hz(static_cast<float>(PWM_FREQ_HZ));
  set_pwm_enable(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  initializeESC(PwmChannel::Ch14);
  initializeESC(PwmChannel::Ch16);
}
