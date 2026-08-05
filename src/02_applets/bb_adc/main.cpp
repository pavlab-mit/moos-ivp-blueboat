/*************************************************************
      Orgn: MIT, Cambridge MA
      File: bb_adc/main.cpp
   Last Ed: 2026-07-25 (navigator-cpp port)
     Brief:
        Minimal BlueBoat battery-voltage reader. Reads the
        Navigator ADC and prints the pack voltage so the boot
        script (bb_init.sh) can use it as a low-battery launch
        gate.

        Built on navigator-cpp (runtime Navigator V1/V2 and
        Pi 4/5 detection) - one build everywhere.

        Default output is a single line:
            VOLTAGE=15.83
        which is trivial to grep/parse (bb_init.sh greps
        'VOLTAGE=[0-9.]+'). Pass --raw to print all four raw
        ADC channels for debugging.
*************************************************************/

#include <cstdint>
#include <cstdio>
#include <string>

#include "nav_bindings.h"

// PSM voltage divider: ADC channel 3 (0-indexed) -> pack volts.
// Matches the constant used by iBBNavigatorInterface / test_bb_adc.
// https://bluerobotics.com/store/comm-control-power/control/psm-asm-r2-rp/
static constexpr float kVoltageScale = 11.132f;
static constexpr int   kVoltageChannel = 3;

static Navigator g_nav;

static void showHelpAndExit() {
  std::printf("Usage: bb_adc [options]\n");
  std::printf("  -h, --help   Show this help message\n");
  std::printf("  --raw        Print all 4 raw ADC channels as well\n");
  std::printf("Output (default): VOLTAGE=<volts>\n");
  std::exit(0);
}

int main(int ac, char* av[]) {
  bool raw = false;
  for (int i = 1; i < ac; ++i) {
    const std::string a = av[i];
    if (a == "-h" || a == "--help") showHelpAndExit();
    else if (a == "--raw") raw = true;
    else {
      std::fprintf(stderr, "Unhandled argument: %s\n", a.c_str());
      return 1;
    }
  }

  {
    std::string err = g_nav.init();
    if (!err.empty())
      std::fprintf(stderr, "navigator init warnings: %s\n", err.c_str());
  }

  NavADCData adc;
  std::string err = g_nav.read_adc_all(adc);
  if (!err.empty()) {
    // A failed read must NOT print a parseable VOLTAGE= line - the boot
    // script fails closed on a missing value rather than gating on 0.00.
    std::fprintf(stderr, "adc read failed: %s\n", err.c_str());
    return 1;
  }

  float voltage = adc.channel[kVoltageChannel] * kVoltageScale;

  if (raw) {
    std::printf("ADC0=%.4f ADC1=%.4f ADC2=%.4f ADC3=%.4f\n",
                adc.channel[0], adc.channel[1], adc.channel[2], adc.channel[3]);
  }
  std::printf("VOLTAGE=%.2f\n", voltage);

  g_nav.shutdown();
  return 0;
}
