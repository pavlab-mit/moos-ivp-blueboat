/*************************************************************
      Orgn: MIT, Cambridge MA
      File: bb_adc/main.cpp
     Brief:
        Minimal BlueBoat battery-voltage reader. Reads the
        Navigator ADC and prints the pack voltage so the boot
        script (bb_init.sh) can use it as a low-battery launch
        gate.

        Builds against navigator-lib 0.0.6 (NAVOS v1) by default
        and 0.1.2 (NAVOS v2) when IBBNAV_NAVOS_V2 is defined by
        the build system. The ADC bulk-read API changed between
        the two releases, so the read is gated below.

        Default output is a single line:
            VOLTAGE=15.83
        which is trivial to grep/parse. Pass --raw to print all
        four raw ADC channels for debugging.
*************************************************************/

#include <cstdint>
#include <cstdio>
#include <string>

#include "bindings.h"

// PSM voltage divider: ADC channel 3 (0-indexed) -> pack volts.
// Matches the constant used by iBBNavigatorInterface / test_bb_adc.
// https://bluerobotics.com/store/comm-control-power/control/psm-asm-r2-rp/
static constexpr float kVoltageScale = 11.132f;
static constexpr int   kVoltageChannel = 3;

static void showHelpAndExit() {
  std::printf("Usage: bb_adc [options]\n");
  std::printf("  -h, --help   Show this help message\n");
  std::printf("  --raw        Print all 4 raw ADC channels as well\n");
  std::printf("Output (default): VOLTAGE=<volts>\n");
  std::exit(0);
}

// Read all four ADC channels into adc[0..3]. The bulk-read signature
// differs between NAVOS versions, so isolate it here.
static void readAdcChannels(float adc[4]) {
#if defined(IBBNAV_NAVOS_V2) && IBBNAV_NAVOS_V2
  // 0.1.2: void read_adc_all(float* buf, uintptr_t len)
  read_adc_all(adc, 4);
#else
  // 0.0.6: ADCData read_adc_all() returns a struct with .channel[4]
  ADCData data = read_adc_all();
  for (int i = 0; i < 4; ++i)
    adc[i] = data.channel[i];
#endif
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

  // navigator-lib 0.1.2 requires explicit hardware selection BEFORE init().
  // v1 (0.0.6) does not expose these symbols, so gate them out.
#if defined(IBBNAV_NAVOS_V2) && IBBNAV_NAVOS_V2
  set_navigator_version(NavigatorVersion::Version2);
  #if defined(IBBNAV_RASPBERRY_PI5) && IBBNAV_RASPBERRY_PI5
  set_raspberry_pi_version(Raspberry::Pi5);
  #else
  set_raspberry_pi_version(Raspberry::Pi4);
  #endif
#endif
  init();

  float adc[4] = {0, 0, 0, 0};
  readAdcChannels(adc);

  float voltage = adc[kVoltageChannel] * kVoltageScale;

  if (raw) {
    std::printf("ADC0=%.4f ADC1=%.4f ADC2=%.4f ADC3=%.4f\n",
                adc[0], adc[1], adc[2], adc[3]);
  }
  std::printf("VOLTAGE=%.2f\n", voltage);

  return 0;
}
