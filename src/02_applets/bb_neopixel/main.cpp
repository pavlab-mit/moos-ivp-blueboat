/*************************************************************
      Name: Raymond Turrisi (orig. test_discoturtle)
      Orgn: MIT, Cambridge MA
      File: bb_neopixel/main.cpp
     Brief:
        NeoPixel (sk6812 / WS2812B) status control for BlueBoat.
        Ported from moos-ivp-seascout's test_discoturtle and
        updated for NAVOS v1 / v2 dual-target builds.

        Boats do not currently ship with an LED strip installed,
        so bb_init.sh leaves this disabled by default (BB_NEOPIXEL
        in /etc/default/boat). It is kept building so the strip can
        be dropped back in without re-porting:
            bb_neopixel rainbow [-d <secs>]   idle / not launched
            bb_neopixel navlights              launched (green/red)
            bb_neopixel off

        Built on navigator-cpp (runtime Navigator V1/V2 and
        Pi 4/5 detection) - one build everywhere.
*************************************************************/

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <string>
#include <thread>

#include "nav_bindings.h"

static Navigator g_nav;

// Strip length. The original discoturtle ring was 24 LEDs; override with
// --count if your strip differs.
static int g_num_leds = 24;
static uint8_t g_rgbw[256][4];  // max 256 LEDs; only g_num_leds used

static void push() { g_nav.neopixel_set_rgbw(g_rgbw, g_num_leds); }

static void allOff() {
  std::memset(g_rgbw, 0, sizeof(g_rgbw));
  push();
}

static void signalHandler(int signum) {
  allOff();
  std::exit(signum);
}

// Nav lights: first half green, second half red.
static void navLights() {
  std::memset(g_rgbw, 0, sizeof(g_rgbw));
  for (int led = 0; led < g_num_leds; ++led) {
    if (led < g_num_leds / 2) g_rgbw[led][1] = 255;  // green
    else                      g_rgbw[led][0] = 255;  // red
  }
  push();
}

// Rotating rainbow for `duration` seconds (<=0 means run until killed).
static void rainbow(double duration) {
  const auto start = std::chrono::steady_clock::now();
  float offset = 0;
  while (true) {
    std::memset(g_rgbw, 0, sizeof(g_rgbw));
    for (int led = 0; led < g_num_leds; ++led) {
      float hue = std::fmod((led / (float)g_num_leds * 360.0f) + offset, 360.0f);
      float c = 1.0f;
      float x = c * (1 - std::fabs(std::fmod(hue / 60.0f, 2.0f) - 1));
      float r, g, b;
      if      (hue <  60) { r = c; g = x; b = 0; }
      else if (hue < 120) { r = x; g = c; b = 0; }
      else if (hue < 180) { r = 0; g = c; b = x; }
      else if (hue < 240) { r = 0; g = x; b = c; }
      else if (hue < 300) { r = x; g = 0; b = c; }
      else                { r = c; g = 0; b = x; }
      g_rgbw[led][0] = static_cast<uint8_t>(r * 255);
      g_rgbw[led][1] = static_cast<uint8_t>(g * 255);
      g_rgbw[led][2] = static_cast<uint8_t>(b * 255);
    }
    push();

    offset += 2;
    if (offset >= 360) offset -= 360;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    if (duration > 0) {
      double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - start).count() / 1000.0;
      if (elapsed >= duration) break;
    }
  }
  allOff();
}

static void showHelpAndExit() {
  std::printf("BlueBoat NeoPixel status utility\n");
  std::printf("Usage: bb_neopixel <command> [options]\n");
  std::printf("Commands:\n");
  std::printf("  rainbow            Rotating rainbow (idle / not launched)\n");
  std::printf("  navlights          Green/red navigation lights (launched)\n");
  std::printf("  off                Turn all LEDs off\n");
  std::printf("Options:\n");
  std::printf("  -d, --duration <s> Duration for rainbow (default: run until killed)\n");
  std::printf("      --count <n>    Number of LEDs on the strip (default 24)\n");
  std::printf("  -h, --help         Show this help message\n");
  std::exit(0);
}

int main(int ac, char* av[]) {
  if (ac < 2) showHelpAndExit();

  std::string cmd;
  double duration = 0.0;  // 0 == run until killed

  for (int i = 1; i < ac; ++i) {
    const std::string a = av[i];
    if (a == "-h" || a == "--help") showHelpAndExit();
    else if (a == "rainbow" || a == "navlights" || a == "off") cmd = a;
    else if (a == "-d" || a == "--duration") { if (++i < ac) duration = std::stod(av[i]); }
    else if (a == "--count") {
      if (++i < ac) {
        g_num_leds = std::stoi(av[i]);
        if (g_num_leds < 1) g_num_leds = 1;
        if (g_num_leds > 256) g_num_leds = 256;
      }
    }
    else {
      std::fprintf(stderr, "Unhandled argument: %s\n", a.c_str());
      return 1;
    }
  }

  if (cmd.empty()) showHelpAndExit();

  std::signal(SIGINT,  signalHandler);
  std::signal(SIGTERM, signalHandler);

  {
    std::string err = g_nav.init();
    if (!err.empty())
      std::fprintf(stderr, "navigator init warnings: %s\n", err.c_str());
  }

  if (cmd == "off")            allOff();
  else if (cmd == "navlights") navLights();
  else                         rainbow(duration);

  return 0;
}
