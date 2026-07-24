/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: test_bb_adc/main.cpp
   Last Ed:  2024-03-25. 
   9/24/25 Ethan Phan modified CalibrationParams for 4 batts
     Brief:
        A command line utility for testing ADC readings with
        different battery configurations for BlueBoat.
        Supports --nbats parameter to test calibration quality
        with different numbers of batteries.
*************************************************************/

#include "nav_bindings.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/utsname.h>
#include <unistd.h>
#include <csignal>
#include <chrono>
#include <thread>
#include <ctime>
#include <string>

using namespace std;

static Navigator g_nav;

// navigator-lib compatibility shim: keep the ADCData/read_adc_all()
// call shape the rest of this file was written against.
struct ADCData {
  float channel[4] = {0, 0, 0, 0};
};

static ADCData read_adc_all() {
  NavADCData d;
  g_nav.read_adc_all(d);
  ADCData out;
  for (int i = 0; i < 4; i++) out.channel[i] = d.channel[i];
  return out;
}

// Local time helpers (replaces lib_rpi_utils dependency)
namespace {
  double getTimeSinceEpoch() {
    auto now = std::chrono::system_clock::now();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch()).count();
    return static_cast<double>(millis) / 1000.0;
  }

  std::string getTimeStamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S",
                  std::localtime(&now_time));
    return std::string(buffer);
  }
}

// Calibration lookup table for different battery configurations
struct CalibrationParams {
    double offset;
    double gain;
};

// Lookup table indexed by number of batteries (1-8)
const CalibrationParams BATTERY_CALIBRATIONS[] = {
    {0.3235, 37.8788},  // 1 battery (default)
    {0.3235, 37.8788},  // 2 batteries - TODO: calibrate
    {0.3235, 37.8788},  // 3 batteries - TODO: calibrate
    // {1.6266, 100},  // 4 batteries - TODO: calibrate
    {1.616, 37.8788}, //4 batteries, calibrated I think?
    {0.3235, 37.8788},  // 5 batteries - TODO: calibrate
    {0.3235, 37.8788},  // 6 batteries - TODO: calibrate
    {0.3235, 37.8788},  // 7 batteries - TODO: calibrate
    {0.3235, 37.8788}   // 8 batteries - TODO: calibrate
};

void showHelpAndExit() {
    printf("Usage: test_bb_adc [OPTIONS]\n");
    printf("Options:\n");
    printf("  -s, --single      Single measurement (for logging)\n");
    printf("  -c, --continuous  Continuous mode\n");
    printf("  --nbats=N         Number of batteries (1-8, default=1)\n");
    printf("  --auto            Auto-calibrate offset for current battery configuration\n");
    printf("  -h, --help        Show this help message\n");
    exit(0);
};

void signalHandler(int signum)
{
  exit(signum);
}

void autoCalibrateOffset(int num_batteries) {
    const float TARGET_CURRENT = 0.45;  // Baseline system current (A)
    const int NUM_SAMPLES = 30;
    
    printf("=== Auto-Calibrating Offset for %d Battery(ies) ===\n", num_batteries);
    printf("Target current: %.2f A (baseline system load)\n", TARGET_CURRENT);
    printf("Collecting %d samples...\n", NUM_SAMPLES);
    
    CalibrationParams cal = BATTERY_CALIBRATIONS[num_batteries - 1];
    
    float sum_adc = 0;
    float sum_current = 0;
    
    // Collect samples
    for(int i = 0; i < NUM_SAMPLES; i++) {
        ADCData adc = read_adc_all();
        float current = (adc.channel[2] - cal.offset) * cal.gain;
        
        sum_adc += adc.channel[2];
        sum_current += current;
        
        printf("Sample %2d: ADC2=%.4f, Current=%.4f A\n", i+1, adc.channel[2], current);
        usleep(100000);  // 100ms delay
    }
    
    float avg_adc = sum_adc / NUM_SAMPLES;
    float avg_current = sum_current / NUM_SAMPLES;
    
    // Calculate new offset based on BlueOS method
    float new_offset = avg_adc - (TARGET_CURRENT / cal.gain);
    
    printf("\n=== Calibration Results ===\n");
    printf("Average ADC reading: %.4f\n", avg_adc);
    printf("Average current: %.4f A\n", avg_current);
    printf("Current offset: %.4f\n", cal.offset);
    printf("Recommended new offset: %.4f\n", new_offset);
    printf("Offset change: %.4f\n", new_offset - cal.offset);
    
    // Validation (reasonable range check)
    if (new_offset > 0.2 && new_offset < 0.5) {
        printf("✓ New offset is within reasonable range\n");
    } else {
        printf("⚠ Warning: New offset (%.4f) may be outside normal range (0.2-0.5)\n", new_offset);
    }
    
    printf("\nTo use this calibration, update BATTERY_CALIBRATIONS[%d] = {%.4f, %.4f};\n", 
           num_batteries-1, new_offset, cal.gain);
}

int main(int ac, char* av[]) {

  signal(SIGINT, signalHandler);

  bool single = false;
  bool continuous = false;
  bool auto_calibrate = false;
  int num_batteries = 1;

  // Parse user arguments
  for (int i = 1; i < ac; i++)
  {
    string argi = av[i];
    if ((argi == "-s") || (argi == "--single"))
      single = true;
    else if ((argi == "-c") || (argi == "--continuous"))
      continuous = true;
    else if (argi == "--auto")
      auto_calibrate = true;
    else if ((argi == "-h") || (argi == "--help"))
      showHelpAndExit();
    else if (argi.find("--nbats=") == 0) {
      string nbats_str = argi.substr(8);
      num_batteries = stoi(nbats_str);
      if (num_batteries < 1 || num_batteries > 8) {
        printf("Error: Number of batteries must be between 1 and 8\n");
        exit(1);
      }
    }
  }

  {
    std::string err = g_nav.init();
    if (!err.empty())
      fprintf(stderr, "navigator init warnings: %s\n", err.c_str());
  }

  // Get calibration parameters for the specified number of batteries
  CalibrationParams cal = BATTERY_CALIBRATIONS[num_batteries - 1];
  
  printf("Using calibration for %d battery(ies): offset=%.4f, gain=%.4f\n", 
         num_batteries, cal.offset, cal.gain);

  // Auto-calibrate if requested
  if (auto_calibrate) {
    autoCalibrateOffset(num_batteries);
    return 0;
  }

  if(!single){
  ADCData adc = read_adc_all();
  printf("<Testing ADC - %d Battery(ies)>\n", num_batteries);
  printf("Reading ADC Channels: \n\t- ADC0  = %f \n\t- ADC1  = %f \n\t- ADC2* "
         "= %f \n\t- ADC3* = %f\n",
         adc.channel[0], adc.channel[1], adc.channel[2], adc.channel[3]);

  // https://bluerobotics.com/store/comm-control-power/control/psm-asm-r2-rp/
  float current = (adc.channel[2] - cal.offset) * cal.gain;
  float voltage = adc.channel[3] * 11.132;

  printf("Calibrated Current: %0.4f A\n", current);
  printf("Supplied Voltage: %0.2f V\n", voltage);
  printf("Number of Batteries: %d\n", num_batteries);

  printf("</Testing ADC>\n");
  
  // If continuous mode, keep updating
  if (continuous) {
    while (true) {
      adc = read_adc_all();
      current = (adc.channel[2] - cal.offset) * cal.gain;
      voltage = adc.channel[3] * 11.132;
      
      printf("\r[%d bat] Current: %7.4f A, Voltage: %5.2f V, ADC2: %7.4f  ", 
             num_batteries, current, voltage, adc.channel[2]);
      fflush(stdout);
      
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
} else {
  
  ADCData adc = read_adc_all();

  float current = (adc.channel[2] - cal.offset) * cal.gain;
  float voltage = adc.channel[3] * 11.132;

  printf("%s: %12.2f, NBats: %d, Current: %7.4f, Voltage: %5.3f, ADC0: %2.3f, ADC1: %2.3f, ADC2: %2.3f, ADC3: %2.3f\n",
          getTimeStamp().c_str(),
          getTimeSinceEpoch(),
          num_batteries,
          current, 
          voltage, 
          adc.channel[0], 
          adc.channel[1], 
          adc.channel[2], 
          adc.channel[3]);
}
  return 0;
}
