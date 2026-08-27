/*************************************************************
 *  NavSensors -- AHRS, power, environment, lights.
 *
 *  The half of iBBNavigatorInterface that cannot spin a
 *  propeller. It shares only the navigator-cpp handle with
 *  NavActuator.
 *
 *  Three fixes carried in from the audit (plan section 16.3):
 *
 *   - read_mag(), not read_mag_ak09915(). navigator-cpp main
 *     dispatches V1 -> AK09915 and V2 -> MMC5983 with an AK
 *     fallback. The old hardcoded call read a magnetometer that
 *     may not exist on a V2 board -- latent only because the
 *     fleet runs use_mag = false.
 *
 *   - the loop sleeps against an absolute deadline. The old one
 *     computed its sleep from the PREVIOUS iteration's total
 *     period rather than this iteration's work, so it alternated
 *     sleeping and running flat out and averaged
 *     (work + nominal)/2 -- a thread configured for 150 Hz ran
 *     nearer 200-250.
 *
 *   - the achieved rate is measured and published. That class of
 *     timing bug was invisible for as long as nobody could see
 *     the rate; NVGR_SENSOR_RATE_HZ makes the next one obvious.
 *
 *  Author: Jeremy Wenger
 *************************************************************/

#include "BBNavigatorInterface.h"

#include "MBUtils.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <thread>

using namespace std;

extern Navigator g_nav;

// Clamp in place. Free function because the sensor thread and the
// original actuation path both used it; only the sensor half
// still does.
void rclamp(double &val, double min, double max)
{
  if (val < min)      val = min;
  else if (val > max) val = max;
}

// Battery current calibration, indexed by battery count.
// Preserved verbatim from the pre-rewrite app: only the 4-battery
// row has ever been measured.
struct CalibrationParams { double offset; double gain; };
static const CalibrationParams BATTERY_CALIBRATIONS[] = {
    {0.3235, 37.8788},  // 1 battery
    {0.3235, 37.8788},  // 2 batteries - TODO: calibrate
    {0.3235, 37.8788},  // 3 batteries - TODO: calibrate
    {1.616,  37.8788},  // 4 batteries - calibrated
    {0.3235, 37.8788},  // 5 batteries - TODO: calibrate
    {0.3235, 37.8788},  // 6 batteries - TODO: calibrate
    {0.3235, 37.8788},  // 7 batteries - TODO: calibrate
    {0.3235, 37.8788}   // 8 batteries - TODO: calibrate
};

static void mat3_mult(const double A[9], const double B[9], double C[9])
{
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      C[3 * r + c] = A[3 * r + 0] * B[0 + c] +
                     A[3 * r + 1] * B[3 + c] +
                     A[3 * r + 2] * B[6 + c];
}

static void mat3_vec(const double A[9], const double v[3], double out[3])
{
  for (int r = 0; r < 3; r++)
    out[r] = A[3 * r] * v[0] + A[3 * r + 1] * v[1] + A[3 * r + 2] * v[2];
}

bool BBNavigatorInterface::readMagCalFile(std::string filename)
{
  std::ifstream file(filename);
  if (!file.is_open())
  {
    reportRunWarning("Could not open calibration file: " + filename);
    dbg_print("Could not open file\n");
    return false;
  }

  std::string line;
  while (std::getline(file, line))
  {
    if (line.empty() || line[0] == '#')
      continue;

    string arg = biteString(line, '=');
    arg = removeWhite(arg);

    if (arg == "b")
    {
      std::vector<string> values = parseString(line, ',');
      if (values.size() >= 3)
        for (int i = 0; i < 3; i++)
          m_mag_bias[i] = stod(values[i]);
    }
    else if (arg == "A")
    {
      std::vector<string> values = parseString(line, ',');
      if (values.size() >= 9)
        for (int i = 0; i < 9; i++)
          m_mag_scale[i] = stod(values[i]);
    }
  }

  file.close();
  m_have_mag_cal = true;
  return true;
}

// Accepts both cal-file formats:
//   gyro_bias  = x,y,z   /  accel_bias = x,y,z   (interface format)
//   bias_x = v / bias_y = v / bias_z = v          (calibration-script format)
// The second form fixes the long-standing mismatch where the gyro cal
// script's output couldn't be parsed and the bias silently stayed zero.
bool BBNavigatorInterface::readImuCalFile(std::string filename)
{
  std::ifstream file(filename);
  if (!file.is_open())
  {
    reportRunWarning("Could not open IMU calibration file: " + filename);
    return false;
  }

  std::string line;
  for (int i = 0; i < 3; i++)
  {
    m_gyro_bias[i] = 0.0;
    m_accel_bias[i] = 0.0;
  }

  while (std::getline(file, line))
  {
    if (line.empty() || line[0] == '#')
      continue;

    string arg = biteString(line, '=');
    arg = removeWhite(arg);
    string rest = stripBlankEnds(line);

    if (arg == "gyro_bias")
    {
      std::vector<string> values = parseString(rest, ',');
      if (values.size() >= 3)
        for (int i = 0; i < 3; i++)
          m_gyro_bias[i] = stod(values[i]);
    }
    else if (arg == "accel_bias")
    {
      std::vector<string> values = parseString(rest, ',');
      if (values.size() >= 3)
        for (int i = 0; i < 3; i++)
          m_accel_bias[i] = stod(values[i]);
    }
    else if (arg == "bias_x")
      m_gyro_bias[0] = stod(rest);
    else if (arg == "bias_y")
      m_gyro_bias[1] = stod(rest);
    else if (arg == "bias_z")
      m_gyro_bias[2] = stod(rest);
  }

  file.close();
  return true;
}

//---------------------------------------------------------
// Small fixed-size linear algebra (replaces Armadillo).

void BBNavigatorInterface::sensorSamplingThread()
{
  auto prev_time = std::chrono::high_resolution_clock::now();

  double roll_offset_rad = m_roll_offset * M_PI / 180;
  double pitch_offset_rad = m_pitch_offset * M_PI / 180;
  double yaw_offset_rad = m_yaw_offset * M_PI / 180;

  // Mounting-offset rotation R_offset = R_yaw * R_pitch * R_roll
  const double R_roll[9] = {1, 0, 0,
                            0, cos(roll_offset_rad), -sin(roll_offset_rad),
                            0, sin(roll_offset_rad), cos(roll_offset_rad)};
  const double R_pitch[9] = {cos(pitch_offset_rad), 0, sin(pitch_offset_rad),
                             0, 1, 0,
                             -sin(pitch_offset_rad), 0, cos(pitch_offset_rad)};
  const double R_yaw[9] = {cos(yaw_offset_rad), -sin(yaw_offset_rad), 0,
                           sin(yaw_offset_rad), cos(yaw_offset_rad), 0,
                           0, 0, 1};
  double R_tmp[9], R_offset[9];
  mat3_mult(R_pitch, R_roll, R_tmp);
  mat3_mult(R_yaw, R_tmp, R_offset);

  const double nominal_dt = 1.0 / m_sample_frequency;
  const auto period_dur =
      std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
          std::chrono::duration<double>(nominal_dt));

  auto next_deadline     = std::chrono::high_resolution_clock::now();
  auto rate_window_start = next_deadline;
  long rate_samples      = 0;

  while (m_ahrs_running)
  {
    auto current_time = std::chrono::high_resolution_clock::now();

    // Read sensor data from Navigator. navigator-cpp returns error
    // strings instead of throwing; count failures and skip the fusion
    // step for incomplete samples so a transient bus error can never
    // inject zeros into the estimator.
    NavAxisData accel_raw, gyro_raw;
    std::string err_a = g_nav.read_accel(accel_raw);
    std::string err_g = g_nav.read_gyro(gyro_raw);

    NavAxisData mag_raw;
    bool mag_ok = false;
    if (m_use_mag)
      mag_ok = g_nav.read_mag(mag_raw).empty();

    // Plausibility screen. Corrupted SPI transactions (observed under
    // bus contention on zoe) return SUCCESSFULLY with full-scale
    // register patterns - e.g. gyro exactly +/-250 dps (4.3657 rad/s),
    // accel exactly +/-2 g (19.61 m/s^2) or all-zero - so an error
    // check alone cannot catch them. Reject any sample whose accel
    // norm is non-physical or whose gyro sits at/beyond the +/-250 dps
    // configured full scale (a surface boat never legitimately gets
    // there); a bad sample must never reach the estimator or the
    // published gyro.
    bool sample_ok = err_a.empty() && err_g.empty();
    if (sample_ok)
    {
      const double anorm = sqrt((double)accel_raw.x * accel_raw.x +
                                (double)accel_raw.y * accel_raw.y +
                                (double)accel_raw.z * accel_raw.z);
      const double gyro_fs = 4.3;  // just under 250 dps full scale, rad/s
      if (anorm < 2.0 || anorm > 25.0 ||
          fabs(accel_raw.x) > 19.0 || fabs(accel_raw.y) > 19.0 ||
          fabs(accel_raw.z) > 19.0 ||
          fabs(gyro_raw.x) > gyro_fs || fabs(gyro_raw.y) > gyro_fs ||
          fabs(gyro_raw.z) > gyro_fs)
      {
        sample_ok = false;
        std::lock_guard<std::mutex> lock(m_ahrs_mutex);
        m_imu_glitch_count++;
      }
    }

    if (sample_ok)
    {
      // Apply calibrations (bias subtract) in the sensor frame
      double gyro_cal[3] = {gyro_raw.x - m_gyro_bias[0],
                            gyro_raw.y - m_gyro_bias[1],
                            gyro_raw.z - m_gyro_bias[2]};
      double acc_cal[3] = {accel_raw.x - m_accel_bias[0],
                           accel_raw.y - m_accel_bias[1],
                           accel_raw.z - m_accel_bias[2]};

      // Apply body frame rotation
      double gyro_bff[3], acc_bff[3];
      mat3_vec(R_offset, gyro_cal, gyro_bff);
      mat3_vec(R_offset, acc_cal, acc_bff);

      double mag_bff[3] = {0.0, 0.0, 0.0};
      if (m_use_mag && mag_ok)
      {
        double mag_c[3] = {mag_raw.x - m_mag_bias[0],
                           mag_raw.y - m_mag_bias[1],
                           mag_raw.z - m_mag_bias[2]};
        double mag_cal[3];
        mat3_vec(m_mag_scale, mag_c, mag_cal);
        mat3_vec(R_offset, mag_cal, mag_bff);
      }

      // Coerce dt: wall-clock hiccups (scheduling, NTP steps) must not
      // reach the integrator. Allgeuer recommends clamping to a band
      // around the nominal period.
      double dt = std::chrono::duration<double>(
                      current_time - prev_time).count();
      if (dt < 0.5 * nominal_dt) dt = 0.5 * nominal_dt;
      if (dt > 3.0 * nominal_dt) dt = 3.0 * nominal_dt;

      g_nav.ahrs_update(dt,
                        gyro_bff[0], gyro_bff[1], gyro_bff[2],
                        acc_bff[0], acc_bff[1], acc_bff[2],
                        mag_bff[0], mag_bff[1], mag_bff[2]);

      NavAttitudeData att;
      g_nav.ahrs_get_attitude(att);

      {
        std::lock_guard<std::mutex> lock(m_ahrs_mutex);
        m_roll = att.roll * 180.0 / M_PI;
        m_pitch = att.pitch * 180.0 / M_PI;
        m_yaw = att.yaw * 180.0 / M_PI;
        m_heading = fmod((m_yaw + m_declination_deg + m_operating_heading_offset), 360.0);
        if (m_heading < 0)
          m_heading += 360.0;

        m_qw = att.qw;
        m_qx = att.qx;
        m_qy = att.qy;
        m_qz = att.qz;

        m_gyro_x = gyro_bff[0];
        m_gyro_y = gyro_bff[1];
        m_gyro_z = gyro_bff[2];

        m_accel_x = acc_bff[0];
        m_accel_y = acc_bff[1];
        m_accel_z = acc_bff[2];

        // Level-frame yaw rate: rotate the body angular velocity into
        // the world frame with the attitude quaternion and take the z
        // component. Unlike the Euler yaw-rate projection
        // (q*sin(phi)+r*cos(phi))/cos(theta), this is bounded by
        // |omega| for ANY attitude - there is no cos(pitch)
        // singularity to guard, which was failure mode B of the
        // NAV_HEADING spike investigation. Third row of R(q):
        const double zr0 = 2.0 * (m_qx * m_qz - m_qw * m_qy);
        const double zr1 = 2.0 * (m_qy * m_qz + m_qw * m_qx);
        const double zr2 = 1.0 - 2.0 * (m_qx * m_qx + m_qy * m_qy);
        double yaw_rate = zr0 * gyro_bff[0] + zr1 * gyro_bff[1] +
                          zr2 * gyro_bff[2];

        // Physical plausibility clamp (belt and suspenders - the
        // projection above is already bounded by the gyro range).
        rclamp(yaw_rate, -m_yaw_rate_clamp, m_yaw_rate_clamp);
        m_yaw_rate = yaw_rate;
      }
    }
    else if (!err_a.empty() || !err_g.empty())
    {
      std::lock_guard<std::mutex> lock(m_ahrs_mutex);
      m_imu_read_errors++;
      m_last_sensor_error = !err_a.empty() ? err_a : err_g;
    }

    // Pace against an ABSOLUTE deadline.
    //
    // The old loop computed its sleep as
    //     period - (current_time - prev_time)
    // where that difference is the PREVIOUS iteration's total
    // period, not this iteration's work. It therefore alternated
    // between sleeping a full period and not sleeping at all, and
    // averaged (work + period)/2. dt was measured so integration
    // survived -- but only until the short half fell below the
    // 0.5*nominal clamp, at which point the estimator was told
    // more time had passed than had.
    //
    // A deadline advancing by exactly one period cannot drift and
    // cannot alternate. On overrun it re-bases to now rather than
    // catching up in a burst, which would hand the estimator a
    // run of tiny dt values.
    prev_time = current_time;
    next_deadline += period_dur;
    const auto after_work = std::chrono::high_resolution_clock::now();
    if (next_deadline < after_work)
      next_deadline = after_work;
    else
      std::this_thread::sleep_until(next_deadline);

    // Publish what the loop ACHIEVED, not what it was configured
    // for. The timing bug above stayed invisible for as long as
    // nobody could see the rate.
    ++rate_samples;
    const double window =
        std::chrono::duration<double>(current_time - rate_window_start).count();
    if (window >= 1.0) {
      m_sensor_rate_hz.store(rate_samples / window);
      rate_samples = 0;
      rate_window_start = current_time;
    }
  }
}


//---------------------------------------------------------
// Sensor telemetry, published from Iterate() at AppTick.
//
// The sensor thread runs far faster; this decimates it to the
// MOOS rate under one lock rather than notifying from the thread
// (which would put MOOS comms on the sampling path and jitter it).

void BBNavigatorInterface::publishSensorTelemetry()
{
  double roll, pitch, yaw, heading;
  double gx, gy, gz, yaw_rate;
  uint64_t read_errors, glitches;
  {
    std::lock_guard<std::mutex> lock(m_ahrs_mutex);
    roll = m_roll; pitch = m_pitch; yaw = m_yaw; heading = m_heading;
    gx = m_gyro_x; gy = m_gyro_y; gz = m_gyro_z; yaw_rate = m_yaw_rate;
    read_errors = m_imu_read_errors; glitches = m_imu_glitch_count;
  }

  Notify(ahrsName("NAV_ROLL"),    roll);
  Notify(ahrsName("NAV_PITCH"),   pitch);
  Notify(ahrsName("NAV_YAW"),     yaw);
  Notify(ahrsName("NAV_HEADING"), heading);
  Notify(imuName("GYRO_X"),       gx);
  Notify(imuName("GYRO_Y"),       gy);
  Notify(imuName("GYRO_Z"),       gz);
  Notify(imuName("GYRO_Z_LVL"),   yaw_rate);
  Notify("NVGR_SENSOR_RATE_HZ",   m_sensor_rate_hz.load());
  Notify("NVGR_IMU_READ_ERRORS",  (double)read_errors);
  Notify("NVGR_IMU_GLITCHES",     (double)glitches);

  // --- power / environment, decimated ---
  //
  // Everything below is an I2C transaction (or a sysfs read) on
  // the MOOS thread, and the bus is shared with the PWM writer
  // and the sensor thread. At AppTick=50, reading every Iterate
  // would be 150 bus hits/s for data that changes on the scale of
  // seconds. 5 Hz preserves the old 10 Hz app's information rate;
  // the actuator path above is untouched and stays at 50.
  const double telem_now = MOOSTime();
  if (telem_now - m_last_slow_poll < SLOW_POLL_PERIOD_SEC)
    return;
  m_last_slow_poll = telem_now;

  NavADCData adc;
  if (g_nav.read_adc_all(adc).empty()) {
    m_adc_1 = adc.channel[0];
    m_adc_2 = adc.channel[1];
    m_adc_3 = adc.channel[2];
    m_adc_4 = adc.channel[3];

    // Channel assignment is wiring, not preference: CURRENT is
    // ADC3 and VOLTAGE is ADC4. Current uses the per-battery-count
    // calibration table; only the 4-battery row is measured.
    const int idx = (m_num_batteries >= 1 && m_num_batteries <= 8)
                        ? (m_num_batteries - 1) : 3;
    const CalibrationParams cal = BATTERY_CALIBRATIONS[idx];
    m_latest_current = (m_adc_3 - cal.offset) * cal.gain;
    m_latest_voltage = (m_adc_4 - m_voltage_offset) * m_voltage_scale;

    m_rolling_voltage_window.push_back(m_latest_voltage);
    m_rolling_current_window.push_back(m_latest_current);
    m_rolling_power_window.push_back(m_latest_voltage * m_latest_current);
    while (m_rolling_voltage_window.size() > m_rolling_window_size) {
      m_rolling_voltage_window.erase(m_rolling_voltage_window.begin());
      m_rolling_current_window.erase(m_rolling_current_window.begin());
      m_rolling_power_window.erase(m_rolling_power_window.begin());
    }
    auto mean = [](const std::vector<double> &v) {
      if (v.empty()) return 0.0;
      double s = 0; for (double x : v) s += x; return s / v.size();
    };
    m_rolling_voltage = mean(m_rolling_voltage_window);
    m_rolling_current = mean(m_rolling_current_window);
    m_rolling_power   = mean(m_rolling_power_window);

    Notify("NVGR_ROLLING_VOLTAGE", m_rolling_voltage);
    Notify("NVGR_ROLLING_CURRENT", m_rolling_current);
    Notify("NVGR_ROLLING_POWER",   m_rolling_power);
  }

  // --- environment ---
  NavBaroData baro;
  if (g_nav.read_baro(baro).empty()) {
    m_nav_temp     = baro.temperature_c;
    m_nav_pressure = baro.pressure_kpa;   // already kPa
    Notify("NVGTR_IT_C",   m_nav_temp);
    Notify("NVGTR_IP_KPA", m_nav_pressure);
  }

  bool leak = false;
  if (g_nav.read_leak(leak).empty()) {
    if (leak != m_leak_detected)
      reportRunWarning(leak ? "LEAK DETECTED" : "Leak cleared");
    m_leak_detected = leak;
    Notify("NVGR_LEAK", leak ? "true" : "false");
  }

  // Pi CPU temperature, restored from the pre-rewrite app -- the
  // rewrite dropped it and pBB_Health / pBB_Status (and the
  // broker to the back seat) still consume it. On a non-Pi host
  // the file is absent and nothing is published.
  if (telem_now - m_last_rpi_poll >= 1.0) {
    m_last_rpi_poll = telem_now;
    std::ifstream temp_file("/sys/class/thermal/thermal_zone0/temp");
    if (temp_file.is_open()) {
      int milli_c = 0;
      temp_file >> milli_c;
      m_rpi_temp = milli_c / 1000.0;
      Notify("RPI_TEMP", m_rpi_temp);
    }
  }

  Notify("NVGR_ESC_ARMED", m_pwm_output_enabled.load() ? "true" : "false");
}

//---------------------------------------------------------
// Nav lights: port half red, starboard half green.
//
// The previous version wrote rgb_array[i] and rgb_array[2*i] in
// the SAME loop over i < 12, so starboard overwrote port on every
// even index 0-10, filled evens 12-22, and left odd indices 13-23
// black. Nobody had looked at the lights and believed them.

void BBNavigatorInterface::setNavLights()
{
  static const int kLedCount = 24;
  uint8_t rgb[kLedCount][4];
  memset(rgb, 0, sizeof(rgb));

  for (int i = 0; i < kLedCount / 2; i++) {
    for (int c = 0; c < 4; c++) {
      rgb[i][c]                    = (uint8_t)m_port_side[c];
      rgb[i + kLedCount / 2][c]    = (uint8_t)m_starboard_side[c];
    }
  }
  g_nav.neopixel_set_rgbw(rgb, kLedCount);
}
