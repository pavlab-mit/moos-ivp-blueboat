/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: pBB_DGPS_EKF/BB_DGPS_EKF_Model.hpp
   Last Ed: 2026-03-25
     Brief:
        Standalone armadillo-based EKF model for BlueBat DGPS
        navigation. Fuses dual-antenna GPS (position, heading,
        speed, COG) with level-frame gyroscope for smoothed
        heading estimation.

        State vector: [x, y, phi, gamma, s]
          x, y   - position in local grid (m)
          phi    - true heading from DGPS + gyro fusion (rad)
          gamma  - course over ground from GPS (rad)
          s      - speed over ground (m/s)
*************************************************************/

#ifndef BB_DGPS_EKF_MODEL_HPP
#define BB_DGPS_EKF_MODEL_HPP

#include <armadillo>
#include <cmath>
#include <string>

class BB_DGPS_EKF_Model
{
public:
  // State dimension: [x, y, phi, gamma, s]
  static constexpr int DIM_STATE = 5;

  // State indices
  static constexpr int X_IDX = 0;      // Position X (m)
  static constexpr int Y_IDX = 1;      // Position Y (m)
  static constexpr int PHI_IDX = 2;    // Heading from DGPS (rad, Cartesian: 0=East, CCW+)
  static constexpr int GAMMA_IDX = 3;  // Course over ground (rad, Cartesian)
  static constexpr int S_IDX = 4;      // Speed over ground (m/s)

  // Noise configuration
  struct NoiseConfig
  {
    // Process noise standard deviations
    double sigma_x = 0.1;        // Position X process noise (m)
    double sigma_y = 0.1;        // Position Y process noise (m)
    double sigma_phi = 0.01;     // Heading process noise (rad) - small since gyro-driven
    double sigma_gamma = 0.05;   // COG process noise (rad)
    double sigma_s = 0.1;        // Speed process noise (m/s)

    // Base measurement noise standard deviations (scaled by HDOP/H_ACC)
    double sigma_gps_pos = 1.0;     // GPS position base noise (m), scaled by H_ACC
    double sigma_gps_hdg = 0.05;    // GPS heading base noise (rad ~3 deg), scaled by heading acc
    double sigma_gps_cog = 0.1;     // GPS COG base noise (rad ~6 deg)
    double sigma_gps_spd = 0.1;     // GPS speed noise (m/s)
  };

  // GPS measurement bundle (parsed from FIX_STATE_DGNSS)
  struct GPSMeasurement
  {
    double timestamp = 0.0;
    double nav_lat = 0.0;        // Latitude (degrees)
    double nav_lon = 0.0;        // Longitude (degrees)
    double nav_x = 0.0;          // Local X (m) - populated by EKF app via geodesy
    double nav_y = 0.0;          // Local Y (m) - populated by EKF app via geodesy
    double speed = 0.0;
    double cog = 0.0;            // Course over ground (degrees, compass: 0=N, CW+)
    double heading = 0.0;        // DGPS heading (degrees, compass: 0=N, CW+)
    double heading_acc = 0.0;    // Heading accuracy (degrees)
    double hdop = 1.0;
    double h_acc = 1.0;          // Horizontal accuracy (m)
    bool heading_valid = false;
    bool gps_lock = false;
    std::string fix_type = "NONE";

    bool isValid() const {
      return gps_lock && (fix_type != "NONE") && (nav_lat != 0.0 || nav_lon != 0.0);
    }
  };

public:
  BB_DGPS_EKF_Model()
    : m_state(arma::zeros<arma::vec>(DIM_STATE)),
      m_covariance(arma::eye<arma::mat>(DIM_STATE, DIM_STATE)),
      m_initialized(false),
      m_last_position(arma::zeros<arma::vec>(2)),
      m_integrated_distance(0.0)
  {
    // Initial covariance
    m_covariance(X_IDX, X_IDX) = 10.0;     // 3m position uncertainty
    m_covariance(Y_IDX, Y_IDX) = 10.0;     // 3m position uncertainty
    m_covariance(PHI_IDX, PHI_IDX) = 0.1;  // ~18 deg heading uncertainty
    m_covariance(GAMMA_IDX, GAMMA_IDX) = 0.1;  // ~18 deg COG uncertainty
    m_covariance(S_IDX, S_IDX) = 0.25;     // 0.5 m/s speed uncertainty
  }

  explicit BB_DGPS_EKF_Model(const NoiseConfig& noise_cfg)
    : BB_DGPS_EKF_Model()
  {
    m_noise = noise_cfg;
  }

  // Initialize filter with first GPS fix
  void initialize(const GPSMeasurement& gps)
  {
    if (!gps.isValid())
      return;

    m_state(X_IDX) = gps.nav_x;
    m_state(Y_IDX) = gps.nav_y;
    m_state(S_IDX) = gps.speed;

    // Convert compass heading to Cartesian (0=East, CCW+)
    double phi_cart = compassToCartesian(gps.heading);
    double gamma_cart = compassToCartesian(gps.cog);

    m_state(PHI_IDX) = phi_cart;
    m_state(GAMMA_IDX) = gamma_cart;

    // Initial covariance based on GPS accuracy
    m_covariance = arma::eye<arma::mat>(DIM_STATE, DIM_STATE);
    double pos_var = gps.h_acc * gps.h_acc;
    m_covariance(X_IDX, X_IDX) = pos_var;
    m_covariance(Y_IDX, Y_IDX) = pos_var;
    m_covariance(PHI_IDX, PHI_IDX) = 0.1;  // Conservative initial heading uncertainty
    m_covariance(GAMMA_IDX, GAMMA_IDX) = 0.1;
    m_covariance(S_IDX, S_IDX) = 0.25;

    m_last_position(0) = gps.nav_x;
    m_last_position(1) = gps.nav_y;
    m_integrated_distance = 0.0;
    m_initialized = true;
  }

  void initialize(double x, double y, double phi, double gamma, double s)
  {
    m_state(X_IDX) = x;
    m_state(Y_IDX) = y;
    m_state(PHI_IDX) = wrapAngle(phi);
    m_state(GAMMA_IDX) = wrapAngle(gamma);
    m_state(S_IDX) = s;

    m_last_position(0) = x;
    m_last_position(1) = y;
    m_integrated_distance = 0.0;
    m_initialized = true;
  }

  // Accessors
  const arma::vec& state() const { return m_state; }
  const arma::mat& covariance() const { return m_covariance; }
  bool isInitialized() const { return m_initialized; }
  double integratedDistance() const { return m_integrated_distance; }

  NoiseConfig& noiseConfig() { return m_noise; }
  const NoiseConfig& noiseConfig() const { return m_noise; }

  // Getters for individual states
  double getX() const { return m_state(X_IDX); }
  double getY() const { return m_state(Y_IDX); }
  double getHeading() const { return wrapAngle(m_state(PHI_IDX)); }
  double getCOG() const { return wrapAngle(m_state(GAMMA_IDX)); }
  double getSpeed() const { return std::max(0.0, m_state(S_IDX)); }

  // Get heading in compass convention (0=North, CW+, degrees)
  double getHeadingCompass() const {
    return cartesianToCompass(getHeading());
  }

  double getCOGCompass() const {
    return cartesianToCompass(getCOG());
  }

  // Standard deviations from covariance
  double getStdX() const { return std::sqrt(m_covariance(X_IDX, X_IDX)); }
  double getStdY() const { return std::sqrt(m_covariance(Y_IDX, Y_IDX)); }
  double getStdPhi() const { return std::sqrt(m_covariance(PHI_IDX, PHI_IDX)); }
  double getStdGamma() const { return std::sqrt(m_covariance(GAMMA_IDX, GAMMA_IDX)); }
  double getStdSpeed() const { return std::sqrt(m_covariance(S_IDX, S_IDX)); }

  //--------------------------------------------------------------
  // Core EKF operations
  //--------------------------------------------------------------

  // Prediction step using gyroscope angular velocity
  // gyro_z: angular velocity in level frame (rad/s), NED convention (CW positive around down)
  void predict(double dt, double gyro_z)
  {
    if (!m_initialized || dt <= 0.0)
      return;

    // Clamp dt to reasonable range
    dt = std::min(std::max(dt, 0.001), 1.0);

    double s = m_state(S_IDX);
    double gamma = m_state(GAMMA_IDX);
    double phi = m_state(PHI_IDX);

    // State prediction
    arma::vec x_pred = m_state;

    // Position propagates using course over ground (direction of travel)
    x_pred(X_IDX) += s * std::cos(gamma) * dt;
    x_pred(Y_IDX) += s * std::sin(gamma) * dt;

    // Heading propagates using gyro
    // gyro_z convention: positive = rotating right (CW in body frame), matching pBB_EKF
    x_pred(PHI_IDX) = wrapAngle(phi + gyro_z * dt);

    // COG and speed are random walk (updated by measurements)
    // x_pred(GAMMA_IDX) unchanged
    // x_pred(S_IDX) unchanged

    // Jacobian F = dF/dx
    arma::mat F = arma::eye<arma::mat>(DIM_STATE, DIM_STATE);
    F(X_IDX, GAMMA_IDX) = -s * std::sin(gamma) * dt;
    F(X_IDX, S_IDX) = std::cos(gamma) * dt;
    F(Y_IDX, GAMMA_IDX) = s * std::cos(gamma) * dt;
    F(Y_IDX, S_IDX) = std::sin(gamma) * dt;

    // Build process noise matrix
    arma::mat Q = buildProcessNoise(dt);

    // Covariance prediction
    m_covariance = F * m_covariance * F.t() + Q;
    m_state = x_pred;

    // Enforce symmetry and limit growth
    symmetrizeCovariance();
    limitCovarianceGrowth();

    // Update integrated distance
    arma::vec2 current_pos = {m_state(X_IDX), m_state(Y_IDX)};
    arma::vec2 delta = current_pos - m_last_position;
    m_integrated_distance += arma::norm(delta);
    m_last_position = current_pos;
  }

  // Update with full GPS measurement (adaptive noise)
  // speed_threshold: minimum speed for COG updates (m/s)
  void updateGPS(const GPSMeasurement& gps, double speed_threshold = 0.3)
  {
    if (!m_initialized || !gps.isValid())
      return;

    // Position update
    updatePosition(gps.nav_x, gps.nav_y, gps.h_acc);

    // Speed update (always valid with GPS lock)
    updateSpeed(gps.speed);

    // COG update (only meaningful at speed)
    if (gps.speed > speed_threshold) {
      double cog_cart = compassToCartesian(gps.cog);
      updateCOG(cog_cart, gps.hdop);
    }

    // Heading update (only if DGPS heading is valid)
    if (gps.heading_valid && gps.heading_acc > 0 && gps.heading_acc < 10.0) {
      double phi_cart = compassToCartesian(gps.heading);
      updateHeading(phi_cart, gps.heading_acc * M_PI / 180.0);
    }
  }

  // Update with GPS position (x, y in local grid, h_acc is 1-sigma in meters)
  void updatePosition(double x_meas, double y_meas, double h_acc)
  {
    if (!m_initialized)
      return;

    // Minimum measurement noise
    h_acc = std::max(h_acc, 0.1);

    arma::vec z = {x_meas, y_meas};
    arma::mat H = arma::zeros<arma::mat>(2, DIM_STATE);
    H(0, X_IDX) = 1.0;
    H(1, Y_IDX) = 1.0;

    // Adaptive measurement noise based on GPS accuracy
    double pos_var = h_acc * h_acc;
    arma::mat R = arma::eye<arma::mat>(2, 2) * pos_var;

    // Innovation
    arma::vec y = z - H * m_state;

    // Kalman update
    arma::mat S = H * m_covariance * H.t() + R;
    arma::mat K = m_covariance * H.t() * arma::inv(S);

    m_state += K * y;
    m_state(PHI_IDX) = wrapAngle(m_state(PHI_IDX));
    m_state(GAMMA_IDX) = wrapAngle(m_state(GAMMA_IDX));
    if (m_state(S_IDX) < 0) m_state(S_IDX) = 0;

    // Joseph form covariance update
    arma::mat I = arma::eye<arma::mat>(DIM_STATE, DIM_STATE);
    m_covariance = (I - K * H) * m_covariance * (I - K * H).t() + K * R * K.t();
    symmetrizeCovariance();

    // Reset integrated distance after absolute position fix
    m_integrated_distance = 0.0;
    m_last_position(0) = m_state(X_IDX);
    m_last_position(1) = m_state(Y_IDX);
  }

  // Update with DGPS heading (phi in Cartesian radians, acc is 1-sigma in radians)
  void updateHeading(double phi_meas, double heading_acc)
  {
    if (!m_initialized)
      return;

    heading_acc = std::max(heading_acc, 0.01);  // Min ~0.5 deg

    arma::mat H = arma::zeros<arma::mat>(1, DIM_STATE);
    H(0, PHI_IDX) = 1.0;

    double R = heading_acc * heading_acc;

    // Angular innovation with proper wrapping
    double y = wrapAngle(phi_meas - m_state(PHI_IDX));

    double S = arma::as_scalar(H * m_covariance * H.t()) + R;
    if (S <= 0.0) return;

    arma::vec K = m_covariance * H.t() / S;

    m_state += K * y;
    m_state(PHI_IDX) = wrapAngle(m_state(PHI_IDX));
    m_state(GAMMA_IDX) = wrapAngle(m_state(GAMMA_IDX));
    if (m_state(S_IDX) < 0) m_state(S_IDX) = 0;

    arma::mat I = arma::eye<arma::mat>(DIM_STATE, DIM_STATE);
    m_covariance = (I - K * H) * m_covariance * (I - K * H).t() + K * R * K.t();
    symmetrizeCovariance();
  }

  // Update with GPS course over ground (gamma in Cartesian radians)
  void updateCOG(double gamma_meas, double hdop = 1.0)
  {
    if (!m_initialized)
      return;

    // COG noise scales with HDOP
    double cog_noise = m_noise.sigma_gps_cog * std::max(hdop, 1.0);

    arma::mat H = arma::zeros<arma::mat>(1, DIM_STATE);
    H(0, GAMMA_IDX) = 1.0;

    double R = cog_noise * cog_noise;

    double y = wrapAngle(gamma_meas - m_state(GAMMA_IDX));

    double S = arma::as_scalar(H * m_covariance * H.t()) + R;
    if (S <= 0.0) return;

    arma::vec K = m_covariance * H.t() / S;

    m_state += K * y;
    m_state(PHI_IDX) = wrapAngle(m_state(PHI_IDX));
    m_state(GAMMA_IDX) = wrapAngle(m_state(GAMMA_IDX));
    if (m_state(S_IDX) < 0) m_state(S_IDX) = 0;

    arma::mat I = arma::eye<arma::mat>(DIM_STATE, DIM_STATE);
    m_covariance = (I - K * H) * m_covariance * (I - K * H).t() + K * R * K.t();
    symmetrizeCovariance();
  }

  // Update with GPS speed
  void updateSpeed(double speed_meas)
  {
    if (!m_initialized)
      return;

    arma::mat H = arma::zeros<arma::mat>(1, DIM_STATE);
    H(0, S_IDX) = 1.0;

    double R = m_noise.sigma_gps_spd * m_noise.sigma_gps_spd;

    double y = speed_meas - m_state(S_IDX);

    double S = arma::as_scalar(H * m_covariance * H.t()) + R;
    if (S <= 0.0) return;

    arma::vec K = m_covariance * H.t() / S;

    m_state += K * y;
    if (m_state(S_IDX) < 0) m_state(S_IDX) = 0;

    arma::mat I = arma::eye<arma::mat>(DIM_STATE, DIM_STATE);
    m_covariance = (I - K * H) * m_covariance * (I - K * H).t() + K * R * K.t();
    symmetrizeCovariance();
  }

  // Check filter health
  bool isHealthy() const
  {
    if (!m_initialized) return false;
    if (m_state.has_nan()) return false;
    if (m_covariance.has_nan()) return false;

    // Check for excessive covariance
    for (int i = 0; i < DIM_STATE; i++) {
      if (m_covariance(i, i) > 1000.0 || m_covariance(i, i) < 0)
        return false;
    }
    return true;
  }

private:
  arma::mat buildProcessNoise(double dt) const
  {
    arma::mat Q = arma::zeros<arma::mat>(DIM_STATE, DIM_STATE);

    // Position noise (grows with velocity uncertainty)
    Q(X_IDX, X_IDX) = m_noise.sigma_x * m_noise.sigma_x * dt;
    Q(Y_IDX, Y_IDX) = m_noise.sigma_y * m_noise.sigma_y * dt;

    // Heading noise (small since gyro-driven)
    Q(PHI_IDX, PHI_IDX) = m_noise.sigma_phi * m_noise.sigma_phi * dt;

    // COG noise
    Q(GAMMA_IDX, GAMMA_IDX) = m_noise.sigma_gamma * m_noise.sigma_gamma * dt;

    // Speed noise
    Q(S_IDX, S_IDX) = m_noise.sigma_s * m_noise.sigma_s * dt;

    return Q;
  }

  void symmetrizeCovariance()
  {
    m_covariance = 0.5 * (m_covariance + m_covariance.t());
  }

  void limitCovarianceGrowth()
  {
    const double max_var = 1000.0;
    for (int i = 0; i < DIM_STATE; i++) {
      if (m_covariance(i, i) > max_var) {
        m_covariance(i, i) = max_var;
      }
    }
  }

  // Angle wrapping to [-pi, pi]
  static double wrapAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  // Convert compass heading (0=North, CW+, degrees) to Cartesian (0=East, CCW+, radians)
  static double compassToCartesian(double compass_deg)
  {
    double compass_rad = compass_deg * M_PI / 180.0;
    return wrapAngle(M_PI / 2.0 - compass_rad);
  }

  // Convert Cartesian heading (0=East, CCW+, radians) to compass (0=North, CW+, degrees)
  static double cartesianToCompass(double cart_rad)
  {
    double compass_rad = M_PI / 2.0 - cart_rad;
    if (compass_rad < 0) compass_rad += 2.0 * M_PI;
    return compass_rad * 180.0 / M_PI;
  }

private:
  arma::vec m_state;        // [x, y, phi, gamma, s]
  arma::mat m_covariance;   // 5x5
  bool m_initialized;

  NoiseConfig m_noise;

  arma::vec m_last_position;  // Last XY for integrated distance
  double m_integrated_distance;
};

//======================================================================
// DriftEstimator - Simple 2-state EKF for current/drift estimation
//======================================================================
// State: [v_x, v_y] - drift velocity in local frame (m/s)
// When thrusters are off, observed motion is purely due to currents.
// This estimator runs independently and is updated only when drifting.

class DriftEstimator
{
public:
  static constexpr int DIM_STATE = 2;
  static constexpr int VX_IDX = 0;
  static constexpr int VY_IDX = 1;

  struct DriftNoiseConfig
  {
    // Process noise - currents change slowly
    double sigma_vx = 0.001;   // Drift velocity X process noise (m/s per sqrt(s))
    double sigma_vy = 0.001;   // Drift velocity Y process noise (m/s per sqrt(s))

    // Measurement noise - high because COG at low speed is noisy
    double sigma_meas = 0.3;   // Velocity measurement noise (m/s)
  };

public:
  DriftEstimator()
    : m_state(arma::zeros<arma::vec>(DIM_STATE)),
      m_covariance(arma::eye<arma::mat>(DIM_STATE, DIM_STATE)),
      m_initialized(false),
      m_active(false),
      m_sample_count(0)
  {
    // Initial covariance - high uncertainty
    m_covariance(VX_IDX, VX_IDX) = 1.0;  // 1 m/s uncertainty
    m_covariance(VY_IDX, VY_IDX) = 1.0;
  }

  // Reset the estimator (call when starting a new drift period)
  void reset()
  {
    m_state.zeros();
    m_covariance = arma::eye<arma::mat>(DIM_STATE, DIM_STATE);
    m_covariance(VX_IDX, VX_IDX) = 1.0;
    m_covariance(VY_IDX, VY_IDX) = 1.0;
    m_initialized = false;
    m_active = false;
    m_sample_count = 0;
  }

  // Start drift estimation (call when drift period threshold met)
  void startDrifting()
  {
    if (!m_active) {
      reset();
      m_active = true;
    }
  }

  // Stop drift estimation (call when thrusters engaged)
  void stopDrifting()
  {
    m_active = false;
  }

  bool isActive() const { return m_active; }
  bool isInitialized() const { return m_initialized; }
  unsigned int getSampleCount() const { return m_sample_count; }

  DriftNoiseConfig& noiseConfig() { return m_noise; }
  const DriftNoiseConfig& noiseConfig() const { return m_noise; }

  // Prediction step - random walk model (currents change slowly)
  void predict(double dt)
  {
    if (!m_active || dt <= 0.0)
      return;

    dt = std::min(std::max(dt, 0.001), 1.0);

    // State prediction: v_drift stays constant (random walk)
    // No change to state

    // Process noise
    arma::mat Q = arma::zeros<arma::mat>(DIM_STATE, DIM_STATE);
    Q(VX_IDX, VX_IDX) = m_noise.sigma_vx * m_noise.sigma_vx * dt;
    Q(VY_IDX, VY_IDX) = m_noise.sigma_vy * m_noise.sigma_vy * dt;

    // Covariance prediction (F = I for random walk)
    m_covariance += Q;
    symmetrizeCovariance();
    limitCovarianceGrowth();
  }

  // Update with observed velocity from main EKF
  // speed: speed over ground from nav EKF (m/s)
  // cog: course over ground in Cartesian radians (0=East, CCW+)
  void updateVelocity(double speed, double cog)
  {
    if (!m_active)
      return;

    // Convert polar (speed, cog) to Cartesian velocity
    double vx_meas = speed * std::cos(cog);
    double vy_meas = speed * std::sin(cog);

    // Initialize on first measurement
    if (!m_initialized) {
      m_state(VX_IDX) = vx_meas;
      m_state(VY_IDX) = vy_meas;
      m_initialized = true;
      m_sample_count = 1;
      return;
    }

    // Measurement model: z = [v_x, v_y]
    arma::vec z = {vx_meas, vy_meas};
    arma::mat H = arma::eye<arma::mat>(DIM_STATE, DIM_STATE);

    // Measurement noise - high for low-speed COG
    double var = m_noise.sigma_meas * m_noise.sigma_meas;
    arma::mat R = arma::eye<arma::mat>(DIM_STATE, DIM_STATE) * var;

    // Innovation
    arma::vec y = z - H * m_state;

    // Kalman update
    arma::mat S = H * m_covariance * H.t() + R;
    arma::mat K = m_covariance * H.t() * arma::inv(S);

    m_state += K * y;

    // Joseph form covariance update
    arma::mat I = arma::eye<arma::mat>(DIM_STATE, DIM_STATE);
    m_covariance = (I - K * H) * m_covariance * (I - K * H).t() + K * R * K.t();
    symmetrizeCovariance();

    m_sample_count++;
  }

  // Getters
  double getDriftVx() const { return m_state(VX_IDX); }
  double getDriftVy() const { return m_state(VY_IDX); }

  // Get drift speed and direction
  double getDriftSpeed() const {
    return std::sqrt(m_state(VX_IDX) * m_state(VX_IDX) +
                     m_state(VY_IDX) * m_state(VY_IDX));
  }

  // Get drift direction in Cartesian radians (direction current is flowing TO)
  double getDriftDirection() const {
    return std::atan2(m_state(VY_IDX), m_state(VX_IDX));
  }

  // Get drift direction in compass convention (degrees, 0=N, CW+)
  double getDriftDirectionCompass() const {
    double cart_rad = getDriftDirection();
    double compass_rad = M_PI / 2.0 - cart_rad;
    if (compass_rad < 0) compass_rad += 2.0 * M_PI;
    return compass_rad * 180.0 / M_PI;
  }

  // Standard deviations
  double getStdVx() const { return std::sqrt(m_covariance(VX_IDX, VX_IDX)); }
  double getStdVy() const { return std::sqrt(m_covariance(VY_IDX, VY_IDX)); }

  const arma::vec& state() const { return m_state; }
  const arma::mat& covariance() const { return m_covariance; }

private:
  void symmetrizeCovariance()
  {
    m_covariance = 0.5 * (m_covariance + m_covariance.t());
  }

  void limitCovarianceGrowth()
  {
    const double max_var = 10.0;  // Max 3 m/s uncertainty
    for (int i = 0; i < DIM_STATE; i++) {
      if (m_covariance(i, i) > max_var) {
        m_covariance(i, i) = max_var;
      }
    }
  }

private:
  arma::vec m_state;        // [v_x, v_y]
  arma::mat m_covariance;   // 2x2
  bool m_initialized;
  bool m_active;            // Whether we're currently in drift mode
  unsigned int m_sample_count;

  DriftNoiseConfig m_noise;
};

#endif // BB_DGPS_EKF_MODEL_HPP
