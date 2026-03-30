#ifndef NAV_EKF_HPP
#define NAV_EKF_HPP

#include <armadillo>
#include <cmath>

using namespace arma;

class NavEKF {
public:
    // State indices
    static constexpr int X_IDX = 0;
    static constexpr int Y_IDX = 1;
    static constexpr int U_IDX = 2;
    static constexpr int PHI_IDX = 3;
    static constexpr int B_IDX = 4;
    static constexpr int STATE_DIM = 5;
    
    // Bias constraints (±30 degrees)
    static constexpr double MAX_BIAS = M_PI / 6.0;  // 30 degrees in radians
    
    // Constructor
    NavEKF() : m_initialized(false), m_speed_threshold(0.5) {
        // Initialize state and covariance
        m_x = zeros<vec>(STATE_DIM);
        m_P = eye<mat>(STATE_DIM, STATE_DIM);
        
        // Default process noise
        m_Q = diagmat(vec({0.05, 0.05, 0.01, 0.001, 0.0001}));
        
        // Default measurement noise (all as variance, not stddev)
        m_R_pos = eye<mat>(2, 2) * (3.0 * 3.0);      // Position noise variance (m^2)
        m_R_spd = mat(1, 1, fill::ones) * (0.3 * 0.3); // Speed noise variance (m/s)^2
        m_R_mag = mat(1, 1, fill::ones) * pow(5.0 * M_PI/180.0, 2); // Mag heading variance (rad^2)
        m_R_bias = mat(1, 1, fill::ones) * pow(10.0 * M_PI/180.0, 2); // Bias variance (rad^2)
    }
    
    // Configuration
    void setProcessNoise(double pos_var, double speed_var, double heading_var, double bias_var) {
        m_Q(X_IDX, X_IDX) = pos_var;
        m_Q(Y_IDX, Y_IDX) = pos_var;
        m_Q(U_IDX, U_IDX) = speed_var;
        m_Q(PHI_IDX, PHI_IDX) = heading_var;
        m_Q(B_IDX, B_IDX) = bias_var;
    }
    
    void setMeasurementNoise(double pos_stddev, double speed_stddev, double mag_stddev, double bias_stddev) {
        m_R_pos = eye<mat>(2, 2) * (pos_stddev * pos_stddev);
        m_R_spd(0, 0) = speed_stddev * speed_stddev;
        m_R_mag(0, 0) = mag_stddev * mag_stddev;
        m_R_bias(0, 0) = bias_stddev * bias_stddev;
    }
    
    void setSpeedThreshold(double threshold) {
        m_speed_threshold = threshold;
    }
    
    // Initialize filter with first GNSS fix and magnetic heading
    void initialize(double x, double y, double speed, double heading_gnss, double heading_mag) {
        m_x(X_IDX) = x;
        m_x(Y_IDX) = y;
        m_x(U_IDX) = speed;
        m_x(PHI_IDX) = wrapAngle(heading_gnss);
        m_x(B_IDX) = constrainBias(wrapAngle(heading_gnss - heading_mag));
        
        // Initial covariance - more conservative
        m_P = eye<mat>(STATE_DIM, STATE_DIM);
        m_P(X_IDX, X_IDX) = 4.0;    // 2m position uncertainty
        m_P(Y_IDX, Y_IDX) = 4.0;    // 2m position uncertainty
        m_P(U_IDX, U_IDX) = 0.25;   // 0.5 m/s speed uncertainty
        m_P(PHI_IDX, PHI_IDX) = 0.1; // ~18 deg heading uncertainty
        m_P(B_IDX, B_IDX) = 0.1;    // ~18 deg bias uncertainty
        
        m_initialized = true;
    }
    
    // Prediction step
    void predict(double dt, double gyro_z) {
        if (!m_initialized) return;
        
        double u = m_x(U_IDX);
        double phi = m_x(PHI_IDX);
        
        // State prediction
        vec x_pred = m_x;
        x_pred(X_IDX) += u * cos(phi) * dt;
        x_pred(Y_IDX) += u * sin(phi) * dt;
        // u remains the same (random walk model)
        x_pred(PHI_IDX) = wrapAngle(phi + gyro_z * dt);
        // bias remains the same (random walk model)
        
        // Jacobian
        mat F = eye<mat>(STATE_DIM, STATE_DIM);
        F(X_IDX, U_IDX) = cos(phi) * dt;
        F(X_IDX, PHI_IDX) = -u * sin(phi) * dt;
        F(Y_IDX, U_IDX) = sin(phi) * dt;
        F(Y_IDX, PHI_IDX) = u * cos(phi) * dt;
        
        // Covariance prediction with proper dt scaling
        mat Q_scaled = m_Q * dt;  // Scale process noise by time step
        m_P = F * m_P * F.t() + Q_scaled;
        m_x = x_pred;
        
        // Enforce symmetry
        m_P = 0.5 * (m_P + m_P.t());
        
        // Limit covariance growth to prevent divergence
        for (int i = 0; i < STATE_DIM; i++) {
            if (m_P(i, i) > 100.0) {
                m_P(i, i) = 100.0;
            }
        }
    }
    
    // Update with GNSS position
    void updatePosition(double x_meas, double y_meas) {
        if (!m_initialized) return;
        
        vec z = {x_meas, y_meas};
        mat H = zeros<mat>(2, STATE_DIM);
        H(0, X_IDX) = 1.0;
        H(1, Y_IDX) = 1.0;
        
        vec y = z - H * m_x;
        mat S = H * m_P * H.t() + m_R_pos;
        mat K = m_P * H.t() * inv(S);
        
        m_x = m_x + K * y;
        
        // Wrap angles and constrain bias after update
        m_x(PHI_IDX) = wrapAngle(m_x(PHI_IDX));
        m_x(B_IDX) = constrainBias(m_x(B_IDX));
        
        // Joseph form covariance update for numerical stability
        mat I_KH = eye<mat>(STATE_DIM, STATE_DIM) - K * H;
        m_P = I_KH * m_P * I_KH.t() + K * m_R_pos * K.t();
        
        // Enforce symmetry
        m_P = 0.5 * (m_P + m_P.t());
    }
    
    // Update with GNSS speed
    void updateSpeed(double speed_meas) {
        if (!m_initialized) return;
        
        mat H = zeros<mat>(1, STATE_DIM);
        H(0, U_IDX) = 1.0;
        
        double y = speed_meas - m_x(U_IDX);
        mat S = H * m_P * H.t() + m_R_spd;
        mat K = m_P * H.t() / as_scalar(S);
        
        m_x = m_x + K * y;
        
        // Ensure speed is non-negative and constrain bias
        if (m_x(U_IDX) < 0) m_x(U_IDX) = 0;
        m_x(PHI_IDX) = wrapAngle(m_x(PHI_IDX));
        m_x(B_IDX) = constrainBias(m_x(B_IDX));
        
        // Joseph form covariance update
        mat I_KH = eye<mat>(STATE_DIM, STATE_DIM) - K * H;
        m_P = I_KH * m_P * I_KH.t() + K * m_R_spd * K.t();
        
        // Enforce symmetry
        m_P = 0.5 * (m_P + m_P.t());
    }
    
    // Update with magnetic heading (always applied)
    void updateMagHeading(double heading_mag) {
        if (!m_initialized) return;
        
        // Measurement model: z = phi - b
        mat H = zeros<mat>(1, STATE_DIM);
        H(0, PHI_IDX) = 1.0;
        H(0, B_IDX) = -1.0;
        
        double y = wrapAngle(heading_mag - (m_x(PHI_IDX) - m_x(B_IDX)));
        mat S = H * m_P * H.t() + m_R_mag;
        mat K = m_P * H.t() / as_scalar(S);
        
        m_x = m_x + K * y;
        m_x(PHI_IDX) = wrapAngle(m_x(PHI_IDX));
        m_x(B_IDX) = constrainBias(m_x(B_IDX));
        
        // Joseph form covariance update
        mat I_KH = eye<mat>(STATE_DIM, STATE_DIM) - K * H;
        m_P = I_KH * m_P * I_KH.t() + K * m_R_mag * K.t();
        
        // Enforce symmetry
        m_P = 0.5 * (m_P + m_P.t());
    }
    
    // Update bias estimate (only when speed > threshold)
    void updateBias(double heading_gnss, double heading_mag) {
        if (!m_initialized || m_x(U_IDX) <= m_speed_threshold) return;
        
        // Measurement: z = heading_gnss - heading_mag = bias
        mat H = zeros<mat>(1, STATE_DIM);
        H(0, B_IDX) = 1.0;
        
        double bias_meas = wrapAngle(heading_gnss - heading_mag);
        double y = wrapAngle(bias_meas - m_x(B_IDX));
        
        mat S = H * m_P * H.t() + m_R_bias;
        mat K = m_P * H.t() / as_scalar(S);
        
        m_x = m_x + K * y;
        
        // Constrain bias to ±30 degrees
        m_x(B_IDX) = constrainBias(m_x(B_IDX));
        
        // Joseph form covariance update
        mat I_KH = eye<mat>(STATE_DIM, STATE_DIM) - K * H;
        m_P = I_KH * m_P * I_KH.t() + K * m_R_bias * K.t();
        
        // Enforce symmetry
        m_P = 0.5 * (m_P + m_P.t());
    }
    
    // Getters
    double getX() const { return m_x(X_IDX); }
    double getY() const { return m_x(Y_IDX); }
    double getSpeed() const { return m_x(U_IDX); }
    double getHeading() const { return wrapAngle(m_x(PHI_IDX)); }
    double getBias() const { return m_x(B_IDX); }
    vec getState() const { return m_x; }
    mat getCovariance() const { return m_P; }
    bool isInitialized() const { return m_initialized; }
    
    // Get heading for output
    double getOutputHeading(double current_mag_heading) const {
        if (!m_initialized) return current_mag_heading;
        
        if (m_x(U_IDX) <= m_speed_threshold) {
            // When slow/stopped, use magnetic heading corrected by learned bias
            return wrapAngle(current_mag_heading + m_x(B_IDX));
        } else {
            // When moving, use the EKF true heading (not corrected by bias)
            // This is the GNSS/gyro fused heading without magnetic influence
            return wrapAngle(m_x(PHI_IDX));
        }
    }
    
    // Check filter health
    bool isHealthy() const {
        if (!m_initialized) return false;
        
        // Check for NaN in state
        if (m_x.has_nan()) return false;
        
        // Check for NaN or non-positive definite covariance
        if (m_P.has_nan()) return false;
        
        // Check if covariance is getting too large (filter divergence)
        double max_variance = 100.0;  // Conservative threshold
        for (int i = 0; i < STATE_DIM; i++) {
            if (m_P(i, i) > max_variance || m_P(i, i) < 0) {
                return false;
            }
        }
        
        return true;
    }
    
private:
    vec m_x;     // State vector [x, y, u, phi, b]
    mat m_P;     // Covariance matrix
    mat m_Q;     // Process noise
    mat m_R_pos; // Position measurement noise
    mat m_R_spd; // Speed measurement noise  
    mat m_R_mag; // Magnetic heading measurement noise
    mat m_R_bias; // Bias measurement noise
    
    double m_speed_threshold;
    bool m_initialized;
    
    static double wrapAngle(double angle) {
        return atan2(sin(angle), cos(angle));
    }
    
    static double constrainBias(double bias) {
        // Constrain bias to ±30 degrees (±π/6 radians)
        if (bias > MAX_BIAS) {
            return MAX_BIAS;
        } else if (bias < -MAX_BIAS) {
            return -MAX_BIAS;
        }
        return bias;
    }
};

#endif // NAV_EKF_HPP