/************************************************************/
/*    NAME: Karan Mahesh                                    */
/*    ORGN: MIT / Project Greece                            */
/*    FILE: BBPIDEngine.h                                   */
/*    DATE: 2026/06/19                                      */
/*    Brief: Cascaded PID controller for the BlueBoat.      */
/*                                                          */
/*    Two control problems, each a TRUE positional PID      */
/*    (not the incremental "thrust += delta" form used by   */
/*    pMarinePID, which behaves as a pure integrator on     */
/*    speed regardless of Kp/Kd):                           */
/*                                                          */
/*      Speed loop:  e = des_speed - nav_speed              */
/*                   -> DESIRED_THRUST                      */
/*                                                          */
/*      Yaw cascade: outer  e = des_hdg - nav_hdg           */
/*                          -> desired yaw rate (deg/s)     */
/*                   inner  e = des_rate - meas_rate        */
/*                          -> DESIRED_RUDDER               */
/*                                                          */
/*    The inner loop closes on measured yaw rate, so the    */
/*    speed-dependent turn authority of the deep hull is    */
/*    rejected directly. The outer desired-rate clamp       */
/*    (m_max_yawrate) is the natural place to gain-schedule */
/*    on speed later (kept schedule-ready, flat for now).   */
/************************************************************/

#ifndef BBPID_ENGINE_HEADER
#define BBPID_ENGINE_HEADER

#include <string>
#include "ScalarPID.h"

class BBPIDEngine
{
 public:
  BBPIDEngine();
  ~BBPIDEngine() {}

  // --- Gain / limit configuration (live-updatable) ---
  void setSpeedGains(double kp, double ki, double kd);
  void setHeadingGains(double kp, double ki, double kd);   // outer (hdg->rate)
  void setYawRateGains(double kp, double ki, double kd);   // inner (rate->rudder)

  void setSpeedLimits(double integral_limit, double max_thrust);
  void setHeadingLimits(double integral_limit, double max_yawrate);
  void setYawRateLimits(double integral_limit, double max_rudder);

  void setMaxThrust(double v)     { m_max_thrust = v;  }
  void setMaxRudder(double v)     { m_max_rudder = v;  }
  void setMaxYawRate(double v)    { m_max_yawrate = v; }
  void setAllowReverse(bool v)    { m_allow_reverse = v; }
  void setYawRateScale(double v)  { m_yawrate_scale = v;  }  // raw->deg/s (+ sign flip)
  void setRudderPolarity(double v){ m_rudder_polarity = v; } // +1 or -1

  // --- Main computation: one control tick ---
  // nav_yawrate_raw is the unscaled feedback (e.g. GYRO_Z_LVL_IMU).
  void update(double curr_time,
              double desired_speed,   double nav_speed,
              double desired_heading, double nav_heading,
              double nav_yawrate_raw,
              double& out_thrust, double& out_rudder);

  // --- Current gain accessors (for read-modify-write live updates) ---
  double speedKp()   const { return m_speed_pid.getKP();   }
  double speedKi()   const { return m_speed_pid.getKI();   }
  double speedKd()   const { return m_speed_pid.getKD();   }
  double headingKp() const { return m_heading_pid.getKP(); }
  double headingKi() const { return m_heading_pid.getKI(); }
  double headingKd() const { return m_heading_pid.getKD(); }
  double yawrateKp() const { return m_yawrate_pid.getKP(); }
  double yawrateKi() const { return m_yawrate_pid.getKI(); }
  double yawrateKd() const { return m_yawrate_pid.getKD(); }

  // --- Reporting accessors ---
  double getSpeedError()     const { return m_speed_error;   }
  double getHeadingError()   const { return m_heading_error; }
  double getYawRateError()   const { return m_yawrate_error; }
  double getDesiredYawRate() const { return m_desired_yawrate; }
  double getMeasYawRate()    const { return m_meas_yawrate;  }
  double getThrust()         const { return m_out_thrust;    }
  double getRudder()         const { return m_out_rudder;    }

 protected:
  // PID cores (true positional PID; SetGains order is Kp,Kd,Ki)
  ScalarPID m_speed_pid;
  ScalarPID m_heading_pid;   // outer: heading error -> desired yaw rate
  ScalarPID m_yawrate_pid;   // inner: yaw-rate error -> rudder

  // Limits / conventions
  double m_max_thrust;
  double m_max_rudder;
  double m_max_yawrate;      // deg/s, clamp on outer-loop output
  double m_yawrate_scale;    // multiply raw feedback -> deg/s (sign-correcting)
  double m_rudder_polarity;  // +1 / -1 to match hull/mixer convention
  bool   m_allow_reverse;    // allow negative thrust

  // Last-tick state (reporting)
  double m_speed_error;
  double m_heading_error;
  double m_yawrate_error;
  double m_desired_yawrate;
  double m_meas_yawrate;
  double m_out_thrust;
  double m_out_rudder;
};

#endif
