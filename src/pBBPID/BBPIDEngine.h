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
#include <vector>
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
  double getMaxYawRate()    const { return m_max_yawrate; }
  void setAllowReverse(bool v)    { m_allow_reverse = v; }
  void setYawRateScale(double v)  { m_yawrate_scale = v;  }  // raw->deg/s (+ sign flip)
  void setRudderPolarity(double v){ m_rudder_polarity = v; } // +1 or -1

  // Yaw-rate feedback source: external gyro (default) vs derived from
  // heading (for gyro-less setups like sim). alpha is the LPF weight on
  // the fresh derivative sample in [0,1] (higher = less smoothing).
  void setYawRateDerive(bool v)   { m_yawrate_derive = v; }
  void setYawRateFilter(double a) { m_yr_lpf_alpha = a;   }

  // Feedforward (identified static thrust->motion map, inverted):
  //   common c = c0 + cv*v* + crr*r*^2     (-> DESIRED_THRUST)
  //   diff   d = d0 + dr*r* + dvr*v* * r*  (-> DESIRED_RUDDER, scaled)
  void setFeedforwardEnable(bool v)      { m_ff_enable = v; }
  bool feedforwardEnabled() const        { return m_ff_enable; }
  void setFeedforwardSpeed(double c0, double cv, double crr)
       { m_ff_c0 = c0; m_ff_cv = cv; m_ff_crr = crr; }
  void setFeedforwardYaw(double d0, double dr, double dvr)
       { m_ff_d0 = d0; m_ff_dr = dr; m_ff_dvr = dvr; }
  void setFeedforwardRudderScale(double s) { m_ff_rudder_scale = s; }
  double getFFThrust() const { return m_ff_thrust; }
  double getFFRudder() const { return m_ff_rudder; }

  // --- Speed-scheduled gain table (yaw-rate loop + turn-rate cap) ---
  // Each breakpoint pins the inner yaw-rate PID gains and the max
  // commanded yaw rate at a given speed. Between breakpoints the values
  // are linearly interpolated; below the first / above the last point
  // the endpoint values are held (no extrapolation).
  void enableGainSchedule(bool v) { m_schedule_enabled = v; }
  bool gainScheduleEnabled() const { return m_schedule_enabled; }
  void addSchedulePoint(double speed, double kp, double ki,
                        double kd, double max_yawrate);
  void clearSchedule() { m_schedule.clear(); }
  unsigned int scheduleSize() const { return (unsigned int)m_schedule.size(); }
  double getSchedSpeed() const { return m_sched_speed; }

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
  // Linearly interpolate the active yaw-rate gains + cap for a given
  // speed and push them into the inner loop / clamp. No-op if the
  // schedule is empty.
  void applySchedule(double speed);

  // PID cores (true positional PID; SetGains order is Kp,Kd,Ki)
  ScalarPID m_speed_pid;
  ScalarPID m_heading_pid;   // outer: heading error -> desired yaw rate
  ScalarPID m_yawrate_pid;   // inner: yaw-rate error -> rudder

  // Speed-scheduled gain table (sorted ascending by speed)
  struct SchedPoint {
    double speed;
    double kp, ki, kd;
    double max_yawrate;
  };
  std::vector<SchedPoint> m_schedule;
  bool   m_schedule_enabled;
  double m_sched_speed;      // last speed the schedule was evaluated at

  // Heading-derived yaw-rate fallback (gyro-less / sim)
  bool   m_yawrate_derive;
  double m_yr_lpf_alpha;     // LPF weight on fresh derivative sample
  double m_prev_heading;
  double m_prev_heading_time;
  bool   m_have_prev_heading;

  // Feedforward (identified from field data)
  bool   m_ff_enable;
  double m_ff_c0, m_ff_cv, m_ff_crr;     // common-mode (speed) FF
  double m_ff_d0, m_ff_dr, m_ff_dvr;     // differential (yaw) FF
  double m_ff_rudder_scale;              // differential-% -> rudder units
  double m_ff_thrust, m_ff_rudder;       // last FF terms (telemetry)

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
