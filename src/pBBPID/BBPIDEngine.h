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

  // Clear the accumulated integral (and derivative history) of all loops.
  void resetIntegrators();

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
  // Per-axis FF toggles (ANDed under the master ff_enable): speed term ->
  // DESIRED_THRUST, yaw term -> DESIRED_RUDDER. Disable one to isolate.
  void setFeedforwardSpeedEnable(bool v) { m_ff_speed_enable = v; }
  void setFeedforwardYawEnable(bool v)   { m_ff_yaw_enable = v;   }
  bool ffSpeedEnabled() const            { return m_ff_speed_enable; }
  bool ffYawEnabled() const              { return m_ff_yaw_enable;   }
  void setFeedforwardSpeed(double c0, double cv, double crr)
       { m_ff_c0 = c0; m_ff_cv = cv; m_ff_crr = crr; }
  void setFeedforwardYaw(double d0, double dr, double dvr)
       { m_ff_d0 = d0; m_ff_dr = dr; m_ff_dvr = dvr; }
  void setFeedforwardRudderScale(double s) { m_ff_rudder_scale = s; }
  double getFFThrust() const { return m_ff_thrust; }
  double getFFRudder() const { return m_ff_rudder; }
  // FF coefficient accessors (for the param snapshot)
  double ffC0()  const { return m_ff_c0;  }
  double ffCv()  const { return m_ff_cv;  }
  double ffCrr() const { return m_ff_crr; }
  double ffD0()  const { return m_ff_d0;  }
  double ffDr()  const { return m_ff_dr;  }
  double ffDvr() const { return m_ff_dvr; }
  double ffRudderScale() const { return m_ff_rudder_scale; }

  // Low-pass time constant [s] on the desired yaw rate (0 = off). Smooths
  // the value used by the yaw FF and the inner-loop setpoint.
  void setDesYawRateFilter(double tau) { m_des_yawrate_tau = tau; }
  double getDesYawRateFilter() const { return m_des_yawrate_tau; }
  double getYawRateFilter()    const { return m_yr_lpf_alpha; }

  // --- Yaw-priority speed governor ---
  // Following a path of curvature k at speed v needs yaw rate r = k*v. Once
  // the commanded rate saturates the vehicle's turn capability the ONLY way
  // to tighten the achievable radius is to slow down, so trade speed for
  // heading tracking whenever the yaw budget is nearly spent. gain=0 = off.
  void   setYawPriorityGain(double v) { m_yaw_priority_gain = v; }
  void   setYawPriorityKnee(double v) { m_yaw_priority_knee = v; }
  void   setMinSpeedFrac(double v)    { m_min_speed_frac = v; }
  void   setDerateFilter(double v)    { m_derate_tau = v; }
  double getYawPriorityGain() const { return m_yaw_priority_gain; }
  double getYawPriorityKnee() const { return m_yaw_priority_knee; }
  double getMinSpeedFrac()    const { return m_min_speed_frac; }
  double getDerateFilter()    const { return m_derate_tau; }
  double getSpeedDerate()     const { return m_speed_derate_filt; }
  double getGovSpeedCmd()     const { return m_gov_speed_cmd; }

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

  // Update + return the measured yaw rate from the configured source
  // (independent of the control loop, so the scope works when idle).
  double computeMeasYawRate(double curr_time, double nav_heading,
                            double nav_yawrate_raw);

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
  bool   m_ff_speed_enable;              // gate the common (speed) FF term
  bool   m_ff_yaw_enable;                // gate the differential (yaw) FF term
  double m_ff_c0, m_ff_cv, m_ff_crr;     // common-mode (speed) FF
  double m_ff_d0, m_ff_dr, m_ff_dvr;     // differential (yaw) FF
  double m_ff_rudder_scale;              // differential-% -> rudder units
  double m_ff_thrust, m_ff_rudder;       // last FF terms (telemetry)

  // Desired-yaw-rate feedforward: LPF of the numerical derivative of the
  // DESIRED heading (dt-aware first order). This reference-derivative term
  // is added to the heading-PID feedback to form the inner-loop setpoint,
  // so a moving command yields turn rate (and yaw FF) even with gains at 0.
  // The derivative and the LPF run on DIFFERENT clocks: the derivative steps
  // only when the command changes (helm tick), the LPF steps every iterate.
  double m_des_yawrate_tau;              // LPF time constant [s], 0 = off
  double m_des_yawrate_filt;             // filter state (filtered ff rate)
  double m_des_lpf_prev_time;            // prev ITERATE time (dt for LPF alpha)
  bool   m_have_des_lpf;                 // have a previous desired-heading sample
  double m_prev_des_heading;             // last CHANGED desired heading [deg]
  double m_des_chg_time;                 // time of that change [s] (dt for deriv)
  double m_des_yawrate_raw;              // held raw command rate [rad/s]

  // Yaw-priority speed governor state
  double m_yaw_priority_gain;            // 0 = off; 1 = full derate at cap
  double m_yaw_priority_knee;            // yaw-budget fraction where derate starts
  double m_min_speed_frac;               // floor on the derate factor
  double m_derate_tau;                   // LPF time const [s] on the derate
  double m_speed_derate;                 // instantaneous derate factor
  double m_speed_derate_filt;            // filtered derate (what is applied)
  double m_gov_speed_cmd;                // governed speed setpoint [m/s]
  double m_gov_prev_time;                // prev iterate time for the derate LPF
  bool   m_have_gov_time;

  // Limits / conventions
  double m_max_thrust;
  double m_max_rudder;
  double m_max_yawrate;      // rad/s, clamp on outer-loop output
  double m_speed_ilim;       // speed-PID integral limit (for resetIntegrators)
  double m_yawrate_ilim;     // yaw-rate-PID integral limit
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
