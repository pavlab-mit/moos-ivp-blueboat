/*************************************************************
      Name: Raymond Turrisi
      Orgn: MIT, Cambridge MA
      File: pDiffThrustPID_v2/DiffThrustPID_v2.h
   Last Ed: 2026-06-20
     Brief:
        Differential-thrust controller for the BlueBoat. Two
        feedforward + integrator regulators (surge, yaw) summed
        into left/right thrust, with a speed-scheduled
        differential cap. See docs/pDiffThrustPID_v2_spec.md.
*************************************************************/

#ifndef DiffThrustPID_v2_HEADER
#define DiffThrustPID_v2_HEADER

#include <string>
#include <vector>
#include <utility>
#include <cstdarg>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

// Sorted (speed, value) breakpoints read by clamped linear interpolation.
class Schedule
{
 public:
  void set(const std::string &spec);          // "v,val : v,val : ..."
  double eval(double x) const;
  std::string repr() const;

 private:
  std::vector<std::pair<double, double>> m_pts;
};

class DiffThrustPID_v2 : public AppCastingMOOSApp
{
 public:
   DiffThrustPID_v2();
   ~DiffThrustPID_v2() {}

 protected:
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   bool buildReport();

   void registerVariables();
   bool dbg_print(const char *format, ...);

   bool setParam(const std::string &key, const std::string &val);  // live-tunable knobs
   void handleUpdates(const std::string &raw);                     // PDIFF_THRUST_UPDATES
   void publishState();                                            // PDIFF_THRUST_STATE

 private: // Subscribed variable names
   std::string m_setpoint_heading_var;
   std::string m_feedback_heading_var;
   std::string m_setpoint_speed_var;
   std::string m_feedback_speed_var;
   std::string m_feedback_yaw_var;
   std::string m_desired_thrust_l_var;
   std::string m_desired_thrust_r_var;

 private: // Parameters
   Schedule m_ff_surge;     // desired speed -> total thrust [%]
   double   m_speed_kp;     // P on speed error e_v        [% / (m/s)]
   double   m_speed_ki;
   double   m_speed_kd;     // D on measured speed (-Kd*dv) [% / (m/s/s)]
   double   m_speed_i_max;  // cap on surge integral term [% thrust]

   double   m_theta_b;      // bank angle [deg]
   double   m_max_yaw_rate; // yaw rate at full turn [deg/s]
   double   m_yaw_c0;       // differential per yaw rate at rest [%/(deg/s)]
   double   m_yaw_c1;       // ... extra per m/s
   double   m_yaw_kp;       // P on yaw-rate error e_r      [% / (deg/s)]
   double   m_yaw_ki;
   double   m_yaw_kd;       // D on measured yaw rate (-Kd*dr) [% / (deg/s/s)]
   double   m_yaw_i_max;    // cap on yaw integral term [% differential]

   Schedule m_delta_cap;    // max differential vs speed [%]
   double   m_u_max;

 private: // State
   double m_v_des, m_v;
   double m_psi_des, m_psi;
   double m_r;              // measured yaw rate [deg/s] (P/I/D feedback)
   bool   m_deploy;

   double m_Jv, m_Jr;       // integral terms [% thrust / % differential]
   double m_prev_v, m_prev_r;  // for derivative-on-measurement
   double m_prev_time;
   double m_last_state_pub; // last PDIFF_THRUST_STATE heartbeat time

   double m_T, m_D, m_TL, m_TR, m_m, m_r_des;  // for the report

 private: // Debug
   bool   m_debug;
   FILE  *m_debug_stream;
   static const uint16_t m_fname_buff_size = 256;
   char   m_fname[m_fname_buff_size];
   std::string m_app_name;
};

#endif
