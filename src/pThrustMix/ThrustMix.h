/************************************************************/
/*    NAME: J. Wenger                                       */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ThrustMix.h                                     */
/*    DATE: Oct 19, 2025                                    */
/*    Brief: Differential thrust mixer for BlueBoat        */
/*           Converts DESIRED_THRUST + DESIRED_RUDDER       */
/*           to asymmetric DESIRED_THRUST_L/R               */
/************************************************************/

#ifndef ThrustMix_HEADER
#define ThrustMix_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <string>
#include <vector>
#include <utility>

class ThrustMix : public AppCastingMOOSApp
{
 public:
   ThrustMix();
   ~ThrustMix();

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
   bool buildReport();

 protected:
   void registerVariables();
   double getSpeedGainFactor(double speed);
   bool parseSpeedGainPoints(const std::string& config_str);

 private: // Configuration variables

   // Basic mixing gains
   double m_k_inner_base;
   double m_k_outer_base;

   // Speed-dependent scaling
   bool m_enable_speed_scaling;
   std::vector<std::pair<double, double>> m_speed_gain_table; // (speed, gain) pairs

   // Yaw rate feedback
   bool m_enable_yaw_feedback;
   double m_k_yaw_correction;
   double m_max_yaw_rate;      // deg/s
   double m_k_heading_to_rate; // heading error -> desired rate

   // Variable names for inputs/outputs
   std::string m_desired_thrust_var;
   std::string m_desired_rudder_var;
   std::string m_feedback_speed_var;
   std::string m_feedback_yaw_var;
   std::string m_feedback_heading_var;
   std::string m_desired_heading_var;
   std::string m_desired_thrust_l_var;
   std::string m_desired_thrust_r_var;

 private: // State variables

   // Input values
   double m_desired_thrust;
   double m_desired_rudder;
   double m_nav_speed;
   double m_nav_yaw_rate;
   double m_nav_heading;
   double m_desired_heading;

   // Computed values
   double m_mixer;          // Normalized mixer value [-1, 1]
   double m_k_inner;        // Active inner gain
   double m_k_outer;        // Active outer gain
   double m_speed_gain_factor;
   double m_thrust_l;
   double m_thrust_r;

   // Flags
   bool m_have_thrust;
   bool m_have_rudder;
   bool m_have_speed;
   bool m_have_yaw_rate;
   bool m_have_heading;
   bool m_have_desired_heading;
};

#endif 
