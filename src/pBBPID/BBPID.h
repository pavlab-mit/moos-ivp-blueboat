/************************************************************/
/*    NAME: Karan Mahesh                                    */
/*    ORGN: MIT / Project Greece                            */
/*    FILE: BBPID.h                                         */
/*    DATE: 2026/06/19                                      */
/*    Brief: Cascaded PID controller MOOS app for BlueBoat. */
/*           DESIRED_SPEED/DESIRED_HEADING (+ nav feedback) */
/*           -> DESIRED_THRUST / DESIRED_RUDDER.            */
/*           Gains are live-updatable over the MOOSDB.      */
/************************************************************/

#ifndef BBPID_HEADER
#define BBPID_HEADER

#include <string>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "BBPIDEngine.h"

class BBPID : public AppCastingMOOSApp
{
 public:
   BBPID();
   ~BBPID() {}

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
   bool buildReport();

 protected:
   void registerVariables();
   bool handleConfigLine(const std::string& param, const std::string& value);
   bool parseSchedulePoint(const std::string& value);
   void handleLiveGainMail(const std::string& key, double dval);
   void applyEngineLimits();

 private: // Engine
   BBPIDEngine m_engine;

 private: // Input variable names (configurable)
   std::string m_desired_speed_var;
   std::string m_desired_heading_var;
   std::string m_nav_speed_var;
   std::string m_nav_heading_var;
   std::string m_nav_yawrate_var;

 private: // Output variable names (configurable)
   std::string m_thrust_var;
   std::string m_rudder_var;

 private: // Latest input state
   double m_desired_speed;
   double m_desired_heading;
   double m_nav_speed;
   double m_nav_heading;
   double m_nav_yawrate;

   bool   m_have_des_speed;
   bool   m_have_des_heading;
   bool   m_have_nav_speed;
   bool   m_have_nav_heading;
   bool   m_have_nav_yawrate;

   // When true, yaw rate is derived from NAV_HEADING (no external gyro var),
   // so the iterate gate must not wait on m_have_nav_yawrate.
   bool   m_yawrate_derive;

 private: // Bookkeeping
   bool   m_active;          // false until first desired+nav mail
   double m_last_thrust;
   double m_last_rudder;
   unsigned int m_iterations_run;
   double m_tstamp_last_cmd; // time of last fresh desired command
   double m_cmd_stale_thresh;// s; zero outputs if no fresh desired beyond this

 private: // Limit configuration (applied to engine after startup)
   double m_max_thrust;
   double m_max_rudder;
   double m_max_yawrate;     // deg/s, also the outer-loop integral clamp
   double m_speed_integral_limit;
   double m_yawrate_integral_limit;
};

#endif
