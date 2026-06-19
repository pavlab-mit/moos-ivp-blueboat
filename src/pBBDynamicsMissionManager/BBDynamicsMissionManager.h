/************************************************************/
/*    NAME: Karan Mahesh                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BBDynamicsMissionManager.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef BBDynamicsMissionManager_HEADER
#define BBDynamicsMissionManager_HEADER

#include <vector>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class BBDynamicsMissionManager : public AppCastingMOOSApp
{
  struct RunPoint {
    double speed;
    double radius;
  };

 public:
   BBDynamicsMissionManager();
   ~BBDynamicsMissionManager();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private:
   void buildRunGrid();
   void postLegRunUpdate(unsigned int index);

 private: // Configuration variables

 double m_min_speed;
 double m_max_speed;
 double m_speed_interval;
 double m_min_radius;
 double m_max_radius;
 double m_radius_interval;
 double m_min_battery_voltage;
 double m_curr_voltage;
 bool m_need_update;
 int m_num_turns;
 int m_run_index;
 std::string m_legrun_update_var;
 std::vector<RunPoint> m_run_grid;

 private: // State variables
};

#endif 
