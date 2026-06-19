/************************************************************/
/*    NAME: Karan Mahesh                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BBDynamicsMissionManager.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "BBDynamicsMissionManager.h"
#include <cmath>

using namespace std;

//---------------------------------------------------------
// Constructor()

BBDynamicsMissionManager::BBDynamicsMissionManager()
{
  m_min_speed = 0.0;
  m_max_speed = 0.0;
  m_speed_interval = 0.0;
  m_min_radius = 0.0;
  m_max_radius = 0.0;
  m_radius_interval = 0.0;
  m_min_battery_voltage = 22.0; // same as low voltage threshold in BB_Health
  m_curr_voltage = 100;
  m_legrun_update_var = "LEGRUN_UPDATE";
  m_need_update = false;
  m_num_turns = 0;
  m_run_index = 0;
}

//---------------------------------------------------------
// Destructor

BBDynamicsMissionManager::~BBDynamicsMissionManager()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool BBDynamicsMissionManager::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;

  bool handled;

  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    if(key == "NVGR_ROLLING_VOLTAGE"){
      m_curr_voltage = msg.GetDouble(); 
      handled = true;
    }

    if(key == "COMPLETED_TURN"){
      bool completed_turn;
      handled = setBooleanOnString(completed_turn, msg.GetString());

      if (completed_turn){
        m_num_turns ++;

        if (m_num_turns % 2 == 0){
          m_run_index ++;
          m_need_update = true;
        }
      }
    } 

    else if(key != "APPCAST_REQ"){ // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
    }
   }
	
   return(handled);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool BBDynamicsMissionManager::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool BBDynamicsMissionManager::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if (m_curr_voltage <= m_min_battery_voltage) {
    // End mission
    // voltage value not posted in XSIM moded
    // Notify("RETURN", "true");
    Notify("FAKE_RETURN", "true");
  }
  else {

    if (m_need_update) {
      if (m_run_grid.empty()) {
        reportRunWarning("LEGRUN update requested but run grid is empty");
      }
      else {
        postLegRunUpdate(m_run_index);
      }
      m_need_update = false;
    }

  }
  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool BBDynamicsMissionManager::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "min_speed") {
      m_min_speed = atof(value.c_str());
      handled = true;
    }
    else if(param == "max_speed") {
      m_max_speed = atof(value.c_str());
      handled = true;
    }
    else if(param == "speed_interval") {
      m_speed_interval = atof(value.c_str());
      handled = true;
    }
    else if(param == "min_radius") {
      m_min_radius = atof(value.c_str());
      handled = true;
    }
    else if(param == "max_radius") {
      m_max_radius = atof(value.c_str());
      handled = true;
    }
    else if(param == "radius_interval") {
      m_radius_interval = atof(value.c_str());
      handled = true;
    }
    else if(param == "min_battery_voltage") {
      m_min_battery_voltage = atof(value.c_str());
      handled = true;
    }
    else if(param == "legrun_update_var") {
      m_legrun_update_var = value;
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  buildRunGrid();
  if (m_run_grid.empty())
    reportConfigWarning("No speed/radius run grid; check min/max/interval params");
  else
    postLegRunUpdate(m_run_index);

  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void BBDynamicsMissionManager::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();

  // Register("NVGR_ROLLING_VOLTAGE", 0);
  Register("COMPLETED_TURN", 0);

}

//---------------------------------------------------------
// Procedure: buildRunGrid()

void BBDynamicsMissionManager::buildRunGrid()
{
  m_run_grid.clear();

  if((m_speed_interval <= 0) || (m_radius_interval <= 0))
    return;
  if(m_max_speed < m_min_speed)
    return;
  if(m_max_radius < m_min_radius)
    return;

  vector<double> speeds;
  for(double spd = m_min_speed; spd <= m_max_speed + 1e-9; spd += m_speed_interval)
    speeds.push_back(spd);

  vector<double> radii;
  for(double rad = m_min_radius; rad <= m_max_radius + 1e-9; rad += m_radius_interval)
    radii.push_back(rad);

  for(unsigned int i=0; i<speeds.size(); i++) {
    for(unsigned int j=0; j<radii.size(); j++) {
      RunPoint pt;
      pt.speed = speeds[i];
      pt.radius = radii[j];
      m_run_grid.push_back(pt);
    }
  }
}

//---------------------------------------------------------
// Procedure: postLegRunUpdate()

void BBDynamicsMissionManager::postLegRunUpdate(unsigned int index)
{
  if(m_run_grid.empty())
    return;

  index = index % m_run_grid.size();
  const RunPoint& pt = m_run_grid[index];

  string update = "turn_rad=" + doubleToStringX(pt.radius, 2)
                + "#speed=" + doubleToStringX(pt.speed, 2);
  Notify(m_legrun_update_var, update);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool BBDynamicsMissionManager::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "BB Dynamics Mission Manager                  " << endl;
  m_msgs << "============================================" << endl;

  m_msgs << "Voltage:        " << m_curr_voltage << endl;
  m_msgs << "Min voltage:    " << m_min_battery_voltage << endl;
  m_msgs << "Completed turns:" << m_num_turns << endl;
  m_msgs << "Run index:      " << m_run_index << endl;
  m_msgs << "Grid size:      " << m_run_grid.size() << endl;

  if(!m_run_grid.empty()) {
    unsigned int idx = m_run_index % m_run_grid.size();
    m_msgs << "Active speed:   " << m_run_grid[idx].speed << endl;
    m_msgs << "Active radius:  " << m_run_grid[idx].radius << endl;
  }

  return(true);
}




