/*************************************************************
      Name:
      Orgn: MIT, Cambridge MA
      File: pBB_Status/BB_Status.h
   Last Ed: 2026-06-03
     Brief: Front-seat status consolidator for the BlueBoat.
            Subscribes to native front-seat hardware/RC/nav
            variables plus a small set of autonomy variables
            brokered over from the backseat, and publishes ONE
            consolidated BB_STATUS string for shoreside
            monitoring (Subsystem B). Read-only aggregator; it
            commands nothing.
*************************************************************/

#ifndef BB_Status_HEADER
#define BB_Status_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <netinet/in.h>
#include <string>

class BB_Status : public AppCastingMOOSApp
{
 public:
   BB_Status();
   ~BB_Status();

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
   bool buildReport();

 protected:
   void registerVariables();
   std::string deriveMode() const;     // fused control mode
   bool missionRunning() const;        // deploy && helm DRIVE && fresh
   std::string buildStatusString();    // assemble BB_STATUS
   std::string staleList() const;      // which input groups are stale
   bool setupSocket();                 // open the shoreside UDP socket
   bool resolveDest();                 // resolve tx_ip into m_dest

 private: // Configuration
   std::string m_status_var;     // MOOS var to publish (default BB_STATUS)
   std::string m_vname;
   double      m_stale_time;     // sec before an input is "stale"
   double      m_low_voltage_thresh;
   double      m_publish_interval;  // sec between BB_STATUS posts
   double      m_last_publish_time;

 private: // Shoreside UDP transport (pushes BB_STATUS to the collector)
   std::string        m_tx_ip;   // collector IP or hostname; "" disables UDP
   int                m_tx_port;
   int                m_sockfd;   // -1 when disabled / not open
   struct sockaddr_in m_dest;
   bool               m_dest_ok;  // m_dest holds a resolved address
   double             m_last_resolve_try;
   unsigned int       m_tx_sent;
   unsigned int       m_tx_errs;

 private: // ---- Power (native front seat: iBBNavigatorInterface) ----
   double m_volt, m_curr, m_power;
   double m_volt_time, m_curr_time, m_power_time;

 private: // ---- Thermal / enclosure (native front seat) ----
   double m_rpi_temp, m_int_temp, m_int_kpa;
   double m_rpi_temp_time, m_int_temp_time, m_int_kpa_time;

 private: // ---- RC / override (native front seat: iRCReader/Navigator) ----
   bool   m_rc_connected;
   bool   m_rc_failsafe;
   bool   m_rc_deadman;
   double m_rc_ch6;
   double m_rc_time;

 private: // ---- Laptop teleop (native front seat: iTeleop/Navigator) ----
   bool   m_teleop_engaged;    // NVGR_TELEOP_ENGAGED
   double m_teleop_time;

 private: // ---- Propulsion (applied native; commanded brokered) ----
   double m_thr_l, m_thr_r;        // applied (NVGR_THRUST_*)
   double m_des_l, m_des_r;        // commanded from backseat (DESIRED_THRUST_*)
   bool   m_thr_timeout;
   double m_thr_time, m_des_time;

 private: // ---- Navigation / GPS (native front seat: iUnicore) ----
   double      m_nav_lat, m_nav_lon, m_nav_spd, m_nav_hdg;
   double      m_nav_time;
   std::string m_fix_type;
   int         m_num_sats;
   double      m_hdop;
   double      m_gps_time;

 private: // ---- Autonomy (brokered from backseat; may be absent) ----
   std::string m_helm_state;   // IVPHELM_STATE: DRIVE/PARK/STANDBY/...
   std::string m_mode;         // mission MODE string
   bool        m_deploy;
   bool        m_all_stop;
   double      m_helm_time, m_mode_time, m_deploy_time, m_allstop_time;

 private: // ---- Derived staleness flags (recomputed each Iterate) ----
   bool m_stale_power, m_stale_temp, m_stale_rc;
   bool m_stale_thr, m_stale_nav, m_stale_gps, m_stale_helm;

 private: // counters
   unsigned int m_posts;
};

#endif
