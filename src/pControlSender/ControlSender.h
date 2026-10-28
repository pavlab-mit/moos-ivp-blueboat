/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ControlSender.h                                 */
/*    DATE: 2026-10-28                                      */
/*                                                          */
/*    Brief: Re-sends pBBPID's command frame with the       */
/*    barrier filter's thrust/rudder swapped in.            */
/*                                                          */
/*      AUTONOMY_CMD_PRECBF ]                               */
/*      DESIRED_THRUST      ]-> AUTONOMY_CMD                */
/*      DESIRED_RUDDER      ]                               */
/*     [CBF_SEQ]            ]                               */
/*                                                          */
/*    Every rule lives in ControlSenderCore, which has no   */
/*    MOOS in it and is unit tested against the real        */
/*    arbiter. This app is mail in, publication out.        */
/*                                                          */
/*    WHAT THIS APP MUST NOT LEARN TO DO:                   */
/*      - mint a sequence number. It re-sends pBBPID's      */
/*        epoch/seq, which is what lets a dead pBBPID still */
/*        stop the boat through the arbiter's lease.        */
/*      - send a valid frame without a fresh filter output. */
/*        A frame with stale values is how a dead filter    */
/*        would keep the boat driving.                      */
/*      - fall back to pBBPID's raw values when the filter  */
/*        is unhealthy. It stops instead (no silent         */
/*        fallthrough, as in the arbiter).                  */
/*      - publish from OnNewMail. One decision per cycle.   */
/************************************************************/

#ifndef ControlSender_HEADER
#define ControlSender_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include "ControlSenderCore.h"

#include <memory>
#include <string>

class ControlSender : public AppCastingMOOSApp
{
 public:
   ControlSender();
   ~ControlSender() {}

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
   bool buildReport();

 protected:
   void registerVariables();
   bool readNumber(CMOOSMsg& msg, double& out);
   bool isOldMail(CMOOSMsg& msg, double now);
   void handleOutputSeen(const std::string& text);
   void publishState(const std::string& state);

 private: // Configuration
   pcs::ControlSenderConfig m_cfg;

   std::string m_precbf_var;     // pBBPID's frame
   std::string m_thrust_var;     // filter output
   std::string m_rudder_var;     // filter output
   std::string m_filter_seq_var; // optional; "" = arrival mode
   std::string m_output_var;     // to the arbiter

   double m_precbf_warn_sec;            // warn if pBBPID's frame absent
   double m_max_input_age_sec;          // drop replayed (old) mail
   bool   m_all_stop_on_foreign_writer; // post ALL_STOP on a bypass

 private: // State
   std::unique_ptr<pcs::ControlSenderCore> m_core;
   bool   m_config_parsed;

   double m_start_time;
   double m_last_frame_rx;

   pcs::SendDecision m_last;
   bool   m_have_last;
   std::string m_state;

   bool   m_warn_no_precbf;
   std::string m_no_precbf_warning;
   bool   m_warn_filter;
   std::string m_filter_warning;

   uint64_t m_frame_rejects;
   uint64_t m_frame_out_of_order;
   uint64_t m_old_inputs_dropped;
   std::string m_last_reject;

   uint64_t m_foreign_frames;
   uint64_t m_unparseable_outputs;
   std::string m_last_foreign;
   bool   m_foreign_pending;
   double m_last_all_stop_post;
   uint64_t m_all_stop_posts;
};

#endif
