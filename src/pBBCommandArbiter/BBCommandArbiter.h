/************************************************************/
/*    NAME: J. Wenger                                       */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BBCommandArbiter.h                              */
/*    DATE: 2026-08-27                                      */
/*                                                          */
/*    Brief: The only process allowed to decide who is      */
/*    driving the boat.                                     */
/*                                                          */
/*      RC_INPUT_STATE  ]                                   */
/*      TELEOP_CMD      ]-> BB_SELECTED_CMD                 */
/*      AUTONOMY_CMD    ]                                   */
/*                                                          */
/*    Priority is RC > TELEOP > AUTONOMY, but priority is    */
/*    not the interesting part. Two rules are:               */
/*                                                          */
/*      COMMAND IS NOT AUTHORITY. A source publishing a      */
/*      valid command does not thereby get to drive. RC must */
/*      hold the guarded mode switch; teleop must assert a   */
/*      claim. Moving a stick in AUTO means nothing.         */
/*                                                          */
/*      NO SILENT FALLTHROUGH. If a manual owner goes stale  */
/*      or invalid, the boat STOPS. It does not quietly hand */
/*      control to autonomy. An operator whose link just     */
/*      died is not asking for an autonomous mission.        */
/*                                                          */
/*    Every rule lives in lib_bb_command's AuthorityArbiter, */
/*    tested without MOOS. This app is mail in, publication  */
/*    out.                                                   */
/*                                                          */
/*    WHAT THIS APP MUST NOT LEARN TO DO:                    */
/*      - mix. It emits semantic surge/yaw. Left/right       */
/*        allocation belongs to pThrustMix, PWM to the       */
/*        Navigator.                                         */
/*      - enforce RC kill. It observes RC_KILL_ASSERTED for  */
/*        the trace only; the Navigator enforces it on the   */
/*        hardware side where it survives a failure of this  */
/*        process. See plan section 12(b).                   */
/*      - implement the RC deadman. That is a Navigator      */
/*        blanket (plan 12(d)). What lives here is the       */
/*        SOURCE LEASE, which is never disableable.          */
/************************************************************/

#ifndef BBCommandArbiter_HEADER
#define BBCommandArbiter_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include "authority.h"

#include <string>

class BBCommandArbiter : public AppCastingMOOSApp
{
 public:
  BBCommandArbiter();
  ~BBCommandArbiter() {}

 protected:
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();
  bool buildReport();

 protected:
  void registerVariables();
  void handleSourceMail(const std::string& key, const std::string& sval,
                        bb::CommandMailbox& box, const char* label);

 private:
  bb::ArbiterConfig     m_cfg;
  bb::AuthorityArbiter* m_arb;      // built in OnStartUp, after config

  bb::CommandMailbox m_rc;
  bb::CommandMailbox m_teleop;
  bb::CommandMailbox m_autonomy;

  bb::SafetyInputs m_safety;

  // Wire names, configurable for bench rigs.
  std::string m_rc_var;
  std::string m_teleop_var;
  std::string m_autonomy_var;
  std::string m_selected_var;

  bb::AuthorityDecision m_last;
  bool m_have_last;

  uint64_t m_iterations;
  uint64_t m_stop_cycles;
  uint64_t m_fail_closed_events;
};

#endif
