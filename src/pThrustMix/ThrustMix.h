/************************************************************/
/*    NAME: J. Wenger                                       */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ThrustMix.h                                     */
/*    DATE: 2026-08-27 (rewrite)                            */
/*                                                          */
/*    Brief: Front-seat thrust mixer.                       */
/*                                                          */
/*      BB_SELECTED_CMD  ->  BB_MIXED_CMD                   */
/*                                                          */
/*    Converts the arbiter's semantic surge/yaw into         */
/*    physical left/right motor effort using the ArduRover   */
/*    4.7 skid-steer allocation, and copies the upstream     */
/*    lineage onward untouched.                              */
/*                                                          */
/*    This app is deliberately thin. Every rule that matters */
/*    lives in lib_bb_command's MixerStage, where it is      */
/*    tested without MOOS, a clock, or a boat. What is left  */
/*    here is mail in, publication out.                      */
/*                                                          */
/*    WHAT THIS APP MUST NOT LEARN TO DO:                    */
/*      - subscribe to RC, teleop, autonomy or ALL_STOP.     */
/*        It sees one input on purpose; a mixer that can     */
/*        observe the sources will eventually be asked to    */
/*        arbitrate between them, and then there are two     */
/*        authority policies in the system.                  */
/*      - emit PWM or apply ESC inversion. Motor effort is   */
/*        physical and forward-positive all the way to the   */
/*        Navigator's EscMapper (invariant 9).               */
/*      - close any feedback loop. The old mixer had optional*/
/*        speed-gain scheduling and a yaw-rate correction;   */
/*        both are gone. Yaw control belongs to pBBPID, and  */
/*        two interacting yaw loops is not a design.         */
/*                                                          */
/*    Replaces the k_inner/k_outer differential mixer. See   */
/*    docs/control_refactor_plan.md section 14.1 for the A/B */
/*    against 72 minutes of recorded commands.               */
/************************************************************/

#ifndef ThrustMix_HEADER
#define ThrustMix_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include "mixer_stage.h"

#include <string>

class ThrustMix : public AppCastingMOOSApp
{
 public:
  ThrustMix();
  ~ThrustMix() {}

 protected:
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();
  bool buildReport();

 protected:
  void registerVariables();

 private:
  // Owned logic.
  bb::MixerConfig     m_cfg;
  bb::MixerStage*     m_stage;        // constructed in OnStartUp, after config
  bb::DecisionMailbox m_selected;

  // Wire names, configurable so a bench rig can rename them
  // without recompiling.
  std::string m_selected_var;
  std::string m_mixed_var;

  // Cycle timing. dt is measured, not assumed from AppTick: a
  // descheduled app that pretended it ran on schedule would hand
  // the slew limiter a lie.
  double m_last_iterate_time;

  // Latest result, for buildReport().
  bb::MixedCommand m_last;
  bool             m_have_last;

  uint64_t m_iterations;
  uint64_t m_stop_cycles;
};

#endif
