/************************************************************/
/*    NAME: J. Wenger                                       */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BBCommandArbiter.cpp                            */
/*    DATE: 2026-08-27                                      */
/************************************************************/

#include "BBCommandArbiter.h"
#include "BBCommandArbiter_Info.h"

#include "MBUtils.h"
#include "ACTable.h"
#include "ColorParse.h"

#include <cstdio>

using namespace std;

BBCommandArbiter::BBCommandArbiter()
  : m_arb(0),
    m_rc(bb::CommandSource::RC),
    m_teleop(bb::CommandSource::TELEOP),
    m_autonomy(bb::CommandSource::AUTONOMY),
    m_rc_var("RC_INPUT_STATE"),
    m_teleop_var("TELEOP_CMD"),
    m_autonomy_var("AUTONOMY_CMD"),
    m_selected_var("BB_SELECTED_CMD"),
    m_have_last(false),
    m_iterations(0),
    m_stop_cycles(0),
    m_fail_closed_events(0)
{
}

//---------------------------------------------------------
// Mail: validate and cache. Never decide, never publish. A
// callback that publishes makes the decision depend on mail
// arrival order rather than on a coherent cycle, and two sources
// arriving in one tick would produce two different answers.

void BBCommandArbiter::handleSourceMail(const string& key, const string& sval,
                                        bb::CommandMailbox& box,
                                        const char* label)
{
  // Local arrival time, never the sender's stamp: front and back
  // seat clocks are not synchronised (invariant 8).
  const bb::AcceptResult r = box.accept(sval, MOOSTime());

  if (r == bb::AcceptResult::REJECTED) {
    reportRunWarning(string("rejected ") + key + ": " + box.last_reject_reason());
    Notify("BB_ARB_REJECT_SOURCE", label);
    Notify("BB_ARB_REJECT_REASON", box.last_reject_reason());
    Notify("BB_ARB_REJECT_COUNT", (double)(m_rc.reject_count() +
                                           m_teleop.reject_count() +
                                           m_autonomy.reject_count()));
  }
  else if (r == bb::AcceptResult::OUT_OF_ORDER) {
    // Not a warning-worthy fault on a lossy link, but it must be
    // visible: a source whose sequences regress is either
    // restarting without changing epoch or has two writers.
    Notify("BB_ARB_REJECT_SOURCE", label);
    Notify("BB_ARB_REJECT_REASON", bb::to_string(r));
  }
}

bool BBCommandArbiter::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    const string key = msg.GetKey();

    if (key == m_rc_var)
      handleSourceMail(key, msg.GetString(), m_rc, "RC");
    else if (key == m_teleop_var)
      handleSourceMail(key, msg.GetString(), m_teleop, "TELEOP");
    else if (key == m_autonomy_var)
      handleSourceMail(key, msg.GetString(), m_autonomy, "AUTONOMY");
    else if (key == "ALL_STOP") {
      // Accept either convention; missions have used both.
      if (msg.IsDouble()) m_safety.autonomy_all_stop = (msg.GetDouble() != 0.0);
      else {
        const string v = tolower(msg.GetString());
        m_safety.autonomy_all_stop = (v == "true" || v == "1");
      }
    }
    else if (key == "RC_KILL_ASSERTED") {
      // Trace only. The Navigator enforces this (plan 12(b)).
      if (msg.IsDouble()) m_safety.rc_kill_asserted = (msg.GetDouble() != 0.0);
      else {
        const string v = tolower(msg.GetString());
        m_safety.rc_kill_asserted = (v == "true" || v == "1");
      }
    }
    else if (key != "APPCAST_REQ")
      reportRunWarning("Unhandled mail: " + key);
  }
  return true;
}

bool BBCommandArbiter::OnConnectToServer()
{
  registerVariables();
  return true;
}

//---------------------------------------------------------

bool BBCommandArbiter::Iterate()
{
  AppCastingMOOSApp::Iterate();

  const double now = MOOSTime();
  const bb::AuthorityDecision d =
      m_arb->decide(now, m_rc, m_teleop, m_autonomy, m_safety);

  m_last = d;
  m_have_last = true;
  ++m_iterations;
  if (d.hard_stop) ++m_stop_cycles;

  // Primary contract, published every cycle including stop
  // cycles: a gap in the trace is indistinguishable from a dead
  // arbiter, and the mixer's lease depends on a steady sequence.
  Notify(m_selected_var, bb::serialize_decision(d));

  // Transitions are published immediately, not decimated. This is
  // the variable an incident review reads first.
  if (d.authority_changed || d.stop_reason_changed) {
    char ev[256];
    snprintf(ev, sizeof(ev),
             "seq=%llu,from=%s,to=%s,stop=%s,fail_closed=%d,kill=%d",
             (unsigned long long)d.decision_seq,
             bb::to_string(d.previous_source),
             bb::to_string(d.selected_source),
             bb::to_string(d.stop_reason),
             d.fail_closed ? 1 : 0,
             d.rc_kill_observed ? 1 : 0);
    Notify("BB_AUTHORITY_EVENT", ev);

    if (d.authority_changed)
      reportEvent(string("authority ") + bb::to_string(d.previous_source) +
                  " -> " + bb::to_string(d.selected_source));
    if (d.fail_closed) {
      ++m_fail_closed_events;
      reportRunWarning(string("FAIL-CLOSED: manual owner lost, stop reason ") +
                       bb::to_string(d.stop_reason));
    }
  }

  // Scalar mirrors (design doc 11) and fault reporting (12).
  // TELEMETRY ONLY -- nothing downstream may consume these as a
  // command; the coherent string above is the command.
  Notify("BB_CMD_AUTHORITY",      bb::to_string(d.selected_source));
  Notify("BB_CMD_STOP_REASON",    bb::to_string(d.stop_reason));
  Notify("BB_CMD_SURGE_SELECTED", d.surge);
  Notify("BB_CMD_YAW_SELECTED",   d.yaw);
  Notify("BB_ARB_LAST_VALID_RC_AGE",       d.rc.age);
  Notify("BB_ARB_LAST_VALID_TELEOP_AGE",   d.teleop.age);
  Notify("BB_ARB_LAST_VALID_AUTONOMY_AGE", d.autonomy.age);

  AppCastingMOOSApp::PostReport();
  return true;
}

//---------------------------------------------------------

bool BBCommandArbiter::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if (param == "rc_timeout_sec")
      handled = setDoubleOnString(m_cfg.rc_timeout_sec, value);
    else if (param == "teleop_timeout_sec")
      handled = setDoubleOnString(m_cfg.teleop_timeout_sec, value);
    else if (param == "autonomy_timeout_sec")
      handled = setDoubleOnString(m_cfg.autonomy_timeout_sec, value);
    else if (param == "allow_autonomy_before_first_rc")
      handled = setBooleanOnString(m_cfg.allow_autonomy_before_first_rc, value);
    else if (param == "rc_input_var")    { m_rc_var = value;       handled = true; }
    else if (param == "teleop_cmd_var")  { m_teleop_var = value;   handled = true; }
    else if (param == "autonomy_cmd_var"){ m_autonomy_var = value; handled = true; }
    else if (param == "selected_cmd_var"){ m_selected_var = value; handled = true; }

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  // Invalid configuration FAILS STARTUP. An arbiter with a
  // nonsensical timeout should refuse to run rather than discover
  // it with props in the water.
  const string err = m_cfg.validate();
  if (!err.empty()) {
    reportConfigWarning("FATAL: " + err);
    cout << termColor("red")
         << "pBBCommandArbiter: invalid configuration: " << err << endl
         << termColor();
    return false;
  }

  m_arb = new bb::AuthorityArbiter(m_cfg, bb::make_epoch("arb"));

  registerVariables();
  return true;
}

void BBCommandArbiter::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register(m_rc_var, 0);
  Register(m_teleop_var, 0);
  Register(m_autonomy_var, 0);
  Register("ALL_STOP", 0);
  Register("RC_KILL_ASSERTED", 0);
}

//---------------------------------------------------------

static string eval_line(const char* name, const bb::SourceEvaluation& e)
{
  char b[192];
  snprintf(b, sizeof(b), "%-9s %-14s age %7.3f  %s%s%s%s",
           name, e.reason, e.age,
           e.ever_seen ? "seen " : "unseen ",
           e.requesting_authority ? "requesting " : "",
           e.valid ? "valid " : "",
           e.fresh ? "fresh" : "stale");
  return string(b);
}

bool BBCommandArbiter::buildReport()
{
  if (!m_arb) { m_msgs << "NOT CONFIGURED" << endl; return true; }

  m_msgs << "Decision epoch: " << m_arb->epoch()
         << "   seq: " << m_arb->seq() << endl;
  m_msgs << "Timeouts:       rc " << doubleToString(m_cfg.rc_timeout_sec, 2)
         << "  teleop " << doubleToString(m_cfg.teleop_timeout_sec, 2)
         << "  autonomy " << doubleToString(m_cfg.autonomy_timeout_sec, 2)
         << " s" << endl;
  m_msgs << "Pre-RC autonomy: "
         << (m_cfg.allow_autonomy_before_first_rc ? "allowed" : "inhibited") << endl;
  m_msgs << endl;

  if (!m_have_last) { m_msgs << "No cycle completed yet." << endl; return true; }

  m_msgs << "SELECTED: " << bb::to_string(m_last.selected_source);
  if (m_last.hard_stop)
    m_msgs << "   *** HARD STOP: " << bb::to_string(m_last.stop_reason) << " ***";
  m_msgs << endl;
  m_msgs << "  surge " << doubleToString(m_last.surge, 2)
         << "   yaw " << doubleToString(m_last.yaw, 2) << endl;
  if (m_last.fail_closed)
    m_msgs << "  FAIL-CLOSED: a manual owner lost the boat; NOT handed to autonomy"
           << endl;
  if (m_last.rc_kill_observed)
    m_msgs << "  RC KILL asserted (enforced at the Navigator, not here)" << endl;
  m_msgs << endl;

  m_msgs << "Sources" << endl;
  m_msgs << "  " << eval_line("RC",       m_last.rc)       << endl;
  m_msgs << "  " << eval_line("TELEOP",   m_last.teleop)   << endl;
  m_msgs << "  " << eval_line("AUTONOMY", m_last.autonomy) << endl;
  m_msgs << endl;

  ACTable actab(5);
  actab << "Source | Accepted | Dup | OutOfOrder | Rejected";
  actab.addHeaderLines();
  actab << "RC"       << uintToString(m_rc.accepted_count())
        << uintToString(m_rc.duplicate_count())
        << uintToString(m_rc.out_of_order_count())
        << uintToString(m_rc.reject_count());
  actab << "TELEOP"   << uintToString(m_teleop.accepted_count())
        << uintToString(m_teleop.duplicate_count())
        << uintToString(m_teleop.out_of_order_count())
        << uintToString(m_teleop.reject_count());
  actab << "AUTONOMY" << uintToString(m_autonomy.accepted_count())
        << uintToString(m_autonomy.duplicate_count())
        << uintToString(m_autonomy.out_of_order_count())
        << uintToString(m_autonomy.reject_count());
  m_msgs << actab.getFormattedString() << endl << endl;

  m_msgs << "Safety:  ALL_STOP " << (m_safety.autonomy_all_stop ? "TRUE" : "false")
         << "   RC_KILL " << (m_safety.rc_kill_asserted ? "ASSERTED" : "clear") << endl;
  m_msgs << "Cycles:  " << m_iterations
         << "   stopped " << m_stop_cycles
         << " (" << doubleToString(m_iterations ? 100.0 * m_stop_cycles / m_iterations : 0.0, 1)
         << "%)   fail-closed events " << m_fail_closed_events << endl;
  return true;
}
