/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ControlSender.cpp                               */
/*    DATE: 2026-10-28                                      */
/************************************************************/

#include <cmath>
#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "ColorParse.h"
#include "wire_format.h"
#include "ControlSender.h"

using namespace std;

namespace {
// Constant so retractRunWarning() matches what was reported.
const char* kFilterWarn =
    "Barrier filter unhealthy: valid pBBPID frames are being sent as valid=0";
}

//---------------------------------------------------------
// Constructor

ControlSender::ControlSender()
  : m_precbf_var("AUTONOMY_CMD_PRECBF"),
    m_thrust_var("DESIRED_THRUST"),
    m_rudder_var("DESIRED_RUDDER"),
    m_filter_seq_var(""),
    m_output_var("AUTONOMY_CMD"),
    m_precbf_warn_sec(3.0),
    m_max_input_age_sec(1.0),
    m_all_stop_on_foreign_writer(true),
    m_config_parsed(false),
    m_start_time(0.0),
    m_last_frame_rx(0.0),
    m_have_last(false),
    m_warn_no_precbf(false),
    m_warn_filter(false),
    m_frame_rejects(0),
    m_frame_out_of_order(0),
    m_old_inputs_dropped(0),
    m_foreign_frames(0),
    m_unparseable_outputs(0),
    m_foreign_pending(false),
    m_last_all_stop_post(-1.0e9),
    m_all_stop_posts(0)
{
}

//---------------------------------------------------------
// readNumber: filter outputs may arrive as doubles or as
// strings; read either with the shared strict dialect.

bool ControlSender::readNumber(CMOOSMsg& msg, double& out)
{
  if (msg.IsDouble()) { out = msg.GetDouble(); return true; }
  if (msg.IsString() && bb::wire::parse_double(msg.GetString(), out)) return true;
  reportRunWarning(msg.GetKey() + " is not a number");
  return false;
}

//---------------------------------------------------------
// isOldMail: MOOSDB hands a new subscriber the LAST stored value
// of every variable it registers for, however old. After a
// restart of this app that would be pBBPID's last frame and the
// filter's last output -- possibly from before pBBPID died. Mail
// whose publisher stamp is older than max_input_age_sec is not a
// command, it is history, and is dropped.
//
// This compares the publisher's stamp with our clock, which is
// sound because pControlSender, pBBPID and the filter all run on
// the backseat computer. Freshness itself is still judged on
// local arrival time, in the core.

bool ControlSender::isOldMail(CMOOSMsg& msg, double now)
{
  if ((now - msg.GetTime()) <= m_max_input_age_sec)
    return false;
  ++m_old_inputs_dropped;
  return true;
}

//---------------------------------------------------------
// OnNewMail: validate and cache only. Never send a frame from
// here -- the decision is made once per cycle in Iterate().

bool ControlSender::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    const string key = msg.GetKey();
    const double rx  = MOOSTime();     // local arrival (invariant 8)

    if(key == "APPCAST_REQ")
      continue;
    if(!m_core) {
      reportRunWarning("Mail before configuration: " + key);
      continue;
    }

    if(key == m_precbf_var) {
      if(!msg.IsString()) {
        reportRunWarning(m_precbf_var + " is not a string");
        continue;
      }
      if(isOldMail(msg, rx))
        continue;
      const bb::AcceptResult r = m_core->onFrame(msg.GetString(), rx);
      if(r == bb::AcceptResult::ACCEPTED)
        m_last_frame_rx = rx;
      else if(r == bb::AcceptResult::REJECTED) {
        ++m_frame_rejects;
        m_last_reject = m_core->frameMailbox().last_reject_reason();
        reportRunWarning("rejected " + key + ": " + m_last_reject);
        Notify("CTLSEND_REJECT_REASON", m_last_reject);
      }
      else if(r == bb::AcceptResult::OUT_OF_ORDER) {
        // pBBPID's seq went backwards without an epoch change:
        // two writers, or a restart that kept its epoch.
        ++m_frame_out_of_order;
        Notify("CTLSEND_REJECT_REASON", bb::to_string(r));
      }
    }
    else if(key == m_thrust_var) {
      double v = 0.0;
      if(!isOldMail(msg, rx) && readNumber(msg, v))
        m_core->onThrust(v, msg.GetTime(), rx);
    }
    else if(key == m_rudder_var) {
      double v = 0.0;
      if(!isOldMail(msg, rx) && readNumber(msg, v))
        m_core->onRudder(v, msg.GetTime(), rx);
    }
    else if(!m_filter_seq_var.empty() && (key == m_filter_seq_var)) {
      double v = 0.0;
      if(isOldMail(msg, rx) || !readNumber(msg, v))
        continue;
      // An exact non-negative integer, as published by the filter
      // from the frame's seq field. 2^53 bounds exact doubles.
      if(!std::isfinite(v) || (v < 0.0) || (v != std::floor(v)) || (v > 9.0e15)) {
        reportRunWarning(key + " is not a valid seq: " + doubleToStringX(v, 3));
        continue;
      }
      m_core->onFilterSeq((uint64_t)v, msg.GetTime(), rx);
    }
    else if(key == m_output_var) {
      if(msg.IsString())
        handleOutputSeen(msg.GetString());
    }
    else
      reportRunWarning("Unhandled Mail: " + key);
  }

  return(true);
}

//---------------------------------------------------------
// handleOutputSeen: the bypass guard.
//
// We subscribe to our own output. Our frames carry cbf=1 and are
// ignored. Anything else on it is a second writer -- almost always
// pBBPID still on its default autonomy_cmd_var -- and its frames
// reach the arbiter WITHOUT the barrier filter. When pBBPID is the
// culprit, AUTONOMY_CMD_PRECBF also goes quiet and this app sends
// nothing, so the raw frames are all the arbiter sees.

void ControlSender::handleOutputSeen(const string& text)
{
  const pcs::OutputCheck oc = m_core->checkOutput(text);
  if(oc.origin == pcs::OutputOrigin::OURS)
    return;

  if(oc.origin == pcs::OutputOrigin::UNPARSEABLE) {
    ++m_unparseable_outputs;
    reportRunWarning(m_output_var + " carries a frame the arbiter will reject: " +
                     oc.detail);
    return;
  }

  ++m_foreign_frames;
  const string who = "producer=" + oc.producer + ",epoch=" + oc.epoch;
  if(who != m_last_foreign) {
    m_last_foreign = who;
    reportRunWarning("SECOND WRITER on " + m_output_var + " (" + who +
                     "): commands reach the arbiter WITHOUT the barrier filter."
                     " Check pBBPID's autonomy_cmd_var.");
    Notify("CTLSEND_FOREIGN_WRITER", who);
  }
  m_foreign_pending = true;   // ALL_STOP, if configured, goes out in Iterate
}

//---------------------------------------------------------

void ControlSender::publishState(const string& state)
{
  if(state == m_state)
    return;
  m_state = state;
  Notify("CTLSEND_STATE", state);
}

//---------------------------------------------------------
// OnConnectToServer
//
// The connect callback can fire before OnStartUp has parsed the
// config, which would subscribe the DEFAULT variable names and
// orphan any override. Register only once names are final;
// OnStartUp does the first registration itself.

bool ControlSender::OnConnectToServer()
{
  if(m_config_parsed)
    registerVariables();
  return(true);
}

//---------------------------------------------------------
// Iterate: at most one frame per cycle, decided by the core.

bool ControlSender::Iterate()
{
  AppCastingMOOSApp::Iterate();
  if(!m_core) {
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  const double now = MOOSTime();
  const pcs::SendDecision d = m_core->tick(now);

  if(d.kind != pcs::SendKind::NONE) {
    Notify(m_output_var, d.text);
    m_last = d;
    m_have_last = true;

    // Telemetry. CTLSEND_LATENCY is pBBPID's tick -> this send,
    // on the one backseat clock: the delay the filter path adds.
    Notify("CTLSEND_LATENCY",    d.latency);
    Notify("CTLSEND_SEQ_LAG",    (double)d.seq_lag);
    Notify("CTLSEND_FILTER_AGE", m_core->filterAge(now));
    if(d.kind == pcs::SendKind::FILTERED) {
      Notify("CTLSEND_SURGE_DELTA", d.sent.surge - d.raw_surge);
      Notify("CTLSEND_YAW_DELTA",   d.sent.yaw   - d.raw_yaw);
    }
    publishState(pcs::to_string(d.kind));
  }

  // Filter health, as it affects the boat: only while valid
  // frames are being turned into stops.
  if(d.kind == pcs::SendKind::FILTER_STALE) {
    const string why = d.sent.field(pcs::ControlSenderCore::kFaultKey);
    if(!m_warn_filter || (why != m_filter_warning)) {
      reportEvent("filter unhealthy (" + why + "): sending valid=0");
      m_filter_warning = why;
    }
    if(!m_warn_filter) {
      reportRunWarning(kFilterWarn);
      m_warn_filter = true;
    }
  }
  else if((d.kind == pcs::SendKind::FILTERED) && m_warn_filter) {
    retractRunWarning(kFilterWarn);
    reportEvent("filter healthy again");
    m_warn_filter = false;
    m_filter_warning.clear();
  }

  // pBBPID's frame absent. Normal before pBBPID starts; after
  // that, the usual cause is pBBPID still publishing straight to
  // the output variable, i.e. bypassing the filter.
  const double since = m_core->hasFrame() ? (now - m_last_frame_rx)
                                          : (now - m_start_time);
  if(!m_core->hasFrame() || (since > m_precbf_warn_sec))
    publishState("WAITING_FOR_PRECBF");
  if(since > m_precbf_warn_sec) {
    if(!m_warn_no_precbf) {
      reportRunWarning(m_no_precbf_warning);
      m_warn_no_precbf = true;
    }
  }
  else if(m_warn_no_precbf) {
    retractRunWarning(m_no_precbf_warning);
    m_warn_no_precbf = false;
  }

  // Bypass guard. ALL_STOP gates autonomy only (never RC or
  // teleop) and is latched by the arbiter until someone posts
  // false. Re-posted at most once a second while the second
  // writer persists, so clearing it does not quietly resume an
  // unfiltered mission.
  if(m_foreign_pending) {
    m_foreign_pending = false;
    if(m_all_stop_on_foreign_writer && ((now - m_last_all_stop_post) > 1.0)) {
      Notify("ALL_STOP", "true");
      m_last_all_stop_post = now;
      if(++m_all_stop_posts == 1)
        reportEvent("posted ALL_STOP: second writer on " + m_output_var);
    }
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// OnStartUp: read configuration. Invalid configuration FAILS
// STARTUP, as in the arbiter and mixer: a command relay with a
// nonsensical timeout should refuse to run, not find out at sea.

bool ControlSender::OnStartUp()
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
    if(param == "precbf_var")           { m_precbf_var = value;     handled = true; }
    else if(param == "thrust_var")      { m_thrust_var = value;     handled = true; }
    else if(param == "rudder_var")      { m_rudder_var = value;     handled = true; }
    else if(param == "filter_seq_var")  { m_filter_seq_var = value; handled = true; }
    else if(param == "output_var")      { m_output_var = value;     handled = true; }
    else if(param == "producer")        { m_cfg.producer = value;   handled = true; }
    else if(param == "filter_timeout_sec")
      handled = setDoubleOnString(m_cfg.filter_timeout_sec, value);
    else if(param == "pair_tolerance_sec")
      handled = setDoubleOnString(m_cfg.pair_tolerance_sec, value);
    else if(param == "precbf_warn_sec")
      handled = setDoubleOnString(m_precbf_warn_sec, value);
    else if(param == "max_input_age_sec")
      handled = setDoubleOnString(m_max_input_age_sec, value);
    else if(param == "all_stop_on_foreign_writer")
      handled = setBooleanOnString(m_all_stop_on_foreign_writer, value);

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  m_cfg.use_filter_seq = !m_filter_seq_var.empty();

  string err = m_cfg.validate();
  if(err.empty()) {
    if(m_precbf_var.empty() || m_thrust_var.empty() ||
       m_rudder_var.empty() || m_output_var.empty())
      err = "precbf_var, thrust_var, rudder_var and output_var must be set";
    else if(m_output_var == m_precbf_var)
      err = "output_var must differ from precbf_var (the frame would loop)";
    else if((m_thrust_var == m_rudder_var) ||
            (m_thrust_var == m_output_var) || (m_rudder_var == m_output_var) ||
            (m_thrust_var == m_precbf_var) || (m_rudder_var == m_precbf_var))
      err = "every input and output variable must be distinct";
    else if(!m_filter_seq_var.empty() &&
            ((m_filter_seq_var == m_thrust_var) || (m_filter_seq_var == m_rudder_var) ||
             (m_filter_seq_var == m_precbf_var) || (m_filter_seq_var == m_output_var)))
      err = "filter_seq_var must differ from every other variable";
    else if(!std::isfinite(m_precbf_warn_sec) || (m_precbf_warn_sec <= 0.0))
      err = "precbf_warn_sec must be finite and > 0";
    else if(!std::isfinite(m_max_input_age_sec) || (m_max_input_age_sec <= 0.0))
      err = "max_input_age_sec must be finite and > 0";
  }
  if(!err.empty()) {
    reportConfigWarning("FATAL: " + err);
    cout << termColor("red") << "pControlSender: invalid configuration: "
         << err << endl << termColor();
    return(false);
  }

  m_no_precbf_warning = "No " + m_precbf_var + " from pBBPID for over " +
      doubleToStringX(m_precbf_warn_sec, 1) + " s. If pBBPID is running, "
      "check that its autonomy_cmd_var is " + m_precbf_var + ".";

  m_core.reset(new pcs::ControlSenderCore(m_cfg));
  m_start_time = MOOSTime();

  m_config_parsed = true;   // names are final; see OnConnectToServer
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// registerVariables

void ControlSender::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register(m_precbf_var, 0);
  Register(m_thrust_var, 0);
  Register(m_rudder_var, 0);
  if(!m_filter_seq_var.empty())
    Register(m_filter_seq_var, 0);
  // Our own output, for the bypass guard.
  Register(m_output_var, 0);
}

//------------------------------------------------------------
// buildReport

bool ControlSender::buildReport()
{
  m_msgs << "pControlSender: pBBPID frame + barrier filter -> " << m_output_var << endl;
  m_msgs << "============================================================" << endl;
  if(!m_core) {
    m_msgs << "NOT CONFIGURED" << endl;
    return(true);
  }

  const double now = MOOSTime();
  const pcs::ControlSenderConfig& cfg = m_core->config();

  m_msgs << "In:      " << m_precbf_var << " (pBBPID)" << endl;
  m_msgs << "Filter:  " << m_thrust_var << ", " << m_rudder_var;
  if(!m_filter_seq_var.empty())
    m_msgs << ", " << m_filter_seq_var;
  m_msgs << "   match by " << (cfg.use_filter_seq ? "seq" : "arrival") << endl;
  m_msgs << "Out:     " << m_output_var << "   producer=" << cfg.producer
         << "   marker " << pcs::ControlSenderCore::kMarkerKey << "=1" << endl;
  m_msgs << "Timing:  filter_timeout " << doubleToString(cfg.filter_timeout_sec, 2)
         << " s   pair_tolerance " << doubleToString(cfg.pair_tolerance_sec, 3)
         << " s   max_input_age " << doubleToString(m_max_input_age_sec, 2) << " s" << endl;
  m_msgs << "Guard:   " << (m_all_stop_on_foreign_writer ? "ALL_STOP on a second writer"
                                                         : "warn only on a second writer")
         << endl << endl;

  m_msgs << "State:   " << (m_state.empty() ? "-" : m_state)
         << "   filter " << pcs::to_string(m_core->filterHealth(now))
         << " (last output " << doubleToString(m_core->filterAge(now), 3) << " s ago)" << endl;
  if(m_core->hasFrame()) {
    const bb::SemanticCommand& f = m_core->newestFrame();
    m_msgs << "Newest:  " << f.epoch << "/" << f.seq
           << (f.valid ? "  valid" : "  INVALID")
           << "  age " << doubleToString(now - f.rx_time, 3) << " s" << endl;
  }
  else
    m_msgs << "Newest:  no frame from pBBPID yet" << endl;
  m_msgs << endl;

  const bb::CommandMailbox& mb = m_core->frameMailbox();
  ACTable ftab(6);
  ftab << "pBBPID frames | Accepted | Dup | OutOfOrder | Rejected | Restarts";
  ftab.addHeaderLines();
  ftab << m_precbf_var << uintToString(mb.accepted_count())
       << uintToString(mb.duplicate_count()) << uintToString(mb.out_of_order_count())
       << uintToString(mb.reject_count()) << uintToString(mb.epoch_change_count());
  m_msgs << ftab.getFormattedString() << endl;
  if(!m_last_reject.empty())
    m_msgs << "  last reject: " << m_last_reject << endl;
  m_msgs << endl;

  ACTable ptab(6);
  ptab << "Filter | Outputs | Mismatched | NonFinite | Unmatched | Unused";
  ptab.addHeaderLines();
  ptab << "pairs" << uintToString(m_core->pairsFormed())
       << uintToString(m_core->pairMismatches()) << uintToString(m_core->nonfiniteRejects())
       << uintToString(m_core->unmatchedPairs()) << uintToString(m_core->unusedPairs());
  m_msgs << ptab.getFormattedString() << endl;
  m_msgs << "  old mail dropped (startup replay): " << uintToString(m_old_inputs_dropped) << endl;
  m_msgs << endl;

  ACTable stab(5);
  stab << "Sent | Filtered | ForwardedInvalid | FilterStale | Skipped";
  stab.addHeaderLines();
  stab << m_output_var << uintToString(m_core->sentFiltered())
       << uintToString(m_core->sentForwarded()) << uintToString(m_core->sentStale())
       << uintToString(m_core->framesSkipped());
  m_msgs << stab.getFormattedString() << endl;
  m_msgs << "  clamped to +/-100: " << uintToString(m_core->clampCount()) << endl << endl;

  if(m_have_last) {
    m_msgs << "Last sent: " << pcs::to_string(m_last.kind) << "   "
           << m_last.sent.epoch << "/" << m_last.sent.seq
           << (m_last.sent.valid ? "  valid" : "  INVALID");
    const string fault = m_last.sent.field(pcs::ControlSenderCore::kFaultKey);
    if(!fault.empty())
      m_msgs << "  " << pcs::ControlSenderCore::kFaultKey << "=" << fault;
    m_msgs << endl;
    ACTable ltab(3);
    ltab << "Field | pBBPID | Sent";
    ltab.addHeaderLines();
    ltab << "surge" << doubleToString(m_last.raw_surge, 2) << doubleToString(m_last.sent.surge, 2);
    ltab << "yaw"   << doubleToString(m_last.raw_yaw, 2)   << doubleToString(m_last.sent.yaw, 2);
    m_msgs << ltab.getFormattedString() << endl;
    m_msgs << "  latency " << doubleToString(m_last.latency, 3) << " s   seq lag "
           << uintToString(m_last.seq_lag) << endl << endl;
  }

  m_msgs << "Bypass guard: foreign frames " << uintToString(m_foreign_frames);
  if(!m_last_foreign.empty())
    m_msgs << " (last " << m_last_foreign << ")";
  m_msgs << "   unparseable " << uintToString(m_unparseable_outputs)
         << "   ALL_STOP posts " << uintToString(m_all_stop_posts) << endl;

  return(true);
}
