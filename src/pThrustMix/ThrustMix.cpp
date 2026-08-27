/************************************************************/
/*    NAME: J. Wenger                                       */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ThrustMix.cpp                                   */
/*    DATE: 2026-08-27 (rewrite)                            */
/************************************************************/

#include "ThrustMix.h"
#include "ThrustMix_Info.h"

#include "MBUtils.h"
#include "ACTable.h"
#include "ColorParse.h"

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <unistd.h>

using namespace std;

//---------------------------------------------------------
// A per-launch epoch. Deliberately NOT wall-clock: two boats
// booting in the same second must not be able to collide, and a
// clock that steps must not be able to make a new session look
// like an old one. Random plus pid is enough for a log join key.

static string make_epoch(const char* prefix)
{
  unsigned seed = (unsigned)::getpid();
  seed ^= (unsigned)::time(NULL) * 2654435761u;
  ::srandom(seed);
  char buf[32];
  snprintf(buf, sizeof(buf), "%s-%06lx", prefix, (long)(::random() & 0xFFFFFF));
  return string(buf);
}

//---------------------------------------------------------

ThrustMix::ThrustMix()
  : m_stage(0),
    m_selected_var("BB_SELECTED_CMD"),
    m_mixed_var("BB_MIXED_CMD"),
    m_last_iterate_time(0.0),
    m_have_last(false),
    m_iterations(0),
    m_stop_cycles(0)
{
}

//---------------------------------------------------------
// OnNewMail: validate and cache only. Never publish from here --
// a callback that writes actuator state makes the output depend
// on mail arrival order rather than on a coherent cycle.

bool ThrustMix::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    if (msg.GetKey() == m_selected_var) {
      // Local arrival time, not the sender's stamp (invariant 8).
      const bb::AcceptResult r = m_selected.accept(msg.GetString(), MOOSTime());
      if (r == bb::AcceptResult::REJECTED) {
        reportRunWarning("rejected " + m_selected_var + ": " +
                         m_selected.last_reject_reason());
        Notify("BB_MIX_INPUT_REJECT_REASON", m_selected.last_reject_reason());
      }
    }
    else if (msg.GetKey() != "APPCAST_REQ") {
      reportRunWarning("Unhandled mail: " + msg.GetKey());
    }
  }
  return true;
}

bool ThrustMix::OnConnectToServer()
{
  registerVariables();
  return true;
}

//---------------------------------------------------------
// Iterate: one coherent result per cycle, always. Including
// stop cycles -- a gap in the trace is indistinguishable from a
// dead app.

bool ThrustMix::Iterate()
{
  AppCastingMOOSApp::Iterate();

  const double now = MOOSTime();

  // Measured dt. The first cycle has no previous timestamp, so
  // give the limiter nothing rather than a fabricated interval.
  double dt = 0.0;
  if (m_last_iterate_time > 0.0) dt = now - m_last_iterate_time;
  m_last_iterate_time = now;

  const bb::MixedCommand out = m_stage->update(now, dt, m_selected);

  m_last = out;
  m_have_last = true;
  ++m_iterations;
  if (out.hard_stop) ++m_stop_cycles;

  // Primary contract.
  Notify(m_mixed_var, bb::serialize_mixed(out));

  // Plot-friendly scalar mirrors (design doc 11). These are
  // TELEMETRY: nothing downstream may consume them as a command.
  Notify("BB_MIX_LEFT",          out.left_effort);
  Notify("BB_MIX_RIGHT",         out.right_effort);
  Notify("BB_MIX_SURGE_SHAPED",  out.surge_shaped);
  Notify("BB_MIX_YAW_SHAPED",    out.yaw_shaped);
  Notify("BB_MIX_INPUT_AGE",     out.input_age);
  Notify("BB_MIX_HARD_STOP",     out.hard_stop ? "true" : "false");
  Notify("BB_MIX_STOP_REASON",   bb::to_string(out.stop_reason));
  Notify("BB_MIX_SATURATION",    out.alloc.saturation_value);

  AppCastingMOOSApp::PostReport();
  return true;
}

//---------------------------------------------------------

bool ThrustMix::OnStartUp()
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
    if (param == "mixer_model") {
      m_cfg.mixer_model = value; handled = true;
    }
    else if (param == "selected_cmd_timeout_sec") {
      handled = setDoubleOnString(m_cfg.selected_cmd_timeout_sec, value);
    }
    else if (param == "slew_rate_pct_sec") {
      handled = setDoubleOnString(m_cfg.slew_rate_pct_sec, value);
    }
    else if (param == "slew_max_dt_sec") {
      handled = setDoubleOnString(m_cfg.slew_max_dt_sec, value);
    }
    else if (param == "slew_reset_on_handoff") {
      handled = setBooleanOnString(m_cfg.slew_reset_on_handoff, value);
    }
    else if (param == "thrust_asymmetry") {
      handled = setDoubleOnString(m_cfg.skid.thrust_asymmetry, value);
    }
    else if (param == "steering_throttle_mix") {
      handled = setDoubleOnString(m_cfg.skid.steering_throttle_mix, value);
    }
    else if (param == "selected_cmd_var") {
      m_selected_var = value; handled = true;
    }
    else if (param == "mixed_cmd_var") {
      m_mixed_var = value; handled = true;
    }

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  // Configuration validation FAILS STARTUP. A mixer with a
  // nonsensical asymmetry or an unnamed model should refuse to
  // run, not discover the problem with props in the water.
  const string err = m_cfg.validate();
  if (!err.empty()) {
    reportConfigWarning("FATAL: " + err);
    cout << termColor("red")
         << "pThrustMix: invalid configuration: " << err << endl
         << termColor();
    return false;
  }

  m_stage = new bb::MixerStage(m_cfg, make_epoch("mix"));

  registerVariables();
  return true;
}

void ThrustMix::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // ONE input. If this list ever grows, re-read the header.
  Register(m_selected_var, 0);
}

//---------------------------------------------------------

bool ThrustMix::buildReport()
{
  if (!m_stage) {
    m_msgs << "NOT CONFIGURED" << endl;
    return true;
  }

  m_msgs << "Model:      " << m_cfg.mixer_model << endl;
  m_msgs << "Mix epoch:  " << m_stage->epoch()
         << "   seq: " << m_stage->seq() << endl;
  m_msgs << "Slew:       " << doubleToString(m_cfg.slew_rate_pct_sec, 1)
         << " %/s   state " << doubleToString(m_stage->slew_state(), 2)
         << "   reset_on_handoff: "
         << (m_cfg.slew_reset_on_handoff ? "true" : "false") << endl;
  m_msgs << "Skid:       A=" << doubleToString(m_cfg.skid.thrust_asymmetry, 2)
         << "  m=" << doubleToString(m_cfg.skid.steering_throttle_mix, 2) << endl;
  m_msgs << endl;

  m_msgs << "Input (" << m_selected_var << ")" << endl;
  m_msgs << "  accepted " << m_selected.accepted_count()
         << "  dup " << m_selected.duplicate_count()
         << "  out-of-order " << m_selected.out_of_order_count()
         << "  rejected " << m_selected.reject_count() << endl;
  if (!m_selected.last_reject_reason().empty())
    m_msgs << "  last reject: " << m_selected.last_reject_reason() << endl;
  m_msgs << endl;

  if (!m_have_last) {
    m_msgs << "No cycle completed yet." << endl;
    return true;
  }

  ACTable actab(4);
  actab << "Field | In | Shaped | Out";
  actab.addHeaderLines();
  actab << "surge" << doubleToString(m_last.surge_in, 2)
        << doubleToString(m_last.surge_shaped, 2) << "";
  actab << "yaw"   << doubleToString(m_last.yaw_in, 2)
        << doubleToString(m_last.yaw_shaped, 2) << "";
  actab << "left"  << "" << "" << doubleToString(m_last.left_effort, 2);
  actab << "right" << "" << "" << doubleToString(m_last.right_effort, 2);
  m_msgs << actab.getFormattedString() << endl << endl;

  m_msgs << "Selected:   " << bb::to_string(m_last.selected)
         << (m_last.hard_stop ? "   HARD STOP: " : "   running (")
         << bb::to_string(m_last.stop_reason)
         << (m_last.hard_stop ? "" : ")") << endl;
  m_msgs << "Input age:  " << doubleToString(m_last.input_age, 3) << " s" << endl;
  m_msgs << "Saturation: q=" << doubleToString(m_last.alloc.saturation_value, 3)
         << "  f=" << doubleToString(m_last.alloc.fair_scaler, 3)
         << (m_last.alloc.saturated ? "  SATURATED" : "") << endl;
  m_msgs << "Limits:     "
         << (m_last.alloc.limit_steer_left     ? "steer_left "  : "")
         << (m_last.alloc.limit_steer_right    ? "steer_right " : "")
         << (m_last.alloc.limit_throttle_lower ? "thr_lower "   : "")
         << (m_last.alloc.limit_throttle_upper ? "thr_upper "   : "")
         << ((!m_last.alloc.limit_steer_left && !m_last.alloc.limit_steer_right &&
              !m_last.alloc.limit_throttle_lower && !m_last.alloc.limit_throttle_upper)
                 ? "none" : "") << endl;
  m_msgs << endl;

  m_msgs << "Lineage:    src " << (m_last.source_producer.empty() ? "-" : m_last.source_producer)
         << " " << (m_last.source_epoch.empty() ? "-" : m_last.source_epoch)
         << "/" << m_last.source_seq
         << "  ->  arb " << (m_last.decision_epoch.empty() ? "-" : m_last.decision_epoch)
         << "/" << m_last.decision_seq
         << "  ->  mix " << m_last.mix_epoch << "/" << m_last.mix_seq << endl;
  m_msgs << "Cycles:     " << m_iterations
         << "   stopped " << m_stop_cycles
         << " (" << doubleToString(m_iterations ? 100.0 * m_stop_cycles / m_iterations : 0.0, 1)
         << "%)" << endl;
  return true;
}
