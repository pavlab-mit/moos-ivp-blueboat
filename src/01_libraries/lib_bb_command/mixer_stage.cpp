#include "mixer_stage.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>

namespace bb {

std::string MixerConfig::validate() const
{
  if (mixer_model.empty())
    return "mixer_model is required and has no default -- name the equation "
           "that produces the motor pair";
  if (!std::isfinite(selected_cmd_timeout_sec) || selected_cmd_timeout_sec <= 0.0)
    return "selected_cmd_timeout_sec must be finite and > 0";
  if (!std::isfinite(slew_rate_pct_sec) || slew_rate_pct_sec < 0.0)
    return "slew_rate_pct_sec must be finite and >= 0";
  if (!std::isfinite(slew_max_dt_sec) || slew_max_dt_sec <= 0.0)
    return "slew_max_dt_sec must be finite and > 0";
  return skid.validate();
}

MixerStage::MixerStage(const MixerConfig& cfg, const std::string& mix_epoch)
  : m_cfg(cfg),
    m_epoch(mix_epoch),
    m_seq(0),
    m_slew(cfg.slew_rate_pct_sec, cfg.slew_max_dt_sec),
    m_previous_selected(CommandSource::NONE)
{
}

MixedCommand MixerStage::update(double now, double dt,
                                const DecisionMailbox& selected)
{
  MixedCommand m;
  m.mix_epoch   = m_epoch;
  m.mix_seq     = ++m_seq;
  m.mix_time    = now;
  m.mixer_model = m_cfg.mixer_model;
  m.input_age   = selected.age(now);

  const bool have  = selected.has_decision();
  const bool fresh = selected.fresh(now, m_cfg.selected_cmd_timeout_sec);

  // --- Input failure, or an upstream stop -------------------
  //
  // All three collapse to the same actuator outcome, but they get
  // DIFFERENT stop reasons, because they point at different
  // faults: no arbiter at all, an arbiter that has gone quiet, or
  // an arbiter that deliberately stopped. A log that cannot tell
  // those apart cannot be debugged.
  if (!have || !fresh || selected.snapshot().hard_stop) {
    m.hard_stop = true;

    if (!have)       m.stop_reason = StopReason::MIXER_INPUT_INVALID;
    else if (!fresh) m.stop_reason = StopReason::MIXER_INPUT_STALE;
    else             m.stop_reason = selected.snapshot().stop_reason;

    // Invariant 7: bypass the limiter and reset it, so resuming
    // starts from rest rather than from wherever the ramp had got
    // to when the fault hit.
    m_slew.reset(0.0);

    if (have) {
      const DecisionSnapshot& d = selected.snapshot();
      m.decision_epoch  = d.decision_epoch;
      m.decision_seq    = d.decision_seq;
      m.selected        = d.selected;
      m.source_producer = d.source_producer;
      m.source_epoch    = d.source_epoch;
      m.source_seq      = d.source_seq;
    }

    m_previous_selected = CommandSource::NONE;
    return m;   // left/right stay at zero
  }

  // --- Normal path ------------------------------------------
  const DecisionSnapshot& d = selected.snapshot();

  m.decision_epoch  = d.decision_epoch;
  m.decision_seq    = d.decision_seq;
  m.selected        = d.selected;
  m.source_producer = d.source_producer;
  m.source_epoch    = d.source_epoch;
  m.source_seq      = d.source_seq;
  m.hard_stop       = false;
  m.stop_reason     = StopReason::NONE;
  m.surge_in        = d.surge;
  m.yaw_in          = d.yaw;

  // Plan decision (c). A handoff is a change of who is driving,
  // not a fault, so by default the common-throttle state carries
  // and the new owner picks up from the current speed.
  m.handoff = (m_previous_selected != CommandSource::NONE) &&
              (d.selected != m_previous_selected);
  if (m.handoff && m_cfg.slew_reset_on_handoff)
    m_slew.reset(0.0);
  m_previous_selected = d.selected;

  m.surge_shaped = m_slew.update(d.surge, dt);
  m.slew_limited = m_slew.limited();

  // Yaw is NOT slewed. See the header.
  m.yaw_shaped = d.yaw;

  m.alloc        = skid_mix(m.surge_shaped, m.yaw_shaped, m_cfg.skid);
  m.left_effort  = m.alloc.left_effort;
  m.right_effort = m.alloc.right_effort;

  return m;
}

std::string serialize_mixed(const MixedCommand& m)
{
  char buf[768];
  std::snprintf(buf, sizeof(buf),
                "v=1,mixer_model=%s,mix_epoch=%s,mix_seq=%llu,mix_time=%.3f,"
                "decision_epoch=%s,decision_seq=%llu,"
                "selected=%s,hard_stop=%d,stop=%s,"
                "source_producer=%s,source_epoch=%s,source_seq=%llu,"
                "surge_in=%.3f,yaw_in=%.3f,"
                "surge_shaped=%.3f,yaw_shaped=%.3f,"
                "left_effort=%.3f,right_effort=%.3f,"
                "q=%.4f,f=%.4f,limits=%d%d%d%d",
                m.mixer_model.c_str(),
                m.mix_epoch.c_str(),
                (unsigned long long)m.mix_seq,
                m.mix_time,
                m.decision_epoch.empty() ? "-" : m.decision_epoch.c_str(),
                (unsigned long long)m.decision_seq,
                to_string(m.selected),
                m.hard_stop ? 1 : 0,
                to_string(m.stop_reason),
                m.source_producer.empty() ? "-" : m.source_producer.c_str(),
                m.source_epoch.empty() ? "-" : m.source_epoch.c_str(),
                (unsigned long long)m.source_seq,
                m.surge_in, m.yaw_in,
                m.surge_shaped, m.yaw_shaped,
                m.left_effort, m.right_effort,
                m.alloc.saturation_value, m.alloc.fair_scaler,
                m.alloc.limit_steer_left ? 1 : 0,
                m.alloc.limit_steer_right ? 1 : 0,
                m.alloc.limit_throttle_lower ? 1 : 0,
                m.alloc.limit_throttle_upper ? 1 : 0);
  return std::string(buf);
}



// =========================================================
//  Consumer side: parsing BB_MIXED_CMD and leasing it.
// =========================================================

namespace {

const double kMixedNeverAge = 1.0e9;

bool mx_split(const std::string& text,
              std::map<std::string, std::string>& out, std::string& bad)
{
  size_t pos = 0;
  while (pos <= text.size()) {
    const size_t comma = text.find(',', pos);
    const std::string tok = (comma == std::string::npos)
                                ? text.substr(pos)
                                : text.substr(pos, comma - pos);
    if (tok.empty()) { bad = "empty token"; return false; }
    const size_t eq = tok.find('=');
    if (eq == std::string::npos || eq == 0) { bad = tok; return false; }
    const std::string key = tok.substr(0, eq);
    if (out.count(key)) { bad = key; return false; }
    out[key] = tok.substr(eq + 1);
    if (comma == std::string::npos) break;
    pos = comma + 1;
  }
  return true;
}

bool mx_double(const std::string& s, double& out)
{
  if (s.empty()) return false;
  const char* b = s.c_str();
  char* e = nullptr;
  const double v = std::strtod(b, &e);
  if (e == b || *e != '\0' || !std::isfinite(v)) return false;
  out = v;
  return true;
}

bool mx_u64(const std::string& s, uint64_t& out)
{
  if (s.empty()) return false;
  for (size_t i = 0; i < s.size(); ++i)
    if (s[i] < '0' || s[i] > '9') return false;
  out = std::strtoull(s.c_str(), nullptr, 10);
  return true;
}

MixedParseResult mxfail(const char* reason, const std::string& detail)
{
  MixedParseResult r;
  r.ok = false;
  r.reject_reason = reason;
  r.detail = detail;
  return r;
}

} // namespace

MixedParseResult parse_mixed(const std::string& text)
{
  if (text.empty()) return mxfail(reject::kMalformed, "empty");

  std::map<std::string, std::string> kv;
  std::string bad;
  if (!mx_split(text, kv, bad))
    return kv.count(bad) ? mxfail(reject::kDuplicateKey, bad)
                         : mxfail(reject::kMalformed, bad);

  std::map<std::string, std::string>::iterator it = kv.find("v");
  if (it == kv.end()) return mxfail(reject::kMissingField, "v");
  uint64_t ver = 0;
  if (!mx_u64(it->second, ver)) return mxfail(reject::kMalformed, "v");
  if (ver != kCommandContractVersion)
    return mxfail(reject::kUnsupportedVersion, it->second);

  const char* req[] = {"mixer_model", "mix_epoch", "mix_seq", "mix_time",
                       "hard_stop", "stop", "left_effort", "right_effort",
                       "selected"};
  for (size_t i = 0; i < sizeof(req)/sizeof(req[0]); ++i)
    if (!kv.count(req[i])) return mxfail(reject::kMissingField, req[i]);

  MixedSnapshot m;
  m.mixer_model = kv["mixer_model"];
  m.mix_epoch   = kv["mix_epoch"];
  if (m.mix_epoch.empty()) return mxfail(reject::kMissingField, "mix_epoch");

  if (!mx_u64(kv["mix_seq"], m.mix_seq))
    return mxfail(reject::kMalformed, "mix_seq");
  if (!mx_double(kv["mix_time"], m.mix_time))
    return mxfail(reject::kNonFinite, "mix_time");

  bool ok = false;
  m.selected = source_from_string(kv["selected"], ok);
  if (!ok) return mxfail(reject::kInvalidFlag, "selected");
  m.stop_reason = stop_from_string(kv["stop"], ok);
  if (!ok) return mxfail(reject::kInvalidFlag, "stop");

  if      (kv["hard_stop"] == "1") m.hard_stop = true;
  else if (kv["hard_stop"] == "0") m.hard_stop = false;
  else return mxfail(reject::kInvalidFlag, "hard_stop");

  if (!mx_double(kv["left_effort"],  m.left_effort))
    return mxfail(reject::kNonFinite, "left_effort");
  if (!mx_double(kv["right_effort"], m.right_effort))
    return mxfail(reject::kNonFinite, "right_effort");
  if (m.left_effort  < -100.0 || m.left_effort  > 100.0)
    return mxfail(reject::kOutOfRange, "left_effort");
  if (m.right_effort < -100.0 || m.right_effort > 100.0)
    return mxfail(reject::kOutOfRange, "right_effort");

  // A frame claiming a stop while carrying effort is
  // self-contradictory. Refuse rather than choose a half to trust
  // -- this is the last parse before the water.
  if (m.hard_stop && (m.left_effort != 0.0 || m.right_effort != 0.0))
    return mxfail(reject::kOutOfRange, "hard_stop with nonzero effort");

  if (kv.count("decision_epoch")) m.decision_epoch = kv["decision_epoch"];
  if (kv.count("decision_seq") && !mx_u64(kv["decision_seq"], m.decision_seq))
    return mxfail(reject::kMalformed, "decision_seq");
  if (kv.count("source_producer")) m.source_producer = kv["source_producer"];
  if (kv.count("source_epoch"))    m.source_epoch    = kv["source_epoch"];
  if (kv.count("source_seq") && !mx_u64(kv["source_seq"], m.source_seq))
    return mxfail(reject::kMalformed, "source_seq");

  MixedParseResult r;
  r.ok = true;
  r.mixed = m;
  return r;
}

MixedMailbox::MixedMailbox()
  : m_has(false), m_accepted(0), m_duplicates(0),
    m_out_of_order(0), m_rejects(0)
{
}

AcceptResult MixedMailbox::accept(const std::string& text, double arrival_time)
{
  const MixedParseResult pr = parse_mixed(text);
  if (!pr.ok) {
    ++m_rejects;
    m_last_reject_reason = pr.reject_reason;
    return AcceptResult::REJECTED;
  }
  if (!std::isfinite(arrival_time)) {
    ++m_rejects;
    m_last_reject_reason = reject::kNonFinite;
    return AcceptResult::REJECTED;
  }
  m_last_reject_reason.clear();

  MixedSnapshot in = pr.mixed;

  if (m_has && in.mix_epoch == m_mixed.mix_epoch) {
    if (in.mix_seq == m_mixed.mix_seq) {
      ++m_duplicates;
      return AcceptResult::DUPLICATE;
    }
    if (in.mix_seq < m_mixed.mix_seq) {
      ++m_out_of_order;
      return AcceptResult::OUT_OF_ORDER;
    }
  }

  in.rx_time = arrival_time;
  m_mixed = in;
  m_has = true;
  ++m_accepted;
  return AcceptResult::ACCEPTED;
}

double MixedMailbox::age(double now) const
{
  if (!m_has || !std::isfinite(now)) return kMixedNeverAge;
  const double a = now - m_mixed.rx_time;
  return (a < 0.0) ? 0.0 : a;
}

bool MixedMailbox::fresh(double now, double timeout_sec) const
{
  if (!m_has) return false;
  if (!std::isfinite(timeout_sec) || timeout_sec <= 0.0) return false;
  return age(now) <= timeout_sec;
}

} // namespace bb
