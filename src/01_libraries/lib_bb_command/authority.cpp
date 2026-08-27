#include "authority.h"
#include "wire_format.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>

namespace bb {

const char* to_string(StopReason r)
{
  switch (r) {
    case StopReason::NONE:              return "NONE";
    case StopReason::STARTUP:           return "STARTUP";
    case StopReason::RC_INVALID:        return "RC_INVALID";
    case StopReason::RC_STALE:          return "RC_STALE";
    case StopReason::TELEOP_ESTOP:      return "TELEOP_ESTOP";
    case StopReason::TELEOP_INVALID:    return "TELEOP_INVALID";
    case StopReason::TELEOP_STALE:      return "TELEOP_STALE";
    case StopReason::AUTONOMY_ALL_STOP: return "AUTONOMY_ALL_STOP";
    case StopReason::AUTONOMY_INVALID:  return "AUTONOMY_INVALID";
    case StopReason::AUTONOMY_STALE:    return "AUTONOMY_STALE";
    case StopReason::MIXER_INPUT_STALE: return "MIXER_INPUT_STALE";
    case StopReason::MIXER_INPUT_INVALID: return "MIXER_INPUT_INVALID";
    case StopReason::NAV_INPUT_STALE:   return "NAV_INPUT_STALE";
    case StopReason::NAV_INPUT_INVALID: return "NAV_INPUT_INVALID";
    case StopReason::RC_KILL:           return "RC_KILL";
    case StopReason::RC_DEADMAN:        return "RC_DEADMAN";
    case StopReason::PWM_DISARMED:      return "PWM_DISARMED";
    case StopReason::HARDWARE_FAULT:    return "HARDWARE_FAULT";
    case StopReason::SHUTDOWN:          return "SHUTDOWN";
    case StopReason::INTERNAL_FAULT:    return "INTERNAL_FAULT";
  }
  return "INTERNAL_FAULT";
}

std::string ArbiterConfig::validate() const
{
  if (!std::isfinite(rc_timeout_sec) || rc_timeout_sec <= 0.0)
    return "rc_timeout_sec must be finite and > 0";
  if (!std::isfinite(teleop_timeout_sec) || teleop_timeout_sec <= 0.0)
    return "teleop_timeout_sec must be finite and > 0";
  if (!std::isfinite(autonomy_timeout_sec) || autonomy_timeout_sec <= 0.0)
    return "autonomy_timeout_sec must be finite and > 0";
  return "";
}

namespace {

SourceEvaluation evaluate(const CommandMailbox& mb, double now, double timeout,
                          bool requesting)
{
  SourceEvaluation e;
  e.source              = mb.source();
  e.ever_seen           = mb.has_command();
  e.age                 = mb.age(now);
  e.fresh               = mb.fresh(now, timeout);
  e.valid               = mb.has_command() && mb.snapshot().valid;
  e.requesting_authority = requesting;
  e.eligible            = e.fresh && e.valid && requesting;

  if (!e.ever_seen)            e.reason = "never_produced";
  else if (!requesting)        e.reason = "not_requesting";
  else if (!e.valid)           e.reason = "invalid";
  else if (!e.fresh)           e.reason = "stale";
  else                         e.reason = "eligible";
  return e;
}

void copy_lineage(AuthorityDecision& d, const SemanticCommand& c)
{
  d.source_producer = c.producer;
  d.source_epoch    = c.epoch;
  d.source_seq      = c.seq;
}

// Manual authority cap. Policy, not mixing and not ESC
// calibration -- it belongs to the arbiter because it is a
// statement about how much of the boat this operator is allowed,
// which is an authority question.
void apply_authority_limit(AuthorityDecision& d, const SemanticCommand& c)
{
  double k = c.authority_limit / 100.0;
  if (!std::isfinite(k)) k = 0.0;
  if (k < 0.0) k = 0.0;
  if (k > 1.0) k = 1.0;
  d.surge = c.surge * k;
  d.yaw   = c.yaw   * k;
}

} // namespace

AuthorityArbiter::AuthorityArbiter(const ArbiterConfig& cfg,
                                   const std::string& decision_epoch)
  : m_cfg(cfg),
    m_epoch(decision_epoch),
    m_seq(0),
    m_previous_source(CommandSource::NONE),
    m_previous_stop_reason(StopReason::STARTUP)
{
}

AuthorityDecision AuthorityArbiter::decide(double now,
                                           const CommandMailbox& rc,
                                           const CommandMailbox& teleop,
                                           const CommandMailbox& autonomy,
                                           const SafetyInputs& safety)
{
  AuthorityDecision d;
  d.decision_epoch  = m_epoch;
  d.decision_seq    = ++m_seq;
  d.decision_time   = now;
  d.previous_source = m_previous_source;
  d.rc_kill_observed = safety.rc_kill_asserted;

  // --- What is each source asking for? ---------------------
  //
  // RC requests authority by the guarded mode switch, NOT by
  // moving the sticks. Nonzero sticks in AUTO mode mean nothing;
  // that separation is what stops a knocked stick from stealing
  // the boat mid-mission.
  //
  // The last known mode is used even when RC has gone stale.
  // iRCInterface retains the last valid guarded mode on link loss
  // rather than synthesising NON_MANUAL (design doc 5.1), which
  // is what makes the fail-closed branch below reachable: an
  // operator who was driving and lost the link still "holds" the
  // boat, and we stop rather than hand it to autonomy.
  const bool rc_manual =
      rc.has_command() && rc.snapshot().field("mode") == "MANUAL";

  // Teleop requests authority by an explicit claim. Command
  // presence alone is not a claim (invariant 3).
  const bool teleop_claim =
      teleop.has_command() && teleop.snapshot().field_bool("claim", false);

  // Autonomy is the residual: it never "requests", it takes the
  // boat when no manual source wants it and it is allowed to.
  const bool autonomy_requesting = !rc_manual && !teleop_claim;

  d.rc       = evaluate(rc,       now, m_cfg.rc_timeout_sec,       rc_manual);
  d.teleop   = evaluate(teleop,   now, m_cfg.teleop_timeout_sec,   teleop_claim);
  d.autonomy = evaluate(autonomy, now, m_cfg.autonomy_timeout_sec, autonomy_requesting);

  // --- RC: highest priority ---------------------------------
  if (rc_manual) {
    if (d.rc.eligible) {
      d.selected_source = CommandSource::RC;
      d.hard_stop = false;
      d.stop_reason = StopReason::NONE;
      copy_lineage(d, rc.snapshot());
      apply_authority_limit(d, rc.snapshot());
    } else {
      // Invariant 6. The operator is holding the boat and cannot
      // command it; that is a stop, not a handover.
      d.hard_stop = true;
      d.fail_closed = true;
      d.stop_reason = d.rc.valid ? StopReason::RC_STALE : StopReason::RC_INVALID;
      copy_lineage(d, rc.snapshot());
    }
  }
  // --- Teleop -----------------------------------------------
  else if (teleop_claim) {
    const bool estop = teleop.snapshot().field_bool("estop", false);
    if (estop) {
      d.hard_stop = true;
      d.stop_reason = StopReason::TELEOP_ESTOP;
      copy_lineage(d, teleop.snapshot());
    } else if (d.teleop.eligible) {
      d.selected_source = CommandSource::TELEOP;
      d.hard_stop = false;
      d.stop_reason = StopReason::NONE;
      copy_lineage(d, teleop.snapshot());
      apply_authority_limit(d, teleop.snapshot());
    } else {
      d.hard_stop = true;
      d.fail_closed = true;
      d.stop_reason = d.teleop.valid ? StopReason::TELEOP_STALE
                                     : StopReason::TELEOP_INVALID;
      copy_lineage(d, teleop.snapshot());
    }
  }
  // --- Autonomy ---------------------------------------------
  else {
    // ALL_STOP gates autonomy and nothing else. It must not be
    // able to paralyse a manual rescue, which is why it is tested
    // here and not at the top.
    if (safety.autonomy_all_stop) {
      d.hard_stop = true;
      d.stop_reason = StopReason::AUTONOMY_ALL_STOP;
    }
    // Decision (e): the pre-first-frame window. A boat that has
    // never heard a handset is not "in AUTO"; it simply has no RC.
    // Refusing to run would strand every deployment without a
    // link, so this is allowed by default and configurable.
    else if (!rc.has_command() && !m_cfg.allow_autonomy_before_first_rc) {
      d.hard_stop = true;
      d.stop_reason = StopReason::STARTUP;
    }
    else if (d.autonomy.eligible) {
      d.selected_source = CommandSource::AUTONOMY;
      d.hard_stop = false;
      d.stop_reason = StopReason::NONE;
      copy_lineage(d, autonomy.snapshot());
      // Autonomy carries no authority cap: the cap is an operator
      // affordance for manual driving. A mission is either
      // permitted to run or it is not.
      d.surge = autonomy.snapshot().surge;
      d.yaw   = autonomy.snapshot().yaw;
    }
    else {
      d.hard_stop = true;
      d.stop_reason = d.autonomy.valid ? StopReason::AUTONOMY_STALE
                                       : StopReason::AUTONOMY_INVALID;
      copy_lineage(d, autonomy.snapshot());
    }
  }

  if (d.hard_stop) { d.surge = 0.0; d.yaw = 0.0; }

  // Non-finite must never leave this function, whatever a source
  // claimed (invariant 12). The parser should have caught it; this
  // is the belt to that braces.
  if (!std::isfinite(d.surge) || !std::isfinite(d.yaw)) {
    d.surge = 0.0; d.yaw = 0.0;
    d.hard_stop = true;
    d.stop_reason = StopReason::INTERNAL_FAULT;
    d.selected_source = CommandSource::NONE;
  }

  d.authority_changed   = (d.selected_source != m_previous_source);
  d.stop_reason_changed = (d.stop_reason != m_previous_stop_reason);

  m_previous_source      = d.selected_source;
  m_previous_stop_reason = d.stop_reason;
  return d;
}

std::string serialize_decision(const AuthorityDecision& d)
{
  return wire::formatf(
      "v=1,decision_epoch=%s,decision_seq=%llu,decision_time=%.3f,"
      "selected=%s,hard_stop=%d,stop=%s,"
      "source_producer=%s,source_epoch=%s,source_seq=%llu,"
      "surge=%.3f,yaw=%.3f",
      d.decision_epoch.empty() ? "-" : d.decision_epoch.c_str(),
      (unsigned long long)d.decision_seq,
      d.decision_time,
      to_string(d.selected_source),
      d.hard_stop ? 1 : 0,
      to_string(d.stop_reason),
      d.source_producer.empty() ? "-" : d.source_producer.c_str(),
      d.source_epoch.empty() ? "-" : d.source_epoch.c_str(),
      (unsigned long long)d.source_seq,
      d.surge, d.yaw);
}



// =========================================================
//  Consumer side: parsing BB_SELECTED_CMD and leasing it.
// =========================================================

namespace {

const double kDecisionNeverAge = 1.0e9;

// The shared wire dialect (wire_format.h). The private copies this
// replaced had already drifted from the envelope parser's
// semantics -- see the header there.
using wire::parse_double;
using wire::parse_u64;

} // namespace

CommandSource source_from_string(const std::string& s, bool& ok)
{
  ok = true;
  if (s == "RC")            return CommandSource::RC;
  if (s == "TELEOP")        return CommandSource::TELEOP;
  if (s == "AUTONOMY")      return CommandSource::AUTONOMY;
  if (s == "LEGACY_DIRECT") return CommandSource::LEGACY_DIRECT;
  if (s == "NONE")          return CommandSource::NONE;
  ok = false;
  return CommandSource::NONE;
}

StopReason stop_from_string(const std::string& s, bool& ok)
{
  ok = true;
  if (s == "NONE")              return StopReason::NONE;
  if (s == "STARTUP")           return StopReason::STARTUP;
  if (s == "RC_INVALID")        return StopReason::RC_INVALID;
  if (s == "RC_STALE")          return StopReason::RC_STALE;
  if (s == "TELEOP_ESTOP")      return StopReason::TELEOP_ESTOP;
  if (s == "TELEOP_INVALID")    return StopReason::TELEOP_INVALID;
  if (s == "TELEOP_STALE")      return StopReason::TELEOP_STALE;
  if (s == "AUTONOMY_ALL_STOP") return StopReason::AUTONOMY_ALL_STOP;
  if (s == "AUTONOMY_INVALID")  return StopReason::AUTONOMY_INVALID;
  if (s == "AUTONOMY_STALE")    return StopReason::AUTONOMY_STALE;
  if (s == "MIXER_INPUT_STALE") return StopReason::MIXER_INPUT_STALE;
  if (s == "MIXER_INPUT_INVALID") return StopReason::MIXER_INPUT_INVALID;
  if (s == "NAV_INPUT_STALE")   return StopReason::NAV_INPUT_STALE;
  if (s == "NAV_INPUT_INVALID") return StopReason::NAV_INPUT_INVALID;
  if (s == "RC_KILL")           return StopReason::RC_KILL;
  if (s == "RC_DEADMAN")        return StopReason::RC_DEADMAN;
  if (s == "PWM_DISARMED")      return StopReason::PWM_DISARMED;
  if (s == "HARDWARE_FAULT")    return StopReason::HARDWARE_FAULT;
  if (s == "SHUTDOWN")          return StopReason::SHUTDOWN;
  if (s == "INTERNAL_FAULT")    return StopReason::INTERNAL_FAULT;
  ok = false;
  return StopReason::INTERNAL_FAULT;
}

namespace {

DecisionParseResult dfail(const char* reason, const std::string& detail)
{
  DecisionParseResult r;
  r.ok = false;
  r.reject_reason = reason;
  r.detail = detail;
  return r;
}

} // namespace

DecisionParseResult parse_decision(const std::string& text)
{
  if (text.empty()) return dfail(reject::kMalformed, "empty");

  std::map<std::string, std::string> kv;
  const wire::SplitResult sr = wire::kv_split(text, kv);
  // A duplicate key and a malformed token are different faults and
  // get different tokens, because they point at different bugs
  // upstream. (The old inference from kv.count(bad) mislabelled a
  // malformed token whose text matched an existing key.)
  if (sr.fault == wire::SplitFault::DUPLICATE_KEY)
    return dfail(reject::kDuplicateKey, sr.detail);
  if (sr.fault != wire::SplitFault::NONE)
    return dfail(reject::kMalformed, sr.detail);

  std::map<std::string, std::string>::iterator it = kv.find("v");
  if (it == kv.end()) return dfail(reject::kMissingField, "v");
  uint64_t ver = 0;
  if (!parse_u64(it->second, ver)) return dfail(reject::kMalformed, "v");
  if (ver != kCommandContractVersion)
    return dfail(reject::kUnsupportedVersion, it->second);

  DecisionSnapshot d;

  const char* req[] = {"decision_epoch", "decision_seq", "decision_time",
                       "selected", "hard_stop", "stop", "surge", "yaw"};
  for (size_t i = 0; i < sizeof(req)/sizeof(req[0]); ++i)
    if (!kv.count(req[i])) return dfail(reject::kMissingField, req[i]);

  d.decision_epoch = kv["decision_epoch"];
  if (d.decision_epoch.empty()) return dfail(reject::kMissingField, "decision_epoch");

  if (!parse_u64(kv["decision_seq"], d.decision_seq))
    return dfail(reject::kMalformed, "decision_seq");
  if (!parse_double(kv["decision_time"], d.decision_time))
    return dfail(reject::kNonFinite, "decision_time");

  bool ok = false;
  d.selected = source_from_string(kv["selected"], ok);
  if (!ok) return dfail(reject::kInvalidFlag, "selected");
  d.stop_reason = stop_from_string(kv["stop"], ok);
  if (!ok) return dfail(reject::kInvalidFlag, "stop");

  if      (kv["hard_stop"] == "1") d.hard_stop = true;
  else if (kv["hard_stop"] == "0") d.hard_stop = false;
  else return dfail(reject::kInvalidFlag, "hard_stop");

  if (!parse_double(kv["surge"], d.surge)) return dfail(reject::kNonFinite, "surge");
  if (!parse_double(kv["yaw"],   d.yaw))   return dfail(reject::kNonFinite, "yaw");
  if (d.surge < -100.0 || d.surge > 100.0) return dfail(reject::kOutOfRange, "surge");
  if (d.yaw   < -100.0 || d.yaw   > 100.0) return dfail(reject::kOutOfRange, "yaw");

  // A frame that says it stopped but carries thrust is
  // self-contradictory. Reject rather than guess which half to
  // believe.
  if (d.hard_stop && (d.surge != 0.0 || d.yaw != 0.0))
    return dfail(reject::kOutOfRange, "hard_stop with nonzero command");

  // Lineage is optional in the sense that a stop cycle may have
  // none, but if present it must parse.
  if (kv.count("source_producer")) d.source_producer = kv["source_producer"];
  if (kv.count("source_epoch"))    d.source_epoch    = kv["source_epoch"];
  if (kv.count("source_seq") && !parse_u64(kv["source_seq"], d.source_seq))
    return dfail(reject::kMalformed, "source_seq");

  DecisionParseResult r;
  r.ok = true;
  r.decision = d;
  return r;
}

DecisionMailbox::DecisionMailbox()
  : m_has(false), m_accepted(0), m_duplicates(0),
    m_out_of_order(0), m_rejects(0)
{
}

AcceptResult DecisionMailbox::accept(const std::string& text, double arrival_time)
{
  const DecisionParseResult pr = parse_decision(text);
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

  DecisionSnapshot in = pr.decision;

  if (m_has && in.decision_epoch == m_decision.decision_epoch) {
    if (in.decision_seq == m_decision.decision_seq) {
      ++m_duplicates;
      return AcceptResult::DUPLICATE;
    }
    if (in.decision_seq < m_decision.decision_seq) {
      ++m_out_of_order;
      return AcceptResult::OUT_OF_ORDER;
    }
  }

  in.rx_time = arrival_time;
  m_decision = in;
  m_has = true;
  ++m_accepted;
  return AcceptResult::ACCEPTED;
}

double DecisionMailbox::age(double now) const
{
  if (!m_has || !std::isfinite(now)) return kDecisionNeverAge;
  const double a = now - m_decision.rx_time;
  return (a < 0.0) ? 0.0 : a;
}

bool DecisionMailbox::fresh(double now, double timeout_sec) const
{
  if (!m_has) return false;
  if (!std::isfinite(timeout_sec) || timeout_sec <= 0.0) return false;
  return age(now) <= timeout_sec;
}

} // namespace bb
