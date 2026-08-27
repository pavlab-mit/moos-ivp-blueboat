/*************************************************************
 *  authority -- who is allowed to drive the boat, and why.
 *
 *  Pure decision logic, lifted out of pBBCommandArbiter so it can
 *  be exhaustively tested without MOOS, a clock, or a boat. The
 *  app around it does mail and publication; every rule that
 *  matters lives here.
 *
 *  The governing idea is that COMMAND IS NOT AUTHORITY
 *  (invariant 3). A source publishing a perfectly valid command
 *  does not thereby get to drive. It must also be REQUESTING
 *  authority in its own contractual way -- RC by holding the mode
 *  switch in MANUAL, teleop by asserting a claim, autonomy by
 *  being the last resort when neither manual source wants it.
 *
 *  And the second idea: NO SILENT FALLTHROUGH (invariant 6). If a
 *  manual source holds authority and goes stale or invalid, the
 *  boat STOPS. It does not quietly hand control to autonomy. An
 *  operator whose link just died is not expressing a preference
 *  for autonomous operation.
 *
 *  DIVERGENCES FROM design doc section 6, all from decisions
 *  recorded in docs/control_refactor_plan.md section 12:
 *
 *   - RC kill does NOT gate here. Per decision (b) it is enforced
 *     at the Navigator, on the hardware side, where it survives a
 *     failure of this code. The arbiter observes it for the trace
 *     only, so the log explains a standstill. The design doc's
 *     pseudocode hard-stops on kill; that would put the safety
 *     property behind the software it is meant to outlive.
 *
 *   - There is no UNKNOWN mode branch. Per decision (e) CH6 is a
 *     two-position switch, so with the link up the mode is
 *     binary. What remains is the window BEFORE the first valid
 *     frame, and per decision (d) that must not inhibit autonomy:
 *     a boat launched with the handset off has to be able to run.
 *     `allow_autonomy_before_first_rc` covers exactly that window
 *     and nothing else.
 *
 *   - The RC deadman is absent. Per decision (d) it is a separate
 *     Navigator-enforced blanket, not an arbitration rule. The
 *     SOURCE LEASE lives here and is never disableable; the
 *     deadman is opt-in and lives there. Conflating the two is
 *     what made the earlier contract self-contradictory.
 *
 *  Author: Jeremy Wenger
 *************************************************************/

#ifndef BB_AUTHORITY_HEADER
#define BB_AUTHORITY_HEADER

#include "command_envelope.h"
#include "command_mailbox.h"

#include <cstdint>
#include <string>

namespace bb {

// Stable tokens; these reach logs and tests, so do not renumber
// or rename casually.
enum class StopReason {
  NONE,
  STARTUP,
  RC_INVALID,
  RC_STALE,
  TELEOP_ESTOP,
  TELEOP_INVALID,
  TELEOP_STALE,
  AUTONOMY_ALL_STOP,
  AUTONOMY_INVALID,
  AUTONOMY_STALE,
  MIXER_INPUT_STALE,
  MIXER_INPUT_INVALID,
  INTERNAL_FAULT
};

const char* to_string(StopReason r);

struct ArbiterConfig
{
  // Per decision (7): start lenient, tighten only once the
  // measured age distributions justify it. These defaults are
  // deliberately generous; they are not the final values and the
  // arbiter publishes the ages so the numbers can be chosen from
  // evidence rather than taste.
  double rc_timeout_sec       = 1.0;
  double teleop_timeout_sec   = 1.5;
  double autonomy_timeout_sec = 2.0;

  // Decision (e): the pre-first-frame window only. Once ANY valid
  // RC frame has been seen, the mode switch is authoritative and
  // this flag is irrelevant. Setting it false makes a boat refuse
  // to run autonomously until it has heard from a handset at
  // least once.
  bool allow_autonomy_before_first_rc = true;

  std::string validate() const;
};

// Safety signals are not command sources (design doc 5.4).
struct SafetyInputs
{
  bool autonomy_all_stop = false;

  // Observed for the trace only -- see the divergence note above.
  // The Navigator enforces this.
  bool rc_kill_asserted = false;
};

struct SourceEvaluation
{
  CommandSource source = CommandSource::NONE;
  bool   ever_seen           = false;
  bool   requesting_authority = false;
  bool   fresh               = false;
  bool   valid               = false;
  bool   eligible            = false;   // fresh && valid && requesting
  double age                 = 0.0;
  const char* reason         = "";      // why not eligible, for the trace
};

struct AuthorityDecision
{
  std::string   decision_epoch;
  uint64_t      decision_seq  = 0;
  double        decision_time = 0.0;

  CommandSource previous_source = CommandSource::NONE;
  CommandSource selected_source = CommandSource::NONE;

  bool          hard_stop   = true;
  StopReason    stop_reason = StopReason::STARTUP;

  // What the mixer should act on. Zero whenever hard_stop is set,
  // and scaled by the owning source's authority_limit otherwise.
  double surge = 0.0;
  double yaw   = 0.0;

  // Lineage: copied verbatim so the chain resolves backwards.
  std::string source_producer;
  std::string source_epoch;
  uint64_t    source_seq = 0;

  SourceEvaluation rc;
  SourceEvaluation teleop;
  SourceEvaluation autonomy;

  bool rc_kill_observed = false;   // trace only
  bool authority_changed = false;  // selected_source != previous_source
  bool stop_reason_changed = false;

  // True when a manual owner lost the boat and we STOPPED rather
  // than falling through. Called out separately because it is the
  // single most important thing a log can tell you after an
  // incident.
  bool fail_closed = false;
};

class AuthorityArbiter
{
 public:
  AuthorityArbiter(const ArbiterConfig& cfg, const std::string& decision_epoch);

  // One control cycle. Pure with respect to its inputs apart from
  // the sequence counter and the previous-source memory.
  AuthorityDecision decide(double now,
                           const CommandMailbox& rc,
                           const CommandMailbox& teleop,
                           const CommandMailbox& autonomy,
                           const SafetyInputs& safety);

  const std::string& epoch() const { return m_epoch; }
  uint64_t           seq()   const { return m_seq; }

  const ArbiterConfig& config() const { return m_cfg; }

 private:
  ArbiterConfig m_cfg;
  std::string   m_epoch;
  uint64_t      m_seq;
  CommandSource m_previous_source;
  StopReason    m_previous_stop_reason;
};

// Serialise a decision as BB_SELECTED_CMD (design doc 6.1).
std::string serialize_decision(const AuthorityDecision& d);

// ---------------------------------------------------------
// Consumer side.
//
// pThrustMix does not need the arbiter's full evaluation record --
// only what it must act on and what it must copy onward. Keeping
// the consumer view narrow is deliberate: a mixer that cannot see
// the per-source eligibility record cannot accidentally start
// making authority decisions of its own.

struct DecisionSnapshot
{
  std::string   decision_epoch;
  uint64_t      decision_seq = 0;
  double        decision_time = 0.0;
  CommandSource selected = CommandSource::NONE;
  bool          hard_stop = true;
  StopReason    stop_reason = StopReason::STARTUP;
  std::string   source_producer;
  std::string   source_epoch;
  uint64_t      source_seq = 0;
  double        surge = 0.0;
  double        yaw = 0.0;
  double        rx_time = 0.0;   // local arrival, drives freshness
};

struct DecisionParseResult
{
  bool             ok = false;
  std::string      reject_reason;
  std::string      detail;
  DecisionSnapshot decision;
};

DecisionParseResult parse_decision(const std::string& text);

// Same lease discipline as CommandMailbox, keyed on
// (decision_epoch, decision_seq). An arbiter restart changes the
// epoch and rebases the sequence; a repeat does not refresh
// freshness.
class DecisionMailbox
{
 public:
  DecisionMailbox();

  AcceptResult accept(const std::string& text, double arrival_time);

  bool has_decision() const { return m_has; }
  const DecisionSnapshot& snapshot() const { return m_decision; }
  double age(double now) const;
  bool   fresh(double now, double timeout_sec) const;

  uint64_t accepted_count()     const { return m_accepted; }
  uint64_t duplicate_count()    const { return m_duplicates; }
  uint64_t out_of_order_count() const { return m_out_of_order; }
  uint64_t reject_count()       const { return m_rejects; }
  const std::string& last_reject_reason() const { return m_last_reject_reason; }

 private:
  DecisionSnapshot m_decision;
  bool     m_has;
  uint64_t m_accepted, m_duplicates, m_out_of_order, m_rejects;
  std::string m_last_reject_reason;
};

} // namespace bb

#endif
