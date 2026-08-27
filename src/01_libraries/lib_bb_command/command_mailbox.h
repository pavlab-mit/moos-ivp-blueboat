/*************************************************************
 *  command_mailbox -- accepts source snapshots, enforces the
 *  epoch/seq lease, and answers "is this source still driving?"
 *
 *  The three rules that matter, and why:
 *
 *  1. A REPEATED epoch+seq does not refresh freshness. The
 *     back-seat broker re-transmits the latest field map every
 *     tick, so mail arriving is not evidence that the controller
 *     is alive. Only a NEW sequence is.
 *
 *  2. Freshness uses LOCAL arrival time, never source_time. Front
 *     and back seat clocks are not synchronised, so a producer's
 *     own timestamp cannot be trusted to decide expiry
 *     (invariant 8).
 *
 *  3. A NEW EPOCH resets the sequence baseline but grants
 *     nothing. A producer that restarts gets a clean slate for
 *     ordering; it does not thereby acquire authority
 *     (invariant 3).
 *
 *  This class holds no policy about who may drive. It answers
 *  factual questions only; the arbiter decides.
 *
 *  Author: Jeremy Wenger
 *************************************************************/

#ifndef BB_COMMAND_MAILBOX_HEADER
#define BB_COMMAND_MAILBOX_HEADER

#include "command_envelope.h"

#include <cstdint>
#include <string>

namespace bb {

enum class AcceptResult {
  ACCEPTED,      // new sequence, snapshot updated, lease refreshed
  DUPLICATE,     // same epoch+seq as current; lease NOT refreshed
  OUT_OF_ORDER,  // same epoch, older seq; rejected
  REJECTED       // failed parsing or validation
};

const char* to_string(AcceptResult r);

class CommandMailbox
{
 public:
  explicit CommandMailbox(CommandSource source = CommandSource::NONE);

  // Offer a wire string. arrival_time is local MOOSTime. On
  // ACCEPTED the snapshot and rx_time are updated; on anything
  // else the previous snapshot stands untouched.
  AcceptResult accept(const std::string& text, double arrival_time);

  // True once any snapshot has ever been accepted. Distinct from
  // "fresh" and from "valid": the arbiter needs all three to tell
  // NEVER_PRODUCED apart from STALE apart from INVALID, which are
  // different stop reasons.
  bool has_command() const { return m_has_command; }

  const SemanticCommand& snapshot() const { return m_command; }

  // Seconds since the last ACCEPTED sample. Large sentinel when
  // nothing has ever been accepted, so callers comparing against
  // a timeout fail closed without a special case.
  double age(double now) const;

  bool fresh(double now, double timeout_sec) const;

  // Convenience: fresh AND the producer marked it valid. Still
  // not an authority decision.
  bool usable(double now, double timeout_sec) const;

  CommandSource source() const { return m_source; }

  // --- counters, for BB_ARB_REJECT_* telemetry
  uint64_t accepted_count()     const { return m_accepted; }
  uint64_t duplicate_count()    const { return m_duplicates; }
  uint64_t out_of_order_count() const { return m_out_of_order; }
  uint64_t reject_count()       const { return m_rejects; }
  uint64_t epoch_change_count() const { return m_epoch_changes; }

  // Reason token from the most recent REJECTED accept(), for the
  // reject telemetry. Empty if the last accept was not a reject.
  const std::string& last_reject_reason() const { return m_last_reject_reason; }
  const std::string& last_reject_detail() const { return m_last_reject_detail; }

  void reset();

 private:
  CommandSource   m_source;
  SemanticCommand m_command;
  bool            m_has_command;

  uint64_t m_accepted;
  uint64_t m_duplicates;
  uint64_t m_out_of_order;
  uint64_t m_rejects;
  uint64_t m_epoch_changes;

  std::string m_last_reject_reason;
  std::string m_last_reject_detail;
};

} // namespace bb

#endif
