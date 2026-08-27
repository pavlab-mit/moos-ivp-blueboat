#include "command_mailbox.h"

#include <cmath>

namespace bb {

namespace {
// Returned by age() when nothing has ever been accepted. Any
// sane timeout comparison then reports "stale", which is the
// safe reading of "we have never heard from this source".
const double kNeverAge = 1.0e9;
}

const char* to_string(AcceptResult r)
{
  switch (r) {
    case AcceptResult::ACCEPTED:     return "accepted";
    case AcceptResult::DUPLICATE:    return "duplicate";
    case AcceptResult::OUT_OF_ORDER: return "out_of_order_seq";
    case AcceptResult::REJECTED:     return "rejected";
  }
  return "unknown";
}

CommandMailbox::CommandMailbox(CommandSource source)
  : m_source(source),
    m_has_command(false),
    m_accepted(0),
    m_duplicates(0),
    m_out_of_order(0),
    m_rejects(0),
    m_epoch_changes(0)
{
}

void CommandMailbox::reset()
{
  m_command = SemanticCommand();
  m_has_command = false;
  m_last_reject_reason.clear();
  m_last_reject_detail.clear();
  // Counters deliberately survive a reset: they are lifetime
  // diagnostics, and zeroing them would hide a source that is
  // flapping.
}

AcceptResult CommandMailbox::accept(const std::string& text, double arrival_time)
{
  const ParseResult pr = parse_command(text, m_source);

  if (!pr.ok) {
    ++m_rejects;
    m_last_reject_reason = pr.reject_reason;
    m_last_reject_detail = pr.detail;
    return AcceptResult::REJECTED;
  }

  // A non-finite arrival time would poison every subsequent
  // freshness test, so treat it as a reject rather than storing it.
  if (!std::isfinite(arrival_time)) {
    ++m_rejects;
    m_last_reject_reason = reject::kNonFinite;
    m_last_reject_detail = "arrival_time";
    return AcceptResult::REJECTED;
  }

  m_last_reject_reason.clear();
  m_last_reject_detail.clear();

  SemanticCommand incoming = pr.command;

  if (m_has_command && incoming.epoch == m_command.epoch) {
    if (incoming.seq == m_command.seq) {
      // Rule 1: a repeat is not evidence of life. Do NOT touch
      // rx_time -- that is the whole point.
      ++m_duplicates;
      return AcceptResult::DUPLICATE;
    }
    if (incoming.seq < m_command.seq) {
      ++m_out_of_order;
      return AcceptResult::OUT_OF_ORDER;
    }
  } else if (m_has_command && incoming.epoch != m_command.epoch) {
    // Rule 3: producer restarted. Accept and rebase ordering.
    ++m_epoch_changes;
  }

  incoming.rx_time = arrival_time;   // Rule 2
  m_command = incoming;
  m_has_command = true;
  ++m_accepted;
  return AcceptResult::ACCEPTED;
}

double CommandMailbox::age(double now) const
{
  if (!m_has_command) return kNeverAge;
  if (!std::isfinite(now)) return kNeverAge;
  const double a = now - m_command.rx_time;
  return (a < 0.0) ? 0.0 : a;   // clock stepped backwards; not stale
}

bool CommandMailbox::fresh(double now, double timeout_sec) const
{
  if (!m_has_command) return false;
  if (!std::isfinite(timeout_sec) || timeout_sec <= 0.0) return false;
  return age(now) <= timeout_sec;
}

bool CommandMailbox::usable(double now, double timeout_sec) const
{
  return fresh(now, timeout_sec) && m_command.valid;
}

} // namespace bb
