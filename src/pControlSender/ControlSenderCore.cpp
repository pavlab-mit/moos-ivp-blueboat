/*************************************************************
 *  ControlSenderCore -- see the header for the rules.
 *************************************************************/

#include "ControlSenderCore.h"

#include <cmath>

namespace pcs {

namespace {

// Frames kept for seq-mode lookup: 1.6 s at pBBPID's 10 Hz, well
// past filter_timeout_sec, beyond which a frame is too old anyway.
const size_t kHistoryLen = 16;

double clamp100(double v, bool& clamped)
{
  if (v >  100.0) { clamped = true; return  100.0; }
  if (v < -100.0) { clamped = true; return -100.0; }
  return v;
}

} // namespace

const char* ControlSenderCore::kMarkerKey = "cbf";
const char* ControlSenderCore::kFaultKey  = "cbf_fault";

//---------------------------------------------------------

std::string ControlSenderConfig::validate() const
{
  if (producer.empty())
    return "producer must not be empty";
  // The wire dialect splits on ',' -- a comma in the producer
  // would make every frame we send fail to parse downstream.
  if (producer.find(',') != std::string::npos)
    return "producer must not contain ','";
  if (!std::isfinite(filter_timeout_sec) || filter_timeout_sec <= 0.0)
    return "filter_timeout_sec must be finite and > 0";
  if (!std::isfinite(pair_tolerance_sec) || pair_tolerance_sec < 0.0)
    return "pair_tolerance_sec must be finite and >= 0";
  return "";
}

const char* to_string(SendKind k)
{
  switch (k) {
    case SendKind::NONE:            return "NONE";
    case SendKind::FORWARD_INVALID: return "FORWARD_INVALID";
    case SendKind::FILTERED:        return "FILTERED";
    case SendKind::FILTER_STALE:    return "FILTER_STALE";
  }
  return "NONE";
}

const char* to_string(FilterHealth h)
{
  switch (h) {
    case FilterHealth::NEVER_HEARD: return "filter_never_heard";
    case FilterHealth::FRESH:       return "fresh";
    case FilterHealth::SILENT:      return "filter_silent";
    case FilterHealth::UNMATCHED:   return "filter_unmatched";
  }
  return "filter_silent";
}

const char* to_string(OutputOrigin o)
{
  switch (o) {
    case OutputOrigin::OURS:        return "ours";
    case OutputOrigin::FOREIGN:     return "foreign";
    case OutputOrigin::UNPARSEABLE: return "unparseable";
  }
  return "unparseable";
}

//---------------------------------------------------------

ControlSenderCore::ControlSenderCore(const ControlSenderConfig& cfg)
  : m_cfg(cfg),
    m_frames(bb::CommandSource::AUTONOMY),
    m_have_pair(false),
    m_last_pair_rx(0.0),
    m_have_matchable(false),
    m_last_matchable_rx(0.0),
    m_have_sent(false),
    m_sent_seq(0),
    m_pairs_formed(0), m_pair_mismatches(0), m_nonfinite(0),
    m_unmatched_pairs(0), m_unused_pairs(0),
    m_sent_filtered(0), m_sent_forwarded(0), m_sent_stale(0),
    m_frames_skipped(0), m_clamps(0)
{
}

//---------------------------------------------------------
// Mail

bb::AcceptResult ControlSenderCore::onFrame(const std::string& text, double rx_time)
{
  // The mailbox gives us the arbiter's own discipline for free:
  // strict parse, duplicate and out-of-order rejection, epoch
  // rebase on a pBBPID restart.
  const bb::AcceptResult r = m_frames.accept(text, rx_time);
  if (r != bb::AcceptResult::ACCEPTED)
    return r;

  const bb::SemanticCommand& c = m_frames.snapshot();

  // pBBPID restarted: frames from the old epoch say nothing about
  // the new one, and their seqs would collide with new ones.
  if (!m_history.empty() && m_history.back().cmd.epoch != c.epoch)
    m_history.clear();

  Frame f;
  f.cmd = c;
  m_history.push_back(f);
  while (m_history.size() > kHistoryLen)
    m_history.pop_front();
  return r;
}

bool ControlSenderCore::updateHalf(Half& h, double value, double src_time,
                                   double rx_time)
{
  if (!std::isfinite(value) || !std::isfinite(src_time) ||
      !std::isfinite(rx_time)) {
    ++m_nonfinite;
    return false;
  }
  h.pending  = true;
  h.value    = value;
  h.src_time = src_time;
  h.rx_time  = rx_time;
  tryFormPair();
  return true;
}

bool ControlSenderCore::onThrust(double value, double src_time, double rx_time)
{
  return updateHalf(m_thrust, value, src_time, rx_time);
}

bool ControlSenderCore::onRudder(double value, double src_time, double rx_time)
{
  return updateHalf(m_rudder, value, src_time, rx_time);
}

bool ControlSenderCore::onFilterSeq(uint64_t seq, double src_time, double rx_time)
{
  if (!m_cfg.use_filter_seq)
    return false;
  m_seq.seq = seq;
  return updateHalf(m_seq, (double)seq, src_time, rx_time);
}

//---------------------------------------------------------
// tryFormPair: complete one filter output from its halves.
//
// A pair forms only when every half has been updated since the
// last pair AND their publisher timestamps agree to within
// pair_tolerance_sec -- i.e. they came out of the same filter
// cycle. On a mismatch the OLDEST half is dropped, so the next
// update can complete a coherent pair instead of the old half
// being glued to a new one.

void ControlSenderCore::tryFormPair()
{
  Half* halves[3] = { &m_thrust, &m_rudder, &m_seq };
  const int n = m_cfg.use_filter_seq ? 3 : 2;

  for (int i = 0; i < n; ++i)
    if (!halves[i]->pending) return;

  int    oldest = 0;
  double lo = halves[0]->src_time, hi = halves[0]->src_time;
  double rx = halves[0]->rx_time;
  for (int i = 1; i < n; ++i) {
    if (halves[i]->src_time < lo) { lo = halves[i]->src_time; oldest = i; }
    if (halves[i]->src_time > hi) hi = halves[i]->src_time;
    if (halves[i]->rx_time  > rx) rx = halves[i]->rx_time;
  }

  if (hi - lo > m_cfg.pair_tolerance_sec) {
    ++m_pair_mismatches;
    halves[oldest]->pending = false;
    return;
  }

  // An unconsulted previous pair is being superseded: count it,
  // it is a filter output that never reached the boat.
  if (m_pair.has && !m_pair.consulted)
    ++m_unused_pairs;

  Pair p;
  p.has     = true;
  p.thrust  = m_thrust.value;
  p.rudder  = m_rudder.value;
  p.rx_time = rx;

  // Which frame does this output belong to?
  if (!m_history.empty()) {
    p.has_target   = true;
    p.target_epoch = m_history.back().cmd.epoch;
    // Arrival mode: the newest frame that had arrived by now. Mail
    // is processed in delivery order, so this is exact even when
    // a frame and a pair land in the same batch.
    p.target_seq   = m_cfg.use_filter_seq ? m_seq.seq
                                          : m_history.back().cmd.seq;
  }

  for (int i = 0; i < n; ++i)
    halves[i]->pending = false;

  m_pair = p;
  ++m_pairs_formed;
  m_have_pair    = true;
  m_last_pair_rx = rx;

  // "Matchable": the pair names a frame we still hold that is
  // young enough to use. A filter echoing the wrong seq (or
  // lagging past filter_timeout_sec) produces pairs that are
  // never matchable, which filterHealth() reports as UNMATCHED.
  if (p.has_target) {
    const Frame* f = findFrame(p.target_epoch, p.target_seq);
    if (f && (rx - f->cmd.rx_time) <= m_cfg.filter_timeout_sec) {
      m_have_matchable    = true;
      m_last_matchable_rx = rx;
    }
    else {
      ++m_unmatched_pairs;
    }
  }
}

//---------------------------------------------------------

const ControlSenderCore::Frame*
ControlSenderCore::findFrame(const std::string& epoch, uint64_t seq) const
{
  for (std::deque<Frame>::const_reverse_iterator it = m_history.rbegin();
       it != m_history.rend(); ++it) {
    if (it->cmd.epoch == epoch && it->cmd.seq == seq)
      return &(*it);
  }
  return 0;
}

bool ControlSenderCore::isNewer(const bb::SemanticCommand& c) const
{
  if (!m_have_sent) return true;
  if (c.epoch != m_sent_epoch) return true;   // pBBPID restarted
  return c.seq > m_sent_seq;
}

const bb::SemanticCommand& ControlSenderCore::newestFrame() const
{
  return m_history.back().cmd;
}

double ControlSenderCore::filterAge(double now) const
{
  if (!m_have_pair) return 1.0e9;
  const double a = now - m_last_pair_rx;
  return (a < 0.0) ? 0.0 : a;
}

FilterHealth ControlSenderCore::filterHealth(double now) const
{
  if (!m_have_pair)
    return FilterHealth::NEVER_HEARD;
  if ((now - m_last_pair_rx) > m_cfg.filter_timeout_sec)
    return FilterHealth::SILENT;
  if (!m_have_matchable ||
      (now - m_last_matchable_rx) > m_cfg.filter_timeout_sec)
    return FilterHealth::UNMATCHED;
  return FilterHealth::FRESH;
}

//---------------------------------------------------------
// tick: one cycle. At most one frame goes out.

SendDecision ControlSenderCore::tick(double now)
{
  SendDecision none;
  if (m_history.empty())
    return none;                    // pBBPID silent: the arbiter's
                                    // own lease handles it

  const Frame& newest = m_history.back();

  // Rule 2: a stop always wins, and never waits for the filter.
  if (isNewer(newest.cmd) && !newest.cmd.valid)
    return send(newest, SendKind::FORWARD_INVALID, now, 0.0, 0.0,
                FilterHealth::FRESH);

  // Rule 3: a fresh pair, consulted exactly once.
  if (m_pair.has && !m_pair.consulted) {
    m_pair.consulted = true;
    const Frame* f = m_pair.has_target
                         ? findFrame(m_pair.target_epoch, m_pair.target_seq)
                         : 0;
    if (f && f->cmd.valid && isNewer(f->cmd) &&
        (now - f->cmd.rx_time) <= m_cfg.filter_timeout_sec)
      return send(*f, SendKind::FILTERED, now, m_pair.thrust, m_pair.rudder,
                  FilterHealth::FRESH);
    // Late (its frame already went out), unknown, invalid or too
    // old. Not an error by itself; filterHealth() decides whether
    // a pattern of these is.
    ++m_unused_pairs;
  }

  // Rule 4: a valid frame is waiting and the filter cannot serve it.
  if (isNewer(newest.cmd) && newest.cmd.valid) {
    const FilterHealth h = filterHealth(now);
    if (h != FilterHealth::FRESH)
      return send(newest, SendKind::FILTER_STALE, now, 0.0, 0.0, h);
  }

  // Otherwise wait: the pair for the newest frame is on its way.
  return none;
}

//---------------------------------------------------------
// send: build the outgoing frame from pBBPID's.

SendDecision ControlSenderCore::send(const Frame& f, SendKind kind, double now,
                                     double thrust, double rudder,
                                     FilterHealth why)
{
  SendDecision d;
  d.kind      = kind;
  d.raw_surge = f.cmd.surge;
  d.raw_yaw   = f.cmd.yaw;

  // Start from pBBPID's frame verbatim: version, epoch, seq,
  // source_time, valid, authority_limit and every extra field.
  bb::SemanticCommand c = f.cmd;
  c.producer = m_cfg.producer;
  c.extra[kMarkerKey] = "1";
  c.extra.erase(kFaultKey);

  if (kind == SendKind::FILTERED) {
    // The parser downstream rejects anything outside [-100, 100]
    // and drops the WHOLE frame, so clamp here (pBBPID does too).
    bool clamped = false;
    c.surge = clamp100(thrust, clamped);
    c.yaw   = clamp100(rudder, clamped);
    d.clamped = clamped;
    if (clamped) ++m_clamps;
    ++m_sent_filtered;
  }
  else if (kind == SendKind::FILTER_STALE) {
    c.valid = false;
    c.surge = 0.0;
    c.yaw   = 0.0;
    c.extra[kFaultKey] = to_string(why);
    ++m_sent_stale;
  }
  else {
    // FORWARD_INVALID: exactly as pBBPID sent it (already zeros).
    ++m_sent_forwarded;
  }

  if (m_have_sent && c.epoch == m_sent_epoch && c.seq > m_sent_seq + 1) {
    d.skipped = c.seq - m_sent_seq - 1;
    m_frames_skipped += d.skipped;
  }

  d.text    = bb::serialize_command(c);
  d.sent    = c;
  d.latency = now - f.cmd.source_time;   // both on the backseat clock
  d.seq_lag = m_history.back().cmd.seq - c.seq;

  m_have_sent  = true;
  m_sent_epoch = c.epoch;
  m_sent_seq   = c.seq;
  return d;
}

//---------------------------------------------------------

OutputCheck ControlSenderCore::checkOutput(const std::string& text) const
{
  OutputCheck oc;
  const bb::ParseResult pr = bb::parse_command(text, bb::CommandSource::AUTONOMY);
  if (!pr.ok) {
    oc.origin = OutputOrigin::UNPARSEABLE;
    oc.detail = pr.reject_reason + (pr.detail.empty() ? "" : ":" + pr.detail);
    return oc;
  }
  oc.producer = pr.command.producer;
  oc.epoch    = pr.command.epoch;
  // The marker, not the producer, identifies our frames: the
  // producer is configurable and may legitimately read "pBBPID".
  oc.origin = (pr.command.field(kMarkerKey) == "1") ? OutputOrigin::OURS
                                                    : OutputOrigin::FOREIGN;
  return oc;
}

} // namespace pcs
