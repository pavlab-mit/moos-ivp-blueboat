/*************************************************************
 *  ControlSenderCore -- the pure logic of pControlSender.
 *
 *    AUTONOMY_CMD_PRECBF  (pBBPID's frame)          ]
 *    DESIRED_THRUST       (barrier filter output)   ]-> AUTONOMY_CMD
 *    DESIRED_RUDDER       (barrier filter output)   ]
 *   [CBF_SEQ]             (optional: seq it filtered)
 *
 *  AUTONOMY_CMD is pBBPID's frame, re-sent with the filtered
 *  thrust/rudder swapped in. epoch, seq, source_time, valid,
 *  authority_limit and every extra field are kept exactly as
 *  pBBPID wrote them. Only three things change:
 *
 *    surge/yaw  <- filtered thrust/rudder (valid frames only)
 *    producer   <- configurable (default "pBBPID_CBF")
 *    cbf=1      <- marker on every frame this app sends
 *
 *  WHY COPY pBBPID's SEQ INSTEAD OF MINTING ONE. The arbiter's
 *  lease refreshes only on a NEW seq (CommandMailbox rule 1).
 *  Because this app never creates one, it cannot make a dead
 *  pBBPID look alive: if pBBPID stops, new seqs stop, and the
 *  arbiter stops the boat with AUTONOMY_STALE exactly as today.
 *
 *  THE RULES, evaluated once per tick():
 *
 *    1. Nothing newer than the last frame sent -> send nothing.
 *    2. Newest frame is valid=0 -> forward it unchanged, at once,
 *       whatever the filter is doing. A stop always wins.
 *    3. A newly formed filter pair names the frame it belongs to:
 *         seq mode     -> the seq the filter published with it;
 *         arrival mode -> the newest frame that had ARRIVED when
 *                         the pair completed (mail order, not
 *                         clocks).
 *       If that frame is valid, newer than the last one sent and
 *       younger than filter_timeout_sec, send it ONCE with the
 *       pair's values. Each pair is consulted once.
 *    4. A valid frame is waiting and the filter is unhealthy
 *       (silent, or producing pairs that match no frame, for
 *       longer than filter_timeout_sec) -> send that frame as
 *       valid=0 with zeros and cbf_fault=<reason>.
 *    5. Never send a seq at or below the last one sent in the
 *       same epoch: the arbiter would drop it as a duplicate.
 *
 *  A valid frame whose pair is merely late is skipped, not
 *  invalidated: an invalid frame is a hard stop, and every hard
 *  stop resets the mixer's throttle ramp, so treating jitter as a
 *  fault would make the boat stutter. Gaps in seq are harmless;
 *  the mailbox only rejects seqs that go backwards.
 *
 *  Pairing thrust with rudder uses the PUBLISHER's timestamps
 *  (msg.GetTime()), which only ever get compared with each other.
 *  Every age uses LOCAL arrival time (invariant 8).
 *
 *  No MOOS, no clock, no I/O. The app feeds it mail and asks it,
 *  once per Iterate, what to send.
 *************************************************************/

#ifndef PCS_CONTROL_SENDER_CORE_HEADER
#define PCS_CONTROL_SENDER_CORE_HEADER

#include "command_envelope.h"
#include "command_mailbox.h"

#include <cstdint>
#include <deque>
#include <string>

namespace pcs {

struct ControlSenderConfig
{
  // Written into every frame sent. "pBBPID" makes the frame
  // byte-identical apart from surge/yaw and the marker; the
  // default makes filtering visible in the frontseat lineage
  // (BB_SELECTED_CMD / BB_MIXED_CMD source_producer). Nothing in
  // the arbiter, mixer or lib_bb_command checks this value.
  std::string producer = "pBBPID_CBF";

  // How long the filter may be silent (or unmatched) before a
  // waiting valid frame is sent as valid=0. Also the oldest a
  // frame may be and still carry filtered values.
  double filter_timeout_sec = 0.5;

  // Max difference between the publisher timestamps of the
  // halves of one filter output (thrust, rudder[, seq]).
  double pair_tolerance_sec = 0.02;

  // true when the filter publishes the seq of the frame it
  // filtered alongside thrust/rudder.
  bool use_filter_seq = false;

  // "" if valid, else why not.
  std::string validate() const;
};

enum class SendKind { NONE, FORWARD_INVALID, FILTERED, FILTER_STALE };
const char* to_string(SendKind k);

enum class FilterHealth { NEVER_HEARD, FRESH, SILENT, UNMATCHED };
const char* to_string(FilterHealth h);

struct SendDecision
{
  SendKind    kind = SendKind::NONE;
  std::string text;               // serialised frame; empty when NONE
  bb::SemanticCommand sent;       // what went out

  double   raw_surge = 0.0;       // pBBPID's own values for this seq
  double   raw_yaw   = 0.0;
  double   latency   = 0.0;       // now - pBBPID's source_time
  uint64_t seq_lag   = 0;         // newest frame seq - sent seq
  uint64_t skipped   = 0;         // frames passed over since last send
  bool     clamped   = false;     // a filtered value exceeded +/-100
};

// Who wrote a frame seen on the output variable.
enum class OutputOrigin { OURS, FOREIGN, UNPARSEABLE };
const char* to_string(OutputOrigin o);

struct OutputCheck
{
  OutputOrigin origin = OutputOrigin::UNPARSEABLE;
  std::string  producer;
  std::string  epoch;
  std::string  detail;            // reject reason when UNPARSEABLE
};

class ControlSenderCore
{
 public:
  static const char* kMarkerKey;  // "cbf"
  static const char* kFaultKey;   // "cbf_fault"

  explicit ControlSenderCore(const ControlSenderConfig& cfg);

  // --- mail -------------------------------------------------
  // pBBPID's frame. rx_time is local MOOSTime at arrival.
  bb::AcceptResult onFrame(const std::string& text, double rx_time);

  // Filter outputs. src_time is the publisher's stamp
  // (msg.GetTime()); rx_time is local arrival. Return false if
  // the sample was rejected (non-finite).
  bool onThrust(double value, double src_time, double rx_time);
  bool onRudder(double value, double src_time, double rx_time);
  bool onFilterSeq(uint64_t seq, double src_time, double rx_time);

  // A frame seen on the output variable (including our own,
  // which come back because the app subscribes to it).
  OutputCheck checkOutput(const std::string& text) const;

  // --- one cycle ----------------------------------------------
  SendDecision tick(double now);

  // --- introspection, for appcast and telemetry ---------------
  FilterHealth filterHealth(double now) const;
  bool   hasFrame() const { return !m_history.empty(); }
  const  bb::SemanticCommand& newestFrame() const;  // hasFrame() first
  double filterAge(double now) const;               // since last pair
  const  bb::CommandMailbox& frameMailbox() const { return m_frames; }
  const  ControlSenderConfig& config() const { return m_cfg; }

  bool        haveSent()  const { return m_have_sent; }
  uint64_t    sentSeq()   const { return m_sent_seq; }
  const std::string& sentEpoch() const { return m_sent_epoch; }

  uint64_t pairsFormed()      const { return m_pairs_formed; }
  uint64_t pairMismatches()   const { return m_pair_mismatches; }
  uint64_t nonfiniteRejects() const { return m_nonfinite; }
  uint64_t unmatchedPairs()   const { return m_unmatched_pairs; }
  uint64_t unusedPairs()      const { return m_unused_pairs; }
  uint64_t sentFiltered()     const { return m_sent_filtered; }
  uint64_t sentForwarded()    const { return m_sent_forwarded; }
  uint64_t sentStale()        const { return m_sent_stale; }
  uint64_t framesSkipped()    const { return m_frames_skipped; }
  uint64_t clampCount()       const { return m_clamps; }

 private:
  struct Frame
  {
    bb::SemanticCommand cmd;      // cmd.rx_time set by the mailbox
  };

  struct Half
  {
    bool     pending  = false;    // updated since the last pair
    double   value    = 0.0;
    uint64_t seq      = 0;        // seq channel only
    double   src_time = 0.0;
    double   rx_time  = 0.0;
  };

  struct Pair
  {
    bool        has        = false;
    bool        consulted  = false;
    bool        has_target = false;
    double      thrust     = 0.0;
    double      rudder     = 0.0;
    std::string target_epoch;
    uint64_t    target_seq = 0;
    double      rx_time    = 0.0;
  };

  bool updateHalf(Half& h, double value, double src_time, double rx_time);
  void tryFormPair();
  const Frame* findFrame(const std::string& epoch, uint64_t seq) const;
  bool isNewer(const bb::SemanticCommand& c) const;
  SendDecision send(const Frame& f, SendKind kind, double now,
                    double thrust, double rudder, FilterHealth why);

  ControlSenderConfig m_cfg;

  bb::CommandMailbox  m_frames;
  std::deque<Frame>   m_history;  // current epoch only, oldest first

  Half m_thrust, m_rudder, m_seq;
  Pair m_pair;

  bool   m_have_pair;
  double m_last_pair_rx;
  bool   m_have_matchable;
  double m_last_matchable_rx;

  bool        m_have_sent;
  std::string m_sent_epoch;
  uint64_t    m_sent_seq;

  uint64_t m_pairs_formed, m_pair_mismatches, m_nonfinite;
  uint64_t m_unmatched_pairs, m_unused_pairs;
  uint64_t m_sent_filtered, m_sent_forwarded, m_sent_stale;
  uint64_t m_frames_skipped, m_clamps;
};

} // namespace pcs

#endif
