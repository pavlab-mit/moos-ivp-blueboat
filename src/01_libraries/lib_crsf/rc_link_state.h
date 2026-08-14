/*************************************************************
 * RC link supervision state machine
 *
 * Transport-agnostic connection model for an RC link: per-frame
 * validity, staleness, failsafe, and asymmetric hysteresis.
 * Extracted from iRCInterface so the semantics are unit-testable
 * on a laptop - no MOOS, no serial port, no threads, no clock
 * (callers pass timestamps in).
 *
 * Two-boolean model, semantics identical to lib_sbus:
 *
 *   frameValid()  - per-frame, instantaneous (safety gate).
 *                   True iff frames are arriving, the newest is
 *                   within the staleness window, and the receiver
 *                   has not declared failsafe.
 *
 *   rcConnected() - debounced (mode/UI/logging). Becomes true
 *                   only after min_good_frames consecutive good
 *                   frames since the last bad event; a single bad
 *                   event (staleness or failsafe) flips it false
 *                   immediately. Asymmetric hysteresis: fast
 *                   disconnect, slow reconnect.
 *
 * Threading contract: the class is deliberately lock-free and
 * copyable. The writer (a serial thread) owns an instance and
 * calls onChannelsFrame() / onLinkStats() / refresh(); readers
 * receive it BY COPY inside a snapshot and MUST call refresh(now)
 * on their copy before trusting the booleans. That reader-side
 * re-evaluation is what prevents a dead or wedged writer from
 * freezing rcConnected()==true forever - which would silently
 * defeat any downstream deadman watchdog keyed on RC_CONNECTED.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#ifndef RC_LINK_STATE_H
#define RC_LINK_STATE_H

#include <stdint.h>

// Link considered stale after this long without an RC frame.
#define RC_SIGNAL_LOSS_TIMEOUT_MS  500

// Consecutive good frames required to flip rcConnected()
// false -> true. A single bad event flips it back immediately.
#define RC_HYSTERESIS_GOOD_FRAMES  3

class RcLinkState {
public:
  explicit RcLinkState(
      uint32_t timeout_ms = RC_SIGNAL_LOSS_TIMEOUT_MS,
      unsigned int min_good_frames = RC_HYSTERESIS_GOOD_FRAMES);

  // A CRC-valid RC channels frame arrived at now_us.
  void onChannelsFrame(uint64_t now_us);

  // A CRC-valid link-statistics frame arrived. CRSF has no
  // failsafe flag in the RC frame; failsafe is the receiver
  // reporting uplink LQ == 0 while stats still flow.
  void onLinkStats(uint8_t uplink_lq, uint64_t now_us);

  // Recompute frameValid()/rcConnected() against now_us. The
  // writer calls this every loop pass; every reader calls it on
  // its own copy with its own clock (see threading contract).
  void refresh(uint64_t now_us);

  bool frameValid()  const { return frame_valid_; }
  bool rcConnected() const { return rc_connected_; }
  bool failsafe()    const { return failsafe_; }

  uint64_t      lastFrameUs()      const { return last_frame_us_; }
  unsigned long framesReceived()   const { return frames_received_; }
  unsigned long consecGoodFrames() const { return consec_good_frames_; }

private:
  uint32_t     timeout_ms_;
  unsigned int min_good_frames_;

  uint64_t last_frame_us_;      // 0 = never
  bool     failsafe_;
  bool     frame_valid_;
  bool     rc_connected_;
  unsigned long consec_good_frames_;
  unsigned long frames_received_;
};

#endif /* RC_LINK_STATE_H */
