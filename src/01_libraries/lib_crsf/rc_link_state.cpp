/*************************************************************
 * RC link supervision state machine implementation
 *
 * See rc_link_state.h for the model and threading contract.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "rc_link_state.h"

RcLinkState::RcLinkState(uint32_t timeout_ms,
                         unsigned int min_good_frames)
  : timeout_ms_(timeout_ms),
    min_good_frames_(min_good_frames),
    last_frame_us_(0),
    failsafe_(false),
    frame_valid_(false),
    rc_connected_(false),
    consec_good_frames_(0),
    frames_received_(0)
{
}

//---------------------------------------------------------
// onChannelsFrame()

void RcLinkState::onChannelsFrame(uint64_t now_us)
{
  last_frame_us_ = now_us;
  frames_received_++;

  // Frames arriving during failsafe do not build reconnect
  // credit; the hysteresis restarts once failsafe clears.
  if (!failsafe_)
    consec_good_frames_++;

  refresh(now_us);
}

//---------------------------------------------------------
// onLinkStats()

void RcLinkState::onLinkStats(uint8_t uplink_lq, uint64_t now_us)
{
  failsafe_ = (uplink_lq == 0);
  refresh(now_us);
}

//---------------------------------------------------------
// refresh()
//
// Staleness is strict-greater-than the timeout, so a frame aged
// exactly timeout_ms is still valid. A single bad event zeroes
// the hysteresis counter; reconnection requires min_good_frames
// fresh consecutive good frames.

void RcLinkState::refresh(uint64_t now_us)
{
  bool stale = (last_frame_us_ == 0) ||
    ((now_us - last_frame_us_) / 1000ull > (uint64_t)timeout_ms_);

  if (stale || failsafe_) {
    frame_valid_  = false;
    rc_connected_ = false;
    consec_good_frames_ = 0;
  } else {
    frame_valid_ = true;
    if (consec_good_frames_ >= min_good_frames_)
      rc_connected_ = true;
  }
}
