/*************************************************************
 * Unit tests for RcLinkState - staleness, hysteresis, failsafe,
 * and the reader-side dead-writer guard.
 *
 * Host-side only: no hardware, no serial port, no MOOS, no
 * threads. Timestamps are passed in, so every timing scenario is
 * exact and deterministic.
 *
 * The dead-writer tests at the bottom encode the failure class
 * observed in the 2026-07-30 rc_hil_bench logs: the first version
 * of iRCInterface deadlocked its own threads on a recursive mutex,
 * and a frozen writer must never be able to keep reporting a live
 * link. The deadlock itself was structural (fixed by the snapshot
 * design); these tests pin the semantic guard that makes any
 * future frozen-writer bug fail SAFE instead of defeating the
 * deadman.
 *
 * Convention follows navigator-cpp: print "FAIL: ..." and return
 * non-zero on failure, "PASS" and 0 on success. No test
 * framework dependency.
 *
 * Build:  ./build.sh --unit_tests
 * Run:    ./bin/test_rc_link_state
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "rc_link_state.h"

#include <cstdio>
#include <string>

//---------------------------------------------------------
// Minimal check harness

static int g_failures = 0;
static int g_checks   = 0;

static void check(bool cond, const std::string& what)
{
  g_checks++;
  if (!cond) {
    g_failures++;
    fprintf(stderr, "FAIL: %s\n", what.c_str());
  }
}

// Timestamps: microseconds. Frames every 10 ms = 100 Hz, the
// planned link rate. An arbitrary nonzero epoch guards against
// code accidentally treating t=0 as special beyond "never".
static const uint64_t T0 = 1000000000ull;   // 1000 s
static const uint64_t MS = 1000ull;

//---------------------------------------------------------
// Initial state: never seen a frame -> everything false.

static void test_initial_state()
{
  RcLinkState s;
  check(!s.frameValid(),  "initial: frameValid false");
  check(!s.rcConnected(), "initial: rcConnected false");
  check(!s.failsafe(),    "initial: failsafe false");

  // refresh() with no frames ever must stay disconnected.
  s.refresh(T0);
  check(!s.frameValid(),  "initial+refresh: frameValid still false");
  check(!s.rcConnected(), "initial+refresh: rcConnected still false");
}

//---------------------------------------------------------
// Reconnect hysteresis: valid immediately, connected only after
// RC_HYSTERESIS_GOOD_FRAMES consecutive good frames.

static void test_hysteresis()
{
  RcLinkState s;

  s.onChannelsFrame(T0);
  check(s.frameValid(),   "1 frame: frameValid true");
  check(!s.rcConnected(), "1 frame: not yet connected");

  s.onChannelsFrame(T0 + 10*MS);
  check(!s.rcConnected(), "2 frames: not yet connected");

  s.onChannelsFrame(T0 + 20*MS);
  check(s.rcConnected(),  "3 frames: connected");
  check(s.framesReceived() == 3, "3 frames counted");
}

//---------------------------------------------------------
// Staleness: strict > timeout. A frame aged exactly the timeout
// is still valid; one microsecond-rounded ms past it is not.

static void test_staleness_boundary()
{
  RcLinkState s;
  for (int i = 0; i < 3; i++)
    s.onChannelsFrame(T0 + i*10*MS);
  const uint64_t last = T0 + 20*MS;

  s.refresh(last + RC_SIGNAL_LOSS_TIMEOUT_MS*MS);
  check(s.frameValid(),  "age == timeout: still valid");
  check(s.rcConnected(), "age == timeout: still connected");

  s.refresh(last + (RC_SIGNAL_LOSS_TIMEOUT_MS+1)*MS);
  check(!s.frameValid(),  "age > timeout: invalid");
  check(!s.rcConnected(), "age > timeout: disconnected");
}

//---------------------------------------------------------
// Recovery after a dropout restarts the hysteresis from zero:
// fast disconnect, slow reconnect.

static void test_dropout_recovery()
{
  RcLinkState s;
  for (int i = 0; i < 5; i++)
    s.onChannelsFrame(T0 + i*10*MS);
  check(s.rcConnected(), "connected before dropout");

  // 600 ms of silence.
  const uint64_t t_gap = T0 + 40*MS + 600*MS;
  s.refresh(t_gap);
  check(!s.rcConnected(), "disconnected after 600 ms gap");
  check(s.consecGoodFrames() == 0, "hysteresis reset by dropout");

  // Frames resume: needs 3 fresh ones again.
  s.onChannelsFrame(t_gap + 10*MS);
  check(s.frameValid(),   "resume 1: valid immediately");
  check(!s.rcConnected(), "resume 1: not yet connected");
  s.onChannelsFrame(t_gap + 20*MS);
  check(!s.rcConnected(), "resume 2: not yet connected");
  s.onChannelsFrame(t_gap + 30*MS);
  check(s.rcConnected(),  "resume 3: reconnected");
}

//---------------------------------------------------------
// Failsafe: uplink LQ 0 disconnects immediately even while RC
// frames keep arriving (ELRS keeps emitting 0x16 briefly after
// declaring a dead uplink). Frames during failsafe build no
// reconnect credit.

static void test_failsafe()
{
  RcLinkState s;
  for (int i = 0; i < 5; i++)
    s.onChannelsFrame(T0 + i*10*MS);
  check(s.rcConnected(), "connected before failsafe");

  uint64_t t = T0 + 50*MS;
  s.onLinkStats(0, t);   // receiver declares dead uplink
  check(s.failsafe(),     "LQ 0: failsafe set");
  check(!s.frameValid(),  "LQ 0: frameValid false immediately");
  check(!s.rcConnected(), "LQ 0: disconnected immediately");

  // RC frames continuing during failsafe must not reconnect.
  for (int i = 1; i <= 5; i++) {
    s.onChannelsFrame(t + i*10*MS);
    check(!s.rcConnected(), "frame during failsafe: stays disconnected");
  }
  check(s.consecGoodFrames() == 0, "failsafe frames build no credit");

  // Link recovers: LQ nonzero, then normal hysteresis applies.
  t += 100*MS;
  s.onLinkStats(72, t);
  check(!s.failsafe(),    "LQ 72: failsafe cleared");
  check(!s.rcConnected(), "failsafe clear: not connected until hysteresis");

  s.onChannelsFrame(t + 10*MS);
  s.onChannelsFrame(t + 20*MS);
  check(!s.rcConnected(), "post-failsafe 2 frames: not yet");
  s.onChannelsFrame(t + 30*MS);
  check(s.rcConnected(),  "post-failsafe 3 frames: reconnected");
}

//---------------------------------------------------------
// Dead-writer guard: a reader that copies the state and calls
// refresh() with ITS OWN clock must see the link die, even though
// the frozen copy was written with rc_connected=true. This is the
// contract Iterate() relies on so a wedged serial thread cannot
// defeat the downstream deadman.

static void test_dead_writer_guard()
{
  RcLinkState writer;
  for (int i = 0; i < 5; i++)
    writer.onChannelsFrame(T0 + i*10*MS);
  check(writer.rcConnected(), "writer connected at freeze time");

  // Writer wedges here. Reader keeps copying the frozen state.
  RcLinkState reader = writer;   // snapshot copy

  // Within the window the frozen truth is still the truth.
  reader.refresh(T0 + 40*MS + 100*MS);
  check(reader.rcConnected(), "reader +100 ms: still connected");

  // Ten seconds later the copy must NOT keep saying connected.
  reader = writer;
  reader.refresh(T0 + 40*MS + 10000*MS);
  check(!reader.rcConnected(), "reader +10 s: frozen writer reads disconnected");
  check(!reader.frameValid(),  "reader +10 s: frameValid false");

  // And the writer's own stored copy is untouched by the reader's
  // refresh (value semantics, no sharing).
  check(writer.rcConnected(), "writer copy unaffected by reader refresh");
}

//---------------------------------------------------------
// 100 Hz sanity: the numbers from the design review. 50 missed
// frames (500 ms) is the disconnect point; 3 frames (30 ms) is
// the reconnect cost.

static void test_100hz_profile()
{
  RcLinkState s;
  uint64_t t = T0;
  for (int i = 0; i < 100; i++) {   // 1 s of clean 100 Hz link
    s.onChannelsFrame(t);
    t += 10*MS;
  }
  check(s.rcConnected(), "100 Hz: connected after 1 s");

  // 49 missed frames (490 ms): still inside the window.
  s.refresh(t - 10*MS + 490*MS);
  check(s.frameValid(), "49 missed frames: still valid");

  // 51 missed frames (510 ms): stale.
  s.refresh(t - 10*MS + 510*MS);
  check(!s.frameValid(), "51 missed frames: stale");
}

//---------------------------------------------------------

int main()
{
  test_initial_state();
  test_hysteresis();
  test_staleness_boundary();
  test_dropout_recovery();
  test_failsafe();
  test_dead_writer_guard();
  test_100hz_profile();

  if (g_failures == 0) {
    printf("PASS (%d checks)\n", g_checks);
    return 0;
  }
  fprintf(stderr, "%d of %d checks FAILED\n", g_failures, g_checks);
  return 1;
}
