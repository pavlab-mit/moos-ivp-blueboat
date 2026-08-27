/*************************************************************
 * Unit tests for CommandMailbox -- the source lease.
 *
 * The duplicate-frame test is the one that matters most. The
 * back-seat broker re-transmits its latest field map every tick
 * whether or not the controller produced anything new, so "mail
 * arrived" is not evidence the controller is alive. If a repeat
 * refreshed freshness, a dead controller would look healthy
 * forever and the boat would keep driving on its last command.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "command_mailbox.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace bb;

static int g_failures = 0;
static int g_checks   = 0;

static void check(bool cond, const std::string& what)
{
  g_checks++;
  if (!cond) { g_failures++; fprintf(stderr, "FAIL: %s\n", what.c_str()); }
}

// Build a wire string with the given epoch/seq/validity.
static std::string frame(const std::string& epoch, unsigned long long seq,
                         bool valid = true, double surge = 10.0)
{
  char buf[256];
  snprintf(buf, sizeof(buf),
           "v=1,producer=pBBPID,epoch=%s,seq=%llu,source_time=%.3f,"
           "valid=%d,surge=%.1f,yaw=0.0",
           epoch.c_str(), seq, (double)seq * 0.05, valid ? 1 : 0, surge);
  return std::string(buf);
}

static void test_initial_state()
{
  CommandMailbox mb(CommandSource::AUTONOMY);
  check(!mb.has_command(), "nothing accepted yet");
  check(!mb.fresh(100.0, 1.0), "never-heard is not fresh");
  check(!mb.usable(100.0, 1.0), "never-heard is not usable");
  check(mb.age(100.0) > 1e6, "age is a large sentinel, so timeouts fail closed");
  check(mb.source() == CommandSource::AUTONOMY, "source recorded");
}

static void test_accept_and_freshness()
{
  CommandMailbox mb(CommandSource::AUTONOMY);
  check(mb.accept(frame("e1", 1), 100.0) == AcceptResult::ACCEPTED, "first frame");
  check(mb.has_command(), "has a command");
  check(std::fabs(mb.age(100.0)) < 1e-9, "age zero at arrival");
  check(std::fabs(mb.age(100.5) - 0.5) < 1e-9, "age advances with the clock");
  check(mb.fresh(100.5, 1.0), "inside the timeout");
  check(!mb.fresh(101.5, 1.0), "outside the timeout");
  check(mb.accepted_count() == 1, "accepted counter");
}

// Rule 1, and the reason this class exists.
static void test_duplicate_does_not_refresh_lease()
{
  CommandMailbox mb(CommandSource::AUTONOMY);
  mb.accept(frame("e1", 7), 100.0);

  // The broker keeps re-sending the same sequence for a second.
  for (int i = 1; i <= 10; ++i) {
    const double t = 100.0 + i * 0.1;
    check(mb.accept(frame("e1", 7), t) == AcceptResult::DUPLICATE,
          "repeat of the same epoch+seq is a duplicate");
  }

  check(mb.duplicate_count() == 10, "duplicates counted");
  check(std::fabs(mb.age(101.0) - 1.0) < 1e-9,
        "age measured from the ORIGINAL arrival, not the repeats");
  check(!mb.fresh(101.0, 0.5),
        "a source repeating one sequence goes stale on schedule");
}

// Rule 2: source_time must not influence expiry. The back seat's
// clock may be arbitrarily far from ours.
static void test_freshness_ignores_source_time()
{
  CommandMailbox mb(CommandSource::AUTONOMY);
  // source_time here is ~0.05, wildly different from local 5000.
  mb.accept(frame("e1", 1), 5000.0);
  check(std::fabs(mb.age(5000.25) - 0.25) < 1e-9,
        "age uses local arrival, not the producer's clock");
  check(mb.fresh(5000.25, 1.0), "fresh by local time");
}

static void test_out_of_order_rejected()
{
  CommandMailbox mb(CommandSource::AUTONOMY);
  mb.accept(frame("e1", 10), 100.0);
  check(mb.accept(frame("e1", 9), 100.1) == AcceptResult::OUT_OF_ORDER,
        "older sequence rejected");
  check(mb.snapshot().seq == 10, "snapshot unchanged by an out-of-order frame");
  check(std::fabs(mb.age(100.1) - 0.1) < 1e-9,
        "out-of-order frame does not refresh the lease");
  check(mb.out_of_order_count() == 1, "out-of-order counted");

  // Forward progress still works after a rejection.
  check(mb.accept(frame("e1", 11), 100.2) == AcceptResult::ACCEPTED,
        "newer sequence accepted after an out-of-order one");
}

// Rule 3: a restart rebases ordering but grants nothing.
static void test_epoch_change_rebases_sequence()
{
  CommandMailbox mb(CommandSource::AUTONOMY);
  mb.accept(frame("e1", 50000), 100.0);

  // Producer restarts: new epoch, sequence back to 1. Under the
  // old "seq must increase" rule alone this would be rejected
  // forever, and the source could never come back.
  check(mb.accept(frame("e2", 1), 101.0) == AcceptResult::ACCEPTED,
        "new epoch with a lower sequence is accepted");
  check(mb.snapshot().epoch == "e2", "snapshot took the new epoch");
  check(mb.snapshot().seq == 1, "sequence rebased");
  check(mb.epoch_change_count() == 1, "epoch change counted");

  // Ordering is enforced again within the new epoch.
  check(mb.accept(frame("e2", 1), 101.1) == AcceptResult::DUPLICATE,
        "duplicate detection works in the new epoch");
}

static void test_rejects_do_not_disturb_snapshot()
{
  CommandMailbox mb(CommandSource::RC);
  mb.accept(frame("e1", 5, true, 42.0), 100.0);

  check(mb.accept("total garbage", 100.5) == AcceptResult::REJECTED, "garbage rejected");
  check(mb.accept("v=9,producer=p,epoch=a,seq=1,source_time=0,valid=1,surge=0,yaw=0",
                  100.6) == AcceptResult::REJECTED, "bad version rejected");
  check(mb.last_reject_reason() == reject::kUnsupportedVersion,
        "reject reason token exposed for telemetry");

  check(std::fabs(mb.snapshot().surge - 42.0) < 1e-9,
        "snapshot survives rejected mail intact");
  check(std::fabs(mb.age(100.6) - 0.6) < 1e-9,
        "rejected mail does not refresh the lease");
  check(mb.reject_count() == 2, "rejects counted");
}

static void test_nonfinite_arrival_time_rejected()
{
  CommandMailbox mb(CommandSource::RC);
  check(mb.accept(frame("e1", 1), std::nan("")) == AcceptResult::REJECTED,
        "NaN arrival time rejected rather than stored");
  check(!mb.has_command(), "poisoned timestamp never becomes the snapshot");
}

// valid=0 is a live source reporting it cannot be trusted. That is
// different from stale, and the arbiter maps them to different
// stop reasons, so the mailbox must keep them distinguishable.
static void test_valid_flag_separate_from_freshness()
{
  CommandMailbox mb(CommandSource::RC);
  mb.accept(frame("e1", 1, /*valid=*/false), 100.0);
  check(mb.has_command(), "invalid frame is still a frame");
  check(mb.fresh(100.1, 1.0), "invalid frame is fresh");
  check(!mb.usable(100.1, 1.0), "invalid frame is not usable");
}

static void test_clock_step_backwards_is_not_stale()
{
  // NTP or a warp can step MOOSTime backwards. Negative age must
  // not read as a huge positive age and trip a false stop.
  CommandMailbox mb(CommandSource::RC);
  mb.accept(frame("e1", 1), 100.0);
  check(std::fabs(mb.age(99.0)) < 1e-9, "backwards clock clamps age to zero");
  check(mb.fresh(99.0, 0.5), "backwards clock does not read as stale");
}

static void test_zero_timeout_is_never_fresh()
{
  CommandMailbox mb(CommandSource::RC);
  mb.accept(frame("e1", 1), 100.0);
  check(!mb.fresh(100.0, 0.0), "zero timeout fails closed");
  check(!mb.fresh(100.0, -1.0), "negative timeout fails closed");
  check(!mb.fresh(100.0, std::nan("")), "NaN timeout fails closed");
}

int main()
{
  printf("test_command_mailbox\n");
  test_initial_state();
  test_accept_and_freshness();
  test_duplicate_does_not_refresh_lease();
  test_freshness_ignores_source_time();
  test_out_of_order_rejected();
  test_epoch_change_rebases_sequence();
  test_rejects_do_not_disturb_snapshot();
  test_nonfinite_arrival_time_rejected();
  test_valid_flag_separate_from_freshness();
  test_clock_step_backwards_is_not_stale();
  test_zero_timeout_is_never_fresh();

  if (g_failures) {
    fprintf(stderr, "\n%d/%d checks FAILED\n", g_failures, g_checks);
    return 1;
  }
  printf("PASS (%d checks)\n", g_checks);
  return 0;
}
