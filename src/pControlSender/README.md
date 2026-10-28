# pControlSender

Puts the barrier filter between pBBPID and the arbiter.

```
pBBPID ──AUTONOMY_CMD_PRECBF──────────────────────────────► pControlSender ──AUTONOMY_CMD──► pBBCommandArbiter
       ──DESIRED_THRUST_PRECBF / DESIRED_RUDDER_PRECBF──► filter ──DESIRED_THRUST / DESIRED_RUDDER──┘
```

`AUTONOMY_CMD` is pBBPID's frame re-sent with the filter's thrust/rudder swapped
in. Epoch, seq, `source_time`, `valid`, `authority_limit` and every extra field
are pBBPID's. Only three things change: `surge`/`yaw`, `producer`, and a `cbf=1`
marker.

pControlSender never creates a sequence number. The arbiter only renews
autonomy's lease on a *new* seq, so a dead pBBPID still stops the boat exactly as
it does today.

## Files

| File | What it is |
|---|---|
| `ControlSenderCore.h/.cpp` | All the rules. No MOOS; unit tested. |
| `ControlSender.h/.cpp` | The MOOS app: mail in, one decision per Iterate, publication out. |
| `ControlSender_Info.cpp` | `-h`, `-e`, `-i` text. |
| `test/test_control_sender_core.cpp` | Unit tests, including runs through the real `CommandMailbox` and `AuthorityArbiter`. |
| `plug_pControlSender.moos` | Config block to include in the mission. |

## Setup

1. **pBBPID.** No code change. Add three lines to its config block:
   ```
   thrust_var       = DESIRED_THRUST_PRECBF
   rudder_var       = DESIRED_RUDDER_PRECBF
   autonomy_cmd_var = AUTONOMY_CMD_PRECBF
   ```
2. **Build.** Add `ADD_SUBDIRECTORY(pControlSender)` to the `src` CMakeLists,
   after `lib_bb_command`. The app links `bb_command`.
3. **Tests.** Run `./build.sh --unit_tests`, then `ctest -R test_control_sender_core`.
4. **Launch.** Include `plug_pControlSender.moos` and add pControlSender to
   pAntler, on the same computer as pBBPID and the filter.

## What the filter must do

- **Publish thrust and rudder together.** Send `DESIRED_THRUST` and
  `DESIRED_RUDDER` back to back in the same cycle, with the default (current)
  timestamp. pControlSender only pairs two values whose publisher timestamps are
  within `pair_tolerance_sec` (20 ms).
- **Use contract units.** Surge is positive forward; yaw is positive to
  starboard. Both are in [-100, 100]; anything outside is clamped, and the clamp
  is counted.
- **When the filter's own timeout fires, stop publishing. Don't publish zeros.**
  - Silence is turned into a labelled stop: within `filter_timeout_sec` (0.5 s),
    `valid=0` with `cbf_fault=filter_silent`.
  - Zeros would be sent as a *valid* zero command. The arbiter keeps autonomy
    selected and the mixer ramps throttle down, instead of stopping the boat.
- **Optional: publish the seq it filtered.** Put the `seq` of the
  `AUTONOMY_CMD_PRECBF` frame it used into `CBF_SEQ`, in the same cycle as the
  two values, and set `filter_seq_var = CBF_SEQ`. Each filtered value is then
  sent under exactly the frame it came from.
  - Without it, each output goes to the newest frame that had arrived when the
    output did. That's exact while the filter answers within pBBPID's 100 ms
    tick, and up to one tick late otherwise.
  - To know the seq, the filter has to read `AUTONOMY_CMD_PRECBF`.
- **Never write `AUTONOMY_CMD`.**

## Behaviour

| Situation | pControlSender sends | Arbiter |
|---|---|---|
| Normal | pBBPID's frame with filtered values | AUTONOMY drives |
| pBBPID holding off, or helm stale | pBBPID's `valid=0` frame, unchanged | `AUTONOMY_INVALID` (as today) |
| One filter output late or missing | nothing for that tick (seq skipped) | keeps the last command (lease 2.0 s) |
| Filter silent > `filter_timeout_sec` | `valid=0`, zeros, `cbf_fault=filter_silent` | `AUTONOMY_INVALID` |
| Filter's seqs match no frame (seq mode) | `valid=0`, `cbf_fault=filter_unmatched` | `AUTONOMY_INVALID` |
| Valid frames before the filter has ever spoken | `valid=0`, `cbf_fault=filter_never_heard` | `AUTONOMY_INVALID` |
| pBBPID dead | nothing | `AUTONOMY_STALE` after 2.0 s (as today) |
| pControlSender dead | nothing | `AUTONOMY_STALE` after 2.0 s |
| pBBPID still on its default `autonomy_cmd_var` | nothing; run warning; `ALL_STOP=true` | `AUTONOMY_ALL_STOP` |

pControlSender never falls back to pBBPID's unfiltered values. If the filter
dies, the boat runs on the last filtered command for up to `filter_timeout_sec`
before it stops.

## Telemetry

| Variable | Meaning |
|---|---|
| `CTLSEND_STATE` | `WAITING_FOR_PRECBF`, `FILTERED`, `FORWARD_INVALID` or `FILTER_STALE`, posted on change |
| `CTLSEND_LATENCY` | delay from pBBPID's tick to the send, in s. All on the backseat clock, so this is the real delay the filter path adds. |
| `CTLSEND_SURGE_DELTA`, `CTLSEND_YAW_DELTA` | filtered value minus pBBPID's: how much the filter changed |
| `CTLSEND_SEQ_LAG` | newest pBBPID seq minus the seq just sent |
| `CTLSEND_FILTER_AGE` | seconds since the last filter output |
| `CTLSEND_FOREIGN_WRITER` | producer and epoch of anything else writing `AUTONOMY_CMD` |

For a per-tick before/after comparison, match `AUTONOMY_CMD_PRECBF` and
`AUTONOMY_CMD` frames on `seq` in the alog.

## Bench checklist

1. **Identity filter.** Copy `*_PRECBF` to `DESIRED_*` in one cycle.
   - `CTLSEND_STATE` should read `FILTERED`.
   - `AUTONOMY_CMD` should match `AUTONOMY_CMD_PRECBF` seq for seq, apart from
     `producer` and `cbf=1`. The deltas should be 0.
   - Note `CTLSEND_LATENCY`.
2. **Kill the filter.** Within about 0.5 s, `CTLSEND_STATE` should read
   `FILTER_STALE` and `BB_CMD_STOP_REASON` should read `AUTONOMY_INVALID`.
   Restart it and it should return to `FILTERED`.
3. **Kill pBBPID.** No more `AUTONOMY_CMD`, then `AUTONOMY_STALE` at 2.0 s.
4. **Kill pControlSender.** `AUTONOMY_STALE` at 2.0 s. Restart it while pBBPID is
   down: nothing should be sent. The replayed old mail is dropped.
5. **Stop the helm.** You should see `FORWARD_INVALID` and `AUTONOMY_INVALID`.
6. **Bypass check.** Run pBBPID with its default `autonomy_cmd_var`. You should see
   `CTLSEND_FOREIGN_WRITER`, `ALL_STOP`, and `AUTONOMY_ALL_STOP`.
7. **If using `CBF_SEQ`:** `CTLSEND_SEQ_LAG` should stay small and steady.

## Known limits

- **pBBPID can wind up.** It doesn't know when the filter holds it back, so
  watch `BBPID_*_ITERM` during interventions.
- **Frontseat logs only show the producer.** Extra fields don't pass the
  arbiter, so `BB_SELECTED_CMD`/`BB_MIXED_CMD` show only `producer=pBBPID_CBF`
  and pBBPID's epoch/seq. To see the frame details, look up that seq in the
  backseat `AUTONOMY_CMD` log.
- **`ALL_STOP` has to reach the arbiter.** The bypass guard posts `ALL_STOP` in
  the backseat, so check that the bridge forwards it.
- **The old-mail check assumes one clock.** It compares publisher timestamps
  with local time, which is sound only because pBBPID, the filter and
  pControlSender share the backseat computer.
