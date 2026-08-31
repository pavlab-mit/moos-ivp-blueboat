# handling_block — the deliberate maneuver block (plan §8.3)

The mission that produces the handling acceptance numbers the archives
could not: held heading steps, a zigzag, pivots, and a straight run,
each with exact setpoints and exact hold times, driven by
`uTimerScript` directly into `pBBPID`. **No helm.** The first on-water
run of the refactor (2026-08-27) proved the pipeline; this block turns
"feels great" into t63 / overshoot / peak-yaw-rate / cross-track
numbers comparable against the ArduRover baseline (plan §2.1).

## How it stays safe and stays in the box

- The script advances while autonomy **has the boat or is being
  offered it**: `(BB_CMD_AUTHORITY = AUTONOMY) or
  (BB_CMD_STOP_REASON = AUTONOMY_INVALID)`, both bridged from the
  front seat. The second clause is load-bearing — it breaks the
  bootstrap deadlock (pBBPID is not valid until this script posts,
  and the arbiter will not select AUTONOMY until pBBPID is valid;
  "CH6 in AUTO, nothing commanding" reads as `AUTONOMY_INVALID`,
  which is the go). **Flip CH6 to RC and the block pauses** — script
  clock frozen; reposition; flip back and it resumes mid-sequence.
- **Reposition with CH6, not the kill switch.** Kill neutralizes the
  props but leaves authority with AUTONOMY, so the script clock keeps
  running and that phase's data is garbage (rerun it by restarting
  the mission if it mattered). Kill remains the emergency stop.
- The DEPLOY button does nothing here — no helm, no
  uFldMessageHandler. CH6 is the only go-switch.
- Setpoints re-post every second, so `pBBPID`'s 1.5 s staleness gate,
  the arbiter's lease, and every fail-closed path work exactly as in
  alpha_pavlab. When the script pauses, commanding stops, autonomy
  goes `AUTONOMY_INVALID`, and RC has the boat — which it already did.
- No `pDeadManPost` / shoreside heartbeat: the RC operator is the
  safety authority for this mission. Do not run it beyond RC range.

## Two modes

| Mode | Command | Who drives | Scored with |
|---|---|---|---|
| **Session 0** — open-loop calibration | `./launch_vehicle.sh --rc-cal` | you, on RC, the whole run | `analysis/rc_cal.py` |
| **Session 1+** — the handling block | `./launch_vehicle.sh --base=150` | `uTimerScript` → `pBBPID` | `analysis/handling_block.py` |

Session order and what each session is allowed to change:
[`docs/tuning_playbook.md`](../../docs/tuning_playbook.md) §1.

## Running the block

```
./launch_vehicle.sh --base=150            # on the back seat, boat side
```

Identity (VNAME, front-seat IP) comes from `get_robot_info.sh` as in
alpha; override with `--vname=` / `--fseat=`. `--base=` regenerates the
maneuver script for the site's water: the block needs roughly a
300 m × 300 m box around headings 030–330 relative to base, and the
operator repositions between phases as needed (the pause makes this
free). Launch the front seat as usual (debug flag for the .alog).

Sequence (~5.4 min of autonomy time, pauses excluded):
warmup → steps +30/−60/+120 @ 0.5 m/s (25 s holds) → same @ 1.5 m/s →
±20° zigzag ×6 @ 1.0 m/s → two 180° pivots at speed 0 → 60 s straight
@ 1.5 m/s → stop.

## Scoring

```
analysis/handling_block.py <backseat>.alog
```

Prints the §8.3 table per phase: steps (t63, overshoot, peak yaw rate,
settled error), pivots (sustained yaw rate), straight (cross-track
RMS). Samples taken while RC held the boat are excluded automatically.

## Session 0: the RC calibration run card

```
./launch_vehicle.sh --rc-cal              # timer script INERT; RC only
```

The `--rc-cal` script is `paused = true` with zero events, so nothing can
command the boat under autonomy even if CH6 gets bumped mid-pivot. Front
seat launches as usual. **Leave CH6 in RC for the entire run** — the
scorer aborts if it finds any `BB_CMD_AUTHORITY=AUTONOMY` samples.

Before you start: **CH11 THRUST_LIMIT at 100%** and leave it there. It
scales applied effort, and while the scorer reads post-limit applied
values (so the math survives), moving it mid-run fragments the segments.

### The card (~7 min of water time)

| # | What | Sticks | Hold |
|---|---|---|---|
| 1 | straight, slow | left stick ahead ~⅓, right stick centred | 20 s |
| 2 | straight, medium | left ~⅔ | 20 s |
| 3 | straight, fast | left ~full | 20 s |
| 4 | pivot CW | **left stick centred**, right stick full right | 20 s (≥1 full turn) |
| 5 | pivot CCW | left centred, right full left | 20 s |
| 6 | pivot CW, softer | left centred, right ~⅔ right | 20 s |
| 7 | pivot CCW, softer | left centred, right ~⅔ left | 20 s |

Neutral both sticks for ~6 s between every item — that gap is what
segments the run. Tap **MARK (SH)** at the start of each item if you like;
`RC_MARK` is bridged to the back seat now, and it makes the log readable
by eye, but the scorer segments from applied effort and does not need it.

Three things that decide whether the run is usable:

- **Hold each item steady.** The exact stick position does not matter at
  all — the scorer reads the effort that actually came out. Steadiness
  does: it splits a segment when applied effort moves more than ~6%.
- **Left stick truly centred on the pivots.** It is spring-centred, so
  this is easy, but a leaning throttle turns a pivot into an arc and
  biases the answer. The scorer catches it: it re-derives `A_set` from
  the applied ratio and flags a >5% mismatch against `--asym`.
- **At least one full 360° per pivot.** Wind and current are world-fixed
  and average out of body-frame surge over a whole turn; below one turn
  they do not, and the scorer says so per pivot.

Three distinct straight speeds and both pivot directions are the minimum.
Repeat a pivot pair if the day is gusty.

### Scoring

```
analysis/rc_cal.py <backseat>.alog --asym 1.6
```

Prints the drag curve, a row per pivot, and one recommendation:
`set thrust_asymmetry = <A>`. Also splits out a CW-vs-CCW check, which is
**not** `A` — a rate split over 10% is motor, prop or ESC mismatch and
wants investigating before the `A` number is trusted.

Apply it in
`missions/blueboat_frontseat/plugs/blueboat_fs/plug_pThrustMix.moos`.
It is front-seat config, not live-settable, so it takes effect on the
next front-seat start — which is why this is measured once rather than
swept (playbook §2).

The identity being inverted, and why body-frame surge drift is the right
observable, are documented at the top of `analysis/rc_cal.py`.
`analysis/test_rc_cal.py` checks the recovery against synthetic logs with
known asymmetry, including the null case.

**Acceptance is deferred to session 1:** with the new `A` in place, the
block's pivot phases should show drift ≈ 0.

## Regenerating / tuning the block

`gen_maneuver.py --base <deg> [--slow 0.5 --fast 1.5 --hold 25]`
(or `--rc-cal` for the inert session-0 script)
rewrites `plugs/plug_uTimerScript.moos` (the launcher does this every
launch). The generated plug is a build artifact — edit the generator,
not the plug. Boat-specific config (pBBPID gains, broker, EKF) is
included from `../alpha_pavlab_asv/plugs/` so it lives in one place.
