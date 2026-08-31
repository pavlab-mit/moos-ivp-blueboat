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

### The card (~10 min of water time)

**Straights** (drag curve, backup + speed-loop data). Right stick centred.

| # | left stick | hold |
|---|---|---|
| 1 | crawl, just off centre (~10%) | 20 s |
| 2 | ~20% | 20 s |
| 3 | ~35% | 20 s |

**Null ladders** — the primary measurement. Hold the right stick at a
steady **28%**, then step the left stick through the ladder, 20 s per rung.
At zero drift the two sides' delivered thrust cancels exactly:

```
L + R/A_true = 0        ->        A_true = |applied_R| / |applied_L|
```

No drag curve, and no spin-drag term (zero velocity is zero drag whatever
the drag law) — which is what went wrong on 31 Aug.

At `thrust_asymmetry = 2.8` the ladder straddles the expected null, so it
runs **both ahead and astern** of centre:

| throttle | L% | R% | ratio | expect |
|---|---|---|---|---|
| +8% | 36.0 | −56.0 | 1.56 | drifts aft |
| +5% | 33.0 | −64.4 | 1.95 | drifts aft |
| +3% | 31.0 | −70.0 | 2.26 | drifts aft |
| 0 | 28.0 | −78.4 | 2.80 | near null |
| −3% | 25.0 | −86.8 | 3.47 | drifts fwd |
| −5% | 23.0 | −92.4 | 4.02 | drifts fwd |

Stop at −5%: −8% saturates the mixer and the ratio stops being clean.
Fly the ladder **CW, then CCW** — the GPS lever arm is `+omega*d` one way
and `-omega*d` the other, so the two directions cancel it and agreeing on
the null is a free cross-check.

Regenerate the ladder for a different `A_set` or steering with the mixer
port in `analysis/test_rc_cal.py` (`mixer()`), which is bb::allocate.

Neutral both sticks ~6 s between every item — that gap is what segments the
run. Tap **MARK (SH)** at the start of each item if you like; `RC_MARK` is
bridged now and makes the log readable by eye, though the scorer segments
from applied effort and does not need it.

**The one thing that decides whether the run is usable: you must see the
drift change sign across the ladder.** The method interpolates a zero
crossing between the two rungs that straddle it. If every rung still creeps
forward, keep trimming astern; if every rung creeps back, trim ahead. An
unbracketed ladder is extrapolation, which is the failure this run exists
to escape — the scorer flags it as `NOT BRACKETED` rather than quietly
reporting a number.

Exact stick values do not matter; the scorer reads what actually came out.
Steadiness does — it splits a segment when applied effort moves ~6%.

Before you start: **CH11 THRUST_LIMIT at 100%** and leave it there.

> **Expect slower pivots, and do not read that as a fault.** Raising `A`
> shrinks the mixer's feasible region (max steering at zero throttle is
> `1/A_set`), so a full-stick pivot goes from (62.5, −100) at 1.6 to
> (35.7, −100) at 2.8 — about 27% less yaw authority. The 31 Aug run
> pivoted at ~44 deg/s; expect ~32 deg/s at 2.8, still clear of the
> 25 deg/s bar. Correcting `A` buys drift-free pivots and costs peak turn
> rate.

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
