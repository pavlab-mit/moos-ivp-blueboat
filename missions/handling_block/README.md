# handling_block — the deliberate maneuver block (plan §8.3)

The mission that produces the handling acceptance numbers the archives
could not: held heading steps, a zigzag, pivots, and a straight run,
each with exact setpoints and exact hold times, driven by
`uTimerScript` directly into `pBBPID`. **No helm.** The first on-water
run of the refactor (2026-08-27) proved the pipeline; this block turns
"feels great" into t63 / overshoot / peak-yaw-rate / cross-track
numbers comparable against the ArduRover baseline (plan §2.1).

## How it stays safe and stays in the box

- The script's `condition` holds it to `BB_CMD_AUTHORITY = AUTONOMY`
  (bridged from the front seat). **Flip CH6 to RC and the block
  pauses** — script clock frozen; reposition the boat; flip back and
  it resumes mid-sequence. Kill and (if enabled) deadman are untouched.
- Setpoints re-post every second, so `pBBPID`'s 1.5 s staleness gate,
  the arbiter's lease, and every fail-closed path work exactly as in
  alpha_pavlab. When the script pauses, commanding stops, autonomy
  goes `AUTONOMY_INVALID`, and RC has the boat — which it already did.
- No `pDeadManPost` / shoreside heartbeat: the RC operator is the
  safety authority for this mission. Do not run it beyond RC range.

## Running

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

## Regenerating / tuning the block

`gen_maneuver.py --base <deg> [--slow 0.5 --fast 1.5 --hold 25]`
rewrites `plugs/plug_uTimerScript.moos` (the launcher does this every
launch). The generated plug is a build artifact — edit the generator,
not the plug. Boat-specific config (pBBPID gains, broker, EKF) is
included from `../alpha_pavlab_asv/plugs/` so it lives in one place.
