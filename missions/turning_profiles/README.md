# turning_profiles — turning-capability characterization

Measures the turning radius the BlueBoat achieves at a set of speed
setpoints under two pBBPID **capability profiles**, delivered as plug
blocks. Helm-driven: `BHV_LegRun` carries the boat to each trial at the
trial speed; `BHV_FixedTurn` executes a 355° turn at a 90° heading offset
(a maximum-rate turn) and posts `FT_REPORT`. The whole schedule is static
in a generated `.bhv` plug, so there is no custom sequencing app, nothing
waits on a speed-validity gate, and nothing depends on another repository.
Scored offline by `analysis/turn_radius.py`.

Stock behaviors only (moos-ivp Aug-2026 or later, for `park_on_allstop`).
Boat plugs (broker, EKF, health, logger, …) are included from
`../alpha_pavlab_asv/plugs`; pBBPID is a local copy with the profile blocks
on top, and the launcher refuses to fly if the copy has drifted from alpha.

## The two profiles (`plugs/plug_pBBPID.moos`)

| | `agile` | `slow` |
|---|---|---|
| `max_yawrate` | 3.0 rad/s (172°/s; never binding, the turn is rudder-saturated = the hull's real maximum) | 0.10 rad/s (5.7°/s) |
| `yaw_priority_gain` | 0 (governor off, speed held through the turn) | 0 |
| everything else | alpha's converged plug, verbatim | identical |
| expected min radius v/ω at 0.75 / 1.0 / 1.25 / 1.5 m/s | hull-limited, roughly 2–4 m | 7.5 / 10 / 12.5 / 15 m |

Neither profile forbids reverse at the motor: the front-seat skid mixer
reverses the inside motor whenever yaw demand exceeds surge, and that has
no off switch. `allow_reverse` (common surge floor) stays true in both; on
the 31 Aug grid run it bit on 0.7% of ticks while a motor was negative on
16.6%, so it is not a profile axis. The scorer reports the motor-reverse
fraction per turn so the profile description can say what actually happened.

**Why `max_yawrate` is first in the block:** MOOS hands pBBPID its config
lines with the *first* occurrence of a key winning. The profile and SIM
overrides therefore sit at the top of the block, above alpha's verbatim
body. Putting them after it silently does nothing (found in sim: the
controller kept subscribing to `NAV_SURGE` / `GPS_HEADING_DGNSS`).

## Running

```
# on the boat (identity from get_robot_info.sh, as alpha)
./launch_vehicle.sh --profile=agile                 # 4 speeds x 2 trials, starboard
./launch_vehicle.sh --profile=slow
# shoreside (laptop)
./launch_shoreside.sh

# sim on a laptop (actuator-mode uSimMarineV22 + the real pBBPID)
./launch.sh --profile=slow --speeds=1.0,1.5 --trials=1 10
# headless end-to-end check, scored: PASS/FAIL
./sim_test.sh --profile=agile 10
```

Launcher options: `--speeds=`, `--trials=`, `--dir=star|port`,
`--mod_hdg=`, `--leg=x,y,len,ang`, `--turn_rad=`, `--timeout=`,
`--check_r=`, `--margin=`, `--sync-plug`, `--just_make`, `--sim`.

Defaults: speeds 0.75,1.0,1.25,1.5 · 2 trials · starboard · leg centred
(45,-40) heading 065, 40 m long (the relocated `blueboat_dynamics` track
area, op region from commit 8632255). Per profile: end-turn radius 7/18 m,
turn timeout 75/180 s, clearance check radius 6/16 m. The generator prints
every turn circle's clearance to the op region and refuses to write the
schedule if any is under 12 m.

Time on the water (8 turns each): agile ≈ 12 min, slow ≈ 25 min of
autonomy, plus repositioning.

## On the water

1. Front seat up, back seat launched with the profile, CH6 in RC.
2. **Start the boat beyond either END of the leg, pointing roughly along
   it** (the first leg runs to the far end, so the boat gets a full
   approach leg). Starting beside the midpoint fires the first turn at
   once, before the speed has settled.
3. DEPLOY on the shoreside, then CH6 to AUTO.
4. Each trial: leg at the trial speed → 355° turn to starboard at the
   midpoint → back onto the leg → end turn → next leg. The last turn flags
   `RETURN=true`.
5. **RC takeover mid-turn**: the fixed turn is conditioned on
   `BB_CMD_AUTHORITY = AUTONOMY` (field only), so taking RC idles it; on
   hand-back it restarts from the current heading as a fresh 355° turn.
   Hand back near an end of the leg, not mid-leg, or the leg run's mid-leg
   trigger fires the next turn before the boat has settled. The scorer
   flags any window that was not 100% autonomy and scores only its last
   autonomy segment.
6. Watch the battery yourself. There is no low-voltage return here.
7. Kill switch remains the emergency stop; reposition with CH6.

Switching profile = relaunch the back seat with the other `--profile=`.
(Both knobs are also live via `BBPID_SET`, and `BBPID_PARAMS_ACTIVE` records
either way, but a relaunch keeps the `._moos` snapshot honest.)

## Scoring

```
analysis/turn_radius.py <backseat>.alog            # fixed-turn windows
analysis/turn_radius.py <backseat>.alog --legrun   # BHV_LegRun end turns instead
analysis/turn_radius.py <alog> --json out.json
```

Per turn: `R_fit` (least-squares circle over the steady part of the turn,
after 60° of heading change), `R_kin` (mean surge / mean turn rate),
`R_ft` (BHV_FixedTurn's own avg [min–max]), mean/min/entry speed, turn
rate, rudder p50/p90 and saturated fraction, mixer-saturation fraction,
motor-reverse fraction, autonomy fraction, turn time, heading change; then
a per-setpoint capability table. The profile is read from the `._moos`
snapshot overlaid with `BBPID_PARAMS_ACTIVE`.

Validated on the 31 Aug `blueboat_dynamics` log with `--legrun`: the
commanded 5 / 7 / 9 m LegRun turns score 4.8–5.0 / 7.0–7.3 / 9.0–9.3 m.

## Files

| | |
|---|---|
| `gen_turn_schedule.py` | writes `plugs/plug_turn_schedule.bhv` (LegRun + FixedTurn) and `plugs/plug_opregion.bhv`; geometry check |
| `plugs/plug_pBBPID.moos` | profiles + SIM overrides on top of alpha's verbatim body |
| `plugs/plug_pHelmIvP.moos` | 4 Hz, `park_on_allstop = false` (required by BHV_FixedTurn) |
| `plugs/plug_uSimMarineV22.moos` | actuator mode (no embedded PID) |
| `plugs/plug_uFldNodeBroker.moos` | alpha's bridges + `FT_REPORT`, `LR_MODE_REPORT`, `MODE`, `BBPID_PARAMS_ACTIVE` |
| `sim_test.sh` | headless launch → DEPLOY → wait for every FT_REPORT and RETURN → ktm → score |

Supersedes, for this purpose, `moos-ivp-blueboat-exp/missions/calibration`
(pVelRadCalibration/uSpeedSender from moos-ivp-aarjav): that mission gates
and retries trials on speed-validity thresholds the governor trips, and
was never flown on the new stack.
