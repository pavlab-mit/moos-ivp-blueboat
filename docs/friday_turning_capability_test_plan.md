# Friday on-water test plan: controller wrap-up + two turning-capability profiles

Status: 2026-10-14, mission built and sim-verified; for the next Friday session (yip)
Companions: [`tuning_playbook.md`](tuning_playbook.md) (campaign record, session 6 still open),
[`control_checkpoint_brief.md`](control_checkpoint_brief.md) §5 (open items),
`~/moos/moos-ivp-blueboat-exp/missions/calibration/README` (the fixed-turn mission)

Two goals, in priority order:

1. **Close the controller.** The 31 Aug campaign converged (A = 1.77, `ff_yaw 0,45,80`,
   `ff_speed 0,9.2,30,10.1`, `des_yawrate_filter 0.5`; feedback gains never moved).
   What remains is confirmation, not tuning.
2. **Deliver two vehicle profiles as plugs**, each with a measured table:
   speed setpoint → achievable turning radius.

---

## 1. Where things stand (what the repos and logs say)

- **Controller tuning is done.** Standing gains are the 27 Aug set; the campaign was
  calibration plus one shaping knob. Acceptance bar met on `LOG_YIP_31_8_2026_____14_01_57`.
- **Open controller items** (none need a tuning session):
  - `des_yawrate_filter = 0.5` passed with thin margin (30° t63 1.8 s vs the 2.0 s bar). Re-confirm.
  - Tier-2 policy never evaluated (playbook session 6): governor
    (`yaw_priority_gain 0.5`, `min_speed_frac 0.5`), `steering_throttle_mix 0.6`, gain schedule.
    The 31 Aug grid run shows the governor **is** active in turns: at a 1.0 m/s setpoint
    the boat fell to 0.50 m/s at turn entry (p50 0.89 through the turn).
  - `handling_block.py` speed metrics still read `NAV_SPEED`; should read `NAV_SURGE`. Offline.
  - CW/CCW pivot-rate split (30%) was flagged at starved yaw rates; re-read it from the
    session-4 log at healthy rates. Offline, no water needed.
  - `pBBDynamicsMissionManager` posts `FAKE_RETURN`, not `RETURN`, at 22 V. The low-battery
    return is a no-op; the operator owns the voltage watch.
- **Two candidate missions exist:**

  | Mission | What it measures | Water-proven on new stack? |
  |---|---|---|
  | `moos-ivp-blueboat/missions/blueboat_dynamics` | LegRun grid: commanded radius 5/7/9 m × speed 1.0/1.2/1.4; 2 turns per point | **Yes**, 31 Aug run 3: 5+ grid points, 1250 s autonomy, no faults |
  | `moos-ivp-blueboat-exp/missions/calibration` | 360° fixed turns at a hard heading offset → **minimum** radius at each speed; `FT_REPORT` avg/min/max radius, `TURN_REPORT` speed-validity | **No.** Ported 2026-09-18, no field logs. Apps live in `moos-ivp-aarjav` (`pVelRadCalibration`, `uSpeedSender`) |

  The deliverable ("at speed v we can achieve radius x") is the **fixed-turn** mission. The
  LegRun grid answers a different question (does the boat track a commanded 5/7/9 m arc)
  and is the fallback if the fixed-turn mission is not ready on the boat.

- **Plug drift to know about.** `moos-ivp-blueboat-exp/missions/blueboat_dynamics/plugs/blueboat/plug_pBBPID.moos`
  is **pre-campaign** (ff_yaw 0,20,100, no `NAV_SURGE`, no lifecycle fixes, `max_yawrate 0.3`).
  Do not fly it. The exp `calibration` plug and the main-repo `blueboat_dynamics` plug are
  both identical to alpha's converged plug.
- **Main repo has untracked work:** `analysis/`, `docs/*.md`. Commit before Friday so the
  scorers and the campaign record travel with the logs.

---

## 2. The two profiles (decided 2026-10-14: aggressive split)

Implemented in `missions/turning_profiles/plugs/plug_pBBPID.moos`, selected
with `launch_vehicle.sh --profile=agile|slow`.

| | Profile `agile` | Profile `slow` |
|---|---|---|
| `max_yawrate` | **3.0 rad/s** (172°/s, never binding: the turn is rudder-saturated, i.e. the hull's real maximum) | **0.10 rad/s** (5.7°/s) |
| `yaw_priority_gain` | 0 (governor off: speed held through the turn) | 0 |
| everything else | alpha's converged plug, verbatim | identical |
| predicted R_min = v / ω | hull-limited, ~2–4 m | 7.5 / 10 / 12.5 / 15 m at 0.75 / 1.0 / 1.25 / 1.5 m/s |

- `allow_reverse` stays true in both. It floors only the common surge command
  (0.7% of ticks on the 31 Aug grid run) while the front-seat skid mixer
  reversed a motor on 16.6% of ticks regardless, and the mixer has no option
  to forbid that. Per your rule nothing was added; the scorer reports the
  motor-reverse fraction per turn so each profile's description states what
  actually happened at the motors.
- Governor off in both so the pair differs in exactly one limit. With the
  governor on, the 31 Aug run shed speed to 0.5 m/s at turn entry.
- Gotcha found in sim: MOOS hands pBBPID its block with the FIRST occurrence
  of a key winning, so the profile lines sit at the top of the block.

---

## 3. The mission (built and sim-verified 2026-10-14)

`missions/turning_profiles/` in moos-ivp-blueboat (README there). Stock
`BHV_LegRun` + `BHV_FixedTurn` with a generated static schedule: one leg per
turn at the trial speed, a 355° turn at a 90° heading offset at the midpoint,
`FT_REPORT` per turn, the last turn flags `RETURN`. No pVelRadCalibration /
uSpeedSender, so no dependency on moos-ivp-aarjav and no speed-validity
gate to trip. Boat plugs are included from alpha; the pBBPID copy is
drift-checked against alpha at every launch.

Hardening for the real-world failure modes:

- Helm `park_on_allstop = false` (BHV_FixedTurn leaves one idle iteration
  per completion; newer helms park on it) and 4 Hz (pBBPID's 1.5 s stale gate).
- Turn completion by accumulated heading **or** a per-turn timeout, so a
  heading transient or a slow turn can never stall the schedule.
- Field-only: the fixed turn is conditioned on `BB_CMD_AUTHORITY = AUTONOMY`,
  so an RC takeover idles it and it restarts as a fresh turn on hand-back.
- Op-region clearance of every turn circle and end turn is computed from the
  same polygon the op-region plug is written from; the launcher refuses a
  schedule with less than 12 m to spare (slow profile at 1.5 m/s has 15 m).
- First leg always runs to the far end (`far_turn`) so there is a full
  approach leg whichever end the operator starts from.

Sim shakeout (`sim_test.sh`, pBBPID in the loop, actuator-mode uSimMarineV22):
both profiles PASS end to end; the slow profile turns at 5.8°/s for 8.8 m at
1.0 m/s and 13.5 m at 1.5 m/s, i.e. the cap defines the envelope as intended.

Scorer `analysis/turn_radius.py` validated on the 31 Aug field log: LegRun
turns commanded at 5 / 7 / 9 m score 4.8–5.0 / 7.0–7.3 / 9.0–9.3 m, with
mixer saturation, motor reverse (39–45% of ticks) and authority populated.

---

## 4. Pre-Friday checklist

1. **Commit** `missions/turning_profiles`, `analysis/turn_radius.py` and the
   untracked `docs/` + `analysis/` so the boat's boot-time pull has them.
2. **Boat inventory (ssh yip-bb, was unreachable from here on 2026-10-14):**
   `pHelmIvP` is an Aug-2026+ build (`park_on_allstop` accepted, no config
   warning in the appcast); front seat runs the build with
   `thrust_asymmetry = 1.77`; confirm which front-seat tree is live
   (navigator-cpp migration shakeout was pending). If the fs build changed
   since 31 Aug, block 1 of the run card is mandatory.
3. `./launch_vehicle.sh --profile=agile --just_make` **on the boat** (real
   `get_robot_info.sh`): nsplug clean, drift check passes, return point sane.
4. Offline controller items: CW/CCW split from the session-4 log;
   `handling_block.py` speed metrics → `NAV_SURGE`.
5. Optional: run the two profiles in sim once more on the day
   (`./sim_test.sh --profile=slow 10`) after pulling.

---

## 5. On-water run card (yip)

| # | Block | Launch | Min | Pass / what you get |
|---|---|---|---|---|
| 0 | Launch + handoff | fs up; `turning_profiles --profile=agile`; CH6 RC. DEPLOY, then CH6 AUTO: `AUTONOMY` within seconds; first leg turns the commanded way | 5 | pBBPID appcast clean, `BBPID_GATE` OPEN |
| 1 | Controller confirmation (goal 1) | `handling_block --base=150`, verbatim gains | 8 | matches 31 Aug session 4 (30° t63 ≤ 2 s, ovs ≤ 10°, pivots ≥ 25°/s, drift within ±0.15). Skip only if the boat is byte-identical to 31 Aug |
| 2 | **agile** | `--profile=agile` (defaults: 0.75–1.5 m/s, 2 trials, starboard) | 12 | 8 `FT_REPORT`s, RETURN at the end |
| 3 | **slow** | relaunch back seat `--profile=slow` | 25 | 8 `FT_REPORT`s; the turns are big (up to 15 m radius): watch the op-region margin on the return-leg turns |
| 4 | Port repeat | `--dir=port --trials=1` for each profile, as time allows | 15 | CW/CCW radius split = motor match, not a profile property |
| 5 | Return, pull logs | bs + fs logs with `._moos` | — | |

Operator notes: start beyond an END of the leg, pointing along it, before
DEPLOY. Reposition with CH6, not the kill switch. Hand AUTO back near a leg
end, not mid-leg. No low-voltage return in this mission; watch
`NVGR_ROLLING_VOLTAGE`. Stay in RC range.

---

## 6. Deliverable format

One table per profile plug, straight from `analysis/turn_radius.py`'s
capability table:

| Setpoint (m/s) | Mean surge in turn (m/s) | R circle-fit (m) | R FT_REPORT avg [min–max] (m) | Turn rate (°/s) | Rudder p90 (%) | Mixer sat (%) | Inside-motor reverse (% ticks) | Clean trials |
|---|---|---|---|---|---|---|---|---|
| 0.75 | | | | | | | | n/N |
| 1.00 | | | | | | | | |
| 1.25 | | | | | | | | |
| 1.50 | | | | | | | | |

Report the circle-fit radius as the number; `FT_REPORT` is the cross-check (it averages
180 opposite-heading point pairs, so it is sensitive to drift during the turn). Radius is of
the hull reference point: yip's EKF is lever-arm corrected (`gps_lever_y = -0.356`). State
the mean surge beside every radius, because with the governor on the boat does not turn at
the setpoint speed, and that is the honest capability statement.

Each profile ships as: its `#ifdef PROFILE` block in
`missions/turning_profiles/plugs/plug_pBBPID.moos`, one sentence on what it
constrains and what it does not (`slow`: "yaw rate ≤ 5.7°/s, speed held;
individual motors may still reverse via the mixer"), and its table.

---

## 7. Go / no-go and fallbacks

- **No-go** if block 1 regresses or if `AUTONOMY` handoff fails at the dock.
- **Fallback if `turning_profiles` misbehaves on the boat** (it has only run in
  sim): `blueboat_dynamics` with `min_radius 3, max_radius 9, interval 2` and
  speeds 1.0/1.4 for each profile; score with `turn_radius.py --legrun`. The
  smallest commanded radius the boat still tracks is that profile's envelope.
- **If the slow profile's return-leg turns run close to the op region**, rerun
  with `--speeds=0.75,1.0,1.25` or move the leg south with `--leg=`.
