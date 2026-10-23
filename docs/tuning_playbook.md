# BlueBoat Tuning Playbook

Status: revised 2026-08-31 (tier structure + session order); infrastructure
ready 2026-08-28; first evidence session pending
Companions: [`control_checkpoint_brief.md`](control_checkpoint_brief.md) §4 (the strategy),
[`bb_pid_primitive.md`](bb_pid_primitive.md) (the engine being tuned)

The brief's rule: **one knob-set per block run, the scorer as referee.**
This document maps every tunable to the evidence column that informs it,
and describes the loop that collects that evidence in ~7 minutes of
autonomy time per iteration.

---

## 0. Read this before you plan a session

### 0.1 The 27 Aug run is a plumbing baseline, not a tuning baseline

`missions/alpha_pavlab_asv/logs/handling/LOG_YIP_27_8_2026_____16_59_38`
proved the pipeline. It does **not** establish tunable numbers. Re-scored:

| Phase group | Usable as tuning evidence? |
|---|---|
| stepA/B ×6, zig 2–6 | t63 / ovs / peak / rateRMS yes; **`settled` no** (FF held-derivative bias, ~6°) |
| zig_1 | no — t63 45.9 s across 2 autonomy windows |
| pivot ×2 | **no** — 3.1 and 5.0 °/s at \|rud\| 4.2 / 8.7. That is the FF sign failure, not the hull |
| straight | **no** — 2 autonomy windows, resume windup; mean thrust −7.8% while 0.46 m/s *under* command |
| speed staircase | did not exist (17 phases then, 23 now) |

Ten valid transients, zero valid `settled`, zero valid pivot, zero valid
straight, zero speed data. In particular the pivot line reads like damning asymmetry evidence
(rate CW 3.1 vs CCW 5.0 °/s, a 37% split) and is **noise on top of a
broken phase**. Do not tune on it. `handling_block.py` now splits that
readout in two — common-mode body-surge drift for `thrust_asymmetry`,
CW/CCW rate split for motor match — because reporting them as one number
is what made a broken phase look like a calibration finding.

Consequence: the first block run of the new engine is a *re-baseline*, at
verbatim gains. It is not session 1 of tuning; it is session 1 of having
numbers at all.

### 0.2 The axis is calibration vs tuning, not top-down vs bottom-up

Three tiers. Work them in order; do not interleave.

- **Tier 0 — plant constants.** Measured open-loop, then frozen:
  `thrust_asymmetry`, `ff_speed`, `ff_yaw`, the real achievable yaw rate.
  These are not knobs. Iterating on them with the closed-loop block is a
  category error — you would be tuning a plant model with a controller in
  the way.
- **Tier 1 — feedback gains.** `yawrate_pid` → `heading_pid` →
  `speed_pid`, inner to outer, block as referee.
- **Tier 2 — policy.** `steering_throttle_mix`, the governor
  (`yaw_priority_*`, `min_speed_frac`), `enable_gain_schedule`. These
  encode *what to sacrifice under saturation* — decisions, not plant
  facts. Genuinely last, and only with the saturation telemetry live.

The two ArduRover mixer constants live in the same config struct and in
**different tiers**. `thrust_asymmetry` changes the plant the loops see,
therefore it goes first. `steering_throttle_mix` is a saturation policy,
therefore it goes last. Earlier revisions of this document put both at
the bottom under one rationale; that was wrong for the first of them.

### 0.3 What the baseline already tells us about tier 0

Regressing the steady segments of the 27 Aug log (desired yaw rate
constant ≥3 s, autonomy only) — `analysis/handling_block.py` does not
print this; it was computed directly off the log:

```
at the 25 deg/s rail:  mean |rud| 58.4%   mean achieved 18.3 deg/s   n=155
  -> plant needs ~183 %/(rad/s) of rudder to hold a yaw rate
  -> ff_yaw supplies 20 + 30*v  =  57.5 %/(rad/s) at v=1.25   (~3x low)
                                =  20   %/(rad/s) at pivot    (~9x low)
```

Two consequences fall out of that one number:

- **The boat never reaches its commanded rate.** Commanded 25 °/s,
  achieved 18.3 °/s, with 42% rudder unused and `rudSat = 0%` everywhere
  in the run. Nothing saturates; the loop simply is not asking.
- **The inner loop's DC gain is ~0.33.** `yawrate_pid` kp=60 against a
  plant needing ~183 means P alone can deliver a third of the required
  authority, so `ki=40` carries the rest — at 4.7 %/s it needs ~12 s to
  close, and a 120° step spends ~5 s at the rail. That is why `rateRMS`
  sits at 8.7–17.2 °/s against a 25 °/s cap.

`ff_speed` by contrast is roughly right: steady bins give
cv ≈ 10–11 %/(m/s) against the configured 7.5. ~30% low, not 3× low.

**So `ff_yaw` is the highest-leverage tier-0 item, ahead of
`thrust_asymmetry`.** Both are tier 0; `ff_yaw` moves the numbers more.

### 0.4 Loop-shape before you get wet

`bb_plant_id.py` gave `G(s) = 0.0971/(2.32s+1)·e^(−0.5s)` for
DESIRED_RUDDER → yaw rate. With the two 0.3 s filters
(`yawrate_filter`, `des_yawrate_filter`) and that 0.5 s delay, a
crossover near 0.6 rad/s leaves roughly 70° of phase margin. Do that
arithmetic on paper before session 3 and the inner-loop session becomes
a verification rather than a search. It costs no water time.

---

## 1. Session order

| # | Session | Mission | Changes going in | Acceptance |
|---|---|---|---|---|
| **0** | Open-loop RC calibration | `handling_block --rc-cal` (RC only, script inert) | none | `analysis/rc_cal.py` returns an `A` recommendation and a motor-match verdict |
| **0b** | bench | — | `thrust_asymmetry` ← session 0 | rebuild, `ctest` green |
| **1** | **Re-baseline** | `handling_block` | **none** (verbatim gains, §3 fixes live) | `settled` → ~0, pivots ≈ 25 °/s, no windup, `BB_MIX_SATURATION` present. This is the real baseline |
| **2** | Yaw feedforward | `handling_block` | `ff_yaw` only, from session-1 steady data | steady \|I\| of the yaw-rate loop drops; `rateRMS` drops |
| **3** | Inner loop | `handling_block` | `yawrate_pid` | achieved rate tracks commanded; `rudSat` still ~0 on 60° steps |
| **4** | Outer loop | `handling_block` | `heading_pid` | t63 ≤ 2 s on 30° @ 1.5 m/s, ovs ≤ 10° |
| **5** | Speed loop + FF | `handling_block` | `speed_pid`, `ff_speed` | `ffShare` → ~1 at steady speed, `spdErr` → ~0 |

> **Run record (2026-08-31, all flown same day):** 0 = `11_07_07` (drag
> curve + A bracket), 0-null = `11_41_09` (A=1.80), 1 = `12_56_34`
> (re-baseline after the EKF hull-frame fix; also the ff_yaw steady
> data), 2 = `13_21_56` (ff_yaw 0,45,80 — PASS, brief-§4 bar met on
> steps/pivots), 3 = `13_45_35` (ff_speed 0,9.2,30,10.1 with the new
> quadratic cvv term — PASS; staircase t63 collapsed, steady I → ~0).
> Sessions 3–5 as planned above were never needed as separate gain
> sessions: with tier-0 FF correct, `yawrate_pid`/`heading_pid`/
> `speed_pid` met acceptance at verbatim gains. Session 4 = `14_01_57`
> (`des_yawrate_filter` 0.3→0.5 + A 1.80→1.77): **PASS, campaign
> converged.** Two-sided acceptance held — 30° t63 1.5/1.8 s, clean
> zig reversals ovs 2.3–4.2 (was 13–15 worst-leg), rate spikes gone
> (peaks 20–33 vs 37–50), best rateRMS yet (5.3–8.2). A = 1.77
> verdict: mean lever-corrected pivot surge −0.069 (pass band ±0.15;
> read directly from NAV_SURGE since all four pivots ran CCW and the
> CW/CCW pairing was unavailable — the EKF lever-arm fix makes the
> direct read valid). Two phases carry asterisks from RC takeovers
> (stepB_+120, zig_1/zig_2 entry); their clean siblings all pass.
> Standing gains after the campaign: `speed_pid 1,2,0` /
> `heading_pid 0.8,0,0` / `yawrate_pid 60,40,0` — UNCHANGED from the
> 27 Aug water-proven set. The entire campaign was calibration
> (A, ff_yaw, ff_speed+cvv) and one shaping knob (des_yawrate_filter);
> no feedback gain ever needed to move.
| **6** | Policy | `handling_block` | governor, `steering_throttle_mix`, gain schedule | each justified against a gain=0 / disabled run |

Sessions 2 and 5 change **feedforward, not gains** — there the block is
being used as a measurement instrument, not a search. That distinction is
what "one knob-set per run" misses if you read it as "one gain per run."

---

## 2. The loop

```
edit plug_pBBPID.moos (ONE change)          # or live: BBPID_SET over the DB
  -> run the handling block (~7 min autonomy)
  -> analysis/handling_block.py <bs.alog> --ledger analysis/tuning_ledger.jsonl
  -> analysis/handling_block.py --compare <previous.alog> <this.alog>
  -> decide, repeat
```

- The scorer header prints the run's **parameter provenance** (the `._moos`
  snapshot overlaid with any `BBPID_PARAMS_ACTIVE` live-tuning
  republications), and `--compare` ends with a **param diff** — so "one
  knob per run" is audited, not assumed.
- The ledger (`--ledger`, one JSON line per scored run) accumulates
  params + every metric across sessions.
- Live tuning mid-session: post `BBPID_SET = key=value` (any pBBPID plug
  param); pBBPID republishes the active set so the log stays
  self-describing. **`thrust_asymmetry` and `steering_throttle_mix` are
  NOT live** — they are pThrustMix config on the front seat, and changing
  them costs a front-seat restart (props spin: wait for a reboot, never
  restart the fs mission on the water). Plan them per-session, not
  per-iteration.

## 3. The block (missions/handling_block)

`gen_maneuver.py` emits 23 phases, ~423 s of autonomy:

| Phases | What it measures |
|---|---|
| stepA/B ±30/−60/+120 @ slow/fast | heading + rate loop transients, two speeds |
| zig ×6 | repeatability, settled bias |
| pivot ×2 (±170, baseline-comparable) | turn authority, thrust asymmetry |
| pivot180 ×2 | the `ff_step_limit` guard under true 180s |
| straight 60 s | XTE, standing speed error |
| speed staircase 0.5→1.0→1.5→0.8 | speed-loop step response |

`--pivot180 0` for a legacy-config run; `--speed-steps ""` to skip the
staircase; `--rc-cal` for the session-0 open-loop run (see §4).

## 4. Session 0: open-loop RC calibration

`thrust_asymmetry` cannot be measured from a closed-loop run — the
controller is between you and the plant, and on the 27 Aug log the pivot
phases were broken anyway (§0.1). It also cannot be bisected on the
water, because it is not live-settable (§2).

So it is identified **once, open-loop, from a single RC run**, using the
boat's own drag curve as the calibrated ruler:

```
./launch_vehicle.sh --base=150 --rc-cal      # timer script inert; RC only
    ... fly the card in missions/handling_block/README.md ...
analysis/rc_cal.py <backseat>.alog
```

The identity it inverts. In a pure pivot the mixer commands `left = +u`
and `right = −u·A_set`; reverse delivers `1/A_true` of forward thrust per
unit command, so the residual surge effort is

```
e_net = u·(1 − A_set/A_true)        ->        A_true = A_set / (1 − e_net/u)
```

`e_net` is recovered by measuring the pivot's steady fore/aft drift and
inverting the straight-run drag curve collected in the same log (both are
in the same effort-% units, because a straight run at surge `s` commands
both sides forward at `s` with no reverse compensation in play).

- Drift **forward** in both directions → reverse is weaker than assumed →
  `A_true > A_set` → raise `thrust_asymmetry`.
- Drift **aft** in both directions → lower it.
- A **CW-vs-CCW rate or drift split** is a different fault: motor, prop,
  or ESC mismatch, not `A`. The scorer reports the two separately,
  because conflating them is what the old scorer line did.

Two things bite in practice, both learned on the 31 Aug run:

- **The GPS lever arm.** `NAV_X/NAV_Y` follow the antenna, not the centre
  of rotation. A pontoon-mounted antenna 0.35 m off centreline on a boat
  spinning at 40 °/s reads a spurious ±0.25 m/s of body-frame surge that
  **flips sign with turn direction** — indistinguishable from a left/right
  motor imbalance until you remove it. Measure the offset, pass
  `--gps-offset`. The gyro's offset is irrelevant (angular velocity is the
  same everywhere on a rigid body).
- **The drag curve must cover the drift range.** Pivot drift is
  0.3–0.9 m/s; if the slowest straight is 1.0 m/s, every pivot is scored
  off an extrapolation, and the error is worst exactly where `spin` (the
  ratio of rotational to translational flow over the hull) is largest.

Good to roughly ±20% *when both of those hold* — the pivot's rotational
drag is not the straight-line drag curve. That is the right precision: the question is whether `A` is
1.6 or 2.4, not 1.6 or 1.65. The **null test is the acceptance**: after
setting the new value, session 1's pivot drift should be ≈ 0.

## 5. Knob → evidence map

### Tier 0 — plant constants (measure, freeze)

#### `thrust_asymmetry` (1.6, pThrustMix, front seat)
- Session 0, `analysis/rc_cal.py`. See §4.
- Then verified every block run after: `handling_block.py` reports
  `common-mode surge drift` over the pivot phases — the same observable
  rc_cal inverts. Null (≈0) means A is right. The `motor match` line
  beside it is a *different* fault and must not be read as A.
- Copied from ArduRover BlueBoat120, never validated on this hull.
- Confirmation via per-side current is **not currently possible**: the
  Navigator publishes only `NVGR_ROLLING_CURRENT` (total). Per-side would
  need new instrumentation.

#### `ff_yaw` (0, 45, 80) — d0, dr, dvr, in %/(rad/s)
- The one with the most leverage (§0.3). **Set 2026-08-31 (session 2)**
  from the session-1 re-baseline (`LOG_YIP_31_8_2026_____12_56_34`,
  rudSat 0% — no railed-sample bias): steady-segment regression
  (desired rate held 3 s, tracking within 30%, n=353) gives
  **43.8 %/(rad/s) at pivot** (v<0.3, n=134, resid sd 5.2) and
  **143 %/(rad/s) underway** (v>0.7, n=109) → dr=45,
  dvr=(143−44)/1.2≈80, intercepts ≈0 so d0 stays 0.
- The earlier "~183 flat" number came from n=155 *railed* 27 Aug
  samples — biased high and blind to the speed slope. The plant gain
  RISES with speed (hull yaw damping), which is what the `dvr·v` term
  is for.
- Evidence method: steady segments where desired yaw rate is constant —
  mean `|DESIRED_RUDDER|` per unit of *achieved* rate, binned by speed.
  The speed dependence separates `dr` from `dvr`.
- Cross-check: the yaw-rate loop's steady integral should fall toward 0
  as the FF takes over (session 1 railed it at 50, 4.4% at-limit).
  `yawrate integral` in the run summary is the readout.
- Caveat carried from the plug: the plant ID's VAF was 51% and its
  `dr = 10.30 %/(deg/s)` (= 590 %/(rad/s)) is far above every in-log
  estimate. Trust the log over the ID.

#### `ff_speed` (0, 9.2, 30, 10.1) — c0, cv, crr, cvv
- **Set 2026-08-31 (session 3).** The engine gained an optional 4th
  coefficient `cvv` (quadratic drag, applied as `cvv·v*|v*|`; absent =
  0 = legacy linear). The hull's drag is quadratic — steady speed-hold
  thrust from the session-2 log (n=462) matches the rc_cal drag curve
  `9.2v + 10.1v|v|` within ~1 unit at every staircase point
  ((0.48, 5.1), (0.96, 15.2), (1.44, 33.4) measured vs 6.7/14.9/34.2
  predicted) — so the coefficients come straight from the boat's own
  measured drag curve, and no linear cv can fit both ends.
- `ffShare` (staircase + straight): FF's share of mean thrust at steady
  speed. Share ≪ 1 with a large steady `iterm_speed` → FF model low
  (the integral is doing FF's job); share > 1 with negative I → too
  high. Sessions 1–2 carried I = 8–17 at steady speed; post-cvv the
  steady speed |I| should fall toward 0 at every rung.
- Steady bins on 27 Aug suggest cv ≈ 10–11 vs the configured 7.5, but the
  1.2–1.8 m/s bin was windup-contaminated. Re-measure in session 1.

#### `max_yawrate` (0.4363 rad/s = 25 °/s)
- Not a tuning knob until the loops can actually reach it. On 27 Aug the
  boat achieved 18.3 °/s against a 25 °/s command with rudder to spare —
  the cap was never the binding constraint. Revisit after session 3.

### Tier 1 — feedback gains

#### Inner yaw-rate loop — `yawrate_pid` (60, 40, 0)
- `rateRMS` (per step): RMS of desired-vs-measured yaw rate over the
  transient. High with `rudSat` ≈ 0 → raise kp; high *with* rudder
  saturation → the plant is the limit, not the gains.
- `rudSat` (per step): fraction of the phase with |DESIRED_RUDDER| ≥ 99.
  Persistent saturation on 60° steps is the brief's "inner loop
  saturating" red line.
- `yawrate integral` (summary): steady |I| near its limit → the FF is not
  carrying its share (fix `ff_yaw` first — that is why session 2 precedes
  session 3) or Ki is doing P's job.
- Target from §0.4's loop shaping, not from a search.

#### Outer heading loop — `heading_pid` (0.8, 0, 0)
- `t63`, `ovs`, `settled` (per step): the classics. Post-FF-fix
  prediction: `settled` → ~0 **without** heading Ki. If a standing bias
  survives, check `BBPID_HEADING_ITERM` stays 0 (it should, ki=0) and
  only then consider Ki.
- `XTE rms` (straight): the integrated consequence.

#### Speed loop — `speed_pid` (1, 2, 0)
- `t63` (speed staircase): loop bandwidth per step size and direction.
- `spdErr` (staircase + straight): standing error → Ki evidence.
- `I` / `iterm_speed`: the steady thrust the integral supplies is exactly
  what `ff_speed` fails to predict — tune the two together in session 5.

### Tier 2 — policy

#### Governor — `yaw_priority_gain/knee`, `min_speed_frac`, `derate_filter`
- `govMin` (per step): min governed speed as a fraction of commanded.
  Sitting at `min_speed_frac` (50%) on every big step = the governor is
  railing — decide whether that is buying heading tracking (compare t63
  against a gain=0 run) or just losing ground speed. It railed on most
  phases on 27 Aug, but against a broken inner loop, so that tells us
  nothing yet.

#### Turn-rate cap schedule — `enable_gain_schedule`, `schedule_point`s
- Decide from the **stepA (slow) vs stepB (fast) columns**: if fast steps
  overshoot/oscillate where slow ones do not (or vice versa), the flat
  gains are compromising one regime — evidence for the schedule, not
  symmetry.
- Note the schedule also overrides `max_yawrate`, so enabling it changes
  two things at once. Enable it only after tier 1 is settled.

#### `steering_throttle_mix` (0.6, pThrustMix, front seat)
- Summary row — saturation fraction and mean |heading error| on saturated
  vs unsaturated ticks. Touch only if the saturated column is meaningfully
  worse. 9.2% of cycles on the 5 Aug run; `BB_MIX_SATURATION` was **not**
  in the 27 Aug log (bridged 2026-08-28, still unverified on water), so
  this knob has no evidence column until session 1 confirms the bridge.

### Lifecycle (verify, don't tune) — `max_dt`, `antiwindup`, `integrate_gate`, `ff_hold_time`, `ff_step_limit`
- `integrate gate` transitions (summary): OPEN under autonomy, FROZEN
  across takeovers, no flapping.
- Integral health rows: at-limit fraction ~0 is the windup fixes working.
  A loop parked at its limit is a regression or an FF gap.
- pivot180 phases: sustained rate ≈ the ±170 pivots → retire the
  band-aid; ~2°/s → the guard is not active (check `ff_step_limit` in the
  params header).
- All of this is session 1's job. If any of it fails, session 1 repeats
  and no tuning starts.

## 6. Telemetry the evidence rides on

Published by pBBPID (all in the bs log):
`BBPID_{SPEED,HEADING,YAWRATE}_ITERM` (integral terms, output units) ·
`BBPID_GATE` (OPEN/FROZEN/OFF, on change) · `BBPID_PARAMS_ACTIVE`
(full active param set, on startup + every applied change) · plus the
existing error/rate/FF/governor debug set.

Bridged from the front seat (`plug_iFrontSeatBroker.moos`):
`BB_MIX_SATURATION` · `NVGR_APPLIED_LEFT` / `_RIGHT` · `GYRO_Z_LVL_IMU` ·
`BB_CMD_AUTHORITY` / `_STOP_REASON` · `RC_MARK` (added 2026-08-31 for
session-0 segmentation).

## 7. Acceptance bar (unchanged, brief §4)

t63 ≤ 2 s on 30° steps @ 1.5 m/s · overshoot ≤ 10° · settled ≤ 2° ·
pivot ≥ 25°/s sustained · straight XTE RMS ≤ 1 m — against the
ArduRover reference (2.1° median underway, loaded hull).

## 8. Known gaps in this document's own tooling

- **`bb_plant_id.py` is not in the repo.** It is cited here (§0.4) and in
  `plug_pBBPID.moos`'s `ff_yaw` comment as the source of
  `G(s) = 0.0971/(2.32s+1)e^-0.5s`. The transfer function is recorded;
  the script that produced it is not. Re-derive from a session-1 log
  rather than trusting the citation.
- **`BBPID_PARAMS_DEFAULT` is republished on a timer** (~every 4.5 s, 95
  copies in the 27 Aug log). The scorer's provenance header keys on
  `BBPID_PARAMS_ACTIVE`, which was absent from that log entirely. Confirm
  in session 1 that provenance actually lands, since every "one knob per
  run" claim rides on it.
- **Per-side current is not instrumented** (§5, `thrust_asymmetry`).
