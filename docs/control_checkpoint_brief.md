# BlueBoat Control: Checkpoint and the Road to Correct

Status: **checkpoint, 2026-08-27** — refactor water-proven; controller plumbing next
Companions: [`control_refactor_plan.md`](control_refactor_plan.md) (the route taken),
[`ibb_navigator_command_pipeline.md`](ibb_navigator_command_pipeline.md) (the contracts)

---

## 1. What we gained

The control refactor replaced a 2,000-line Navigator that arbitrated, mixed,
filtered and drove hardware in one opaque loop with four apps, four sentences:
`iRCInterface` decodes the handset; `pBBCommandArbiter` decides who is driving;
`pThrustMix` converts surge/yaw to left/right effort; `iBBNavigatorInterface`
converts effort to pulses and owns the hardware. Everything interesting lives in
a hardware-free library: 14 test suites, ~114k checks, mutation-tested.

All of the following is **measured on the water (yip, 27 Aug)**, not asserted:

| Property | Before (via, 5 Aug) | Now (yip, 27 Aug) |
|---|---|---|
| Heading error under autonomy | p50 5.0°, p90 113° | **p50 0.19°, p90 1.44°** ¹ |
| Stick → actuator-frame latency | ~320 ms mean (computed) | **p50 37 ms, p95 56 ms** |
| Step response (never measurable before) | n=1 usable step in 72 min | **t63 1.5–6 s** (30–126° steps), peak yaw 18–39°/s |
| ESC pulses | 8% outside spec, 2% at 2200 µs | **100% within 1100–1900 µs** |
| Yaw authority commanded (median) | 10.2 | **23.4** (2.3×; LPF + deadband gone) |
| Fail-closed behavior | untestable | kill ×2 live, RC takeovers clean, 0 rejects, 0 watchdog trips |
| Traceability | none | every pulse joins to one stick/controller sample; runs replay offline |

¹ Directional, not acceptance: different hull, day, and speed profile than the baseline.

Equally important: we now own a **measurement instrument**. The
`handling_block` mission drives exact, held setpoints (steps, zigzag, pivots,
straight) with RC-pause/resume built in, and `analysis/handling_block.py` turns
its log into the acceptance table in one command. Handling claims are now
numbers, on demand, in ~25 min of water time.

## 2. What the first block exposed

The instrument paid for itself immediately: two plumbing defects in `pBBPID`,
both invisible before there was a way to command clean steps.

- **Resume windup.** The moos-ivp `ScalarPID` underneath the engine keeps its
  own wall-clock; the first tick after an RC-repositioning gap integrated
  `Ki·e·dt` across the whole gap (43 s), slamming integrals to their limits —
  the boat resumed at surge −13.7 with +1.5 m/s commanded and took ~15 s to
  unwind. *Band-aid in place:* integrators reset on every resume.
- **Feedforward held-derivative.** The 2-DOF reference FF holds the last
  command-step's implied rate until the next change. Correct for a helm's
  moving staircase; wrong for step-and-hold: it forces a standing error of
  `ff/kp` after every settle (predicted 6.0°, measured 6.4°), and at an exact
  180° pivot its sign is numeric noise — FF cancelled the railed feedback and
  the boat pivoted at 2°/s instead of 25. *Band-aid in place:* pivots command
  ±170°.

The deeper diagnosis: **the controller predates a world where its output is not
always in charge.** It cannot tell "my command is reaching the water" from "the
Navigator is holding neutral under a kill," it has no anti-windup under
saturation beyond a clamp, and the engine has zero test coverage.

## 3. Next steps: a correct control scheme (not tuning)

> **Update 2026-08-28:** items 1–4 implemented — see
> [`bb_pid_primitive.md`](bb_pid_primitive.md). One amendment to item 1:
> rather than a from-scratch engine, `bb::Pid` (lib_bb_control) reproduces
> ScalarPID's arithmetic exactly on clean ticks, with ScalarPID retained as
> the CTest parity oracle. Parity replay on BOTH 27 Aug logs (block +
> helm): **PASS, bit-identical at all-defaults on every tick, railed
> included** (see bb_pid_primitive.md §6). Remaining: the §3.5 water
> session.

Same method that fixed the actuation path — pure component, tests, replay,
then hardware. Gains stay verbatim throughout; every change validated by
**parity replay** (old vs new engine on the existing block log: identical on
clean segments, divergent only where the fixes bite).

1. **Own the PID primitive** (library, beside the mixer): explicit
   caller-supplied `dt` with a max-dt clamp, an `integrate_enable` input,
   tracking anti-windup on output saturation, `reset()`, unit + mutation
   tests. Drop-in swap inside `BBPIDEngine`, same gains.
2. **Actuation-aware integration.** `BB_CMD_AUTHORITY` and `NVGR_STOP_REASON`
   already bridge to the back seat (invariant 11's applied-output telemetry,
   built for exactly this). Integrate only while
   `authority = AUTONOMY && stop = NONE` — kills the latent kill-window windup.
   Extension: the mixer's saturation flags are on the wire; the loop demanding
   more than the mixer can allocate stops integrating the excess.
3. **Fix the FF**: decay the held derivative after ~2× the observed command
   cadence (helm behavior unchanged; step-and-hold gets a true zero), publish
   no FF for steps > ~150° (repositioning, not trajectory; sign is noise), cap
   FF at `max_yawrate`.
4. **Test the engine.** `BBPIDEngine` is nearly pure already; wire it into
   CTest: steps, gaps, saturation, the FF cases — mutation-tested.
5. **One water session**: re-run the block; acceptance is the scorer's settled
   column collapsing toward zero and pivots sustaining ~25°/s.

Estimated 1–2 bench days plus that session.

## 4. What tuning looks like from there

Only after §3 lands — tuning on top of a biased FF or winding integrators is
wasted motion. Then it becomes an evidence loop, one knob-set per block run,
the scorer as referee:

- **Inner first (yaw-rate loop).** `yawrate_pid = 60, 40, 0` against the step
  responses; the open-loop plant ID (`bb_plant_id.py`, ff_yaw = 0, 20, 30)
  already gives the FF baseline. Target: rate tracking without the inner loop
  saturating on 60° steps.
- **Outer (heading loop).** `heading_pid = 0.8 P-only` today. With the FF bias
  gone, raise kp toward the t63 target and add Ki only if a standing error
  survives (it should not, post-FF-fix). Decide the speed-gain schedule
  (exists, disabled) from the 0.5 vs 1.5 m/s step columns — evidence, not
  symmetry.
- **Speed loop**, then the two copied mixer constants: `MOT_THST_ASYM = 1.6`
  from pivot wash symmetry + per-side current, `MOT_STR_THR_MIX = 0.6` only if
  saturation data (9.2% of cycles last run) says it matters.
- **Acceptance bar:** the ArduRover reference — 2.1° median underway, 0.4 s
  error-to-rotation — measured on a loaded hull. We start from 0.19° p50 on a
  gentler profile; the block's held steps are where the honest comparison
  happens. Suggested pass line: t63 ≤ 2 s on 30° steps at 1.5 m/s, overshoot
  ≤ 10°, settled ≤ 2°, pivot ≥ 25°/s sustained, straight XTE RMS ≤ 1 m.

Each session: pull → block run (~25 min) → `handling_block.py` → change one
thing → repeat. The instrument makes every claim in this brief re-checkable in
an afternoon.

## 5. Open items ledger (unchanged by this brief)

> ### **DEFECT 2026-08-31 — the speed loop runs away in reverse.**
>
> `NAV_SPEED` is an unsigned **magnitude**; `pBBPID`'s speed loop consumes
> it as a signed surge velocity. Zero commanded speed is therefore an
> **unstable equilibrium in the reverse direction**, and the block's pivot
> phases (`spd = 0`) walk straight into it.
>
> Measured on the 31 Aug run (`LOG_YIP_31_8_2026_____12_05_45`), pivot_1
> onward. Signed surge is computed from `NAV_X/NAV_Y` + `NAV_HEADING`:
>
> ```
>   t     surge(signed)  NAV_SPEED   spdErr   DESIRED_THRUST
>  476       +0.840        0.823     -0.823        +2.89
>  482       +0.055        0.231     -0.231        -9.20    <- through zero
>  488       -0.748        0.752     -0.738       -16.38    <- now reversed
>  494       -1.100        1.085     -1.085       -21.84
>  512       -1.484        1.492     -1.495       -42.87
> ```
>
> Past zero the boat is moving backwards, `NAV_SPEED` reports `|v|`, so the
> error stays negative and the loop commands *more* reverse. Positive
> feedback; it settles at ~1.45 m/s astern where drag balances −42% thrust.
>
> **It then flew the entire `straight` phase backwards at 1.48 m/s while
> reporting `spdErr` ≈ +0.05** — the loop cannot tell, and neither can the
> scorer. The boat was reversed for ~100 s, from pivot_1 through the end of
> the straight leg. Every speed metric in that window is meaningless.
>
> Why it did not show on 27 Aug: the pivots were broken by the FF sign bug
> and the boat never actually decelerated through zero. Fixing the pivots
> exposed this.
>
> **The lever arm below is the trigger.** At 40 deg/s a pivot shows ~0.25 m/s
> of phantom speed even with the hull dead stationary, so a pivot commanded
> from rest starts the runaway on its own. Unsigned speed is the mechanism;
> the lever arm is the match.
>
> **Fix (both defects, one place):** have the EKF publish hull-frame
> kinematics — a **signed body-frame surge**, lever-arm corrected
> (`v_hull = v_antenna - omega x r`, then project on heading). pBBPID needs
> no code change: it already has a `nav_speed_var` config knob, so point it
> at the new variable. Until then the block's pivot, straight and speed
> phases cannot be scored, and **`spd = 0` phases are not safe to fly
> unattended**.
>
> Interim guard worth considering regardless: hold the speed integrator (or
> the whole speed loop) when commanded speed is ~0, so a pivot cannot wind
> the loop at all.
>
> **Update 2026-08-31: implemented (bench), water pending.** The EKF now
> derives signed hull-frame surge/sway from its existing state
> (`u = s·cos(γ−φ)`) and publishes `NAV_SURGE` / `NAV_SWAY`;
> `plug_pBBPID.moos` repoints `nav_speed_var = NAV_SURGE` — no pBBPID code
> change. Also landed: each GNSS epoch is now fused ONCE (TS-gated; the
> spike doc's re-application defect — 3717 re-applications eliminated on
> this log). Validated by `test_dgps_ekf_model` (CTest) and
> `dgps_ekf_replay` on this run: replayed `NAV_SURGE` tracks the signed
> surge above through zero to −1.487 @ t 512 (table says −1.495 from
> positions) with 99.45% sign agreement wherever |v| > 0.4. The speed
> integrator guard was deliberately NOT taken (decision 2026-08-31: keep
> the change surgical). **Water-confirmed 2026-08-31
> (`LOG_YIP_31_8_2026_____12_56_34`):** all 23 phases ran; no reverse
> runaway (worst excursion −0.44 m/s crossing zero into pivot_1,
> recovered in ~8 s — the error sign flipped positive exactly where the
> old loop ran away); `NAV_SURGE` matched position-derived surge 759/759
> on sign; the block's pivot/straight/speed columns are scoreable for
> the first time.
>
> ---
>
> **NEW 2026-08-31 — EKF does not compensate the GPS lever arm.** Both
> antennas sit on yip's PORT pontoon, 14 in (0.356 m) off centreline. The
> 31 Aug calibration run solved that offset back out of `NAV_X/NAV_Y` at
> 0.346 m — i.e. the full physical value — which means `pBB_DGPS_EKF`
> publishes **antenna-frame** position and velocity, uncorrected.
>
> Consequence: under rotation the reported surge carries a spurious
> `-omega * d` term — 0.06 m/s at 10 deg/s, **0.16 m/s at 25 deg/s** —
> signed with the turn direction. Zero in a straight line, so cross-track
> and the speed staircase are clean, but every metric scored *during* a
> turn inherits it: `spdErr` on the step and zigzag phases, and the
> governor's derate decisions, at roughly 10% of a 1.5 m/s command at full
> turn rate. It is a systematic bias, not noise.
>
> **Fix:** apply the lever arm in `pBB_DGPS_EKF` — rotate the measured
> antenna offset by heading and subtract `omega x r` from the velocity (and
> the offset itself from position). Needs the offset as config
> (`gps_offset_x/y`, body frame) rather than a constant. Until then,
> `analysis/rc_cal.py --gps-offset` compensates it downstream for
> calibration only, and session-5 speed-loop evidence should come from
> straight-line phases rather than turning ones.
>
> Sizing this properly also wants a second boat measured — via's antenna
> geometry is not recorded anywhere.
>
> **Update 2026-08-31: implemented (bench), water pending.** `pBB_DGPS_EKF`
> now lever-arm-corrects every measurement before fusion
> (`p_hull = p_ant − R(φ)r`, `v_hull = v_ant − ω×r`; pure function in
> `BB_DGPS_EKF_Model.hpp`, unit-tested against the rc_cal convention and
> the 0.25 m/s @ 40 °/s number). Config `gps_lever_x/y` (body frame,
> default 0,0 = no correction); yip's plug carries `gps_lever_y = −0.356`.
> **Via still needs its geometry measured — its correction is OFF until
> then.** Replay on this log: straight-line speed/heading unchanged
> (p50 Δ 0.009 m/s / 0.07°), position shifted by exactly the lever arm;
> pivot-phase speed becomes turn-direction-independent (the base filter
> flipped ±ω·d with turn sense). **Water-confirmed 2026-08-31:** the
> scorer's common-mode pivot surge drift reads −0.023 m/s (was the
> ~0.25 m/s phantom) — lever arm and A = 1.8 both null on the water.

`iRCInterface` 16→50 Hz (now latency-justified; closes the G2 p95 gap) ·
engine fixes above (implemented 2026-08-28, replay + water pending) ·
yip's AK09915 mag dead
(logged; `use_mag=false`) · `NVGR_DISARM` has no handset producer · ADS1115
poll to its own slow thread (optional; costs one 20 ms iterate in ten) ·
tighten source leases from age data after more runs (autonomy saw a 1.34 s
excursion — keep ≥1.5 s).
