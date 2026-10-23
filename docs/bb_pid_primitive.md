# bb::Pid: the owned control primitive

Status: **implemented 2026-08-28, bench-validated (17/17 CTest); water validation pending**
Companions: [`control_checkpoint_brief.md`](control_checkpoint_brief.md) §3 (the plan this executes),
[`ibb_navigator_command_pipeline.md`](ibb_navigator_command_pipeline.md) (the telemetry it consumes)

## 1. What and why

`lib_bb_control` is the feedback-control sibling of `lib_bb_command`: pure,
no MOOS, no clock, no hardware. It holds two things:

- **`bb::Pid`** ([pid.h](../src/01_libraries/lib_bb_control/pid.h)) — the one
  PID primitive for every controller we own. Replaces moos-ivp's `ScalarPID`
  inside `BBPIDEngine`.
- **`BBPIDEngine`** — moved here from `src/pBBPID/` unchanged in role: the
  cascade, FF, governor and schedule around three `bb::Pid` loops. `pBBPID`
  is now a thin MOOS wrapper that links this library.

The 27 Aug handling block exposed two lifecycle defects (resume windup, the
held-derivative FF) that ScalarPID cannot express fixes for: it destroys its
integral when Ki≤0 (so "pause integration" is impossible), freezes P and D
along with I when fed dt=0, and its integral/derivative state is reachable
only through `protected:`. We weighed a thin subclass against a rewrite; the
subclass would have had to recompute the output formula on every gated tick
anyway — re-deriving the math while staying coupled to the base class's
internals. `pDiffThrustPID_v2` (Turrisi) already ran the key pattern —
commit only the integral increment the water actually sees — so the
primitive was rewritten around it, and the moos-ivp dependency was *moved,
not dropped*: ScalarPID is now the **test oracle**
(`test_pid_scalar_parity.cpp`) that pins the water-proven arithmetic.

## 2. The parity contract

On a **clean tick** — integration enabled, no reported rail, dt under
`max_dt`, output inside its limit, Kd = 0 (every live gain set) — `bb::Pid`
reproduces ScalarPID exactly, by construction and by test (2000 ticks × 3
live gain sets, dt-jittered, tolerance 1e-9):

- integral state in **output units** (`I += Ki·e·dt`): a live Ki change does
  not rescale history;
- integral limit in output units, same clamp; `Ki ≤ 0` zeroes the sum;
- same operation order (integrate → output → clamp), same output clamp;
- first step after construction/`reset()` establishes state, integrates
  nothing.

One deliberate departure: derivative is a single backward difference (or
caller-supplied `d_error` for derivative-on-measurement) instead of
ScalarPID's 9-sample mean — dormant at Kd = 0, better once Kd wakes.

## 3. Where it diverges — exactly where the fixes bite

| Fix | Mechanism | Default |
|---|---|---|
| Gap credit bounded | `max_dt` clamps the dt credited per step | off (config `max_dt`) |
| Authority gate | `set_integrate_enable(false)` freezes I; P/D live | off (config `integrate_gate`) |
| Downstream rails | `set_external_saturation(sign)` skips increments into a reported rail (mixer saturation, post-FF clamp) | off (config `antiwindup`) |
| Tracking anti-windup | at the loop's own rail, the increment commits **only up to the rail's worth of authority**: I parks at the rail, never at its own limit behind it, and comes off on the first reversed tick | off (config `antiwindup`) |

**One rule, learned the hard way: every behavior change is config-gated, so
all-defaults is verbatim legacy on every tick — rails included.** The first
cut left the anti-windup (and the FF cap) always-on, reasoning "they only
act where fixes bite"; replaying the 27 Aug *helm* log (which rails on 235
ticks, unlike the block log) showed why that fails: the integral is memory,
so one refused increment carries divergence into later clean ticks and makes
the replay unauditable. With the gates in place, both logs replay
bit-identical at all-defaults. The parity oracle asserts the enabled fix's
divergence direction too: driven onto a rail and reversed, the old class
stays railed ~10 s burning off its parked integral; with `antiwindup` on the
new one responds on the first tick — and with it off, matches the oracle
exactly *on* the rail.

## 4. The FF fixes (engine-level, brief §3.3)

- `ff_hold_time` — zero the held desired-heading derivative after ~2× the
  helm cadence. Staircase behavior unchanged; step-and-hold gets a true
  zero instead of the `ff/kp` standing error (6.4° measured).
- `ff_step_limit` — steps beyond ~150° publish no reference FF: that is a
  repositioning, and at exact 180° the derivative's sign is numeric noise
  (it cancelled the railed feedback: 2°/s pivots). Retires the ±170°
  band-aid in `gen_maneuver.py` — pending water validation.
- FF capped at `max_yawrate` before summing with feedback — gated with
  `ff_step_limit` (same pathology family; when ff exceeds the cap and fb
  opposes it, `min(ff,cap)+fb ≠ min(ff+fb,cap)`, so an ungated cap broke
  bit-parity during an end-of-run command jump in the helm log).

## 5. App plumbing (pBBPID, brief §3.2)

`integrate_gate = true` subscribes the applied-output telemetry built for
exactly this (invariant 11): integrate only while `BB_CMD_AUTHORITY ==
AUTONOMY && NVGR_STOP_REASON == NONE`; `BB_MIX_SATURATION > 1` stops the
loops winding the excess the mixer cannot allocate (direction = our own
last command's sign). **Fail frozen**: stale gate inputs freeze integration
(P/D live) and raise a run warning — a broken bridge must not silently
re-enable the kill-window windup, and a frozen integral must not read as
"Ki does nothing".

## 6. Validation state

- `test_pid` (44 checks): the incidents as unit tests — 43 s gap, railed
  pivot, freeze-with-live-P, external rails, fail-closed NaN, reset
  semantics.
- `test_pid_scalar_parity` (9 checks): clean-segment exactness + asserted
  divergence at rails and gaps, against the real ScalarPID.
- `test_bbpid_engine` (17 checks): staircase FF alive, step-and-hold decays,
  180° pivot commands the full budget, FF cap, engine-level gap/gate/rails.
- **Parity replay: PASS on two logs (2026-08-28)** — `bb_pid_replay`
  (test/replay/) drives the legacy engine (git-HEAD extract, ScalarPID
  underneath) and the new engine side-by-side:
  - **Block log** (`handling/LOG_YIP_..._16_59_38`, 6,443 ticks, no rails):
    all-defaults **bit-identical**; fixes-on divergence windows *break*
    exactly at the 42.1 s / 12.8 s RC-repositioning gaps, and the post-gap
    resume tick (t=231.4) shows legacy at thrust −4.8 / rudder +79 (the
    windup) vs +18.6 / −11.5 sane.
  - **Helm log** (`bs/LOG_YIP_..._16_01_53`, 7,804 ticks, 235 railed,
    1,148 RC-authority): all-defaults **bit-identical including every
    railed tick**. This log caught two fixes that were initially
    always-on (tracking anti-windup; the FF cap) — both now config-gated,
    which is what made the claim exact.
  - The helm log also validated `ff_hold_time = 2.0` against real helm
    cadence: change intervals p50 1.0 s / p99 5 s; the 47 slower-than-2 s
    holds carry only ~5° mean steps (~1–2°/s implied — zeroing them IS the
    standing-error fix), while big leg transitions (74° max) still fire a
    fresh, capped FF pulse.
  - Caveat on fixes-on magnitudes: replay is open-loop (nav inputs are the
    logged boat, flown by the legacy outputs), so once trajectories diverge
    the new engine's errors persist — divergence *locations* are
    meaningful, sustained magnitudes overstate closed-loop reality.
  - Harness fidelity (legacy replay vs the log itself): rudder |Δ| p50
    0.005/0.0003, small tails at FF change ticks from timestamp jitter.
- **Pending**: the §3.5 water session — settled column → 0, pivots
  ≥ 25°/s. `BB_MIX_SATURATION` bridge line added to
  `plug_iFrontSeatBroker.moos` (was never bridged; the gate's mixer input
  read 0 all of 27 Aug) — takes effect next launch.

Config defaults are legacy (everything off) so the replay is clean;
`plug_pBBPID.moos` enables the fixes for the boat.
