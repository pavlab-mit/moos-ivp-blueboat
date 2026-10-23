# BlueBoat Control Refactor: Design, Implementation, and Test Plan

Status: **AGREED — in progress**
Branch: `control-refactor` (off `crossfire`)

Companion to [`ibb_navigator_command_pipeline.md`](ibb_navigator_command_pipeline.md),
which specifies the *target* contracts. This document is the *route*.

Out of scope: `pBBPID` gains and structure, the EKF, the back-seat helm. §9 says
what that costs, and the log evidence in §2 says it costs more than first
assumed.

---

## 1. Goals, made measurable

| # | Goal | Measurable form | Proven in |
|---|---|---|---|
| G1 | A boat that turns responsively | Heading tracking error and heading-error-to-rotation lag, against the measured ArduRover baseline in §2.1 | §8.3 |
| G2 | ~50 Hz control | Front-seat control apps at `AppTick=50`; measured source-to-PWM latency p95 **< 40 ms** | §8.2, §8.3 |
| G3 | Less obtuse architecture | Four apps, four one-line responsibilities; `iBBNavigatorInterface` under ~900 lines with zero arbitration logic | §6, §7 |
| G4 | Explainable, tractable | Every actuator frame resolves back to one source sample; every on-water frame reproducible offline from the `.alog` | §8.2 |

---

## 2. Baseline: what the existing logs actually show

No new baseline runs. Two existing logs carry enough signal, and they were
analyzed rather than assumed:

- **ArduRover reference** — `gallant`, 26 Aug 2026, MIT Pavilion, 22.7 min.
  `iArduRoverBridge` at 10 Hz (`NAV_HEADING`, `NAV_SPEED`, `IMU_GYRO_Z`);
  10557 samples in `GUIDED`, 3045 in `MANUAL`. Real boat, armed, real GPS.
- **Our controller** — `via`, 5 Aug 2026 16:27, 71.8 min. `pBBPID` at 19.9 Hz
  (`BBPID_DESIRED_YAWRATE`, `BBPID_MEAS_YAWRATE`), `DESIRED_THRUST_L/R` at
  16 Hz, `NAV_HEADING` at 14.5 Hz, `GYRO_Z_LVL_IMU` at 9 Hz.

**Both are BlueBoat120 hulls**, so the comparison is same-class. They are not
same-configuration: `gallant` carries a heavy sensor fit (RBR sonde, winch, ping
sonar); `via` carries none. That asymmetry matters and is used in §2.2.

### 2.1 Measured comparison

Heading tracking error while underway (speed > 0.5 m/s):

| | median | p75 | p90 |
|---|---:|---:|---:|
| ArduRover (`gallant`) | **2.1 deg** | 5.1 | 55.1 |
| pBBPID (`via`) | **5.0 deg** | 18.6 | 113.3 |

Cross-correlation lag, underway:

| Relationship | ArduRover | via |
|---|---:|---:|
| heading error -> yaw rate | **0.40 s** (r=0.56) | **4.40 s** (r=0.33) |
| thrust differential -> yaw rate | not logged | **0.50 s** (r=0.51) |

### 2.2 What that decomposition means

This is the most important result in the plan, and it partly redirects it.

**The hull and the actuators are not the problem.** Once a differential is
commanded, `via` rotates within 0.50 s with good coherence (r=0.51) — and that
0.50 s already contains the ~0.33 s of transport lag measured in §2.4 plus hull
inertia. Peak yaw authority is likewise fine: `via` reached 58.5 deg/s against
`gallant`'s 40.5 deg/s, and that ordering is exactly what the sensor-load
difference predicts — more mass and drag, less achievable yaw rate. The peak-rate
gap is a payload effect, not a control-quality signal.

**The gap is in the path from heading error to a commanded differential**:
4.40 s versus ArduRover's 0.40 s. That is the controller plus the transport
chain, not the mixer allocation law and not the ESC.

So the ordering of §2.5 (mixer math) as a top responsiveness suspect was wrong.
It stays on the list for correctness and explainability, but latency and the
heading loop are where the seconds are.

*Method caveat:* cross-correlation on non-stationary waypoint-following data
gives an indicative lag, not a transfer function. The two missions also ran
different speed regimes (`gallant` median 0.12 m/s, mostly station-keeping and
casting; `via` median 1.40 m/s surveying). Treat 4.40 vs 0.40 as a strong
directional signal, not a calibrated number.

*The payload asymmetry cuts against us, not for us.* `gallant` is the
handicapped boat — loaded, draggier, demonstrably lower peak yaw rate — and it
still holds heading to 2.1 deg median with a 0.40 s error-to-rotation lag.
`via` is the light, more agile boat and manages 5.0 deg and 4.40 s. Correcting
for load makes the control gap **wider** than the raw table shows, not
narrower. Nothing here is explained by `via` being a harder boat to turn.

### 2.3 Why clean step-response targets are not available from these logs

Both missions retask `DESIRED_HEADING` continuously — `gallant` at 0.82 Hz,
`via` at 0.88 Hz. Filtering for undisturbed >=20 deg steps whose command holds
25 s yields **n=1 in each log**. Step-response acceptance numbers must come from
a deliberate maneuver block, which is cheap to append to a run we are already
doing (§8.3).

### 2.4 Transport latency (unchanged, and now corroborated)

One-way mean, PID output to a new ESC pulse:

| Stage | Rate | Mean lag |
|---|---:|---:|
| `pMarinePIDV22` / `pBBPID` (back seat) | 16-20 Hz | 28 ms |
| `pThrustMix` (back seat) | 16 Hz | 31 ms |
| `iBackSeatBroker` | 10 Hz | 50 ms |
| UDP hop | - | ~2 ms |
| `iFrontSeatBroker` | 10 Hz | 50 ms |
| `iBBNavigatorInterface::Iterate` | 10 Hz | 50 ms |
| LPF (§2.6) | tau | ~100 ms |
| PWM thread | 50 Hz | 10 ms |
| **Total** | | **~320 ms mean, ~550 ms worst case** |

Consistent with the measured 0.50 s differential-to-rotation lag once hull
inertia is included.

### 2.5 The ESC command range is partly fiction — quantified

[`BBNavigatorInterface.cpp:63`](../src/iBBNavigatorInterface/BBNavigatorInterface.cpp#L63)
maps `[-100,100]` onto `1500 ± 700 us`. The Basic ESC 500 documents
**1100-1900 us**, so any `|cmd| > 57.1` leaves the documented range.

Measured against the 138,496 single-side commands in the `via` log:

- **8.0%** of all commands, and **9.2%** of non-zero commands, map outside
  1100-1900 us;
- **2.0%** sit at full scale — 2200 us, 300 us beyond spec.

Small in aggregate, but these are concentrated in hard turns, which is exactly
where authority matters. And every saturation decision upstream is allocating
against a feasible region the boat does not have. **Resolved: adopt
1100 / 1510 / 1900 with per-channel reversal.**

### 2.6 Two more confirmed defects

**Deadband applied after mixing, per side**
([`:969-978`](../src/iBBNavigatorInterface/BBNavigatorInterface.cpp#L969-L978)).
`thruster_dead_band = 5` zeroes each side independently, so a gentle correction
whose left/right differential is under 5 is deleted outright. ArduRover applies
no post-mix deadband (`MOT_THR_MIN = 0`).

**LPF on left and right independently**
([`:988-993`](../src/iBBNavigatorInterface/BBNavigatorInterface.cpp#L988-L993)).
Filtering both sides filters their difference — the yaw command. ArduRover
slew-limits *common throttle only* and leaves steering unfiltered. Fixing this
is the cheapest available improvement to turn onset.

### 2.7 Incidental finding: yaw-rate sign

In the `via` log, `GYRO_Z_LVL_IMU` is logged with the **opposite sign** to the
`NAV_HEADING` convention: correlating heading error against yaw rate gives
-0.27 as logged and +0.33 negated. Whatever consumes both must be reconciling
this somewhere, or silently not. This is precisely what design doc §2.9
(canonical physical sign) exists to prevent; worth a look during the EKF work
too.

---

## 3. Phase 0 — Setup (done)

- ✅ `navigator-cpp` switched to `main` (`8e2ec24`), fast-forwarded 5 commits.
- ✅ `test_pca9685_restart.cpp` ported from `jeremy-dev` and registered in
  `src/04_unit_tests/CMakeLists.txt`. Nothing else taken from `jeremy-dev`.
- ✅ `moos-ivp-blueboat` branch `control-refactor` created off `crossfire`.

**navigator-cpp is tabled from here.** The two changes above sit uncommitted in
that repo by decision — the blueboat work does not depend on them landing, and
they are picked back up alongside the arming bench re-run. Outstanding there:
commit the two changes, update the ported test's hardcoded
`pwm_set_frequency(100.0f)` to 50 Hz per decision 2, and delete `jeremy-dev`.

### 3.1 navigator-cpp: why `main` and not `jeremy-dev`

`origin/main` is a strict superset of the `jeremy-dev` arming fix — same three
mechanisms (orderly `ALL_LED_OFF` before `SLEEP`; mask the stale `RESTART` bit
in `pca9685_set_frequency`; clear a pending `RESTART` after wake) — plus OE
readback verification, a default prescale written before wake, explicit
per-channel register init, explicit `MODE2` totem-pole, `pca9685_shutdown()`,
and `navigator_force_pwm_off()`.

Beyond the arming bug it also brings **thread-safe I2C** (per-thread slave
selection plus atomic `I2C_RDWR`), which closes a real race: our sensor thread
and the PWM/ADC path share one bus fd, and `I2C_SLAVE` mutates shared
file-description state.

Use the new API in the rewrite: `pca9685_shutdown()` and
`navigator_force_pwm_off()` replace the hand-rolled
[`safePwmShutdown()`](../src/iBBNavigatorInterface/BBNavigatorInterface.cpp#L79-L114)
/ `atexit` path, and `is_pwm_ready()` becomes a real input to
`NavigatorSafetyState.hardware_healthy`.

---

## 4. Decisions

**Resolved:**

1. **ESC endpoints — `1100 / 1510 / 1900`**, per-channel `reversed`. Thrust
   curve changes; retune follows.
2. **PWM output rate — 50 Hz.** Matches ArduRover; no strong case for 100 Hz.
3. **`AUTONOMY_CMD` is packed in `pBBPID`** — right call, and the code agrees:
   [`BBPID.cpp:228-229`](../src/pBBPID/BBPID.cpp#L228-L229) already computes the
   `thrust`/`rudder` pair together and then publishes it as two separate scalars,
   which is the coherent-pair violation itself. `pBBPID` is also the only app
   that knows its own controller identity and `control_mode`, both required by
   design doc §5.3. Nothing downstream can reconstruct that.

   Adding the publication is **additive** — roughly 30 lines, no change to gains
   or structure — so it does not violate the "don't touch `pBBPID`" constraint,
   which was about restructuring and retuning.

   The alpha mission currently runs `pMarinePIDV22`
   ([`meta_vehicle.moos:36`](../missions/alpha_pavlab_asv/meta_vehicle.moos#L36)),
   so the swap to `pBBPID` is part of this work — see §12(a) for the one
   sequencing consequence.

4. **`pThrustMix` — full rewrite**, not an evolution. `mixer_model` required
   with no default so no log is ambiguous about which equation produced a pair.
7. **Timeouts — start lenient, tighten only as the system proves out.** Every
   timeout is a config parameter with a deliberately generous initial value, and
   the *measured* age distributions (`BB_ARB_LAST_VALID_*_AGE`,
   `NVGR_MIX_INPUT_AGE`) get logged from day one so tightening is evidence-led.

**Re-posed** — the originals were badly framed:

5. ~~"RC kill clear gesture"~~ — invented a requirement. Today's CH5 kill is a
   neutral-lock that clears instantly on switch release, deliberately, and the
   link-loss case is already handled (it latches only while `m_rc_connected`).
   **The real question:** the arbiter and the Navigator will each hold kill
   state — the arbiter for the authority decision, the Navigator as the hardware
   sideband. Are those one latch or two, and does the Navigator's sideband clear
   on its own or only when the arbiter says so? Recommend two independent
   latches, each clearing on its own valid evidence, because the whole point of
   the sideband is to survive an arbiter failure.

6. ~~"Seed slew from applied effort or return to zero"~~ — malformed. The slew
   limiter *is* the anti-step mechanism, and at 200 %/s any transition completes
   within 1 s, so there is no step to avoid on a normal handoff.
   **The real question:** should a *source handoff* (autonomy -> RC) reset the
   common-throttle slew state, or carry it? Hard stop already resets to zero by
   contract (design doc §2.7); this is only about clean handoffs. Recommend
   carrying the state — an operator taking manual control mid-transit should not
   have the boat drop to zero throttle and ramp back up — with the handoff and
   the carried value logged as an authority event.

---

## 5. Build order vs deployment order

**Build bottom-up.** The mixer's feasible region is undefined until the ESC
mapping is fixed. §4 decision 1 settles that, so this is unblocked.

**The Navigator is rewritten last, and that is the only behavior-changing
commit.** Cutting it over first — the literal reading of "start at the navigator
interface" — would mean keeping ~800 lines of legacy arbitration alive inside
the very file we are emptying, because nothing upstream would exist to drive it.
Landing the library and the upstream apps first lets the Navigator be rewritten
**once**, straight onto `BB_MIXED_CMD`, with rollback as a ~30-line adapter that
synthesizes a `BB_MIXED_CMD` from legacy `DESIRED_THRUST_L/R` at the mailbox.

Until that cutover, the new apps publish variables nothing consumes. That is a
property of build order, not a test phase, and it carries no validation burden
of its own — the replay harness (§8.2) is what actually gates them.

---

## 6. Phase 1 — `lib_bb_command`: the pure core

New `src/01_libraries/lib_bb_command/`. **No MOOS, no hardware, no threads.**

| Unit | Responsibility |
|---|---|
| `command_envelope.{h,cpp}` | `SemanticCommand`; strict `key=value` parse/serialize; reject duplicate keys, unsupported major version, non-finite, out-of-range (design doc §4) |
| `command_mailbox.{h,cpp}` | epoch/seq acceptance, duplicate suppression, out-of-order rejection, local `rx_time` lease, freshness query (design doc §7.1) |
| `slew_limiter.{h,cpp}` | symmetric `dt`-based common-throttle limiter with `reset()` |
| `skid_mixer.{h,cpp}` | ArduRover 4.7 port, pure; returns every intermediate (`t,s,l,b,q,f`) and limit flags |
| `esc_mapper.{h,cpp}` | per-channel min/trim/max/reversed piecewise-linear map, effort -> us. **1100 / 1510 / 1900** |

**Porting discipline** (design doc §9.8): copy the branch structure from
`AP_MotorsUGV::output_skid_steering()` at tag `Rover-4.7.0` first; simplify only
after it passes. Do not port vectored thrust, wheel-rate control, `MOT_THST_EXPO`,
or `MOT_REV_DELAY`.

**Golden vectors must be generated, not hand-written.** Compile the ArduPilot
function standalone from the `Rover-4.7.0` tag and dump a CSV over a fine
`throttle x steering` grid, crossed with `MOT_STR_THR_MIX in {0.0,0.3,0.5,0.6,1.0}`
and `MOT_THST_ASYM in {1.0,1.6}`, including the boundaries (`t=b`, `q=1`, zero
crossings, full reverse, all four sign quadrants). That CSV is the fixture;
design doc §9.6 becomes a hand-checked subset. Otherwise "ArduPilot parity" is
an assertion.

**Wire CTest.** `enable_testing()` in the root `CMakeLists.txt`, `add_test()` in
each test directory, retrofitting `lib_crsf`'s three existing tests.
`./build.sh --unit_tests && ctest` becomes the pre-commit gate.

---

## 7. Phases 2-5 — the apps

### 7.1 Phase 2 — producers publish coherent commands

- **`iRCInterface`** gains `RC_INPUT_STATE` (design doc §5.1); existing `RC_CH*`
  scalars stay as diagnostics. `AppTick` 16 -> 50, verified against the
  snapshot/`try_lock` restructure from the threading fix.
- **`pBBPID` publishes `AUTONOMY_CMD`** — one envelope carrying the
  `thrust`/`rudder` pair it already computes together, plus epoch/seq,
  `controller`, and `control_mode`. Additive; existing `DESIRED_THRUST` /
  `DESIRED_RUDDER` scalars stay as diagnostics. Requires the alpha mission to
  swap `pMarinePIDV22` -> `pBBPID` (§12(a)).
- **`iTeleop`** publishes `TELEOP_CMD` with `session`/`claim`/`estop` instead of
  direct left/right thrust.

Nothing consumes these yet.

### 7.2 Phase 3 — `pBBCommandArbiter` and `pThrustMix` (front seat)

- **New `pBBCommandArbiter`**, front seat, `AppTick=50`. Mailboxes from the
  library, state machine from design doc §6, `BB_SELECTED_CMD` out. The whole
  authority policy should fit on one screen.
- **`pThrustMix` rewritten from scratch**: subscribes only to
  `BB_SELECTED_CMD`, runs `SlewLimiter` + `SkidMixer`, publishes `BB_MIXED_CMD`,
  front seat at `AppTick=50`. The back-seat instance is removed from
  `meta_vehicle.moos` in the same change.

The §2.5 mixer difference has been quantified by replaying the `via` log through
the new `SkidMixer` and diffing against the logged `DESIRED_THRUST_L/R` — 72
minutes of real commands, offline and deterministic. **Result in §14.1: the new
mixer commands ~2.3x more yaw authority at the median and more differential in
87.5% of samples, with saturation rising from 4% to 20%.** Running both mixers
on the boat to compare them would have been the same measurement with less
control.

### 7.3 Phase 4 — `iBBNavigatorInterface` rewrite (cutover)

**Delete:** `calculateRCThrust`, `calculateHeadingMixer`, `theta_b`/`turn_scale`,
`m_rc_channels[16]`, teleop arbitration, `ALL_STOP` handling, thrust timeout,
the RC-kill *policy* latch, the `LPF` class, the post-mix deadband, and
`left/right_thruster_invert` as a bare sign multiplier.

**Note — the RC deadman stays here.** An earlier draft of this list deleted it.
Per §12(d) it is a Navigator-enforced safety sideband, not arbitration, and it
sits alongside the RC-kill sideband for the same reason: both exist to stop the
boat when the code above them cannot be trusted.

**Keep and harden:** the PWM writer thread, now fed one **atomic `ActuatorFrame`
with a TTL** and its own watchdog (design doc §7.4); the ESC arm lifecycle
rebuilt on `pca9685_shutdown()` / `navigator_force_pwm_off()`; sensor thread,
AHRS, ADC, baro, leak, LEDs.

**Add:** `EscMapper`, a `BB_MIXED_CMD` mailbox, `NVGR_ACTUATOR_TRACE`, and two
hardware safety sidebands — `RC_KILL_ASSERTED` (§12(b)) and RC link liveness
for the opt-in deadman (§12(d)). Both bypass the command path entirely and
neutralize PWM directly. `NavigatorSafetyState` grows one field
(`rc_link_lost`); nothing else about the input contract widens.

**Structure.** The app does two unrelated jobs. Two MOOS apps would be cleaner
but would both own the PCA9685 and share the I2C bus — a hazard, not a cleanup.
One app, two separated translation units (`NavActuator.cpp`, `NavSensors.cpp`),
no shared mutable state beyond the `Navigator` handle. Target under ~900 lines,
from 2030.

**Rollback:** `command_input = MIXED | LEGACY_DIRECT`, where `LEGACY_DIRECT` is
the thin mailbox adapter, not a preserved legacy branch.

### 7.4 Phase 5 — rates and load

`AppTick = CommsTick = 50` on `iRCInterface`, `pBBCommandArbiter`, `pThrustMix`,
`iBBNavigatorInterface`. Then measure: add `uLoadWatch` to the front-seat mission
(it currently has none) and confirm Pi headroom. G2 is a claim we back with a
number, not an assumption.

Brokers stay at 10-20 Hz. After the refactor only *semantic* autonomy commands
cross the link and the whole 50 Hz inner loop is front-seat.

---

## 8. Test strategy

Two tiers off the boat, then a dock test, then the free run: unit tests (§8.1),
log replay (§8.2), dock checklist (§8.4), on-water acceptance (§8.3).

### 8.1 Tier 1 — host unit tests (`ctest`, seconds)

Every requirement in design doc §15 that does not need MOOS: parser rejection
(malformed / NaN / out-of-range / duplicate key / bad version), duplicate-frame
freshness, epoch restart, out-of-order sequence, mixer golden vectors across
`MOT_STR_THR_MIX` below/at/above 0.5, ESC endpoint and reversal mapping, slew
reset on hard stop, arbiter fail-closed transitions (design doc §10).

Plus a randomized property test on the mixer: for any finite `(surge, yaw)`,
outputs finite, within `[-100,100]`, sign-consistent. Cheap insurance against a
porting typo in a branch the grid missed.

### 8.2 Tier 2 — replay harness (the one that carries the weight)

A host binary that reads an `.alog`, feeds the recorded source stream through
the real arbiter, mixer, and ESC mapper, and asserts the result matches the
logged `NVGR_ACTUATOR_TRACE` exactly.

*Built so far* (`src/02_applets/bb_replay/`, results in §14): the `.alog`
reader, the sample-pairing discipline, and two comparison modes — mixer A/B and
ESC mapping — which run against today's logs. The end-to-end assertion above
needs `pBBCommandArbiter` and logs carrying the new contracts, so it lands with
P3.

This is the gate for everything upstream of the Navigator, so it must be built
**before** Phase 3 ships, not after. It runs against:

- the `via` 5 Aug log (autonomy, 72 min, full `DESIRED_THRUST_L/R` lineage);
- a fresh RC-only log from the bench or a dock test, for the manual path;
- synthetic streams for the cases no real log contains: stale sources, epoch
  restarts, out-of-order sequences, kill assertion, watchdog expiry.

That last bullet matters. Real logs contain the happy path; the fail-closed
transitions in design doc §10 have to be driven by constructed input. Those
synthetic streams are the safety argument, and no amount of running the apps on
a boat would produce them.

### 8.3 Tier 3 — on water

One deliberate maneuver block appended to a normal run, giving the step-response
numbers §2.3 showed the archives cannot provide:

| Maneuver | Metric |
|---|---|
| Held heading steps: 30, 60, 120 deg at 0.5 and 1.5 m/s, command held 25 s | t63, overshoot, peak yaw rate |
| Pivot turn (0 surge, full yaw) | sustained yaw rate, per-side current |
| 20-20 zigzag | time to first overshoot, overshoot angle |
| Straight waypoint run | cross-track RMS |
| Whole run | heading tracking error distribution vs §2.1; latency p50/p95 from the lineage trace |

Run the same block once on the **current** stack before cutover — specifically
**after** the `pMarinePIDV22` -> `pBBPID` swap and **before** the actuation
cutover (§12(a)). That ordering is what makes the before/after attributable to
the actuation refactor instead of blending it with the controller change. It
costs one session and converts §2.1's cross-mission comparison into a same-hull,
same-conditions before/after — which is what the ArduRover comparison cannot be.

### 8.4 Dock test — the gate before the first free run

Boat in the water, tied to the dock, props free to spin. This is the only place
the new pulse endpoints (800/2200 -> 1100/1510/1900) and the new output rate
(100 -> 50 Hz) get exercised against real ESCs under load before the boat is
free-running.

| Check | Pass criterion |
|---|---|
| Neutral | both channels hold trim; no rotation, no creep |
| Endpoint sweep | commanded effort -> measured pulse matches `EscMapper`; both directions reach full scale without clipping |
| Sign | positive physical-forward effort turns each prop the right way |
| Arming | full sequence completes; repeat across 10 alternating launches (the `navigator-cpp` regression) |
| RC kill | neutral within one PWM period; release restores thrust (§12(b)) |
| RC deadman, enabled | handset off -> boat safes in any mode (§12(d)) |
| Watchdog | stop publishing `BB_MIXED_CMD` -> PWM neutralizes at TTL, `NVGR_PWM_WATCHDOG_COUNT` increments |
| Source handoff | autonomy -> RC -> teleop; no thrust step, authority events logged |
| Timing | PWM update interval p95 within 25 ms; zero `Iterate` overruns |

What a dock test cannot give you: anything requiring motion — turn response,
tracking error, the §8.3 metrics. Those need the free run.

Standing rules regardless: never restart `fs-mission` on a live boat (props
spin — wait for a reboot).

---

## 9. Honest limits

**This refactor does not close the gap measured in §2.1 on its own.** The
evidence says the heading-error-to-rotation path is 4.40 s against ArduRover's
0.40 s, and that the actuation half of it — differential to rotation — is only
0.50 s. The refactor removes roughly 0.3 s of transport lag from that 0.5 s and
makes the manual paths crisp. The remaining seconds live in the heading loop,
which is `pBBPID`'s gains plus the seat-crossing rate, and is out of scope by
your call. Note the distinction: `pBBPID` becomes the deployed controller as
part of this work (§12(a)); what stays out of scope is retuning or
restructuring it.

That is the right call for now — the actuation path has to be trustworthy and
explainable before controller tuning means anything — but it should be stated
plainly: expect a clearly better-feeling boat under RC and teleop, a modest
autonomy improvement, and a remaining gap that only the controller work closes.

The contracts are chosen so that work is not blocked later: `AUTONOMY_CMD`
carries `control_mode=HEADING_SPEED`, leaving room for a front-seat inner
yaw-rate loop with the back seat commanding heading and speed. No Phase 2-4
decision should close that door.

**`MOT_THST_ASYM = 1.6` is copied, not validated.** Blue Robotics' number for
the BlueBoat120. Copy first, tune only from evidence.

---

## 10. Work packages

| WP | Status | Content | Depends on | Gate |
|---|---|---|---|---|
| **P0** | **done** | navigator-cpp `main` + restart test; `control-refactor` branch | - | met (§3) |
| **P1** | **done** | `lib_bb_command` + generated golden vectors + CTest wiring | §4.1 | met — `ctest` 8/8, 113,244 checks (§13) |
| **P2** | **in progress** | Replay harness (Tier 2) incl. synthetic fail-closed streams | P1 | reader + mixer/ESC A/B done (§14); full-pipeline replay blocked on P3 |
| **P3** | not started | Mission swap to `pBBPID`; new pipeline apps (§7.1 + §7.2): the three producers, `pBBCommandArbiter`, `pThrustMix` rewrite | P1, P2 | Replay reproduces their output exactly, incl. synthetic fail-closed streams |
| **P4** | not started | `iBBNavigatorInterface` rewrite + cutover flag (§7.3) | P1, P2, P3 | Replay green; dock test passes (§8.4) |
| **P5** | not started | 50 Hz front seat; `uLoadWatch`; latency measurement (§7.4) | P4 | p95 latency < 40 ms; Pi holds 50 Hz under real sensor + comms load |
| **P6** | not started | On-water acceptance (§8.3), current stack then new | P5 | Same-hull before/after recorded |
| **P7** | not started | Remove `LEGACY_DIRECT`; update `pBB_DGPS_EKF` and status consumers to use returned applied effort | P6 | No legacy command path remains |

P2 moves ahead of the app work deliberately: the replay harness is the gate for
P3, so it has to exist before there is anything to gate. P3's three producers
and two consumers were separate packages in an earlier draft, split by a
shadow-mode gate that no longer exists; they share one gate now, so they are one
package.

**P2 and P3 partially interleave, and the table above cannot show that.** The
parts of the harness that replay *existing* contracts are done and have already
paid for themselves (§14). The part that replays the *new* contracts end to end
cannot be written until those contracts have a producer, so P2 closes shortly
after P3 starts rather than strictly before it. The gate is unaffected: no P3
app is considered done until replay reproduces its output.

The one thing running-on-the-boat-early would have proven — that four apps hold
50 Hz on the Pi under real sensor and comms load — is measured at P5, where it
belongs, not inferred from a partial pipeline.

---

## 11. What "done" looks like

- Four apps, four sentences: `iRCInterface` decodes the handset;
  `pBBCommandArbiter` decides who is driving; `pThrustMix` converts surge/yaw to
  left/right effort; `iBBNavigatorInterface` converts effort to pulses and owns
  the hardware.
- The interesting logic is a hardware-free library that `ctest` runs in under a
  second.
- Any actuator frame in any log joins backward to exactly one stick movement or
  one controller output.
- Any on-water behavior reproduces at a desk from the `.alog`.
- Heading tracking error measurably better than the §2.1 `via` baseline on the
  same hull, with the residual gap to ArduRover attributable to the controller
  work still to come.

---

## 12. Decision record

Nothing is open. Everything below is settled; the entries are kept because
several of them constrain how the arbiter and the Navigator get written.

**(a) `pBBPID` is the controller.** Moving fully to it; it is what we test on.
The alpha mission swaps `pMarinePIDV22` -> `pBBPID`, and `pBBPID` publishes
`AUTONOMY_CMD` directly (decision 3). No temporary packer, no held autonomy path.

*Consequence to manage:* the controller swap lands inside the refactor window.
Take the §8.3 baseline run **after** the `pBBPID` swap and **before** the
actuation cutover, so the before/after isolates the actuation refactor instead of
blending two changes.

*Unchanged:* `pBBPID`'s gains and structure stay untouched (§9). Deploying it is
not the same as tuning it.

**(b) RC kill is enforced at the Navigator.** Single enforcement point on the
hardware side, where it survives an arbiter or mixer failure. The arbiter
subscribes to `RC_KILL_ASSERTED` for **trace only** — recording the authority
event and stop reason so the log explains the standstill — and does not gate on
it.

*Consequence, matching today's deliberate behavior:* because the arbiter does not
hard-stop, the mixer keeps running and its slew state stays live, so releasing
kill restores thrust immediately rather than ramping. If that is ever unwanted,
letting the arbiter hard-stop resets the slew and gives a ~0.5 s ramp.

**(c) Slew state carries across a source handoff.** Taken as a call, since you
had no strong view and it is cheap to reverse. Default: carry, logging the
handoff and carried value as an authority event. Ships as
`slew_reset_on_handoff`, default `false`, so it flips without a rebuild.

**(d) The RC deadman: opt-in, blanket, Navigator-enforced.**

> If enabled, RC controller disconnect safes the boat **regardless of current
> mode**. Normally disabled; enabled when running experimental code.

Default off. No mode exceptions, no interaction with authority selection — link
gone, boat safe.

*Where it lives follows from why it exists.* The purpose is a guaranteed stop
when the code above it is not trusted. Implementing it inside the arbiter would
route it through the very code it exists to protect against, which is the same
argument as (b) for the kill switch. So it is **enforced at the Navigator**,
alongside the kill sideband, reading RC link liveness directly.

*This is separate from the source lease, which is never disableable.* The lease
says "never drive on stale commands from whoever is currently driving" and lives
in the arbiter (invariant 6). The deadman says "no RC link, no operation at all"
and lives at the Navigator. Two mechanisms, two homes, no contradiction — the
earlier conflict came from treating them as one.

**(e) No `UNKNOWN` switch state.** CH6 is SF, 2-position (`1 = AUTO, 2 = RC`,
[`rc_controllers.md:248`](rc_controllers.md)), so with the link up the mode is
binary. Two narrow states remain, neither a third switch position:

- *Guard band.* A reading between the bands is invalid; the arbiter holds the
  last valid mode, as the current CH6 latch already does.
- *Before the first valid frame at boot.* Real, and it must **not** inhibit
  autonomy — a boat launched with the handset off or out of range has to run.
  The arbiter treats "no RC ever seen" as not-manual and lets autonomy proceed
  rather than fail-closed-forever. With the deadman enabled, (d) covers that case
  instead, at the Navigator.

**(f) Dock test before the first free run.** Checklist in §8.4. Covers the ESC
endpoint and output-rate change against real ESCs under load, with the boat
restrained.

### 12.1 Deferred by design — scheduled, not open

- **Timeout values.** Decision 7 settles the policy (start lenient); the numbers
  come from the age distributions logged from P3 onward, tightened after P5.
- **`MOT_THST_ASYM = 1.6`.** Copy first, tune only from tank or on-water
  evidence.
- **Whether teleop needs a direct-tank maintenance mode** (design doc §16.8).
  Scope question for `iTeleop` in §7.1; decide when that app is opened.
- **The residual autonomy gap.** `pBBPID` gains plus the seat-crossing rate —
  out of scope by your call, quantified in §9.

### 12.2 Pending actions, not decisions

- **navigator-cpp: tabled** (§3). Three items deferred together — commit the two
  staged changes, retarget the ported test from 100 Hz to 50 Hz per decision 2,
  and delete `jeremy-dev` — picked up with the arming bench re-run.
- Amend design doc §5.1 / §16.9 per (d): the source lease and the RC deadman are
  separate mechanisms with separate homes, and the deadman is a legitimate
  opt-in blanket rather than a bypass to be retired.
- Commit `docs/` — this plan and the design doc it accompanies are still
  untracked; `da05bb8` took the library and tests only.

---

## 13. P1 build log

**Status: complete.** `./build.sh --unit_tests && (cd build && ctest)` — 8/8
passing, 113,244 checks.

`src/01_libraries/lib_bb_command/`:

| Unit | Notes |
|---|---|
| `command_envelope` | strict `key=value` parser, fails closed on every malformed form; unknown *optional* keys preserved for forward compatibility |
| `command_mailbox` | epoch/seq lease; a repeated sequence does not refresh freshness |
| `slew_limiter` | common throttle only, per §2.6; adds a `max_dt` clamp ArduPilot has no need for |
| `skid_mixer` | ArduRover 4.7 port, branch-for-branch |
| `esc_mapper` | 1100 / 1510 / 1900, per-channel reversal, the only place inversion happens |

CTest is now wired repo-wide (`enable_testing()` in the root, `add_test()` per
suite), and `lib_crsf`'s three existing suites were retrofitted into it — they
had been build-only since they were written.

### 13.1 Golden vectors

55,890 vectors generated from a transcription of the real
`AP_MotorsUGV::output_skid_steering()` (local checkout `09c814147c`), swept over
`thrust_asymmetry x steering_throttle_mix x surge x yaw` including the branch
boundaries. Generator kept in `test/tools/` with the upstream provenance and
every edit listed, so the transcription can be audited against upstream by eye.

All five worked examples in design doc §9.6 reproduce exactly from the real
function — the hand-derived arithmetic in that document was correct.

### 13.2 Two findings from getting the tests to pass

**Limit flags are not comparable across precisions at boundaries.** All 55,890
*effort* pairs matched immediately. The *limit flags* did not, in two families.
Both trace to ArduPilot running float32 where this library runs double, with the
flags coming from strict inequalities: at a tie the two precisions land on
opposite sides. The clearest case is `surge=15, A=2.5`, where ArduPilot computes
`15.0f*0.01f` as `0.14999999106` (rounding down) while the steering range rounds
*up* to `0.55000001192`; their difference falls just past the lower throttle
limit, so `q` comes out `1.0000001`, the saturation branch runs, and
`throttle_upper` gets set. In double the same quantities cancel to exactly `1.0`
and the branch does not run.

The test now measures the *margin* on each inequality and exempts a row only
when the tightest margin is below what float32 can resolve — currently 306 rows,
0.548%, with the suite failing if that ever exceeds 1%. A first attempt probed
by nudging the inputs instead, which missed an entire band of yaw values,
because the steering value there is clamped and does not move when the input
does. Worth knowing before trusting a similar probe elsewhere.

**A permissive parser branch.** The envelope parser initially accepted a
trailing comma, via a special case that existed only to permit it. Removed:
`pos` can only reach the end of the string by following a comma, so an empty
token there is a trailing comma and not a normal loop exit. Doubled and trailing
commas are both `malformed` now.

### 13.3 Deviations from ArduPilot, all deliberate

1. **double, not float.** Not on an STM32; the precision is free. Consequence is
   §13.2.
2. **Non-finite input fails closed** rather than propagating NaN. ArduPilot
   trusts its callers; we do not (invariant 12).
3. **`SlewLimiter` has a `max_dt` clamp** with no ArduPilot equivalent.
   ArduPilot runs a fixed-rate scheduler; a MOOS app can be descheduled, and
   without the clamp one long stall grants an unbounded step — the limiter would
   stop limiting at the worst possible moment.
4. **Not ported:** wheel-rate control, `MOT_THR_MIN`, `MOT_THST_EXPO`,
   `MOT_REV_DELAY`. All four are identity at BlueBoat120 defaults; porting
   disabled code would be untested code.

Committed as `da05bb8` (library and tests; `docs/` not included). Next: **P2**.

---

## 14. P2 progress: replay harness

**Status: partial — the reader and comparison scaffolding are in; full-pipeline
replay waits on P3.**

`src/02_applets/bb_replay/` — a host tool (no MOOS, no navigator-cpp, no
hardware) with a minimal `.alog` reader and three modes: `--mode=mixer`,
`--mode=esc`, `--mode=scan`.

What it cannot do yet: replay a whole pipeline against `NVGR_ACTUATOR_TRACE`.
That needs `pBBCommandArbiter` to exist and logs that carry the new contracts.
The reader, the pairing discipline, and the comparison scaffolding are what that
will be built on.

### 14.1 Mixer A/B on 72 minutes of real commands

`via`, 5 Aug 16:27, 69,248 paired samples. New ArduRover mixer (A=1.6, m=0.6)
against the `DESIRED_THRUST_L/R` the old `pThrustMix` actually produced, from
the same logged inputs.

| | old | new |
|---|---:|---:|
| yaw authority commanded, median `\|L-R\|` | 10.24 | **23.37** |
| p90 | 70.53 | **157.57** |
| saturation (either side at full scale) | 4.00% | **20.23%** |

The new mixer commands **more** differential in 87.5% of samples, and roughly
2.3x more at the median. That is the change most directly aimed at G1, and the
old configuration explains why: `k_inner=0.6 / k_outer=0.4` with speed scaling
that drops turn gain to **0.2 at zero speed**, ramping to 1.0 only at 1.0 m/s.
The boat was deliberately given a fifth of its turn authority in exactly the
low-speed regime where §2.2 says trim corrections matter.

Per-side differences split hard by regime: median 21.08 while turning
(`|rudder|>10`) versus 1.48 near-straight. The two mixers agree when going
straight and diverge when turning, which is what should happen.

**Saturation rises 4% -> 20%.** More authority means more time at the rail. That
is not free: it makes the ESC endpoint change (§2.5) matter more rather than
less, and it means `MOT_STR_THR_MIX` is now doing real work — 20% of the time it
is deciding what to give up. Worth watching on the dock test and the first free
run.

### 14.2 A harness bug worth recording

The first run reported **102 turn-direction disagreements** — the two mixers
choosing opposite turns. That would have been alarming if true. It was not.

`pThrustMix` publishes the pair back to back in one `Iterate`:
`Notify(L); Notify(R);` — so R for tick N carries a timestamp fractionally later
than L for tick N. Looking R up with `at_or_before(t_L)` therefore returns tick
**N-1**'s value: a systematic one-tick skew on one side only, manufacturing
differentials that were never commanded. Pairing by index instead dropped the
count from 102 to 10.

The remaining 10 were then classified rather than assumed. Each is checked for
whether the yaw command was steady in a +/-150 ms window, since the mixer input
is up to one tick stale relative to the logged output and a flip found during a
rudder reversal says nothing about the mixers. Result: **10 of 10 occurred
during a transition, 0 with a steady command.** There is no real turn-direction
disagreement between the two mixers.

Both checks stayed in the tool. The index/timestamp pairing warns if the series
lengths differ, and the steady/transient split prints every run, so a future
regression cannot hide behind the same ambiguity.

The general lesson for the rest of P2: **a replay harness that pairs samples
wrongly produces confident, specific, wrong answers.** The full-pipeline replay
will pair far more variables than two, and every one of them needs this
question asked explicitly.

### 14.3 ESC mapping, measured

Over the same log's 138,496 single-side commands, the old 800/2200 mapping put
**7.96%** outside the ESC's documented 1100-1900 us window, 2.00% at 2200 us —
300 us beyond spec. Consistent with the 8.0% computed independently in §2.5.
Under the new mapping every command in `[-100,100]` lands inside the window by
construction, which `test_esc_mapper` pins.


---

## 15. P3 progress: authority state machine

**Status: started.** The authority decision logic is in
`lib_bb_command/authority.{h,cpp}` with `test_authority.cpp` — 107 checks, suite
now 9/9.

It lives in the library rather than inside `pBBCommandArbiter` for the same
reason the mixer does: every rule that matters is then testable without MOOS, a
clock, or a boat, and replayable offline. The app becomes mail handling and
publication around a pure `decide()`.

### 15.1 Three divergences from design doc §6

The doc's pseudocode predates the §12 decisions and contradicts them in three
places. Recorded here because the doc still shows the old form:

1. **RC kill does not gate.** The doc hard-stops on kill inside the arbiter.
   Per decision (b) the Navigator enforces it, so the arbiter only records it
   for the trace. Gating here would put the safety property behind the software
   it is meant to outlive.
2. **No `UNKNOWN` mode branch.** Per decision (e) CH6 is two-position. What
   survives is the pre-first-frame window, covered by
   `allow_autonomy_before_first_rc` (default true) so a boat launched with the
   handset off can still run.
3. **No RC deadman.** Per decision (d) it is a Navigator-enforced opt-in
   blanket. The *source lease* lives in the arbiter and is never disableable.
   Keeping them separate is what resolved the contradiction.

### 15.2 Mutation testing found a vacuous test

All 22 scenarios passed on the first run, which for a safety-critical file is a
reason for suspicion rather than confidence. Three deliberate mutations were
introduced to see whether the tests actually bite:

| Mutation | Caught |
|---|---|
| RC-stale falls through to autonomy (the exact bug invariant 6 forbids) | yes — 4 checks |
| Teleop takes authority without a claim | yes — 2 checks |
| Manual authority cap also applied to autonomy | **no** |

The third passed the entire suite. The autonomy test frames omitted
`authority_limit`, which defaults to 100, so capping autonomy at 100% is a
no-op and the assertion `autonomy is never capped` could not distinguish "not
capped" from "capped at full scale". The test frame now carries an explicit
`authority_limit=40`, and the mutation is caught (2 checks).

Worth generalising: an assertion about something being *ignored* is vacuous
unless the input actually carries a value that would change the answer. There
are probably more of these; mutation testing is cheap and should be run against
`pBBCommandArbiter` and the mixer wrapper as they land.

### 15.3 `pThrustMix` rewritten

Done. `BB_SELECTED_CMD -> BB_MIXED_CMD`, front seat, `AppTick=50`.

New library pieces it needed: `parse_decision()` + `DecisionMailbox` (the
consumer view of `BB_SELECTED_CMD`, with the same epoch/seq lease as
`CommandMailbox`), and `MixerStage` — slew, allocation, lineage propagation and
fail-closed input handling as a pure component. `test_mixer_stage.cpp` adds 93
checks; suite is 10/10.

`serialize_decision()` was missing `decision_epoch`, which the consumer lease
keys on. Without it an arbiter restart looks like a permanent sequence
regression. Fixed.

The app is ~250 lines and holds no control logic. Its header lists what it must
never learn to do — subscribe to a second source, emit PWM, or close a feedback
loop — because each of those is a plausible-sounding future change that would
undo the separation. The old mixer's speed-gain scheduling and yaw-rate
correction are gone: yaw control belongs to `pBBPID`, and two interacting yaw
loops is not a design.

`mixer_model` is required with no default, and configuration validation **fails
startup** rather than warning.

**Three input failures get three stop reasons** — `MIXER_INPUT_INVALID` (never
heard from the arbiter), `MIXER_INPUT_STALE` (arbiter went quiet), and the
arbiter's own reason propagated unchanged. They are different faults and a log
that conflates them cannot be debugged.

### 15.4 Verified end to end on a real MOOSDB

Not just unit tests — `MOOSDB` + `pThrustMix` + `pLogger`, driven by `uPokeDB`:

| Check | Result |
|---|---|
| pivot `surge=0, yaw=100` | L=**62.50**, R=**-100.00** within 23 ms — the exact ArduRover values |
| yaw not slewed | full deflection on the first cycle |
| surge ramp 0 -> 100 | 0.500 s at 200 %/s, matching `MOT_SLEWRATE` |
| lease window after a frame | ~23 cycles at 50 Hz = 0.46 s, matching the 0.5 s timeout |
| parser rejects | 0 of 61 frames |
| lineage on the wire | source -> decision -> mix, intact |

One trap worth recording: `uPokeDB` costs ~2 s per invocation, so pokes arrived
every 2.007 s against a 0.5 s lease and the mixer was correctly stale ~75% of
the time. That looked at first like a bug — a frame arriving and the app
reporting stale immediately after — and it is not. Any future MOOS-level smoke
test needs an injector faster than the lease it is testing, or it measures the
injector.

### 15.5 Mutation testing, again

Four mutations on `MixerStage`, all caught: slewing yaw as well as surge (4
checks), skipping the limiter reset on hard stop (2), ignoring
`slew_reset_on_handoff` (1), and overwriting the arbiter's stop reason instead
of propagating it (1).

### 15.6 `pBBCommandArbiter` built

New app, front seat, `AppTick=50`. Like `pThrustMix` it holds no logic — the
state machine is §15's `AuthorityArbiter`. Configuration validation fails
startup; `make_epoch()` moved into the library so both apps mint epochs
identically rather than drifting apart.

Publishes `BB_SELECTED_CMD` **every cycle including stop cycles** (a gap is
indistinguishable from a dead arbiter, and the mixer's lease needs a steady
sequence), plus `BB_AUTHORITY_EVENT` immediately on any transition, carrying
`fail_closed=1` when a manual owner lost the boat. That is the variable to read
first after an incident.

`BB_ARB_LAST_VALID_*_AGE` is published per source from day one, which is how the
lenient timeouts of decision 7 eventually get tightened from evidence.

### 15.7 Full chain verified: arbiter -> mixer -> effort

`MOOSDB` + `pBBCommandArbiter` + `pThrustMix` + `uTimerScript` + `pLogger`.
Scripted scenario: autonomy alone, then RC takes MANUAL, then RC goes silent
while still in MANUAL — with autonomy publishing valid commands throughout.

```
t=1.71  authority NONE     -> AUTONOMY      (mix stop NONE)
t=4.70  authority AUTONOMY -> RC            (RC outranks a healthy autonomy)
t=8.68  authority RC       -> NONE          RC_STALE, fail_closed=1
```

**The fail-closed case is the one that matters.** Autonomy was fresh and valid
the whole time and did not get the boat. The transition:

```
t=8.6754  arbiter decides NONE / RC_STALE
t=8.6989  mixer publishes L=0.00 R=0.00 / RC_STALE
```

23.5 ms end to end, one 50 Hz cycle, no ramp-down — the hard stop bypassed the
slew limiter as invariant 7 requires. While RC drove, `surge=80, yaw=40`
produced **L=100.00, R=30.667**: the design doc §9.6 worked example, reproduced
through two apps and a MOOSDB rather than in a unit test.

The injector ran at 20 Hz, comfortably faster than every lease under test —
applying the §15.4 lesson rather than re-learning it.

### 15.8 `iTeleop` publishes `TELEOP_CMD`

Done, additively — the legacy `TELEOP_ACTIVE` / `TELEOP_THRUST_L/R` stay while
the Navigator still consumes them.

**The GUI speaks tank, the contract speaks semantic.** `applyCommand()` now
prefers `SURGE`/`YAW` fields if the GUI sends them, and otherwise derives
`surge = (L+R)/2`, `yaw = (L-R)/2`. That derivation is **not a round trip and
the operator will notice**: a GUI asking for a symmetric pivot (L=+100,
R=-100) becomes `surge=0, yaw=100`, which the ArduRover mixer renders as
L=62.5, R=-100 — deliberately unequal so the two *physical* thrusts match. That
is the intended consequence of teleop going through the same allocation as every
other source rather than driving motors directly, but it means a tank GUI's
numbers are now a request rather than a promise. `cmd_form` is on the wire
(`semantic` | `derived_from_tank` | `none`) so a "teleop feels different"
report can be answered from the log.

**`claim=0` is published, not implied by silence.** The contract goes out every
iterate whether or not a session is active, and `deactivateSession()` publishes
immediately rather than waiting a cycle. The arbiter treats an explicit release
differently from silence: release lets autonomy resume at once, silence with a
held claim fail-closes. Going quiet on release would have turned every normal
teleop hand-back into a stop.

Verified on a live MOOSDB: `TELEOP_CMD` publishes while idle with `claim=0`,
`valid=1`, advancing sequence, and the emitted frame parses through the library
parser with `source` stamped by the mailbox rather than read from the text.

### 15.9 Broker check for `AUTONOMY_CMD`

`AUTONOMY_CMD` has to cross the seat boundary, and the contract is full of
commas and `=`. `lib_broker_v2` percent-encodes everything outside
`[A-Za-z0-9_-.:]` and delimits fields with `|`, so the string survives intact as
an opaque value. No escaping work needed; the only cost is that encoding roughly
doubles the string length, which is nothing at 10 Hz.

### 15.10 `pBBPID` publishes `AUTONOMY_CMD`

Additive; `DESIRED_THRUST` / `DESIRED_RUDDER` stay as diagnostics.

`valid` maps the controller's three states: **holding off** (no complete
nav/cmd picture) and **stale** (helm stopped commanding) both publish
`valid=0` with zeros; **active** publishes `valid=1`. All three publish
something — going silent during hold-off would read to the arbiter as staleness,
which is a different fault from alive-but-not-ready.

The two invalid cases already command zero, so the boat stops either way. The
difference is what the log says and what the mixer does: `valid=1` with zeros
makes the arbiter *select* autonomy and pass a controlled zero through the slew
limiter; `valid=0` hard-stops with `AUTONOMY_INVALID`, names the fault, and
resets the limiter so resuming ramps from rest.

**`rudder_polarity` is now deprecated, and the reason is new.** It was an
internal convention knob. Since `rudder` *is* the contract's `yaw`, and the
contract defines `yaw > 0 = starboard`, a mis-set polarity no longer produces an
internally inconsistent sign — it publishes a **contract violation**, turning the
boat the wrong way under autonomy while RC still behaves correctly. Kept
configurable for now by decision, marked DEPRECATED in `-e` output, and to be
removed once the pipeline is fully on one convention.

`lib_broker_v2` percent-encodes the contract safely (§15.9), so no broker work
was needed beyond adding `AUTONOMY_CMD` to `tx_vars` — deferred to the mission
swap so all mission edits land together.

### 15.11 `iRCInterface` publishes `RC_INPUT_STATE`

**`surge = CH3`, `yaw = CH1`, with no sign transform.** Those channels are
already `+ = ahead` and `+ = starboard` in the handset mixes, which is the
contract's convention exactly.

That **retires `rc_stick_convention`**. The v1/v2 knob existed because
`iBBNavigatorInterface`'s RC mixer implemented the legacy RadioLink wire; once
RC emits semantic values there is nothing left to get backwards, and the
"turns correct, throttle reversed" failure mode stops being expressible. The
knob still exists for the legacy path and dies with it in P4.

**The important part is what happens on link loss.** The legacy scalars fall to
their operator-absent defaults, `RC_CH6 -> 1 = AUTO`, which is correct for that
contract. The command contract must NOT do that: publishing `mode=NON_MANUAL`
because the link died would tell the arbiter the operator *chose* autonomy, and
the boat would hand itself to the back seat at the exact moment the operator
lost it — the silent fallthrough invariant 6 exists to prevent.

So `mode` and `kill` are latched only while connected and **retained across a
dropout**, while `valid` goes to 0. The arbiter then sees "the operator still
holds MANUAL and cannot command" and hard-stops with `RC_INVALID`. Same physical
outcome as a stop, but the log names the real fault and autonomy never gets a
look in.

`authority_limit` keeps its enable flag by decision; disabled publishes 100
rather than omitting the field, so a log always shows what cap was in force.

Verified with no receiver attached: `valid=0, surge=0, yaw=0, mode=UNKNOWN,
link=LOST`, sequence advancing. `mode=UNKNOWN` is what lets a boat launched with
the handset off still run (decision (e)). **The retain-across-dropout path
cannot be verified without a handset** — it needs the HIL bench, and it is the
single behaviour in this app most worth bench-proving before water.

### 15.11a Bench data already existed, and it changes two things

`docs/archive/rc/rc_characterization.md` is measured, not a plan: 6,805 frames,
70 s of captures, 2026-08-11. Two findings bear directly on this work.

**The RC source runs at 50 Hz, so `AppTick = 16` is decimating the operator.**
"All 16 channel slots update at ~50 Hz effective (min value-change interval
19.9-20 ms on every continuously-moved channel)". Running `iRCInterface` at
16 Hz throws away roughly two thirds of the stick updates the receiver is
already delivering. Raising it to 50 is not a guess about headroom; it is
matching the rate the data arrives at. Still tabled by decision, but the
justification is now evidence rather than symmetry with the other apps.

**The failsafe signature is known, which de-risks the retain-on-dropout logic.**
"0x16 frames stop DEAD - instantly, no fade, no fabricated values, no failsafe
frames." So on link loss there is nothing to misread: frames simply cease, the
staleness detector fires at ~35 ms, and the mode/kill latch stops updating with
its last real value intact. Recovery is equally clean - frames resume within
~0.4 s "same values +/-2 ticks", so the retained mode and the physical switch
re-sync almost immediately. The mechanism is now characterised rather than
assumed. Bench confirmation is still worth doing, but it is a check rather than
a discovery.

**`link=DEGRADED` now uses the measured LQ decay.** After TX loss the 0x14 link
stats keep arriving for ~1.1 s with uplink LQ decaying linearly 100 -> 0. That
is about a second of real early warning and the only signal separating "about to
lose the operator" from "fine". `RC_INPUT_STATE` reports DEGRADED at LQ < 80
while frames are still valid, and carries `lq` on the wire, so a log shows the
link dying before the command stops and an incident review can tell a range
problem from an app fault.

### 15.12 Next in P3

The `pMarinePIDV22` -> `pBBPID` mission swap (§12(a)), which also adds
`AUTONOMY_CMD` to the back-seat broker `tx_vars` and puts
`pBBCommandArbiter` + the rewritten `pThrustMix` on the front seat.

---

## 16. P4 in progress: the Navigator

### 16.1 Library first: `ActuatorStage`

Before touching the 2,030-line app, its safety logic went into the library as a
pure component — the third and last stage, matching `AuthorityArbiter` and
`MixerStage`. `actuator_stage.{h,cpp}` plus `parse_mixed()` / `MixedMailbox`
(the consumer view of `BB_MIXED_CMD`, same lease discipline as the other two).
`test_actuator_stage.cpp`: 46 checks, suite now 11/11.

`ActuatorStage` owns every rule about *when the props may turn*: the two
sidebands that bypass the command path (RC kill, RC deadman), the three hardware
conditions (disarmed, unhealthy, shutting down), and the three upstream failures
(no mixer, quiet mixer, mixer stopped) — each with its own stop reason. It emits
an `ActuatorFrame` carrying the full source -> decision -> mix lineage plus a TTL,
so the PWM thread consumes one atomic value and neutralises past its expiry.

`StopReason` grew the Navigator-local values (`RC_KILL`, `RC_DEADMAN`,
`PWM_DISARMED`, `HARDWARE_FAULT`, `SHUTDOWN`, `NAV_INPUT_*`). The arbiter never
emits these.

### 16.2 A comment that claimed more than the code did

Mutation testing again, and the interesting one failed to fail.

| Mutation | Caught |
|---|---|
| Deadman trusts only the link flag, ignoring RC observation age | yes — 2 checks |
| Upstream stop reason overwritten instead of propagated | yes — 2 checks |
| Local safety evaluated *after* the command | **no** |

Chasing it down changed what I think the ordering does. The header claimed local
safety runs first "so a kill or a hardware fault cannot be masked by a valid
command arriving in the same cycle." **That is not true, and the code never did
it.** Every branch returns early, so there is no ordering in which a fault gets
driven through — both orderings stop the boat.

What the ordering actually decides is *which reason the log records* when a local
fault and an upstream fault are true in the same cycle. With the mixer quiet AND
the kill asserted, local-first reports `RC_KILL` — the operator's action — rather
than `NAV_INPUT_STALE`, a downstream symptom of it. That is a G4 property, worth
exactly what an incident review is worth, and not a safety one.

The suite missed it because every local-safety test paired the fault with a
*valid* command, and under that pairing both orderings give the same answer. The
new test asserts a kill racing a stale input, and a hardware fault racing one.
Header comment corrected to say what the code does.

Second-order lesson, on top of §15.2's: a mutation that "passes" may not have
applied. The first attempt at this one moved the block back above the upstream
checks — `str.index()` found the first of two `mixed.snapshot()` calls — so the
file compiled, the tests passed, and nothing had changed. **Verify the mutation
landed before believing the test that survived it.**

### 16.3 Audit of the existing app

Read before cutting, checking against navigator-cpp `main`. Eight findings,
ordered by what they could cost.

**(1) The wrong magnetometer on a V2 board.** The app hardcodes
`read_mag_ak09915()` ([`:1903`](../src/iBBNavigatorInterface/BBNavigatorInterface.cpp#L1903)).
`main` added `read_mag()`, which dispatches V1 -> AK09915, V2 -> MMC5983 with an
AK fallback. On a V2 Navigator the app is reading a magnetometer that may not be
there. **Latent, not active** — the fleet runs `use_mag = false` — but it is a
trap armed for whoever turns mag fusion on during the EKF work. Fix: call
`read_mag()`.

**(2) `init()` failure is never checked.** `g_nav.init()` is called in the
CONSTRUCTOR ([`:206`](../src/iBBNavigatorInterface/BBNavigatorInterface.cpp#L206)),
its error string stored in `m_last_sensor_error`, and never tested.
`is_initialized()` and `is_pwm_ready()` are called **zero** times in the app. So
a board that failed to come up yields a running app that publishes sensor values
and accepts thrust commands. Two fixes: check the return, and feed
`is_pwm_ready()` into `NavigatorSafetyState.hardware_healthy`, which is what
that new API exists for. Also worth moving init out of the constructor — it does
hardware I/O before `OnStartUp()` has read any configuration.

**(3) The sensor thread's sleep is computed from the wrong interval.**

```cpp
auto current_time = now();          // top of loop
...work...
auto elapsed = current_time - prev_time;   // the PREVIOUS full period
prev_time = current_time;
sleep(period - elapsed);
```

`elapsed` is the previous iteration's *total* period (work + sleep), not this
iteration's work. So the loop alternates: one cycle sleeps, the next computes a
non-positive sleep and runs flat out. Average period is `(work + nominal)/2`, so
a thread configured for 150 Hz actually runs somewhere near 200-250 Hz with
alternating intervals.

The integration is partly protected because `dt` is *measured* rather than
assumed. But it is then clamped to `[0.5*nominal, 3*nominal]`, and the short
half of the alternation lands under that floor whenever a sensor read takes less
than 3.33 ms — at which point the estimator is told more time passed than did,
and over-integrates on every other sample. **Whether that is happening depends
on the real SPI read time and has not been measured.** Worth measuring on the
boat: it would be a third contributor to the NAV_HEADING spike investigation,
alongside the two already confirmed.

**(4) `armIfNeeded()` blocks for 2 s and can run from `OnNewMail`.** The
`NVGR_DISARM=false` re-arm path ([`:475`](../src/iBBNavigatorInterface/BBNavigatorInterface.cpp#L475))
calls it while the PWM thread is running. Two problems: the MOOS mail thread
stops for two seconds, and the arm sequence does not own the pins it is holding
neutral — the modulation thread rewrites them every 20 ms from its own atomics.
It works today only because `requestDisarm()` zeroed those atomics first and
mail is blocked so nothing can change them. That is a coincidence, not a design.

**(5) The PWM thread free-runs slightly slow.** `sleep_for(20ms)` after the work
gives a period of `20 ms + work`, so it is always under 50 Hz and drifts with
load. `sleep_until` on an absolute deadline fixes it.

**(6) The nav-light LED loop overwrites itself.**

```cpp
for (int i = 0; i < 12; i++) {
  rgb_array[i]     = port;        // 0..11
  rgb_array[2 * i] = starboard;   // 0,2,...,22
}
```

Starboard overwrites port on every even index 0-10, fills evens 12-22, and odd
indices 13-23 are never written at all. The intent was presumably 12 per side.
Cosmetic, but it means nobody has looked at the nav lights and believed what
they saw.

**(7) `PWM_FREQ_HZ = 100`** against decision 2's 50 Hz. Changes with the rewrite.

**(8) `setPinPulseWidth()` reads file-scope globals** for min/max
([`:63`](../src/iBBNavigatorInterface/BBNavigatorInterface.cpp#L63)) because it
is static so the signal handler can reach it. `EscMapper` plus an explicit
neutral-pulse accessor replaces the whole arrangement.

### 16.4 The rewrite

`BBNavigatorInterface.cpp` (MOOS lifecycle + config) 521, `NavActuator.cpp` 265,
`NavSensors.cpp` 499, header 230. **2,428 -> 1,515 lines.**

The line count undersells it. Grepping the arbitration/mixing markers
(`rc_mode`, `teleop`, `all_stop`, `calculateRC`, `mixer`, `deadband`, `LPF`,
`invert`): **149 hits in the old app, 0 in `NavActuator.cpp`**, 2 in the
lifecycle file and both in comments saying what no longer happens. The actuation
path is `m_stage->update(...)` plus a PWM writer. Config parsing went from ~410
lines to ~90; the fifteen retired parameters are named explicitly so an
un-migrated plug says `RETIRED parameter 'thruster_alpha' is ignored` rather
than a generic warning an operator skims past.

What did **not** shrink is the sensor half — 499 lines of AHRS, calibration
parsing, ADC, baro, leak and lights, none of which this refactor touches. That
is roughly half the remaining app. The split is what makes it navigable: the
file you open to reason about thrust is 265 lines and contains no policy.

### 16.5 Two things the rewrite got wrong first

**Guessed struct fields.** `NavADCData` is `channel[4]`, not `a0..a3`;
`NavBaroData` is `pressure_kpa` / `temperature_c`, already in kPa. Worse, the
first draft read voltage from ADC2 and current from ADC1 — the real wiring is
**current on ADC3, voltage on ADC4**, with a per-battery-count calibration table
of which only the 4-battery row has ever been measured. Wrong field *names*
failed to compile; the wrong *channel mapping* would have compiled cleanly and
published plausible battery telemetry that was simply false. Restored verbatim.

**Removed a capability that had just been proven.** The first pass dropped the
`NVGR_DISARM=false` in-run re-arm, on the grounds that it blocked the mail
thread for two seconds and raced the PWM writer for the pins. Both true. But
`rc_controllers.md` records that path being broken by the PCA9685 RESTART trap,
fixed, and **bench-passed 2026-08-14** — "first-ever hardware pass of this path"
— and it exists to serve the RC handset's disarm/arm request, which has to work
mid-mission. Deleting it regressed something that had just been made to work.

Rebuilt as a **state machine owned by the PWM thread** (`DISARMED -> ARMING ->
ARMED`). Mail records only the request. The 2 s hold happens where neutral is
actually written every cycle, so it holds something; nothing blocks; and there
is one arm path whether the request comes from launch or from `NVGR_DISARM`.

`performDisarm()` uses `navigator_force_pwm_off()` for an orderly all-off with
readback before raising OE. `pwm_enable(false)` alone leaves the channels
RUNNING, which is exactly what arms the RESTART trap for the next
`set_frequency` — the mechanism that made in-run re-arm panic the ESCs before
the August fix. The orderly stop is why that bench test passes, and it belongs
on this path permanently.

Kill and disarm stay distinct, as designed: **kill** is a neutral-lock with the
ESCs left ARMED so recovery is instant; **disarm** is the signal cut and
re-arming costs the hold.

### 16.6 A stale library, found by a link error

`libnavigator_cpp.dylib` was dated **Jul 24** — built before `main`. The build
compiled against the current header and linked the old library, so
`is_pwm_ready()` failed to resolve. Rebuilt (a prerequisite, not one of the
tabled navigator-cpp changes).

Worth noting the near miss: this surfaced as a link error only because the
symbol was new. A change that altered behaviour behind an unchanged signature
would have linked silently against stale code. Anything depending on
navigator-cpp `main` needs that library rebuilt, including on the boats.

### 16.7 Full chain, off-vehicle

Four apps, three contracts, on a laptop:

```
t=1.91  arbiter  NONE -> AUTONOMY
t=1.91  mixer    AUTONOMY_INVALID -> NONE  (driving)
t=0.76  nav      HARDWARE_FAULT, PWM not armed, props neutral
469 NVGR_ACTUATOR_TRACE frames
```

The Navigator refusing to drive is the CORRECT result: there is no Navigator
board on a laptop, `init()` fails, `is_pwm_ready()` is false, so
`hardware_healthy` is false and `ActuatorStage` holds neutral. That is finding
(2) of the audit working — the old app would have driven regardless.

The lineage survives all three hops intact:

```
source_epoch=au1/191 -> decision_epoch=arb-0081a9/590 -> mix_epoch=mix-bc09b3/...
```

**What this proves:** the contracts connect, the apps co-exist at 50 Hz, and the
fail-closed path works end to end.

**What it does not:** no PWM was ever written, no ESC mapping was exercised, and
the arm state machine never left DISARMED because arming is gated on
`m_nav_init_ok`. The entire actuation path is still unexercised.

### 16.8 Where testing actually stands

| Layer | State |
|---|---|
| Pure logic (`lib_bb_command`) | **Strong.** 11 suites, mutation-tested, ~113k checks |
| Contract plumbing | **Verified** — §16.7, four apps, lineage intact |
| Arbiter -> mixer timing | **Verified** — 23.5 ms fail-closed, §15.7 |
| Actuation path | **Never run.** No pulse has been written by this code |
| Arm/disarm state machine | **Never run.** Highest-risk code in the refactor |
| Anything on a Pi | **Never run.** All of the above is macOS with stubs |
| RC retain-on-dropout | **Never run.** Needs a handset |
| 50 Hz under real load | **Never measured** |
| Sensor-thread rate fix | **Never measured** — the point of publishing it |

**The structural problem: P2 was supposed to gate P3 and P4, and it is still
half-built.** The replay harness does mixer and ESC A/B against old logs; the
full-pipeline replay and the synthetic fail-closed streams — stale sources,
epoch restarts, kill assertion, watchdog expiry — were never written. Those
streams are the safety argument for going replay -> dock -> water with no
on-water shadow phase (§8.2), and they do not exist. Two work packages have been
built past their own gate.

**And a gap the gate cannot close as designed.** The arm/disarm state machine is
the highest-risk untested code — its predecessor took three attempts and a
PCA9685 datasheet trap to get right — and it is **not reachable by the replay
harness**, because it lives in the app interleaved with hardware calls. Replay
can drive contracts; it cannot drive `pwm_enable()`.

The fix is the one already applied three times: extract it. The transition logic
is pure — `(want_armed, now, hold_start, state) -> (next state, action)` — with
the hardware calls behind a narrow interface. Then it is unit-testable and
mutation-testable like `AuthorityArbiter`, `MixerStage` and `ActuatorStage`, and
the bench is confirming a tested design rather than discovering an untested one.

### 16.9 Closing the gaps in §16.8

**Arm sequencer extracted** (`lib_bb_command/arm_sequencer.{h,cpp}`, 543
checks). The transition logic is now pure — `(want_armed, now, hardware_ok) ->
action` — with the caller performing `BEGIN_ARM` / `HOLD_NEUTRAL` / `DISARM` /
`DRIVE`. It was the last part of the actuation path no test could reach.

Five mutations, all caught. The two that matter both concern **every exit from
ARMING or ARMED going through DISARM**, because DISARM is where the caller
latches the channels off, and an exit that skips it leaves them running and arms
the PCA9685 RESTART trap for the next attempt. An abort mid-hold that merely
returned to DISARMED would have looked correct and broken the next arm.

One deliberate asymmetry, pinned by test: a hardware blip while **ARMED** does
*not* disarm. `ActuatorStage` already holds the props at neutral on
`HARDWARE_FAULT`, which is safe and instantly recoverable; cutting the signal
would cost a 2 s re-arm mid-mission. A blip during **ARMING** does disarm — the
output is already enabled and must be cut.

**Synthetic fail-closed streams written** (`bb_replay --mode=fail`, 14
scenarios, registered in CTest). Each drives the real `AuthorityArbiter`,
`MixerStage` and `ActuatorStage`, serialising and re-parsing at every boundary
so the parsers are exercised too. Nothing is mocked but the clock:

```
RC stale in MANUAL stops, no fallthrough to autonomy
teleop stale with a held claim stops
a repeating broker sequence does not keep the boat alive
producer restart accepted, new epoch in lineage
RC kill stops, and releasing it recovers at once
enabled deadman stops a healthy autonomous boat
ALL_STOP blocks autonomy but never the RC rescue
arm -> disarm -> re-arm, mid-mission, no relaunch
every actuator frame resolves to one source sample
...14 total
```

This is what §8.2 promised and P2 had not delivered. The gate now exists.

**Suite: 13 suites green.**

### 16.10 Mission wiring

Front seat gains `plug_pBBCommandArbiter.moos` and `plug_pThrustMix.moos`, both
at `AppTick=50`; the Navigator plug is rewritten for the new contract with all
fifteen retired parameters removed. Back seat swaps `pMarinePIDV22` +
`pThrustMix` for `pBBPID`, and `iBackSeatBroker` now carries `AUTONOMY_CMD`
instead of `DESIRED_THRUST_L/R` — only semantic surge/yaw crosses the seat
boundary now.

`plug_pBBPID.moos` carries the gains **verbatim** from the via 5 Aug 16:27 run.
Deliberately not retuned: the actuation path underneath them has changed (ESC
endpoints, ~2.3x more commanded yaw authority, LPF removed), and retuning on top
of an unmeasured actuation change would confound the two.

Verified by expanding the real mission with `nsplug` and launching it: **zero
config warnings from all three apps**, correct idle state
(`AUTONOMY_INVALID` -> `HARDWARE_FAULT` -> `DISARMED`), no run warnings.

### 16.11 Dock procedure

[`dock_test_control_refactor.md`](dock_test_control_refactor.md) — bench with no
ESCs, bench with ESCs and no props, then wet and tied. It leads with the
alternating-launch regression and the disarm/re-arm cycle because those are the
failures with history, and it puts the kill-switch check first once the boat is
in the water.

### 16.12 Next

The rewrite: strip arbitration, mixing, the LPF and the post-mix deadband;
rebuild the PWM writer around the atomic `ActuatorFrame` and its TTL; keep the
sensor thread, AHRS, ADC, baro, leak and LEDs; split into `NavActuator` /
`NavSensors` translation units; fix (1)-(8) on the way through.
