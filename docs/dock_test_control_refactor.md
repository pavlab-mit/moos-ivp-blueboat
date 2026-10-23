# Dock test: control refactor

Status: **procedure, not yet run**

The gate between the desk and a free-running boat. Everything upstream of this
has been tested without hardware ([`control_refactor_plan.md`](control_refactor_plan.md)
§16.8); nothing downstream of it is safe to assume.

**Two things change at once in this cutover**, and that is why this test exists:
the ESC pulse endpoints move from `800 / 1500 / 2200` to `1100 / 1510 / 1900`,
and the output rate moves from 100 Hz to 50 Hz. The first time either reaches a
real ESC is here.

Anything not matching the expected result is a **stop and diagnose**, not a
retry.

---

## 0. Prerequisites

- [ ] **`navigator-cpp` rebuilt and installed ON THE PI.** Not just pulled.
      The dev machine had a `libnavigator_cpp` from 24 Jul linked against a
      newer header; it surfaced as a link error only because `is_pwm_ready()`
      was a new symbol. A behavioural change behind an unchanged signature
      would have linked silently.
      ```
      cd ~/moos/navigator-cpp && git log --oneline -1   # expect 8e2ec24 or later
      ./build.sh && ./install.sh
      ```
- [ ] `moos-ivp-blueboat` built on the Pi from `control-refactor`.
- [ ] `./build.sh --unit_tests && (cd build && ctest)` green **on the Pi**, not
      only on the laptop. 14 suites (test_wire_format joined in the audit
      follow-up).
- [ ] `./bin/bb_replay --mode=fail` — 14/14 on the Pi.
- [ ] **Props OFF** for §1 and §2. They go on only at §3.
- [ ] Battery charged; a second person on the kill switch for §3.
- [ ] **Launch with the debug flag.** `pLogger` is DEBUG-gated by decision
      (routine ops must not fill the Pi with logs), so a default launch
      produces NO `.alog` — and §4 requires one. Confirm after launch that
      `pLogger` appears in `uProcessWatch` and a fresh log directory exists.
      `uLoadWatch` is unconditional (§1.7 needs it; it writes nothing).
- [ ] **No `bb-esc-failsafe` unit enabled on the Pi.** The watcher was
      removed from the repo (decision, 2026-08-27); if a boat ever had it
      installed, `systemctl disable --now bb-esc-failsafe` — a stale copy
      still expects 100 Hz / 1500 us and would fight the new configuration.

---

## 1. Bench: board only, no thrusters connected

Navigator board powered, ESCs **unplugged**. A logic analyser or scope on the
two PWM signal pins if available; a multimeter reading DC on the pin is enough
to distinguish alive from dead (1510 us at 50 Hz ~= 7.5% duty ~= 0.25 V of
3.3 V).

### 1.1 Startup and arming

- [ ] Launch the front seat. `iBBNavigatorInterface` appcast shows
      `Board: V1|V2   init OK   pwm_ready yes`.
      **The gate is `pwm_ready`.** `init PARTIAL: [...]` with `pwm_ready yes`
      means a SENSOR failed (dead mag, IMU bus fault): actuation testing may
      proceed, but read the warning and understand it — autonomy will
      self-inhibit without nav data. `pwm_ready NO` is a hard stop: the app
      holds neutral and will not arm, which is the point. (The old app drove
      regardless of any of this.)
- [ ] **First-run trap, seen 2026-08-27:** stale sysfs GPIO exports (from
      anything that ever ran the OLD navigator-cpp this boot) hold the SPI CS
      lines and fail the IMU/mag init with `libgpiod ... Device or resource
      busy` for pins 16/17. Check `ls /sys/class/gpio/` — `gpio16`/`gpio17`
      present means unexport them or reboot. A clean boot running only the
      new stack cannot produce this.
- [ ] `NVGR_ARM_STATE` goes `DISARMED -> ARMING -> ARMED` within ~2.5 s.
- [ ] During `ARMING`, both pins carry a steady trim pulse.
- [ ] `NVGR_STOP_REASON` settles at the reason that names the actual state,
      **not** `HARDWARE_FAULT`:
      - full front-seat mission, no handset, no back seat:
        `AUTONOMY_INVALID` (the arbiter's stop, propagated through the mixer —
        the mixer and arbiter are alive, autonomy has never produced);
      - Navigator alone, no mixer running: `NAV_INPUT_INVALID`;
      - `NONE` appears **only while a source is actually driving** — it is not
        the idle state.

### 1.2 The alternating-launch regression

This is the failure that took three attempts and a PCA9685 datasheet trap to
fix (`rc_controllers.md` §8.3). It presents as ESCs beeping at an enabled line
carrying no pulses, on every other launch.

- [ ] Launch / quit **10 times in a row**. Every launch reaches `ARMED` with
      live pulses. No alternation.
- [ ] Every QUIT (Ctrl-C or kill) leaves the ESCs **silent at neutral** —
      signal still on the wire, no panic beeping. Beeping on exit means the
      exit path cut the signal instead of parking neutral: that regression
      shipped once (2026-08-27, first dock session) and was fixed the same
      day; this check keeps it fixed. (`disarm_on_exit = true` beeping is
      correct — that config asks for the signal cut.)
- [ ] `test_pca9685_restart` (navigator-cpp) run 3x back to back: HEALTHY each
      time. *(Note: that test still hardcodes 100 Hz — see §5.)*

### 1.3 The disarm / re-arm cycle

The handset's disarm request, mid-mission. This bench-passed 2026-08-14 and the
rewrite reimplemented it as a state machine; the orderly channel all-off before
the signal cut is what keeps it passing.

- [ ] `NVGR_DISARM=true` → pulses stop, `NVGR_ARM_STATE=DISARMED`.
- [ ] `NVGR_DISARM=false` → `ARMING` for ~2 s → `ARMED`, pulses return.
- [ ] Repeat **5 times**. No panic, no alternation, `arm cycles` increments each
      time in the appcast.
- [ ] Disarm *during* the arming hold: state returns to `DISARMED` cleanly and a
      subsequent re-arm still works. (An abort that skipped the all-off would
      arm the trap for the next attempt.)

### 1.4 Endpoint sweep — the changed mapping

**Poking `BB_MIXED_CMD` by hand needs two accommodations**, both lessons the
plan already recorded (§15.4): a `uPokeDB` invocation costs ~2 s against a
0.5 s lease, and a re-poked frame with the same `mix_seq` is a duplicate that
does **not** refresh freshness. So:

1. For this section only, set `mixed_cmd_timeout_sec` large (e.g. `3600`) in
   the Navigator plug, and restore it before §1.5 — the watchdog checks are
   meaningless with a long lease.
2. Advance `mix_seq` by hand on every poke. A valid minimal frame:

   ```
   uPokeDB targ_*.moos BB_MIXED_CMD="v=1,mixer_model=BENCH_POKE,mix_epoch=bench,mix_seq=1,mix_time=0,selected=NONE,hard_stop=0,stop=NONE,left_effort=50.0,right_effort=0.0"
   ```

   (`iBBNavigatorInterface` must be ARMED; kill and deadman clear.)

- [ ] Feed `BB_MIXED_CMD` as above across effort `-100 .. +100` in
      steps of 10, on each side independently.
- [ ] Measured pulse matches `EscMapper` within 1 us:
      - effort `0` → **1510** us both sides
      - left `+100` → **1100** us  (SERVO3 is reversed)
      - left `-100` → **1900** us
      - right `+100` → **1900** us
      - right `-100` → **1100** us
- [ ] No commanded value in `[-100, 100]` produces a pulse outside
      **1100-1900**. Under the old mapping 8.0% of real commands did.
- [ ] Output rate measures **50 Hz**, not 100.

### 1.5 Watchdogs and sidebands

*(Restore `mixed_cmd_timeout_sec = 0.5` first if §1.4 raised it.)*

- [ ] Kill `pThrustMix`. Pulses go to trim within ~0.5 s (the
      `mixed_cmd_timeout_sec` lease) and `NVGR_STOP_REASON=NAV_INPUT_STALE`.
      **`NVGR_PWM_WATCHDOG_COUNT` does NOT increment here** — the Navigator's
      own Iterate is still alive and committing fresh neutral frames. The PWM
      watchdog (frame TTL) fires only when *Iterate itself* stalls, which
      cannot be staged from outside the process; that path is pinned by
      `test_actuator_stage` and the fail-closed scenarios instead. If the count
      DOES increment during this check, the Navigator main thread is stalling —
      stop and diagnose.
- [ ] Flip the handset KILL switch (CH5) while a command is flowing → trim
      within one PWM period, `NVGR_STOP_REASON=RC_KILL`, `NVGR_ARM_STATE`
      stays **ARMED**. *(Use the real switch, not `uPokeDB`: with
      `iRCInterface` running, a poked `RC_KILL_ASSERTED` is overwritten by the
      app's own publish within one AppTick.)*
- [ ] Release it → drive resumes immediately, no re-arm. *(Kill is a neutral
      lock; disarm is the signal cut. They must not behave alike.)*
- [ ] With `rc_deadman_enabled=true` and a healthy autonomy command flowing,
      power the handset off → `RC_DEADMAN` stop. Restore → drive resumes.

### 1.6 Sensor rate — audit finding (3)

- [ ] `NVGR_SENSOR_RATE_HZ` reads **~150** against `sample_rate = 150`.
      The pre-rewrite loop slept on the previous period rather than this
      iteration's work and ran nearer 200-250 with nothing showing it. If this
      reads high, the fix did not take.
- [ ] `NVGR_IMU_READ_ERRORS` and `NVGR_IMU_GLITCHES` stay at 0 over 5 minutes.

### 1.7 Load

- [ ] `uLoadWatch` on the front seat: no app over its tick budget with all four
      at 50 Hz. **If the Pi cannot hold 50 Hz, stop here** — that is a plan
      assumption, not a detail.

---

## 2. Bench: ESCs connected, props still OFF

- [ ] Repeat §1.1 with ESCs powered. Arm tones normal, no panic beeping.
- [ ] Endpoint sweep again, listening: motor spins up smoothly in both
      directions, no stutter at the zero crossing.
- [ ] **Direction check.** Command left `+50` alone: the LEFT motor turns in the
      direction that drives the boat FORWARD. Then right `+50`. Getting this
      wrong is now a one-line config fix (`*_esc_reversed`) and there is
      nowhere else in the pipeline it could hide.

---

## 3. Dock: in the water, tied fore and aft, props ON

Second person on the kill switch throughout. Lines short enough that the boat
cannot reach anything.

- [ ] **Kill switch first, before anything else.** Command thrust, assert kill,
      confirm the props stop. Do not proceed until this is proven wet.
- [ ] Straight ahead, `surge = 30`: both props forward, boat pulls evenly
      against the lines.
- [ ] Straight astern, `surge = -30`.
- [ ] **Pivot, `surge = 0, yaw = 100`.** Expect `left_effort = 62.5`,
      `right_effort = -100` — deliberately unequal, so the two *physical*
      thrusts match under the 1.6:1 forward/reverse assumption. Watch the wash:
      it should look roughly symmetric even though the commands are not. **If it
      is visibly lopsided, `MOT_THST_ASYM = 1.6` is wrong for this hull** — it
      was copied from Blue Robotics, not measured (plan §9).
- [ ] Mirror it, `yaw = -100`.
- [ ] **`rudder_polarity` check.** With the back seat commanding a heading
      change to starboard, confirm the boat tries to turn starboard. This is the
      one config value that now publishes a *contract violation* when wrong, and
      the symptom is autonomy turning the wrong way while RC behaves correctly.
- [ ] RC takeover from autonomy mid-command: instant, no lurch.
- [ ] RC link loss while in MANUAL: **props stop**. They do not hand over to
      autonomy. Confirm `BB_AUTHORITY_EVENT` carries `fail_closed=1`.
- [ ] Full disarm / re-arm cycle from the handset, wet.
- [ ] Current draw at full pivot within expectations on both sides.

---

## 4. Record before leaving the dock

- [ ] Full `.alog` from the front seat.
- [ ] `NVGR_ACTUATOR_TRACE` present for the whole session, and
      `bb_replay` can resolve a sample of frames back to their source.
- [ ] Measured: `NVGR_SENSOR_RATE_HZ`, PWM rate, `NVGR_PWM_WATCHDOG_COUNT`,
      `BB_ARB_LAST_VALID_*_AGE` percentiles.

Those age percentiles are how the deliberately lenient timeouts (plan decision
7) get tightened from evidence rather than taste. **Do not tighten them before
this run.**

---

## 5. Known gaps at the time of writing

- `test_pca9685_restart` still hardcodes `pwm_set_frequency(100.0f)`; it should
  be retargeted to 50 Hz so it exercises the shipping configuration. Tabled with
  the rest of the navigator-cpp work.
- `iRCInterface` retain-mode-on-dropout has never been exercised with a real
  handset. Its inputs are characterised (`rc_characterization.md`: frames stop
  dead, no fabricated values) so the mechanism is known, but §1.5's deadman
  check and §3's link-loss check are the first real confirmation.
- `iRCInterface` still runs at `AppTick = 16` while the receiver delivers at
  50 Hz. Tabled by decision; not a blocker.
- **The handset's FM sensor now reflects the ARBITER** (fixed in the audit
  follow-up): `iRCInterface` sources it from `BB_CMD_AUTHORITY` +
  `NVGR_STOP_REASON`, showing `RC` / `TELEOP` / `AUTO` / `STOP` / `DEADMAN`.
  Worth confirming during §3 mode transitions — it has never been seen
  against a live arbiter.
- **`pBB_DGPS_EKF` drift gating re-pointed** (fixed in the audit follow-up)
  at `NVGR_APPLIED_LEFT/RIGHT`, bridged back via the brokers. Confirm during
  §4 that the back seat actually receives `NVGR_APPLIED_*` and that
  `DRIFT_ACTIVE` is 0 while thrust is applied.
- **`NVGR_DISARM` has no producer.** The dock drives it with `uPokeDB`, which
  is fine here, but the plan's "serves the RC handset's disarm/arm request"
  is aspiration until something maps a handset control to it.
- **An unhandled process death (SIGKILL, kernel OOM) leaves the PCA9685
  free-running its last pulses.** The `bb_esc_failsafe` watcher that covered
  this gap was removed by decision (2026-08-27); the frame TTL does not help
  when the whole process dies, only when Iterate stalls. Known and accepted:
  the mitigations are the RC kill switch and the power switch.
