# Proposed BlueBoat Command Pipeline

Status: **PROPOSED / design draft**

Scope: command ownership, safety, ArduRover-compatible thrust mixing, ESC output,
and traceability

Out of scope: controller gains, retuning the copied BlueBoat mixer parameters, and
estimator changes

Implementation route, sequencing, and test gates: [`control_refactor_plan.md`](control_refactor_plan.md).

This document specifies a cleaner command path for the BlueBoat front seat. It is
written as executable-style pseudocode so that it can become both an implementation
plan and a basis for unit tests.

The central rules are:

> `pBBCommandArbiter` is the only process allowed to decide which command source owns
> the actuators. `pThrustMix` is the only process allowed to convert semantic
> `surge/yaw` into physical left/right effort. `iBBNavigatorInterface` is the only
> process allowed to convert physical effort into ESC output.

`iRCInterface` still owns everything specific to the receiver and handset: CRSF
decoding, link validity, channel calibration, deadzones/expo, switch guard bands,
axis signs, and conversion to semantic operator intent. It does **not** mix left and
right thrust. Arbitration, mixing, and hardware I/O are deliberately separate MOOS
apps with versioned contracts between them.

## 1. Target pipeline

```text
 iRCInterface          iTeleop           back-seat autonomy
      |                   |                       |
 RC_INPUT_STATE       TELEOP_CMD             AUTONOMY_CMD
      |                   |                       |
      +-------------------+-----------------------+
                          |
                 pBBCommandArbiter
            ownership, leases, stop policy
                          |
                   BB_SELECTED_CMD
        surge/yaw + decision and source lineage
                          |
                     pThrustMix
       slew, skid mixing, saturation, boat physics
                          |
                    BB_MIXED_CMD
       left/right effort + complete upstream lineage
                          |
                iBBNavigatorInterface
       final safety, ESC mapping, PWM and sensors
                          |
                     ESC outputs
```

Each app has one narrow responsibility and one primary output contract. No callback
writes actuator state. `OnNewMail()` validates and caches snapshots; each app's
`Iterate()` publishes a coherent result. The Navigator has no RC/teleop/autonomy
priority logic and the mixer has no hardware configuration.

| App | Primary input | Primary output | Owns | Explicitly does not own |
|---|---|---|---|---|
| `iRCInterface` | CRSF receiver frames | `RC_INPUT_STATE` | receiver validity and stick/switch semantics | authority, mixing, ESC signs |
| `pBBCommandArbiter` | RC, teleop, autonomy contracts | `BB_SELECTED_CMD` | leases, priority, stop policy, manual authority cap | mixing, PWM, sensors |
| `pThrustMix` | `BB_SELECTED_CMD` | `BB_MIXED_CMD` | slew, skid mixing, saturation, reverse compensation | source priority, ESC calibration |
| `iBBNavigatorInterface` | `BB_MIXED_CMD` | PWM and `NVGR_ACTUATOR_TRACE` | final safety, ESC mapping, hardware watchdog, sensors | source arbitration, skid mixing |

## 2. Normative invariants

The words MUST, SHOULD, and MAY are normative in this document.

1. **Single actuator authority.** Only `iBBNavigatorInterface` MUST write the
   thruster PWM interface.
2. **Semantic source commands.** Normal command sources MUST use `surge` and `yaw`,
   not hardware-signed left/right PWM or thrust.
3. **Command is not authority.** Publishing a valid command MUST NOT, by itself,
   grant that source actuator ownership.
4. **One writer per input contract.** Each source variable MUST have one configured
   producer. A source MUST NOT publish another source's variable.
5. **Coherent pairs.** Surge, yaw, sequence, validity, and safety metadata MUST be
   transported in one atomic MOOS string. Separate scalar mail MUST NOT be combined
   into a command pair.
6. **No silent fallthrough.** If an active manual owner becomes invalid or stale,
   the boat MUST stop. It MUST NOT unexpectedly fall through to autonomy.
7. **Hard stop is final.** A hard stop MUST bypass slew/filter dynamics, reset their
   state, and command neutral at the next PWM update.
8. **Receiver time controls freshness.** Source timestamps are diagnostic. Freshness
   MUST use the local arrival time of a new producer sequence because front- and
   back-seat clocks may not be synchronized.
9. **Physical motor sign is canonical.** Positive left/right motor effort means
   physical forward thrust. Electrical inversion MUST occur only in the ESC mapper.
10. **Traceable output.** Every non-neutral actuator frame MUST identify the source
    `epoch + seq`, arbiter `decision_epoch + decision_seq`, and mixer
    `mix_epoch + mix_seq` that produced it.
11. **Applied output is telemetry.** Allocated effort, mapped PWM, and the actual
    actuator frame MUST be published back to MOOS. They are outputs, not recycled
    command inputs.
12. **Non-finite input fails closed.** NaN, infinity, malformed fields, duplicate
    keys, unsupported versions, and out-of-range required fields MUST invalidate the
    whole snapshot.

## 3. Canonical units and signs

| Field | Range | Sign convention |
|---|---:|---|
| `surge` | `[-100, 100]` | positive is forward |
| `yaw` | `[-100, 100]` | positive is starboard / clockwise |
| `authority_limit` | `[0, 100]` | percent of available manual authority |
| `left_effort` | `[-100, 100]` | positive is physical forward thrust |
| `right_effort` | `[-100, 100]` | positive is physical forward thrust |
| `pwm_*_us` | ESC-specific | electrical output, generated only by ESC mapper |

The semantic command is intentionally dimensionless. A future version may use
physical force and moment, but it must use a new major contract version.

## 4. Common command envelope

Each source publishes a comma-delimited `key=value` snapshot. Version 1 values may
not contain commas. Parsers reject duplicate keys and unknown required enum values.

Required common fields:

```text
v=1
producer=<stable process name>
epoch=<new boot/session identifier>
seq=<strictly increasing unsigned integer within epoch>
source_time=<producer MOOSTime; diagnostic only>
valid=<0|1>
surge=<-100..100>
yaw=<-100..100>
```

Example:

```text
v=1,producer=iRCInterface,epoch=7f2c91,seq=18422,source_time=381.420,
valid=1,surge=42.5,yaw=-8.0,mode=MANUAL,kill=0,authority_limit=65
```

The line break above is only for readability; the actual MOOS value is one line.

Envelope rules:

- `epoch` MUST change whenever the producer restarts or begins a new operator
  session. It should be a random boot identifier rather than wall-clock time.
- `seq` MUST advance whenever the producer emits a new control sample. Repeated
  broker frames carrying the same `epoch + seq` do not refresh command freshness.
- A new epoch resets sequence comparison, but it does not automatically grant
  authority.
- Older or out-of-order sequences MUST be rejected and reported in trace telemetry.
- `source_time` MAY help correlate logs but MUST NOT determine expiration.
- The receiver records `rx_time` locally only when it accepts a new sequence.
- Version 1 parsers SHOULD ignore unknown optional fields, but MUST reject an
  unsupported major version.

Suggested parser result:

```cpp
struct SemanticCommand {
    uint32_t version;
    CommandSource source;       // assigned by receiving mailbox, not trusted from text
    std::string producer;
    std::string epoch;
    uint64_t seq;
    double source_time;         // diagnostic only
    double rx_time;             // local time of accepted new sequence
    bool valid;
    double surge;
    double yaw;
    double authority_limit;     // defaults to 100 for sources without this field
};
```

## 5. Source-specific contracts

### 5.1 `RC_INPUT_STATE`

Producer: `iRCInterface`

Consumer: `pBBCommandArbiter`

Required additional fields:

```text
mode=<MANUAL|NON_MANUAL|UNKNOWN>
kill=<0|1>
link=<VALID|DEGRADED|LOST>
authority_limit=<0..100>
deadman_switch=<ENABLED|DISABLED>
```

Rules:

- `iRCInterface` owns CRSF parsing, CRC/frame checks, link/failsafe state, channel
  calibration, center deadzone, optional expo, axis sign, and guarded switch decode.
- `surge` and `yaw` are already conditioned semantic stick intent.
- `iRCInterface` MUST NOT perform left/right allocation or ESC inversion.
- `valid=1` requires a valid receiver frame and valid required controls.
- `mode=MANUAL` expresses the guarded physical mode-switch state. It is not inferred
  from nonzero sticks.
- On link loss, `iRCInterface` MUST retain the last valid guarded mode and kill state;
  it MUST NOT synthesize `NON_MANUAL` from safe-default channel values. Before the
  first valid switch decode, mode is `UNKNOWN` and the default policy is fail-closed.
- `kill=1` is assert-dominant. `RC_KILL_ASSERTED` is also sent directly to the
  Navigator as the final hardware safety sideband; either assertion causes a hard
  stop.
- Clearing a latched kill requires a new, valid RC snapshot showing `kill=0` and the
  configured operator reset gesture/state.
- Source lease expiry is mandatory even if the legacy deadman switch is disabled.
  `deadman_switch` is retained during migration but MUST NOT permit stale RC effort
  to remain applied indefinitely.
- Raw `RC_CH*` variables remain diagnostic/backward-compatible and MUST NOT be used
  by the new authority or mixer code.

### 5.2 `TELEOP_CMD`

Producer: `iTeleop`

Consumer: `pBBCommandArbiter`

Required additional fields:

```text
session=<operator session identifier>
claim=<0|1>
estop=<0|1>
```

Rules:

- `claim=1` requests teleop ownership; command presence alone does not.
- A claimed session that becomes stale causes a hard stop and does not fall through
  to autonomy.
- A newer valid session may replace an old session only through an explicit session
  handoff rule in `iTeleop`; the arbiter logs the session transition.
- Normal teleop uses `surge/yaw`. Direct tank commands, if retained for diagnostics,
  MUST use a separately named and separately enabled maintenance contract.
- `estop=1` causes a hard stop while teleop owns or is attempting to own authority.

### 5.3 `AUTONOMY_CMD`

Producer: the back-seat command pipeline

Consumer: `pBBCommandArbiter` through `iBackSeatBroker` / `iFrontSeatBroker`

Required additional fields:

```text
controller=<controller/model identifier>
control_mode=<HEADING_SPEED|WAYPOINT|OTHER>
```

Rules:

- The producer should publish one new sequence at every controller update.
- The broker may repeat the latest field map, but a repeated command sequence does
  not prove the controller is alive and does not refresh command freshness.
- `ALL_STOP` gates autonomy independently of the command envelope. This preserves
  manual rescue while the autonomy mission is stopped.
- Back-seat restart changes `epoch`; the arbiter waits for a valid command in the new
  epoch before autonomy can become eligible.
- `controller` is required trace metadata. It does not change arbitration priority.
- During migration, legacy `DESIRED_THRUST_L/R` may be accepted only through an
  explicitly enabled `LEGACY_DIRECT` adapter. New and legacy autonomy inputs MUST
  never be active simultaneously.

### 5.4 Safety state

Safety signals are not command sources. They are divided by responsibility:

```cpp
struct ArbitrationSafetyState {
    bool rc_kill_asserted;
    bool autonomy_all_stop;
};

struct NavigatorSafetyState {
    bool pwm_armed;
    bool hardware_healthy;
    bool shutdown_requested;
    bool rc_kill_asserted;
};
```

`RC_KILL_ASSERTED` is deliberately delivered to both `pBBCommandArbiter` and
`iBBNavigatorInterface`. This is a safety sideband, not a second command path. The
arbiter records the correct authority decision; the Navigator can neutralize the
hardware even if the arbiter or mixer has failed. `ALL_STOP` remains an autonomy
policy input to the arbiter. Local actuator faults, shutdown, and disarm belong only
to the Navigator.

## 6. Authority decision contract

```cpp
enum class CommandSource { NONE, RC, TELEOP, AUTONOMY, LEGACY_DIRECT };

enum class StopReason {
    NONE,
    STARTUP,
    RC_KILL,
    RC_INVALID,
    RC_STALE,
    TELEOP_ESTOP,
    TELEOP_INVALID,
    TELEOP_STALE,
    AUTONOMY_ALL_STOP,
    AUTONOMY_INVALID,
    AUTONOMY_STALE,
    PWM_DISARMED,
    HARDWARE_FAULT,
    SHUTDOWN,
    INTERNAL_FAULT
};

struct SourceEvaluation {
    CommandSource source;
    bool requesting_authority;
    bool eligible;
    double age;
    std::string reason;
    SemanticCommand command;
};

struct AuthorityDecision {
    std::string decision_epoch;
    uint64_t decision_seq;
    double decision_time;
    CommandSource previous_source;
    CommandSource selected_source;
    bool hard_stop;
    StopReason stop_reason;
    SemanticCommand selected_command;
    SourceEvaluation rc;
    SourceEvaluation teleop;
    SourceEvaluation autonomy;
};
```

Priority is `RC > TELEOP > AUTONOMY`, but invalid active ownership is fail-closed:

```pseudo
function decide_authority(now, arbitration_safety, rc, teleop, autonomy):
    decision_seq += 1

    if arbitration_safety.rc_kill_asserted or rc.fresh_kill_asserted:
        return HARD_STOP(RC_KILL)

    rc_eval = evaluate_rc(now, rc)
    teleop_eval = evaluate_teleop(now, teleop)
    auto_eval = evaluate_autonomy(now, autonomy,
                                  arbitration_safety.autonomy_all_stop)

    if rc.mode_is_manual:
        if rc_eval.eligible:
            return SELECT(RC, rc.command)
        if rc.has_never_produced_valid_command or not rc.command.valid:
            return HARD_STOP(RC_INVALID)
        return HARD_STOP(RC_STALE)

    if teleop.claim_is_active:
        if teleop.estop:
            return HARD_STOP(TELEOP_ESTOP)
        if teleop_eval.eligible:
            return SELECT(TELEOP, teleop.command)
        if teleop.has_never_produced_valid_command or not teleop.command.valid:
            return HARD_STOP(TELEOP_INVALID)
        return HARD_STOP(TELEOP_STALE)

    if arbitration_safety.autonomy_all_stop:
        return HARD_STOP(AUTONOMY_ALL_STOP)
    if auto_eval.eligible:
        return SELECT(AUTONOMY, autonomy.command)
    if autonomy.has_never_produced_valid_command or not autonomy.command.valid:
        return HARD_STOP(AUTONOMY_INVALID)
    return HARD_STOP(AUTONOMY_STALE)
```

The timeouts are source-specific configuration values. They must be selected from
measured publication rates and validated on the bench; this draft does not freeze
their numeric values.

`controlled_zero` and `hard_stop` are intentionally different:

- A selected source commanding `surge=0,yaw=0` is a controlled zero and MAY pass
  through normal slew behavior.
- A safety or freshness failure is a hard stop and MUST bypass all smoothing.

### 6.1 `BB_SELECTED_CMD`

Producer: `pBBCommandArbiter`

Consumer: `pThrustMix`

```text
v=1,decision_epoch=arb-a91c,decision_seq=9214,decision_time=392.580,
selected=RC,hard_stop=0,stop=NONE,
source_producer=iRCInterface,source_epoch=7f2c91,source_seq=18422,
surge=42.5,yaw=-8.0
```

The actual value is one line. `decision_epoch` changes whenever the arbiter restarts.
`decision_seq` advances once per arbiter control cycle, including hard-stop cycles.
The output MUST copy the selected source identity exactly. If `hard_stop=1`, the
arbiter publishes `surge=0,yaw=0` and a non-`NONE` reason.

This contract is the only normal input to `pThrustMix`. The mixer does not subscribe
to RC, teleop, autonomy mode, or `ALL_STOP`, so it cannot accidentally implement a
second authority policy.

### 6.2 `BB_MIXED_CMD`

Producer: `pThrustMix`

Consumer: `iBBNavigatorInterface`

```text
v=1,mixer_model=ARDUROVER_4_7_SKID,
mix_epoch=mix-471b,mix_seq=33402,mix_time=392.582,
decision_epoch=arb-a91c,decision_seq=9214,
selected=RC,hard_stop=0,stop=NONE,
source_producer=iRCInterface,source_epoch=7f2c91,source_seq=18422,
surge_in=42.5,yaw_in=-8.0,surge_shaped=41.8,yaw_shaped=-8.0,
left_effective=33.8,right_effective=49.8,
left_effort=33.8,right_effort=49.8,
thrust_asymmetry=1.6,steering_throttle_mix=0.6,q=0.498,f=1.0,limits=none
```

The mixer increments `mix_seq` for each output cycle and copies all upstream lineage
unchanged. The Navigator rejects stale, malformed, unsupported, or regressing mixer
frames. It does not recompute the mix.

## 7. Command pipeline pseudocode

### 7.1 `pBBCommandArbiter`

```pseudo
function OnNewMail(messages):
    parse and cache RC_INPUT_STATE, TELEOP_CMD, and AUTONOMY_CMD
    update RC_KILL_ASSERTED and ALL_STOP policy state
    never publish from this callback

function Iterate():
    now = MOOSTime()
    inputs = snapshot_all_mailboxes_atomically()
    decision = decide_authority(now, inputs)

    if decision selects a manual source:
        // Manual authority cap is policy, not mixing or ESC calibration.
        decision.surge *= decision.authority_limit / 100
        decision.yaw   *= decision.authority_limit / 100

    if decision is hard stop:
        decision.surge = 0
        decision.yaw = 0

    publish BB_SELECTED_CMD(decision)
    publish BB_AUTHORITY_EVENT_if_changed(decision)
    publish arbiter diagnostics
```

Its source mailbox acceptance remains strict and side-effect free:

```pseudo
function CommandMailbox.accept(parsed, arrival_time):
    reject malformed, unsupported, non-finite, or out-of-range snapshots

    if parsed.epoch == current.epoch and parsed.seq == current.seq:
        count duplicate; do not refresh freshness
    else if parsed.epoch == current.epoch and parsed.seq < current.seq:
        reject out-of-order sequence
    else:
        accept parsed snapshot
        record local rx_time = arrival_time
```

### 7.2 `pThrustMix`

The mixer subscribes only to `BB_SELECTED_CMD`. It owns vehicle-level shaping and
allocation, but no source priority and no PWM values.

```pseudo
function OnNewMail(messages):
    if message.key == "BB_SELECTED_CMD":
        selected_mailbox.accept_strict(message, MOOSTime())

function Iterate():
    now = MOOSTime()
    dt = control_clock.sane_dt(now)
    selected = selected_mailbox.snapshot()

    if selected missing, stale, malformed, regressing, or selected.hard_stop:
        surge_limiter.reset(0)
        skid_mixer.reset()
        publish BB_MIXED_CMD(
            left_effort=0,
            right_effort=0,
            hard_stop=1,
            stop=selected reason or MIXER_INPUT_STALE,
            complete available upstream lineage)
        return

    shaped_surge = surge_limiter.update(selected.surge, dt)
    shaped_yaw = selected.yaw
    allocation = skid_mixer.allocate(shaped_surge, shaped_yaw)

    publish BB_MIXED_CMD(
        allocation,
        mix_epoch,
        next mix_seq,
        exact decision and source lineage copied from selected)
    publish mixer diagnostics
```

This is the natural evolution of the existing `pThrustMix`. In the new layout it
runs on the front seat, after arbitration. The back seat transports semantic
autonomy `surge/yaw`; it no longer transports autonomy-only left/right mixing as the
final boat command.

### 7.3 `iBBNavigatorInterface`

The Navigator consumes one command variable, `BB_MIXED_CMD`. It does not know the
source precedence and does not recalculate left/right effort.

```pseudo
function OnNewMail(messages):
    if message.key == "BB_MIXED_CMD":
        mixed_mailbox.accept_strict(message, MOOSTime())
    else if message.key == "RC_KILL_ASSERTED":
        rc_kill_latch.observe(message)
    else:
        update sensor, hardware, arming, or configuration state

function Iterate():
    now = MOOSTime()
    mixed = mixed_mailbox.snapshot()
    safety = snapshot_local_safety()

    if safety requires stop:
        applied = neutral(reason = local safety reason)
    else if mixed missing, stale, malformed, regressing, or mixed.hard_stop:
        applied = neutral(reason = mixed reason or MIXER_INPUT_STALE)
    else:
        applied = esc_mapper.map(mixed.left_effort, mixed.right_effort)

    applied copies source, decision, and mix lineage
    applied.expires_at = now + ACTUATOR_FRAME_TTL
    actuator_exchange.commit(applied)
    publish NVGR_ACTUATOR_TRACE(applied)
    publish sensor telemetry on its own schedule
```

### 7.4 PWM writer

The PWM thread receives one atomic frame. It never reads partially updated left/right
members and it has its own watchdog.

```cpp
struct ActuatorFrame {
    std::string decision_epoch;
    uint64_t decision_seq;
    std::string mix_epoch;
    uint64_t mix_seq;
    CommandSource source;
    std::string source_epoch;
    uint64_t source_seq;
    StopReason stop_reason;
    double left_effort;
    double right_effort;
    int left_pwm_us;
    int right_pwm_us;
    double committed_at;
    double expires_at;
};
```

```pseudo
function pwm_writer_tick(now):
    frame = actuator_exchange.latest_atomic_frame()

    if shutdown_requested or not pwm_armed:
        write_neutral_both()
    else if frame is missing or now > frame.expires_at:
        write_neutral_both()
        report_pwm_watchdog_stop()
    else:
        write_pwm_pair(frame.left_pwm_us, frame.right_pwm_us)
```

`write_pwm_pair` should update the pair as closely together as the hardware permits.
Shutdown and signal handlers continue to own an independent neutral path.

## 8. Mixer and ESC contracts

These are separate pure components so each can be replayed against a log and unit
tested without MOOS or hardware. `SurgeSlewLimiter` and `SkidMixer` live in
`pThrustMix`; `EscMapper` lives in `iBBNavigatorInterface`.

### 8.1 Surge slew limiter

```pseudo
function SurgeSlewLimiter.update(request, dt):
    max_step = slew_rate_pct_per_sec * dt
    state = clamp(request, state - max_step, state + max_step)
    return state
```

This is the ArduRover behavior: a symmetric, `dt`-based limit on common throttle
before mixing. The BlueBoat default is `200 %/s`, so at 50 Hz common surge may change
by at most four percentage points per cycle. A hard stop still bypasses this limiter
and resets it to zero. On a valid source-to-source handoff, it may be seeded from
current applied common thrust to avoid a step; the handoff and seed value must be
logged.

### 8.2 Skid-steer mixer

```cpp
struct AllocationResult {
    double left_effective;
    double right_effective;
    double left_effort;
    double right_effort;
    bool surge_limited;
    bool yaw_limited;
    bool reverse_compensation_active;
    double saturation_value;
    double fair_scaler;
    double steering_priority_scale;
};
```

```pseudo
function SkidMixer.allocate(surge, yaw):
    assert finite_and_in_range(surge, yaw)
    return ardurover_skid_mix(
        throttle = surge / 100,
        steering = yaw / 100,
        thrust_asymmetry = 1.6,
        steering_throttle_mix = 0.6)
```

The complete ArduRover formula and proposed port are in the next section. It is not
equivalent to `left=surge+yaw`, `right=surge-yaw`, followed by normalization. It uses
an asymmetric feasible region, a tunable steering-versus-throttle saturation policy,
and reverse-efficiency compensation.

### 8.3 ESC mapper

```pseudo
function EscMapper.map(left_effort, right_effort):
    physical_left  = clamp(left_effort,  -100, 100)
    physical_right = clamp(right_effort, -100, 100)

    electrical_left  = physical_left  * (left_config.reversed  ? -1 : 1)
    electrical_right = physical_right * (right_config.reversed ? -1 : 1)

    left_pwm  = piecewise_linear(electrical_left,
                                 left_config.min_us,
                                 left_config.trim_us,
                                 left_config.max_us)
    right_pwm = piecewise_linear(electrical_right,
                                 right_config.min_us,
                                 right_config.trim_us,
                                 right_config.max_us)

    return AppliedOutput(physical efforts, electrical efforts, PWM values)
```

Each ESC has separate minimum, trim, maximum, deadband, and reversal configuration.
This mirrors ArduPilot's `SERVOx_*` model. A global sign multiplier elsewhere in the
pipeline is prohibited.

## 9. ArduPilot Rover thrust-mixing math

### 9.1 Reference implementation and BlueBoat parameters

The copy target is the skid-steering path in ArduRover 4.7.0, tag commit
`1511f27194f1dcc3728270883047bdf022b3fd53`,
`AP_MotorsUGV::output_skid_steering()`. The same mixer code is still present in the
locally inspected ArduPilot 4.8 development tree. Blue Robotics' ArduRover 4.7
BlueBoat120 parameter file selects left/right throttle functions and the following
motor parameters:

| Parameter | BlueBoat value | Effect |
|---|---:|---|
| `MOT_SLEWRATE` | `200` | common throttle changes at no more than 200 percentage points/s |
| `MOT_STR_THR_MIX` | `0.6` | slightly favors steering when the mix saturates |
| `MOT_THST_ASYM` | `1.6` | models forward thrust as 1.6 times stronger than reverse thrust |
| `MOT_THR_MIN` | `0` | no nonzero minimum-throttle remap |
| `MOT_THR_MAX` | `100` | full normalized throttle range |
| `MOT_THST_EXPO` | `0` | no post-mix thrust curve |
| `MOT_REV_DELAY` | `0` | no pause on motor-direction reversal |
| `BATT_WATT_MAX` | `0` | battery-watt throttle limiting disabled |
| `MOT_PWM_TYPE` | `0` | normal PWM output |
| `MOT_SAFE_DISARM` | `0` | hold trim PWM while disarmed |
| `MOT_VEC_ANGLEMAX` | `0` | vectored-thrust steering disabled |
| `WENC_TYPE` | `0` | per-wheel rate control is not active |

The output assignment is:

| Physical side | ArduPilot function | Navigator channel | Reversed | PWM min/trim/max |
|---|---:|---:|---:|---:|
| right | `ThrottleRight = 74` | `SERVO1` | no | `1100 / 1510 / 1900` us |
| left | `ThrottleLeft = 73` | `SERVO3` | yes | `1100 / 1510 / 1900` us |

Primary references:

- [ArduRover 4.7 skid-steer mixer](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.0/libraries/AR_Motors/AP_MotorsUGV.cpp#L821-L942)
- [ArduRover throttle slew and output shaping](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.0/libraries/AR_Motors/AP_MotorsUGV.cpp#L307-L317)
- [ArduRover post-mix throttle handling](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.0/libraries/AR_Motors/AP_MotorsUGV.cpp#L1035-L1202)
- [ArduPilot signed servo-to-PWM mapping](https://github.com/ArduPilot/ardupilot/blob/Rover-4.7.0/libraries/SRV_Channel/SRV_Channel.cpp#L198-L212)
- [BlueBoat120 ArduRover 4.7 motor defaults](https://github.com/bluerobotics/Blueos-Parameter-Repository/blob/42c408f9085a25a24aee8cd3a5f14f5f06434e87/params/ardupilot/ArduRover/4.7/Navigator/BlueBoat120.params#L529-L541)
- [BlueBoat120 servo assignments and endpoints](https://github.com/bluerobotics/Blueos-Parameter-Repository/blob/42c408f9085a25a24aee8cd3a5f14f5f06434e87/params/ardupilot/ArduRover/4.7/Navigator/BlueBoat120.params#L854-L868)

The mixer is not a full hydrodynamic or geometry-based control allocator. It does
not use thruster spacing, propeller curves, water speed, battery voltage, or an
online force model. For the BlueBoat defaults, the physics-aware part is a single
static forward/reverse asymmetry factor. The rest is deterministic command shaping
and saturation management. That is still a meaningful improvement over independent
left/right clipping, but it should not be mistaken for force allocation.

### 9.2 Notation and input slew

Let:

- `T_req` be requested surge in `[-100,100]`;
- `S_req` be requested yaw/steering in `[-100,100]`, positive starboard;
- `R` be `MOT_SLEWRATE` in percentage points per second;
- `dt` be the mixer-cycle interval.

ArduPilot first slew-limits common throttle, not each motor independently:

```text
delta_max = R * dt
T = clamp(T_req, T_previous - delta_max, T_previous + delta_max)
```

Yaw is not slew-limited by this motor-layer setting. With the BlueBoat's `R=200` and
50 Hz loop, `delta_max=4` per cycle. A steady request takes 0.5 s to move from zero to
full forward, or 1.0 s to traverse full reverse to full forward.

The mixer then normalizes:

```text
t = T / 100
s = S_req / 100
```

Thus `t,s` nominally lie in `[-1,1]`. In ArduPilot source, steering arrives as
`[-4500,4500]` and is divided by 4500; using `[-100,100]` in our contract is
mathematically identical.

### 9.3 Reverse-asymmetric feasible region

Let the thrust-asymmetry parameter be:

```text
A = max(MOT_THST_ASYM, 1)
```

For the BlueBoat, `A=1.6`. ArduPilot defines the lower internal thrust bound and the
center of the asymmetric feasible interval as:

```text
l = -1 / A                   = -0.625
b = (1 + l) / 2             =  0.1875
```

The pre-compensation motor domain is therefore `[-0.625,1]`, not `[-1,1]`. This is
an effective-thrust coordinate: reverse propulsion is assumed to deliver only
`1/A = 62.5%` as much thrust as forward propulsion for the same full-scale motor
command.

Before combining throttle and steering, ArduPilot limits the available steering:

```text
if t < b:
    steering_range = max(t, 0) - l
else:
    steering_range = 1 - b

s = clamp(s, -steering_range, +steering_range)
t = clamp(t, l, 1)
```

For the BlueBoat:

- at zero or reverse throttle, maximum `|s|` is `0.625`;
- from `t=0` to `t=0.1875`, steering range grows from `0.625` to `0.8125`;
- above `t=0.1875`, maximum `|s|` remains `0.8125`.

This restriction ensures that pure steering can be produced with approximately
balanced physical thrust even though the reverse propeller direction is weaker.

### 9.4 Saturation and steering-versus-throttle priority

The unconstrained pre-compensation motor coordinates would be:

```text
a_left  = t + s
a_right = t - s
```

Both must remain inside `[l,1]`. ArduPilot detects either positive or negative
saturation with:

```text
upper = t + abs(s)
lower = t - abs(s)
q = max(upper, lower / l)
```

Because `l` is negative, `lower/l > 1` exactly when `lower < l`. Therefore the pair
fits if `q <= 1`. If `q > 1`, define the fair proportional scale:

```text
f = 1 / q
m = clamp(MOT_STR_THR_MIX, 0, 1)
```

`m=0.5` is the fair/proportional solution. `m>0.5` preserves more steering;
`m<0.5` preserves more throttle.

For the BlueBoat, `m=0.6`, so the steering-priority branch applies:

```text
k_s = f + ((m - 0.5) / 0.5) * (1 - f)
    = 0.2 + 0.8*f                  // when m=0.6

s = k_s * s

if t >= b:
    t = 1 - abs(s)                 // land on upper boundary
else:
    t = l + abs(s)                 // land on lower boundary
```

For completeness, if `m<0.5`, ArduPilot instead preserves throttle by:

```text
k_t = f + ((0.5 - m) / 0.5) * (1 - f)
t = k_t * t

if t >= b:
    s = sign(s) * (1 - t)
else:
    s = sign(s) * (t - l)
```

The limiting cases are useful for interpreting the parameter:

- `m=0.0`: preserve throttle and sacrifice steering;
- `m=0.5`: scale throttle and steering proportionally by `f`;
- `m=1.0`: preserve steering and sacrifice throttle;
- BlueBoat `m=0.6`: 20% of the way from fair scaling toward full steering priority.

This is materially different from dividing both raw motor values by their largest
magnitude. Simple normalization always preserves curvature. ArduPilot deliberately
changes the throttle/steering balance according to `MOT_STR_THR_MIX`.

### 9.5 Left/right mix and reverse compensation

After saturation handling:

```text
a_left  = t + s
a_right = t - s
```

Each `a` lies in `[l,1]`. ArduPilot then converts the effective-thrust coordinate to
a motor/ESC effort command:

```text
u_i = a_i       if a_i >= 0
u_i = A * a_i   if a_i < 0
```

The published `BB_MIXED_CMD.left_effort/right_effort` should be `100*u_i`, with
positive meaning physical forward rotation/thrust. Electrical channel inversion is
still deferred to `iBBNavigatorInterface`.

The reverse multiplication does not claim that PWM is a perfect force controller.
It is a simple static compensation: if reverse thrust is approximately `1/A` of
forward thrust at equal motor command, multiplying a negative effective-thrust
request by `A` approximately restores the intended force.

### 9.6 Worked BlueBoat examples

These examples show steady-state mixing after common-throttle slew has settled.

| `surge` | `yaw` | Important intermediate result | left effort | right effort |
|---:|---:|---|---:|---:|
| `50` | `0` | no saturation | `50.0` | `50.0` |
| `0` | `100` | steering clamps to `0.625` | `62.5` | `-100.0` |
| `-100` | `0` | `t` clamps to `-0.625`, then reverse compensation applies | `-100.0` | `-100.0` |
| `80` | `40` | `q=1.2`, `f=0.8333`, `s` becomes `0.3467` | `100.0` | `30.67` |
| `80` | `80` | `q=1.6`, `f=0.625`, `s` becomes `0.56`, `t` becomes `0.44` | `100.0` | `-19.2` |

The pivot example is the clearest physical interpretation: the forward motor is
commanded to 62.5%, while the weaker reverse motor is commanded to 100%. Under the
assumed `1.6:1` asymmetry, their physical thrust magnitudes should be similar.

Positive yaw produces `left > right`, hence a starboard/clockwise turn under the
canonical physical sign convention.

### 9.7 BlueBoat PWM mapping

Let `u_left,u_right` be post-asymmetry motor effort in `[-1,1]`. ArduPilot configures
the skid-throttle outputs as signed `[-1000,1000]` servo functions and applies the
BlueBoat channel endpoints piecewise around trim. The formulas below are before the
final integer-PWM truncation.

Right motor, `SERVO1`, not reversed:

```text
PWM_right = 1510 + 390*u_right    when u_right >= 0
PWM_right = 1510 + 410*u_right    when u_right <  0
```

Left motor, `SERVO3`, reversed:

```text
PWM_left = 1510 - 410*u_left      when u_left >= 0
PWM_left = 1510 - 390*u_left      when u_left <  0
```

Consequently:

| Command | Left PWM | Right PWM |
|---|---:|---:|
| neutral | `1510` | `1510` |
| full physical forward | `1100` | `1900` |
| full physical reverse | `1900` | `1100` |

This inversion is wiring-specific and belongs in the Navigator's ESC mapper, not in
`pThrustMix`. `MOT_SAFE_DISARM=0` means ArduPilot holds both outputs at trim while
disarmed instead of disabling PWM.

### 9.8 Proposed MOOS port

The first implementation should be a close, pure port of the ArduRover 4.7 function:

```pseudo
function ardurover_skid_mix(surge_pct, yaw_pct, dt, state, params):
    if hard_stop:
        state.previous_surge = 0
        return neutral_with_stop_metadata()

    surge = clamp(surge_pct,
                  state.previous_surge - params.slew_rate * dt,
                  state.previous_surge + params.slew_rate * dt)
    state.previous_surge = surge

    t = surge / 100
    s = clamp(yaw_pct / 100, -1, 1)

    A = max(params.thrust_asymmetry, 1)
    l = -1 / A
    b = (1 + l) / 2

    steering_range = (max(t, 0) - l) if t < b else (1 - b)
    s = clamp(s, -steering_range, steering_range)
    t = clamp(t, l, 1)

    q = max(t + abs(s), (t - abs(s)) / l)
    if q > 1:
        f = 1 / q
        m = clamp(params.steering_throttle_mix, 0, 1)
        if m >= 0.5:
            s *= lerp(f, 1, (m - 0.5) / 0.5)
            t = (1 - abs(s)) if t >= b else (l + abs(s))
        else:
            t *= lerp(f, 1, (0.5 - m) / 0.5)
            s = sign(s) * ((1 - t) if t >= b else (t - l))

    a_left = t + s
    a_right = t - s
    u_left = A*a_left if a_left < 0 else a_left
    u_right = A*a_right if a_right < 0 else a_right

    return left_effort=100*u_left,
           right_effort=100*u_right,
           all intermediate values and limit flags
```

Porting rules:

1. Copy the saturation math and branch behavior before attempting to simplify it.
2. Keep parameters configurable, with BlueBoat defaults `A=1.6`, `m=0.6`, and
   `slew_rate=200`.
3. Trace `t`, `s`, `l`, `b`, steering range, `q`, `f`, pre-compensation motor
   coordinates, post-compensation effort, and limit flags.
4. Build golden-vector tests against the ArduPilot function, including boundary
   values around `t=b`, `q=1`, zero crossings, full reverse, and all four sign
   combinations.
5. Keep hard-stop bypass, source leases, process epochs, and atomic command contracts
   from this design. Those are safety/traceability improvements around the copied
   mixer, not changes to its steady-state math.
6. Do not port disabled BlueBoat paths such as vectored thrust or wheel-rate control
   into the first mixer implementation.
7. Apply `MOT_THR_MIN` and `MOT_THST_EXPO` only if we intentionally support nonzero
   values later. With current BlueBoat defaults, both transforms are identity.

### 9.9 Difference from the current MOOS mixer

The existing [`pThrustMix`](../src/pThrustMix/ThrustMix.cpp) is not an
ArduPilot-style mixer. Its default path uses
`k_outer=1`, `k_inner=2`, allocates the yaw differential first, and gives common
thrust only the remaining symmetric headroom. It also contains optional speed-based
gain scheduling and an optional heading/yaw-rate correction.

| Behavior | Current `pThrustMix` | ArduRover 4.7 BlueBoat target |
|---|---|---|
| Basic turn term | side-dependent `k_outer/k_inner` differential | symmetric `t+s`, `t-s` in an asymmetric feasible region |
| Reverse weakness | larger default inner-motor gain, but no explicit force coordinate | explicit `A=1.6` reverse bound and post-mix compensation |
| Saturation | yaw-first symmetric headroom or legacy shift | continuous `MOT_STR_THR_MIX=0.6` priority law |
| Common-throttle slew | not part of current mixer | symmetric `200 %/s` before mixing |
| Feedback inside mixer | optional speed and yaw feedback | none for this BlueBoat motor path |
| Hardware mapping | publishes generic left/right commands | separate Ardu servo endpoint/reversal stage |

For example, at `surge=0,yaw=100`, the current default ultimately clamps to roughly
`left=100,right=-100`; ArduPilot produces `left=62.5,right=-100`. The ArduPilot pair
is intentionally unequal in ESC effort so that, under its `1.6:1` forward/reverse
assumption, the two physical thrust magnitudes are approximately equal.

For exact parity, the new `ARDUROVER_4_7_SKID` model should bypass the current
`k_inner/k_outer`, `yaw_priority`, speed-gain, and mixer-level yaw-feedback paths.
The heading/yaw-rate controller belongs upstream in `pBBPID`; repeating feedback in
the mixer would obscure controller ownership and can create two interacting yaw
loops.

During migration the old mixer can remain available as a separately named model for
A/B replay, but model selection must be explicit in both configuration and
`BB_MIXED_CMD`. A log must never leave ambiguous which equation produced the motor
pair.

## 10. Authority transitions

| Current state | Event | Result |
|---|---|---|
| any | kill, local fault, disarm, shutdown | immediate hard stop |
| autonomy | fresh valid RC manual request | RC takes authority |
| teleop | fresh valid RC manual request | RC takes authority |
| RC | RC becomes stale/invalid while manual remains selected | hard stop; no fallback |
| RC | operator leaves manual and teleop has an active valid claim | teleop takes authority |
| RC | operator leaves manual and no teleop claim exists | autonomy only if fresh and allowed |
| autonomy | valid teleop claim | teleop takes authority |
| teleop | session becomes stale/invalid while claim remains active | hard stop; no fallback |
| teleop | explicit claim release | autonomy only if fresh and allowed |
| autonomy | `ALL_STOP=true` | hard stop |
| stopped | stop condition clears | arbiter resumes selection; mixer starts from reset slew state |

All source changes generate an authority event. Handoffs never reuse the previous
source's sequence identity.

## 11. Traceability contract

Traceability is a chain of immutable identifiers, not a giant shared state object:

```text
source_epoch + source_seq
          ↓ copied unchanged
decision_epoch + decision_seq
          ↓ copied unchanged
mix_epoch + mix_seq
          ↓ copied unchanged
NVGR actuator frame + PWM
```

Each app publishes the detail it owns:

- `pBBCommandArbiter` publishes `BB_SELECTED_CMD`, `BB_ARBITER_TRACE`, and
  `BB_AUTHORITY_EVENT`. Its trace explains eligibility and rejection of all sources.
- `pThrustMix` publishes `BB_MIXED_CMD` and `BB_MIXER_TRACE`. Its trace explains
  shaping, saturation, reverse compensation, and left/right effort.
- `iBBNavigatorInterface` publishes `NVGR_ACTUATOR_TRACE`. Its trace explains mixed
  input age, local safety state, ESC mapping, and committed PWM.

Example final trace:

```text
NVGR_ACTUATOR_TRACE =
v=1,t=392.584,selected=RC,stop=NONE,
source_epoch=7f2c91,source_seq=18422,
decision_epoch=arb-a91c,decision_seq=9214,
mix_epoch=mix-471b,mix_seq=33402,mix_age=0.002,
left_effort=33.8,right_effort=49.8,
left_pwm_us=1635,right_pwm_us=1694
```

Given that final frame, a log query can join backward to exactly one mixer result,
one arbitration decision, and one source sample. A process restart cannot create an
identifier collision because each stage has its own epoch.

Plot-friendly scalar mirrors SHOULD also be published by the owning app:

```text
BB_CMD_AUTHORITY
BB_CMD_STOP_REASON
BB_CMD_SURGE_SELECTED
BB_CMD_YAW_SELECTED
BB_MIX_SURGE_SHAPED
BB_MIX_YAW_SHAPED
BB_MIX_LEFT
BB_MIX_RIGHT
NVGR_APPLIED_LEFT
NVGR_APPLIED_RIGHT
NVGR_PWM_LEFT_US
NVGR_PWM_RIGHT_US
NVGR_MIX_INPUT_AGE
```

`BB_AUTHORITY_EVENT` is published immediately by the arbiter on authority, source
epoch, or stop-reason transition. The Navigator separately publishes a local safety
event on arming, hardware-fault, RC-kill, mixed-input-stale, or PWM-watchdog
transition.

Coherent contract records should default to each app's control-cycle rate. If storage
requires decimation, transition events remain immediate and applied-output scalars
remain at actuator rate.

## 12. Fault reporting

Parsing and arbitration failures should be observable without appcast scraping:

```text
BB_ARB_REJECT_SOURCE
BB_ARB_REJECT_REASON
BB_ARB_REJECT_COUNT
BB_ARB_LAST_VALID_RC_AGE
BB_ARB_LAST_VALID_TELEOP_AGE
BB_ARB_LAST_VALID_AUTONOMY_AGE
BB_MIX_INPUT_REJECT_REASON
BB_MIX_INPUT_AGE
NVGR_MIX_INPUT_REJECT_REASON
NVGR_MIX_INPUT_AGE
NVGR_PWM_WATCHDOG_COUNT
```

Reject reasons use stable tokens such as:

```text
malformed,duplicate_key,unsupported_version,nonfinite,out_of_range,
out_of_order_seq,invalid_flag,stale,no_claim,all_stop
```

Human-readable detail belongs in appcast output; stable tokens belong in logs and
automated tests.

## 13. Configuration sketch

```text
ProcessConfig = pBBCommandArbiter
{
  AppTick   = 50
  CommsTick = 50

  command_contract_version = 1
  rc_timeout_sec           = <bench validated>
  teleop_timeout_sec       = <bench validated>
  autonomy_timeout_sec     = <greater than measured broker/controller period>
  startup_unknown_rc_mode  = <STOP|ALLOW_NON_MANUAL>
  legacy_direct_input      = false
}

ProcessConfig = pThrustMix
{
  AppTick   = 50
  CommsTick = 50

  selected_cmd_timeout_sec = <greater than measured arbiter period>
  mixer_model              = ARDUROVER_4_7_SKID
  slew_rate_pct_sec        = 200
  steering_throttle_mix    = 0.6
  thrust_asymmetry         = 1.6
  throttle_min_pct         = 0
  thrust_expo              = 0
  shadow_mode              = true
}

ProcessConfig = iBBNavigatorInterface
{
  AppTick   = 50
  CommsTick = 50

  mixed_cmd_timeout_sec    = <greater than measured mixer period>

  left_esc_min_us          = 1100
  left_esc_trim_us         = 1510
  left_esc_max_us          = 1900
  left_esc_reversed        = true

  right_esc_min_us         = 1100
  right_esc_trim_us        = 1510
  right_esc_max_us         = 1900
  right_esc_reversed       = false

  actuator_trace_hz        = 50
}
```

Each app validates only the configuration it owns. Configuration validation MUST
fail startup for non-positive timeouts, non-positive slew rates, invalid PWM
ordering, invalid reversal values, or simultaneous legacy/new autonomy modes.

## 14. Migration plan

1. Add strict parser/serializer and pure data types with unit tests.
2. Extend `iRCInterface` to publish `RC_INPUT_STATE` while retaining existing scalar
   telemetry.
3. Extend teleop and autonomy/broker paths to publish coherent semantic commands.
4. Add `pBBCommandArbiter` on the front seat and run it in shadow mode, publishing
   `BB_SELECTED_CMD` without affecting hardware.
5. Refactor the existing `pThrustMix` to consume `BB_SELECTED_CMD`, preserve lineage,
   and publish `BB_MIXED_CMD`. Run this mixer on the front seat in shadow mode.
6. Reduce `iBBNavigatorInterface` to one mixed-command input, final safety, ESC
   mapping, PWM, and sensor I/O. Initially publish `NVGR_SHADOW_*` without changing
   the current hardware path.
7. Replay RC, teleop, autonomy, stale-link, kill, and restart logs. Require every
   actuator sample to resolve to an accepted source `epoch + seq`.
8. Bench-test neutral, inversion, endpoints, hard-stop latency, source handoffs, and
   watchdog expiry with propellers removed.
9. Switch hardware output to the new atomic actuator frame behind one configuration
   flag; retain immediate rollback for initial water tests.
10. Remove legacy direct-left/right and in-Navigator arbitration/mixing paths after
    field validation.

When mixing moves to the front seat, `pBB_DGPS_EKF` and status tools must consume
returned `BB_MIX_LEFT/RIGHT` or `NVGR_APPLIED_LEFT/RIGHT` telemetry for low-thrust
detection. They must not keep treating `DESIRED_THRUST_L/R` as the physical command.

## 15. Required tests

At minimum:

- malformed/NaN/out-of-range command rejection;
- duplicate broker frame does not refresh freshness;
- producer restart accepts new epoch and resets sequence baseline;
- out-of-order sequence is rejected;
- RC takeover from autonomy;
- RC loss in manual mode stops instead of falling through;
- teleop loss with active claim stops instead of falling through;
- explicit teleop release permits fresh autonomy;
- `ALL_STOP` blocks autonomy but does not prevent deliberate RC rescue;
- RC kill overrides all sources;
- hard stop bypasses slew and produces neutral within one PWM period;
- PWM writer watchdog neutralizes an expired actuator frame;
- saturation outputs and limit flags match ArduRover golden vectors for
  `MOT_STR_THR_MIX` values below, at, and above `0.5`;
- the worked BlueBoat vectors in Section 9.6 reproduce within floating-point
  tolerance;
- left/right electrical inversion occurs only in ESC mapping;
- every actuator frame has a resolvable decision and source command identity;
- shadow output can be reproduced exactly from an `.alog` input stream.

## 16. Open decisions before implementation

1. Freeze the exact wire names (`RC_INPUT_STATE`, `TELEOP_CMD`, `AUTONOMY_CMD`) and
   serialization helper shared by all producers.
2. Use official ArduRover 4.7.0 as the code baseline, then capture the installed
   firmware version and complete live parameter dump from each boat before claiming
   exact vehicle parity.
3. Bench-measure real update rates and select each source timeout.
4. Decide the RC kill clearing gesture/latch policy.
5. Decide whether a valid source handoff seeds surge slew from current applied common
   effort or deliberately returns to zero first.
6. Confirm each ESC's minimum/trim/maximum/reversed settings and physical sign.
7. Tank-test whether `MOT_THST_ASYM=1.6` remains a good approximation across command,
   battery voltage, and forward speed; copy `1.6` first and tune only from evidence.
8. Determine whether normal teleop needs a direct-tank maintenance mode at all.
9. Decide how to retire or redefine the current `RC_DEADMAN_ENABLED` bypass. A stale
   command lease should not be disableable in the final authority contract.
10. Decide whether startup `mode=UNKNOWN` always inhibits autonomy or may be overridden
   by an explicit deployment configuration for boats operated without an RC link.
11. Decide whether to extend the existing `pThrustMix` wire contract in place or give
    the front-seat version a distinct app/name during migration. Reusing the math is
    desirable; accidentally running both command paths is not.

## 17. Existing code affected

- `src/iBBNavigatorInterface/BBNavigatorInterface.cpp`: current mail-time sign
  transforms, RC/teleop/auto arbitration, smoothing, final output, and PWM handoff.
  It should retain only the final safety, ESC, PWM, and sensor responsibilities.
- `src/iBBNavigatorInterface/BBNavigatorInterface.h`: current fixed-alpha LPF and
  shared source-command state should be removed from the hardware interface.
- New `src/pBBCommandArbiter`: source mailboxes, leases, authority state machine,
  `BB_SELECTED_CMD`, and arbitration trace.
- `src/pThrustMix`: evolve from autonomy-only `DESIRED_THRUST/RUDDER` mixing to the
  lineage-preserving front-seat `BB_SELECTED_CMD -> BB_MIXED_CMD` contract.
- `src/iRCInterface/RCInterface.cpp`: receiver parsing and semantic RC publication.
- `src/iTeleop`: replace normal direct left/right thrust output with a coherent
  session-scoped semantic command.
- `src/iBackSeatBroker`, `src/iFrontSeatBroker`, and
  `src/01_libraries/lib_broker_v2`: transport the atomic autonomy command and returned
  applied-output telemetry.
- `src/pBB_DGPS_EKF` and status consumers: use returned allocated/applied effort.

The existing RC wire and channel definitions remain documented in
[`rc_controllers.md`](rc_controllers.md). This proposal changes command ownership and
transport; it does not redefine the physical handset mapping.
