# RC controllers: how they work, how ours are configured, what's left to build

Single source of truth for the BlueBoat RC system — the TX16S MK3
handsets, the ExpressLRS link, the XR4 receivers, and the vehicle-side
`iRCInterface` app.

Status as of **2026-08-14**: link config frozen, handset models built
and imaged onto several handsets, `lib_crsf` unit-tested, `iRCInterface`
implemented and publishing the full Contract v2 channel map. **No boat
runs it yet**, and **most of the switches we configured do nothing on
the vehicle side** — see [§7](#7-code-changes-required-before-the-switches-do-anything).

This document replaces ten earlier working documents, now in
[`docs/archive/rc/`](archive/rc/). They remain useful as the evidence
chain (bench measurements, protocol comparison, decision history) but
are superseded wherever they disagree with this file. Known drift is
called out inline as **[was wrong in the old docs]**.

---

## 1. How the system works

Control flows one way, telemetry the other, over a single 915 MHz
ExpressLRS link:

- **Uplink (control):** sticks and switches → EdgeTX mixes → 16
  channels → internal Gemini TX module → 915 MHz → XR4 receiver → CRSF
  serial (420k 8N1) on `ttyAMA1` → `iRCInterface` → `RC_*` MOOS
  variables → `iBBNavigatorInterface` drives the thrusters.
- **Downlink (telemetry):** MOOS variables (`NAV_*`, `NVGR_*`) →
  `iRCInterface` → the same serial line → XR4 → RF → handset sensors,
  which feed EdgeTX screens, logical-switch alarms, and voice callouts.

Four config layers stack, and every one of them can silently change what
arrives at the Pi:

1. **EdgeTX mixes** on the handset — which physical control drives which
   channel number, and its polarity.
2. **ELRS OTA packet mode** — how many channels, at what rate and
   resolution, get over the air.
3. **Receiver serial output** — CRSF (not SBUS), on a pad assigned to
   serial duty.
4. **`iRCInterface` decode** — raw ticks → semantic MOOS variables.

The contract between layers 3 and 4 is the *wire truth table* in
[§5](#5-channel-map-the-contract). Everything above it (which switch,
which direction) lives in the handset model; everything below it (what
the value means, what it does) lives in the vehicle code. There is
exactly one place per side to look.

### Why CRSF instead of SBUS

- **Bidirectional on one pair** — telemetry back to the handset is the
  whole reason for the move. SBUS is receiver→FC only.
- **CRC8 integrity.** SBUS has none; corruption presented as
  out-of-range channel values that the old stack guessed at.
- **8N1, not 8E2.** SBUS's even-parity framing is why the Pi mini UART
  couldn't be used and why `disable-bt` was mandatory. That entire
  failure class is gone under CRSF — do not carry the old boot-config
  advice into a CRSF runbook.
- Stick resolution is *identical* (both pack 16 × 11-bit). The common
  "CRSF is higher resolution" claim is wrong at the protocol level.

`iRCReader` / `lib_sbus` stay in the tree untouched as the transitional
fallback. Reverting a boat is a launch-config change, not a rebuild.

---

## 2. Hardware and firmware inventory

| Item | Value |
|---|---|
| Handset | RadioMaster TX16S MK3, internal dual-band Gemini (LR1121) |
| Handset firmware | EdgeTX 2.12 |
| TX module firmware | **ExpressLRS 4.1.0** (FCC915 / ISM2G4 build) |
| Receiver | RadioMaster XR4 Gemini Xrossband |
| Receiver firmware | **ExpressLRS 4.1.0** — major version must match the TX |
| Vehicle UART | `/dev/ttyAMA1` = Navigator SERIAL 3, GPIO 4 (TXD3) / GPIO 5 (RXD3) |
| Wiring | Pi TX (GPIO 4) → RX *Serial RX* pad; Pi RX (GPIO 5) → RX *Serial TX* pad; common ground |

The TX-direction wire is the one most often missing — nothing needed the
return path before telemetry. Verify on any boat before expecting
telemetry to work:

```bash
raspi-gpio get 4      # expect func=TXD3
raspi-gpio get 5      # expect func=RXD3
```

The XR4's serial TX/RX pads are dedicated — no pin reassignment is
involved. (Some other ELRS PWM receivers repurpose channel output pads
for serial duty via the WebUI; that pattern does not apply to the XR4.)
Guidance you find online about "UART1/UART2 at 115200, Betaflight Ports
tab" describes a flight controller, not the XR4 — ignore it.

---

## 3. Link configuration (frozen fleet config)

Set in the ELRS Lua on the handset: `SYS` → `Tools` → `ExpressLRS`.
These are the settings that silently drift; audit them after any
firmware change.

| Setting | Value | Why |
|---|---|---|
| RF band | **915 MHz only** | deliberate — keeps 2.4 GHz clear for the primary comms link |
| Packet rate | **100 Hz Full** | full-resolution mode; 50–150 Hz is the sensible band for an ASV. Higher rates cost receiver sensitivity badly (1000 Hz FSK ≈ −101 dBm vs 25 Hz ≈ −123 dBm) and a boat does not need 500 Hz |
| Switch mode | **16ch Rate/2** | all 16 channels, 10-bit, ~50 Hz effective each. No fast/slow split |
| Telemetry ratio | **1:8** | drift here caused the 2026-08-11 lag. Also see the dynamic-power dependency in [§3.1](#31-tx-power-dynamic-capped-at-500-mw) |
| Antenna mode | gemini | |
| Arm | **"Arm using CH5"** | protocol-reserved arm channel; keeps the map valid under Hybrid/Wide OTA modes including range fallback |
| Model Match | **On** | per receiver, latched to the boat's fleet number |
| Failsafe | **no pulses** | receiver stops emitting on link loss — unambiguous downstream, unlike "last position" |
| Link mode | normal | |

Fleet numbering is hull-based and doubles as the Model Match Receiver
No. (range 0–63): **zoe 31, yip 32, xai 33, …**

### 3.1 TX power: dynamic, capped at 500 mW

Both ends are set to **dynamic** power:

- **Transmitter:** Dynamic power **On**, **Max Power = 500 mW**. The
  module runs as little power as the link needs and ramps toward the
  cap as the link degrades.
- **Receiver:** TX power set to **match transmitter** — the receiver's
  telemetry downlink power tracks whatever the TX is currently running,
  rather than being pinned independently.

Two consequences worth holding onto:

- **Dynamic power depends on telemetry.** The TX decides how much power
  to run from the link stats the receiver reports back over the downlink.
  Telemetry ratio 1:8 provides that comfortably; setting telemetry to
  **Off would silently disable dynamic power's feedback**, leaving it
  pinned. Do not turn telemetry off to "save bandwidth."
- **Bench work must not use dynamic/high power.** Full power into an
  antenna a foot away desensitises the receiver and produces *worse*
  link stats than a real deployment. Drop to 10–25 mW fixed for bench
  sessions, then restore.

The 500 mW cap and the fan threshold are the two open settings — see
[§8](#8-open-questions-and-things-to-test).

---

## 4. Provisioning a handset

The full sequence for a fresh or reflashed TX16S. Steps 2 and 3 are the
ones that get forgotten.

**1. Flash EdgeTX 2.12 and the ELRS TX module firmware (4.1.0).**

**2. Copy the matching ELRS Lua script to the SD card.** ⚠ *This is not
optional and is not part of the firmware flash.* The Lua script version
must match the ELRS firmware version — a stale script talks to the new
module and either fails to enumerate settings or shows the wrong ones.

```
assets/rc/sd/SCRIPTS/TOOLS/elrsV3.lua   →   SD:/SCRIPTS/TOOLS/elrsV3.lua
```

Reachable afterwards at `SYS` → `Tools` → `ExpressLRS`. Whenever the
ELRS firmware is updated, update this file in the same session.

**3. Copy the distance telemetry script.**

```
assets/rc/sd/SCRIPTS/MIXES/dstrx.lua   →   SD:/SCRIPTS/MIXES/dstrx.lua
```

Then per model: Model menu → Custom scripts → add `dstrx`. The vehicle
sends distance-from-origin in a CRSF custom frame (type `0x80`, subtype
`0xB1`, uint16 BE metres) which EdgeTX cannot decode natively; this
script republishes it as a normal `Dst` sensor usable in widgets,
logical switches and alarms. No per-vehicle configuration — the origin
lives on the boat, in the mission file. Identical script on every
handset.

**4. Install the voice pack.**

```
assets/rc/sd/SOUNDS/en/*.wav   →   SD:/SOUNDS/en/
```

11 callouts: `rcmode bb_au bb_sig batlow batcrit tlmlost marked killed
running dmanoff dmanon`. Regenerate with a different voice via
[`scripts/gen_rc_sounds.py`](../scripts/gen_rc_sounds.py) (`sounds_hfcm.zip`
is an alternate-voice cut of the same set). Filenames must stay ≤ 8
characters — the script enforces it.

**5. Copy model images.**

```
assets/rc/<boat>.png   →   SD:/IMAGES/
```

720 × 800 portrait, generated by
[`assets/rc/gen_model_image.py`](../assets/rc/gen_model_image.py) —
white name band, `BLUEBOAT / ID <n>` in yellow, catamaran silhouette.
The ID must match the model's Receiver No. **[was wrong in the old
docs: they specified 192×114; the fleet standard moved to 720×800 on
2026-08-12.]**

**6. Load the model.** Import the boat's `.etx`
(`assets/rc/{via,xai,yip}.etx`) via Companion, or build from the golden
template. Model build detail is in [§9](#9-handset-model-build-yip-as-the-golden-template).

**7. Bind, then latch Model Match.** Bind to the boat's XR4, then ELRS
Lua → Model Match → On. This writes the Receiver No. into that
receiver.

**8. Write the pre-start checklist** to `SD:/MODELS/<MODEL>.txt`:

```
YIP pre-start
[ ] Model = YIP (image + RX 32 match the hull number)
[ ] SF down (AUTO), SW1 dark (running), SW2 dark (deadman ON)
[ ] S1 thrust limit set for today's op
[ ] Telemetry: FM shows boat mode, RxBt sane (4S: ~14.8-16.8)
```

**9. Run the acceptance test** ([§10](#10-test-procedures)).

---

## 5. Channel map: the contract

Two protocol-derived constraints are load-bearing:

- **CH5 = ARM** is reserved by ELRS Hybrid/Wide OTA modes, so the map
  stays valid even if the link falls back to a range mode.
- **Discrete channels must be guard-band decoded.** An *unmixed* channel
  emits exactly 992 (measured: rock solid). Naive midpoint decoding maps
  992 to the *armed edge* of every 2-position channel — so a forgotten
  mix line would produce KILLED / deadman-DISABLED rather than a safe
  state. Guard bands turn "forgot a mix" into a visible fault.

### The wire

| CH | Name | Control | Wire values | MOOS variable | Semantic |
|---|---|---|---|---|---|
| 1 | `STEER` | right stick L/R | 172 ↔ 1810 | `RC_CH1` | ±100, **+ = starboard** |
| 2 | `AUX_AXIS_1` | right stick F/B | 172 ↔ 1810 | `RC_CH2` | ±100, reserved |
| 3 | `THRUST` | left stick F/B (spring-centred) | 172 ↔ 1810 | `RC_CH3` | ±100, **+ = ahead** |
| 4 | `AUX_AXIS_2` | left stick L/R | 172 ↔ 1810 | `RC_CH4` | ±100, reserved |
| 5 | `KILL` | SW1 button, LED red | 172 / 1810 | `RC_CH5` | 1 = RUNNING, 2 = KILLED |
| 6 | `MODE` | SF toggle (mix weight −100) | 172 / 1810 | `RC_CH6` | 1 = AUTO, 2 = RC |
| 7 | `ACTION` | SB 3-pos | 172/992/1810 | `RC_CH7` | 1 = HOLD, 2 = RESUME, 3 = RETURN |
| 8 | `DEADMAN_EN` | SW2 button, LED amber | 172 / 1810 | `RC_CH8` | 1 = ENABLED, 2 = DISABLED |
| 9 | `PAYLOAD_PWR` | SW3 button, LED white | 172 / 1810 | `RC_CH9` | 1 = OFF, 2 = ON |
| 10 | `PAYLOAD_MODE` | SC 3-pos | 172/992/1810 | `RC_CH10` | 1 / 2 / 3 |
| 11 | `THRUST_LIMIT` | S1 pot | 172 → 1810 | `RC_CH11` | percent, floored at 25 |
| 12 | `MARK` | SH momentary | 172 / 1810 | `RC_CH12` | 2 = pressed |
| 13–16 | reserved | unmixed | 992 | `RC_CH13-16` | raw ticks passed through |

Free controls for later: SA, SD, SE, SG, S2, LS, RS, SW4–SW6.

### The two invariants

**State 1 is the operator-absent default.** Every discrete channel is
defined so state 1 = *the state the vehicle should hold when nobody is
intervening*. Lab policy is **armed by default** — an over-the-horizon
boat must not stop because the RC link went away. So state 1 means
AUTO, RUNNING, deadman ENABLED, payload OFF. The fallback is correct by
construction rather than by comment.

**Kill is separate from mode.** v1's single "RC mode" boolean conflated
"outputs enabled" with "who is commanding". `KILL` (operator stop) and
`MODE` (command source) are now independent variables.

The lit-button rule keeps the panel readable: **a glowing button is an
operator intervention** — vehicle killed, watchdog off, payload
energized.

**Consumer-side sign convention (`rc_stick_convention`, added
2026-08-14).** The v2 wire (+ = ahead / + = starboard) landed on the
handsets, but `iBBNavigatorInterface`'s RC mixer still implemented
the legacy RadioLink wire (sticks negated, no `−1` output transform).
The mismatch fingerprint — caught on the bench — is **turns correct,
throttle reversed**: the legacy CH1 negation and the missing `−1`
cancel in yaw only. Fixed with a config knob so the SBUS fallback
stays a pure config swap: `rc_stick_convention = v2` in the fleet
plug (CRSF/TX16S boats — the RC path then uses the exact
`−1 × value × invert` transform of the autonomy/teleop paths);
`v1` is the code default and **required if a boat reverts to
`iRCReader`**. The appcast shows the active convention.

### Guard-band decode, as implemented

[`RCInterface.cpp`](../src/iRCInterface/RCInterface.cpp) —
`MapSwitchGuarded()`, thresholds `RC_GUARD_LOW = 600`,
`RC_GUARD_HIGH = 1400`:

- 2-position: below 600 → state 1; above 1400 → state 2; **the middle is
  a dead band, not a state**.
- 3-position: bands around 172 / 992 / 1810 with invalid gaps between.
- Dead band → run warning + **safe state 1**, except:
- **CH5 KILL latches the last valid state** instead of defaulting. A
  config fault must never silently kill an autonomous boat, nor un-kill
  a killed one.

### Disconnected fallback

Published when `RC_CONNECTED` goes false
([`RCInterface.cpp:695`](../src/iRCInterface/RCInterface.cpp#L695)):

| Channels | Value |
|---|---|
| CH1–4 axes | `0` (zero thrust commanded) |
| CH5–10, CH12 | `1` (RUNNING, AUTO, HOLD, deadman ENABLED, payload OFF, no mark) |
| CH11 | `100` — no operator thrust limit; a lingering 25 % cap must not cripple autonomous ops |
| CH13–16 | 992 (mid, no command bias) |

**Publish order is load-bearing:** `RC_CONNECTED = false` is published
*before* the safe-default `RC_CH*` values within an iterate. The
`RC_DEADMAN` watchdog in `iBBNavigatorInterface` depends on it, and so
does the CH6 mode latch — consumers must gate channel mail on
`m_rc_connected` so these "operator unavailable" tokens are never
mistaken for commands.

**[was wrong in the old docs]** `rc_hil_test.md` expected `RC_CH5` = 3/1
and `RC_CH8` = 3/1 (a 3-state map inherited from the SBUS era). The
implemented v2 decode publishes **1/2** for both. The table above is the
truth; the HIL checklist needs that correction before it is re-run.

### Additive link variables (new under CRSF)

| Variable | Source |
|---|---|
| `RC_LINK_LQ` | CRSF `0x14` uplink link quality, % |
| `RC_LINK_RSSI` | `0x14`, dBm |
| `RC_LINK_SNR` | `0x14`, dB |
| `RC_PROTOCOL` | config — `crsf`, for log forensics |
| `RC_FRAME_LOST`, `RC_FAILSAFE`, `RC_CH17/18` | contract parity with SBUS-era logs |

---

## 6. Telemetry

All frame types verified end-to-end on ELRS 4.1.0.

| Frame | Handset sensors | Source | Rate |
|---|---|---|---|
| `0x08` battery | `RxBt` `Curr` `Capa` `Bat%` | `NVGR_ROLLING_VOLTAGE` / `_CURRENT` | `telem_hz` = 2 |
| `0x02` GPS | `GPS` `GSpd` `Hdg` `Sats` `Alt` | `NAV_LAT/LONG/SPEED/HEADING` | `gps_hz` = 1 |
| `0x21` flight mode | `FM` text (≤ 13 chars) | MOOS mode string | with battery |
| `0x80/0xB1` custom | `Dst` (via `dstrx.lua`) | `hypot(NAV_X, NAV_Y)` | with GPS |
| `0x1E` attitude | `Ptch` `Roll` `Yaw` | — | off unless needed |

**GPS is the highest-value frame** — EdgeTX derives a distance readout
and map widget from it, which for a boat past easy sight range is worth
more than everything else combined.

Three rules learned the hard way:

- **Downlink LQ on the Pi side is always 0.** Not a fault, not a ratio
  problem. ELRS receivers write the `0x14` downlink fields only under
  the `DEBUG_BF_LINK_STATS` build flag (`rx_main.cpp`), so they are
  structurally zero on every normal build at every telemetry ratio. The
  RX cannot measure telemetry reception; the TX module can. **The
  handset `TQly` sensor is the downlink canary** — this cost two rounds
  of chasing wiring that was already fine.
- **`OPOST`/`ONLCR` must be cleared on the tty.** With post-processing
  on, the driver rewrites any `0x0A` byte to `0x0D 0x0A`. The battery
  frame's length byte is `0x0A`, and any payload byte (a latitude, a
  voltage) can be too — so this presents as intermittent glitches, not
  a clean failure. Both tools and the app clear the flags explicitly.
- **Telemetry must never stall RC decode.** Non-blocking writes, drop
  frames rather than queue them, encoder off the read thread's critical
  section. RC control latency is the safety-critical property;
  telemetry is not.

### Handset-side alarms (no vehicle code)

Configured per model after sensor discovery:

- L1: `RQly < 50` → "signal low" every 10 s. LQ decays 100→0 over
  ~1.1 s on link loss — that decay is the early warning.
- L2: `RxBt < 14.0` → "battery low"; L3: `RxBt < 13.2` → "battery
  critical" (4S: 3.5 / 3.3 V per cell — adjust if the fleet mixes packs).
- L4: no `RxBt` update → "telemetry lost". This is the **"RF fine, MOOS
  dead"** detector, and the reason telemetry is worth having even when
  nobody is watching the screen.

Three custom screens: **OPS** (`Bat%`, `RxBt`, `RQly`, `FM`, `GSpd`),
**LINK** (`RQly`, `RSNR`, `TPWR`, `RFMD`, RSSI min/max), **NAV** (map
widget, `Hdg`, `Sats`, `Dst`).

**Idea not yet taken:** EdgeTX alarms only on *numeric* sensors, so the
`FM` string can't drive audio alerts. The battery frame's `Capa` field
is currently zeroed — repurposing it as a fault bitmask (leak, ESC
fault, telemetry stale) would give spoken fault callouts for three bytes
already on the wire. Non-standard use of a standard field; document it
in the channel map or don't do it.

---

## 7. Code changes required before the switches do anything

**This is the gap.** `iRCInterface` publishes the complete Contract v2
map. Almost nothing consumes it.

### What is wired (updated 2026-08-14 — §7.1/7.2/7.3/7.5 IMPLEMENTED)

| Variable | Consumer | Effect |
|---|---|---|
| `RC_CONNECTED` | `iBBNavigatorInterface`, `pBB_Status` | deadman timestamp; mode-latch gate; status display |
| `RC_FRAME_VALID` | `iBBNavigatorInterface` | per-cycle thrust gate — one bad frame zeros thrust |
| `RC_CH1`, `RC_CH3` | `iBBNavigatorInterface` | RC steer + thrust mixer |
| `RC_CH5` **KILL** | `iBBNavigatorInterface` | **neutral-lock final override** (§7.1); publishes `NVGR_RC_KILL`; `pBB_Status` mode shows `KILLED` |
| `RC_CH6` | `iBBNavigatorInterface`, `pBB_Status` | latches `m_rc_mode`, only while connected |
| `RC_CH8` **DEADMAN_EN** | `iBBNavigatorInterface` | edge-triggered toggle of the deadman runtime flag (§7.3) |
| `RC_CH11` **THRUST_LIMIT** | `iBBNavigatorInterface` | authority cap on RC + teleop thrust, behind `rc_thrust_limit_enable` (§7.2) |
| `RC_CH12` **MARK** | `iRCInterface` itself | rising edge → incrementing `RC_MARK` + appcast event (§7.5) |
| `RC_FAILSAFE` | `pBB_Status` | status display |
| `RC_DEADMAN_ENABLED` | `iBBNavigatorInterface` | backseat deadman writer (coexists with CH8, last writer wins) |

**Still unconsumed by design:** CH7 ACTION (deferred, §7.4), CH9/CH10
payload (no hardware, §7.6), `RC_LINK_*` (§7.7 polish). All
implemented features are **bench/HIL/water-test pending** — code
compiled, never run against hardware.

---

### 7.1 CH5 — KILL

**Decision (2026-08-14, revised after the §8.3 bench work):
KILL = neutral-lock, not signal-cut.** Operator-clarified semantics:
"killed" means **the thrust channels hold neutral and nothing may
write to them** — props stopped, ESCs armed and silent, signal
present. Not the OE-high signal-cut (which merely makes the ESCs
lose their input and beep; the ESC itself has no disarm).

- **Now:** `RC_CH5` = 1/2, dead-band-latching in `iRCInterface`. No
  consumer.
- **Design:** new `m_rc_kill` latched in the `RC_CH*` mail handler
  alongside the CH6 block
  ([:568](../src/iBBNavigatorInterface/BBNavigatorInterface.cpp#L568)).
  Kill edge → write neutral to both thrusters once, then **refuse all
  thrust writes** — RC, teleop, autonomy — at the single point where
  pulses are written, until un-killed. Un-kill edge → clear the
  lockout; **thrust resumes immediately** (no 2 s re-arm — the ESCs
  never lost signal). Publish `NVGR_RC_KILL` for `pBB_Status` and
  shoreside.
- **Latch across dropout.** Gate the CH5 mail on `m_rc_connected`
  exactly as CH6 is gated. Otherwise the disconnected-fallback
  `RC_CH5 = 1` would **silently un-kill a killed boat** the moment
  the link drops.
- **Not `ALL_STOP`.** The existing `ALL_STOP` deliberately exempts RC
  and teleop ("a latched backseat ALL_STOP must not paralyze the
  rescue"). KILL is the opposite: it overrides everything, rescue
  included.
- **Honest caveat:** the lockout is software-enforced inside one
  process; signal-cut is a hardware line level. The trade is instant
  silent recovery and never-disarmed ESCs vs a stronger guarantee.
  The hardware hammer stays available (`NVGR_DISARM=true` → OE high)
  as the emergency path, and a killed boat is watcher-safe: if the
  app dies while killed, the chip free-runs the neutral it was
  holding.
- **Consistency bonus:** killed state = the between-mission state =
  the watcher's corrective state. One safe state everywhere:
  neutral-on-the-wire.

### 7.2 CH11 — THRUST_LIMIT

**Decision (2026-08-14): caps RC and teleop only — autonomy is never
limited — behind a config knob in the plug.**

Best quality-of-life per line of code: dial authority down for shallow
water, new operators, first splash.

- **Now:** `RC_CH11` = 25–100 %. No consumer.
- **Design:** scale factor applied where RC thrust
  (`calculateRCThrust()`) and teleop thrust are computed in
  `iBBNavigatorInterface`. Autonomy path untouched — which also means
  **no interaction with the new PID controller's anti-windup**; that
  concern applied only to capping autonomous thrust.
- **Config:** `rc_thrust_limit_enable` in the ProcessConfig block.
  Code default **false** (feature inert unless the plug asks for it);
  the fleet plug ships with it set explicitly, so the mission file
  states the policy either way.
- **Disconnect default is now a non-issue.** `iRCInterface` publishes
  `RC_CH11 = 100` on link loss; since the limit only touches manual
  modes, which require a live link anyway, the fallback value never
  drives a thruster.

### 7.3 CH8 — DEADMAN_EN

The over-the-horizon enabler: turn the RC watchdog off so the boat keeps
running when it leaves radio range.

- **Now:** `RC_CH8` = 1/2 published. The navigator honours a *different*
  variable, `RC_DEADMAN_ENABLED` (string), settable from config or
  backseat ([:609](../src/iBBNavigatorInterface/BBNavigatorInterface.cpp#L609)).
- **Decision (2026-08-14):** the plug config (`rc_deadman_enabled`)
  stays the master switch for whether the watchdog exists at all, as
  today. When it is on, `iBBNavigatorInterface` reads `RC_CH8` with the
  same connected-gate latch as CH6 and toggles the runtime enable from
  the handset switch. The backseat `RC_DEADMAN_ENABLED` variable keeps
  working; switch and backseat both write the same runtime flag,
  **edge-triggered** (act only on a switch state *change*), so neither
  writer fights the other every iterate. Every toggle is logged as an
  event.
- **Latching is mandatory here, not optional.** The operator flips the
  switch *and then* walks out of range. A non-latched implementation
  would re-enable the deadman on the disconnect fallback and safe the
  boat — the exact failure the switch exists to prevent.
- Log loudly. "Deadman disabled from the handset" plus a shoreside
  indicator; a boat running autonomously with the watchdog off should
  never be a quiet state.

### 7.4 CH7 — ACTION (HOLD / RESUME / RETURN)

**Decision (2026-08-14): deferred.** Publish-only for the first boat
install; the notes below are the design for when it lands.

- **Now:** `RC_CH7` = 1/2/3. No consumer.
- **Needs:** helm commands — HOLD → `DEPLOY=false` (or station-keep),
  RESUME → `DEPLOY=true, MOOS_MANUAL_OVERRIDE=false, RETURN=false`,
  RETURN → `RETURN=true`.
- **Not in `iBBNavigatorInterface`** — that app is a thruster driver and
  has no business posting helm commands. Either extend `pBB_Status`
  (already subscribes to RC) or add a thin `pRCCommand`. Recommend the
  latter: the mapping is mission policy and will want its own config.
- **Must be edge-triggered**, not level. Posting `DEPLOY=true` every
  iterate would fight the shoreside operator and the helm continuously.
- **Inert while `RC_CH6 == 2` (RC mode)** — the sticks are driving.

### 7.5 CH12 — MARK

- **Now:** level published as 1/2.
- **Needs:** rising-edge detection → one event into the log. Best done
  **in `iRCInterface`**, which sees every frame: publish an incrementing
  `RC_MARK` count or a timestamped event string on the rising edge only.
  Self-contained, needs no consumer beyond `pLogger`, ~10 lines.

### 7.6 CH9 / CH10 — PAYLOAD_PWR, PAYLOAD_MODE

No payload hardware exists. Publish-only until it does; no code change
needed now. Reserve the names.

### 7.7 `RC_LINK_*`

Nothing consumes link quality. Two cheap wins: a run warning in
`iBBNavigatorInterface` (or `pBB_Status`) below an LQ threshold, and
forwarding `RC_LINK_LQ` shoreside via `iFrontSeatBroker` `tx_vars` so
the operator on the laptop sees the link degrading too.

### 7.8 Deployment prerequisites (not code)

- ~~Swap `plug_iRCReader.moos` → `plug_iRCInterface.moos` in
  `meta_vehicle.moos`~~ — **done 2026-08-14.** The mission now
  launches `iRCInterface`; reverting to SBUS is the two-line swap
  back. Both must never run — two readers on one tty split the byte
  stream and both see corruption.
- Install the **patched navigator-cpp build** on the boat — a
  freshly imaged Pi carries the pre-patch library, and without the
  rebuild the alternating-launch ESC panic comes back. (The
  `bb_esc_failsafe` watcher is deliberately not deployed —
  [§8.3](#83-esc-armdisarm-alternating-launch-panic).)
- Confirm `raspi-gpio get 4/5` on the target boat.
- Set that boat's XR4 output protocol to CRSF (ELRS Lua or the WebUI
  Model tab). Verify rather than assume — the boats were deliberately
  set to SBUS.
- Re-run the HIL checklist with the CH5/CH8 expectation corrected.

### Build order (decided 2026-08-14)

1. **MARK** (7.5) — smallest, self-contained, proves the edge-detect
   pattern.
2. **KILL** (7.1) — neutral-lock in `iBBNavigatorInterface`. Formerly
   blocked on the alternating-launch ESC panic
   ([§8.3](#83-esc-armdisarm-alternating-launch-panic)); that is
   diagnosed, patched, and bench-verified — unblocked.
3. **DEADMAN_EN** (7.3) — same latch pattern as KILL, same file.
4. **THRUST_LIMIT** (7.2) — RC + teleop only, config-gated.
5. **`RC_LINK_*`** (7.7) — polish.
6. **ACTION** (7.4) — deferred past the first install.

Steps 2–4 all land in `iBBNavigatorInterface` and share the
"latch-while-connected" idiom already established by CH6; worth writing
that helper once.

---

## 8. Open questions and things to test

### 8.1 TX power: is 500 mW enough?

We cap dynamic power at 500 mW today; the module can do 1000 mW.
Untested.

**Test:** a range walk at the intended operating distance with the boat
in the water (over-water propagation, antenna at boat height, not a
parking-lot test). Record on the handset: `RQly`, `RSSI`, `TPWR` (what
dynamic power actually selected), and `TQly` for the downlink. Two
questions to answer separately:

- Does dynamic power **reach the 500 mW cap** at working range? If it
  never leaves low power, the cap is irrelevant and 1000 mW buys
  nothing.
- If it sits pinned at 500 with LQ degrading, that's the case for
  raising the cap.

Note the **downlink is the weaker half** — the receiver's telemetry
power tracks the TX ("match transmitter"), but the XR4's antenna and
placement on the boat are worse than the handset's. Expect `TQly` to
degrade before `RQly` does. If telemetry drops out while control is
still solid, that's an antenna/placement problem, not a power one.

Also to record: whether raising the cap changes handset battery life
enough to matter over a field day.

### 8.2 Fan threshold: when should the fan run?

The internal Gemini module has a fan, and ELRS exposes a **Fan Threshold**
— the power level at or above which it runs. Unset for our config.

The interaction with dynamic power is the whole question: if the
threshold sits above where dynamic power actually spends its time, the
fan never runs and the module heats up at 500 mW; too low and the fan
runs constantly, costing handset battery and adding noise on a quiet
boat deck.

**Test:** run the range test with the threshold at **250 mW** as a
starting point, then check module temperature (ELRS Lua reports it) after
a sustained high-power period. Decide from the measured temperature at
the power level the link actually uses, which is an output of §8.1 —
**do §8.1 first**, it tells you what power to design the fan around.

### 8.3 ESC arm/disarm: alternating-launch panic

**Status: CONFIRMED, PATCHED, AND RE-VERIFIED (2026-08-14).** Pin
measurements closed the diagnosis: ch14 read ~0.5 V / **0 V** /
~0.5 V across three back-to-back repro runs — the dead run is the
parity whose init slept a running chip, matching the mission's
launch-2 panic. Fix applied to `pca9685.cpp` (all-channels-off before
any SLEEP write, stale bit 7 masked out of the `set_frequency`
read-modify-write, pending RESTART cleared explicitly after every
wake). Post-patch: three consecutive runs with zero RESTART events
and ~0.5 V on the pin throughout — a brief sag between runs is the
init/arm window (OE high, channels parked) and is now uniform across
every launch. Remaining: a double-launch of the real mission app on
a boat as the final confirmation.

**Side-finding, safety-relevant: PWM pulses persist after the app
exits.** Releasing the OE GPIO line does not reset the pin — OE
stays low and the chip keeps generating the last-written pulses
indefinitely. Consequences, walked through:

- **ESCs stay armed at neutral between missions** (every handled exit
  path writes neutral for 1 s first). "Mission down = thrusters safe"
  is **false** today. The "ESCs disarm between missions / PWM does
  not survive app exit" comments in `BBNavigatorInterface.cpp` are
  wrong and must be corrected before they justify a future design
  decision.
- **The worst case is an *unhandled* death** — SIGKILL, kernel OOM,
  Pi brownout of the process: no `safePwmShutdown` runs, so the chip
  free-runs the **last commanded thrust forever**. Kill −9 the app
  while driving and the props keep turning until power is cut. The
  signal handlers cover every crash they can see; this gap is
  precisely the deaths they can't see. Plausibly related: the standing
  via rule "never restart fs-mission live — props spin".
- **A silver lining for KILL:** OE-high persists the same way, so an
  RC kill (`requestDisarm`) survives even a subsequent app crash —
  killed stays killed.
- **Resolutions (2026-08-14):**
  - **Fleet safe state = neutral-on-the-wire, by design** (operator
    decision): ESCs armed at 1500 µs, silent, props stopped, nothing
    writing to the chip. Signal-cut (OE high) is the fallback hammer,
    not the normal safe state. `disarm_on_exit` stays `false`; all
    the wrong "ESCs disarm between missions" comments corrected in
    `iBBNavigatorInterface` and its config files.
  - **Failsafe watcher: built, never deployed, and REMOVED from the
    repo (operator decision, 2026-08-27).** `scripts/bb_esc_failsafe.sh`
    + `scripts/bb-esc-failsafe.service` were retired during the control
    refactor rather than retargeted to the new 50 Hz / 1510 µs
    configuration; recover from git history if ever wanted again.
    What they were (1 Hz: app
    dead + OE low + non-neutral thruster pulse → force neutral, OE
    cut as fallback). The decision followed the 2026-08-14 bench-Pi
    incident: the Pi hard-locked during the install session and then
    failed boot with 8-blink SDRAM codes — a pre-kernel hardware
    failure the watcher provably could not cause (the script never
    reached `/usr/local/bin` and the unit was never enabled), but
    trust is a valid reason. Two real bugs found in the process are
    fixed in the repo copies: the unit's
    `After=multi-user.target`+`WantedBy=multi-user.target` systemd
    ordering cycle, and boot-time false-trip logic in the script.
    **Consequence: the unhandled-death runaway gap (§ above) is
    ACCEPTED, not mitigated.** Runbook line: never `kill -9`
    `iBBNavigatorInterface` with props in the water; use Ctrl-C /
    SIGTERM, which neutral-holds before exit.
  - **Sweep arm mode DELETED** from `iBBNavigatorInterface`, along
    with the per-boot tmpfs marker machinery that existed only to
    gate it (`esc_arm_mode` / `esc_armed_marker` params accepted
    silently for old plugs). Neutral hold is the only arm procedure.

**Observed (2026-08-14):** within one boot, mission launch 1 arms fine,
launch 2 puts the ESCs into panic mode (continuous beeping), launch 3
is fine again — alternating.

**Hypothesis (code-traced, not yet bench-verified): the PCA9685
RESTART-bit trap.** The chip is powered continuously, so each launch
inherits the previous launch's register state:

- Launch 1 finds the chip in its power-on state (already asleep).
  `pca9685_init` sleeps → wakes it; no PWM was running, so nothing
  special happens. Outputs run, ESCs arm. On exit the app releases the
  OE line (ESCs lose signal) but **leaves the chip awake with PWM
  channels running**.
- Launch 2's `pca9685_init` writes SLEEP **to a chip whose channels are
  active**. Per the datasheet, that sets the internal RESTART-pending
  condition, and after wake **all PWM channels stay OFF until software
  writes 1 to the RESTART bit** — which the driver never does: every
  MODE1 write in `pca9685_init` and `pca9685_set_frequency`
  ([pca9685.cpp](../../navigator-cpp/src/02_interfaces/pca9685/pca9685.cpp))
  has bit 7 = 0, and writing 0 to RESTART is a no-op. Result: OE goes
  low, but the pins carry **no pulses** — the ESCs see an enabled line
  with no signal and beep in panic. The 2 s "neutral hold" arms nothing.
- Launch 3 inherits a chip whose channels are already off, so the
  SLEEP write sets no RESTART condition, wake works, outputs run. Fine
  again. And so on, alternating.

**This also condemns the in-run re-arm path** (`NVGR_DISARM=false` →
`armIfNeeded()`): `requestDisarm()` raises OE but leaves the channels
running, so the subsequent `pwm_set_frequency()` sleep-while-running
sets the same trap — meaning un-kill → re-arm would likely panic the
ESCs every time. **This must be fixed before the CH5 KILL work
(§7.1), which reuses exactly this machinery.**

**Bench results (2026-08-14, MODE1 traces via `test_pca9685_restart`,
3 back-to-back runs):** mechanism confirmed, with one refinement.
Run 2's init slept a running chip and RESTART went pending (`0xE0`) —
the trap as hypothesized — and run 3 did *not* pend after init,
proving the two-state parity is real. Refinement: the pending flag
does not survive to the end of the sequence in either parity — in
run 2 `pwm_set_frequency`'s read-modify-write cleared it; in run 3 it
appeared *inside* `set_frequency` and the subsequent neutral LED
writes cleared it. Since the datasheet permits flag-clear **without**
channel restart, a final `0x60` is not proof the outputs are alive —
registers alone cannot label which parity is the dead one. That last
step needs a pin measurement (below); the repro's verdict logic now
says so (`DEAD` only when bit 7 survives; `SUSPECT` when it pended
and was cleared mid-sequence).

**Hardware validation checklist (the gate before any §7 RC feature
work ships).** Bench navigator, ESCs connected **with props removed**
(or multimeter-only, no ESCs). Sync + rebuild first: navigator-cpp
(patched) *and* moos-ivp-blueboat (sweep/marker deletion) on the Pi,
in that order — the app links `libnavigator_cpp`.

- [ ] **A. Repro tool ×3** — `test_pca9685_restart` back to back:
      every run "healthy by registers", no `0xE0` anywhere, ~0.5 V
      on ch14 during each hold. (Done 2026-08-14 — repeat only if
      anything above it changes.)
- [x] **B. Mission double-launch** — **PASSED 2026-08-14**: arbitrary
      relaunches, each showing the expected two-chime signal-gap beep
      then the initialization tone. The launch-2 dead case is gone.
      (Unrelated finding from the same session: the bench board's
      AK09915 mag NACKs its CNTL2 mode write at init — reported as
      "reg 0x49" by a decimal-printed-as-hex error-format bug in
      navigator-cpp `i2c.cpp`, since fixed; real register is 0x31.
      Mag is unused (`use_mag=false` fleet-wide), does not block
      this validation. Diagnose separately with `test_ak09915` +
      `i2cdetect -y 1` after a full power cycle, app stopped.)
- [x] **C. NVGR_DISARM cycle** — **PASSED 2026-08-14**: `true` cuts
      the signal, `false` re-arms cleanly. First-ever hardware pass
      of this path (it was likely broken pre-patch —
      `set_frequency` slept a running chip). KILL's emergency
      hammer is now proven machinery.
- ~~**D/E. Watcher trip tests**~~ — **N/A: watcher not deployed**
      (operator decision 2026-08-14, see resolutions below). If it
      is ever installed, these tests come back: synthetic `i2cset`
      non-neutral trip, and `kill -9` mid-thrust with props off.
- ~~**F. Soak**~~ — N/A with no watcher; the between-missions
      neutral-persist behavior itself was verified directly on the
      bench.

**Verify on hardware — no ESCs needed** (bare bench Navigator; do not
run beside a live mission):

- Purpose-built repro: `test_pca9685_restart` in navigator-cpp
  (`src/04_unit_tests/`). It mimics one mission launch exactly —
  init → 100 Hz → neutral ch14/16 → enable → **exit with channels
  left running**, the state that arms the trap — and prints MODE1 at
  each step plus a verdict. Run it three times back to back: expect
  HEALTHY / BUG REPRODUCED / HEALTHY. (The pre-existing
  `test_pca9685` / `nav_pwm_example` can *not* reproduce this — they
  exit via `pwm_enable(false)` and never leave the chip running.)
- Shell-only alternative: during a panic launch of the real app,
  `i2cget -y 4 0x40 0x00` (Pi 4; bus 3 on Pi 5) — healthy reads
  `0x60`, bug reads `0xE0` (bit 7 = RESTART pending).
- Physical pin check without a scope: multimeter DC on the ch14
  signal pin — 1500 µs @ 100 Hz = 15 % duty ≈ **0.5 V** average when
  alive, flat **0 V** on a bug run.

**Fix (in navigator-cpp, `pca9685.cpp`):** make init idempotent and
handle RESTART per the datasheet —

1. In `pca9685_init`, start from a known state: either issue an I2C
   SWRST (general call 0x00, byte 0x06) before configuring, or
   explicitly write all-channels-off before sleeping. OE is held high
   (outputs disabled) throughout init, so no glitch reaches the ESCs.
2. After any wake, wait 500 µs, read MODE1, and if RESTART is pending
   write MODE1 with bit 7 = 1 to restart the channels.
3. In `pca9685_set_frequency`, mask bit 7 out of the read-modify-write
   (currently a pending RESTART read back gets rewritten verbatim,
   with order-dependent side effects), and apply step 2 after the
   restore.

### 8.4 Smaller open items

- **Fleet-wide `.etx` regeneration.** `via.etx`, `xai.etx`, `yip.etx`
  exist; the `profile.yaml` → generated-models pipeline described in the
  old architecture doc was never built. Three handsets is under the
  threshold where it pays for itself — revisit at six.
- **Stray files:** `src/01_libraries/lib_crsf/yip.etx` and
  `document1.etx` are model exports sitting in a source directory. Move
  to `assets/rc/` or delete.
- **`Capa` → fault bitmask** ([§6](#6-telemetry)) — decide or drop.
- **`Dst` on every handset** — confirm `dstrx.lua` is installed *and
  added to the model's custom scripts* on all three imaged handsets.
  The file being on the SD card is not enough.
- **One unexplained `rc_probe` run** framed nothing at 464 B/s (vs 2410
  healthy) after a successful auto-detect. Signature matches two readers
  splitting one tty. If it reproduces, check for a running MOOS RC app
  first.
- **`iFrontSeatBroker` `tx_vars`** — decide which `RC_*` variables the
  shoreside needs.

---

## 9. Handset model build (YIP as the golden template)

Built in Companion against the real radio document (Read Models and
Settings from Radio first — never a blank one). Copy the boat's PNG to
`/IMAGES/` before starting so it is selectable.

**Setup tab.** Model name = boat name. **Internal Radio System →
Protocol: CRSF**; **External module: OFF** (one model had this
backwards). Receiver No. = fleet number. Arm using CH5. Throttle Source
THR, **Reverse Throttle OFF** (the stick is a bidirectional ±100 axis,
not an RC-plane throttle). **Custom Throttle Warning: value 0** — with a
centre-sprung stick the correct at-rest position is centre, so warn if
it is *not* at 0 at power-on; the default −100-idle warning would nag on
every boot. Centre beep on Thr. **Switch warnings:** SF down (AUTO) and
SW1 off (not killed).

**Customizable Switches:** SW1 `KIL` 2POS off-dim/on-**red**, SW2 `DMN`
off-dim/on-**amber**, SW3 `PAY` off-dim/on-white. (Names cap at 3 chars.)

**Flight Modes:** FM0 `AUTO` (default, SF down); FM1 `RC` (SF↑). The
active name shows on the main screen and is what the voice calls out.

**Inputs:** four, weight 100, **no expo, no curves** — linearity is part
of the contract; all shaping lives vehicle-side in one place.

**Mixes:** CH1–4 from I1–I4 at weight 100; CH5 ← SW1; **CH6 ← SF at
weight −100** (so SF↑, toward the operator, = 1810 = RC — "closer to
you = you're in control"; the wire semantic 172 = AUTO is contract-fixed,
operator polarity lives here and only here); CH7 ← SB; CH8 ← SW2;
CH9 ← SW3; CH10 ← SC; CH11 ← S1 (full range — the 25 % floor is applied
in `iRCInterface`); CH12 ← SH; CH13–16 unmixed.

**Outputs:** untouched. ±100, no subtrim, no reverse, no curves.

**Special functions:** SF↑ → play `rcmode`; SF↓ → play `auto`; SH↓ →
beep (confirms the MARK registered).

**One known wart:** ELRS's arm convention is high = armed, so with KILL
on CH5 (high = killed) the Lua's config lockout engages while the boat
is *killed* — exactly when you might want to change settings. Un-kill to
configure. Cosmetic: there is no flight controller and no OSD here.

---

## 10. Test procedures

Three checklists, in order. Detail lives in the archived docs; this is
the map and the parts that changed.

**A. Handset acceptance** ([`archive/rc/rc_acceptance_test.md`](archive/rc/rc_acceptance_test.md))
— radio + bench Pi, MOOS stopped. Link config audit, power-on warnings,
the full channel truth table via `rc_probe -p crsf --frames -d
/dev/ttyAMA1`, voice callouts, telemetry sensor discovery, alarm
thresholds, link loss, Model Match wrong-boat drill. **Run this after
any model change, firmware update, or new handset.**

**B. MOOS HIL** ([`archive/rc/rc_hil_test.md`](archive/rc/rc_hil_test.md))
— bench navigator, **run inside the full `blueboat_frontseat`
mission** (props off / thrusters disconnected), which has launched
`iRCInterface` in place of `iRCReader` since 2026-08-14. The
isolated `missions/rc_hil_bench` mission still exists but adds
little now that the features span three apps — keep it only as a
bisection tool if the full stack misbehaves. The checklist is the
test script: scaled `RC_CH*` values, connection model and
safe-default publish order, telemetry from live MOOS variables,
soak — **plus the §7 feature checks: MARK edge, KILL latch across
TX-off, CH8 edge toggle, CH11 scaling (RC scales, autonomy
doesn't)**. ⚠ Correct the CH5/CH8 expectations to 1/2
([§5](#5-channel-map-the-contract)); telemetry sources now arrive
from the live apps, not `uPokeDB`.

**C. Field control parity** — RC drive on CRSF must feel identical to
SBUS and the deadman must behave identically. Not yet run.

### Bench tools

```bash
rc_probe --scan                              # which port, which protocol
rc_probe -d /dev/ttyAMA1                     # auto-detect + full verdict
rc_probe -p crsf --frames -d /dev/ttyAMA1    # live channel values
rc_probe -p crsf --link   -d /dev/ttyAMA1    # live LQ / RSSI / SNR
rc_probe -p crsf --record /tmp/cap.txt -t 10 -d /dev/ttyAMA1
rc_probe --replay /tmp/cap.txt -p crsf       # analyse off-vehicle

rc_telem_test --gps --dist -r 1 -t 0 -d /dev/ttyAMA1   # full telemetry
rc_telem_test -v 13.5 -t 20                            # alarm threshold test
```

Healthy bench link: **~87 Hz `0x16`** (100 Hz OTA × 7/8, every 8th slot
is the telemetry slot), 9.9 Hz `0x14`, **0 CRC failures**, 22-byte
payloads, uplink LQ 100 %.

Run with the MOOS RC app stopped — `rc_probe` refuses a held port, and
that refusal is itself a test that the app owns the port properly.

---

## 11. Failure signatures

Measured behaviour, so a real fault is recognisable without guessing.

### Link loss (TX powered off mid-capture)

| Time from TX kill | On the wire |
|---|---|
| 0 ms | **`0x16` frames stop dead** — no fade, no fabricated values, no failsafe frames |
| 0 – 1.1 s | `0x14` continues at 10 Hz, uplink LQ decays linearly 100 → 0 |
| 1.1 – 3.5 s | `0x14` continues, LQ pinned 0 |
| ~3.5 s | **receiver goes completely silent** |
| TX returns | `0x16` resumes within ~0.4 s at full rate, same values ±2 ticks; LQ ramps to 100 over ~1 s |

Zero CRC failures throughout: loss is presence/absence, never
corruption.

Consequences: **frame staleness (~3 missed periods ≈ 35 ms) is the
correct and complete loss detector**. The old `zoe` bug class — receiver
emitting fabricated failsafe channel values that looked like a healthy
frame rate — is *structurally extinct* under "no pulses" failsafe.
**Total serial silence is an ordinary link-down state, not an error
path**; a dead TX and a cut wire are indistinguishable after 3.5 s, and
the operator response is the same either way.

### Quick diagnosis table

| Symptom | Almost always |
|---|---|
| No frames at all, link LEDs up on both ends | **Wrong model selected** — the Model Match half-connection. Looks exactly like a dead wire the first time. |
| No serial data at all | Receiver still in SBUS output mode, wiring, or receiver power |
| Telemetry: no sensors on the handset | Pi TX wire missing (GPIO 4 → RX *Serial RX*), or `raspi-gpio get 4` isn't `TXD3` |
| Battery sensor missing, others fine | `OPOST` not cleared — the `0x0A` length byte is being rewritten |
| `TQly` missing or 0 | Telemetry ratio drifted off 1:8 |
| Downlink LQ 0 % on the Pi | **Normal. Always. Ignore it.** Judge the downlink on handset `TQly`. |
| Channel reads 992 and won't move | Missing mix line — guard-band decode should have raised a run warning |
| Worse link stats on the bench than in the field | TX power too high into a close antenna; drop to 10–25 mW |
| Frames framing at ~1/5 the expected byte rate | Two readers on one tty — check for a running MOOS RC app |

For **SBUS** boats still on `iRCReader`, the old layer-by-layer runbook
(mini UART parity, `disable-bt`, polarity checks, `sbus_probe` verdict
table) is in [`archive/rc/rc_troubleshooting.md`](archive/rc/rc_troubleshooting.md).
None of it applies to CRSF.

---

## 12. Where the code lives

| Path | Role |
|---|---|
| [`src/01_libraries/lib_crsf/`](../src/01_libraries/lib_crsf/) | `crsf_parser` (pure, no I/O), `crsf_frames` (typed decode/encode), `rc_link_state` (two-boolean connection model). Unit-tested under `test/`. |
| [`src/iRCInterface/`](../src/iRCInterface/) | The MOOS app. Owns `ttyAMA1` in both directions. |
| [`src/02_applets/rc_probe/`](../src/02_applets/rc_probe/) | Bench/field diagnostic, drives the *same* parser as the app. |
| [`src/02_applets/rc_telem_test/`](../src/02_applets/rc_telem_test/) | Telemetry sender, no MOOS required. |
| [`missions/rc_hil_bench/`](../missions/rc_hil_bench/) | HIL test mission. |
| [`missions/blueboat_frontseat/plugs/blueboat_fs/plug_iRCInterface.moos`](../missions/blueboat_frontseat/plugs/blueboat_fs/plug_iRCInterface.moos) | Config block — **not yet wired into `meta_vehicle.moos`**. |
| [`assets/rc/`](../assets/rc/) | Lua scripts, sound packs, model `.etx` exports, images, generators. |

The design rule that made the library testable, worth preserving:
**byte-stream parsing is separate from serial I/O**. `lib_sbus` calls
`read()` inside `update()`, which is why it has zero unit tests and why
`sbus_probe` had to reimplement the parser — leaving two SBUS parsers
that can disagree. `rc_probe --replay` runs the same `CrsfParser` the
app does.

### Threading model of `iRCInterface` (read before editing)

The serial thread must never wait on the app thread — at 420 kbaud the
kernel tty buffer holds about a second of traffic, and a stalled reader
loses bytes with no recovery.

1. The serial thread exclusively owns the port, the parser, and its
   working state. No lock guards them.
2. Cross-thread traffic is two small structs, both published with
   `try_lock`: `m_snap` (serial → app) and `m_telem_in` (app → serial).
   On contention the write is skipped and retried next pass (≤ 20 ms) —
   never blocked.
3. The app thread copies the snapshot under the lock, releases, then
   does **all** `Notify()` / `PostReport()` from the copy. **No mutex is
   ever held across a MOOS call.** `buildReport()` takes no lock at all,
   which makes the recursive-lock deadlock that shipped in the first
   version (Iterate → PostReport → buildReport, all on one mutex)
   structurally impossible rather than merely avoided.

The app thread also calls `refresh()` on its *copy* of the link state
before publishing, so a dead or wedged serial thread cannot freeze
`RC_CONNECTED = true` and silently defeat the navigator's deadman.
