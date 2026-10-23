# Handset architecture: RC Contract v2 for TX16S MK3

Status: **constraints bench-verified 2026-08-11** — see
[rc_characterization.md](rc_characterization.md) for the measurements.
Every item tagged [OPEN] below has been resolved; tags are kept so the
claim-to-evidence chain stays visible. Fleet link config is frozen as:
**915 MHz, 100 Hz Full, 16ch Rate/2, telem ratio 1:8, ELRS 4.1.0 both
ends.**

The only inputs treated as given:

1. The link speaks **CRSF** on the wire.
2. **Left stick = throttle** (now spring-centred), **right stick =
   steering**.
3. The handset is a **TX16S MK3** running EdgeTX 2.12.

Nothing about the RF firmware, channel behaviour, rates, failsafe, or
telemetry scheduling is assumed — including whether the firmware is
ExpressLRS. That is question zero below.

---

## What we actually know

### [SPEC] — true of CRSF regardless of radio firmware

- Frame format `[sync][len][type][payload][crc8]`, CRC8 poly `0xD5`,
  420 kbaud 8N1.
- `0x16` RC_CHANNELS_PACKED always packs **16 × 11-bit slots** per
  frame. Note the SBUS-era observation "CH13/15/16 read 0" was a
  property of the old receiver's *SBUS output mode* and says nothing
  about what occupies those slots under CRSF.
- Channel values land in the same 172/992/1811 range the existing
  scaling math already expects.
- `0x14` LINK_STATISTICS is receiver-generated: uplink/downlink
  RSSI, LQ, SNR.
- Telemetry frame types exist for battery (`0x08`), GPS (`0x02`),
  attitude (`0x1E`), flight mode (`0x21`).

### [VERIFIED] — observed on the TX16S/XR4 bench rig

- **Battery telemetry end-to-end works** (2026-08-10): `0x08` frames
  from the Pi appear on the handset as `RxBt`/`Curr`/`Capa`/`Bat%` and
  go stale when the sender stops.
- **`0x21` flight-mode did NOT appear** as a sensor in the same
  session. Cause unknown. Treat FM as unavailable until diagnosed.
- **Downlink LQ 0% with working telemetry** is a real state, tied to
  the telemetry-ratio setting; the tools now treat it as inconclusive.
- **OPOST tty mangling** corrupts any frame containing a `0x0A` byte —
  not just the battery frame's length byte. Any payload byte (a
  latitude, a voltage) can hit `0x0A`, so with GPS flowing this
  presents as intermittent glitches, not a clean failure. Raw mode
  always, both tools already comply.
- **TX16S switch inventory** (read from the radio's own hardware
  config): 3-pos SA/SB/SC/SD/SE/SG; **SF is the only latching 2-pos**;
  SH is 2-pos momentary; SW1–SW6 are RGB-lit buttons configurable
  2POS/3POS; pots S1/S2, sliders LS/RS.
- **EdgeTX 2.12 stores each model as YAML on the SD card**, and model
  images are 192×114 PNG, filename ≤ 9 chars, in `/IMAGES`.

### [OPEN] — the characterization list

This is the gate for the rest of the document. Each question is paired
with the experiment that answers it; all of them are an afternoon with
`rc_probe` and the bench rig.

| # | Question | How to answer it |
|---|---|---|
| 0 | What RF firmware and version is on the TX module and each receiver? | Radio's module info screen; receiver WebUI. Record exact versions per unit. |
| 1 | How many channel slots carry live data, at what per-channel update rate? | `rc_probe --record` while toggling each handset control CH1→CH16; measure per-slot change latency. |
| 2 | What do unused slots read — 992? 172? frozen? | Same capture, controls untouched. |
| 3 | RC payload length — 22 B or 23 B (arm-status byte)? | Frame-length histogram from `rc_probe`. |
| 4 | What does the receiver emit on link loss — silence, failsafe values, flag? | Power off the handset mid-capture. This decides how `iRCInterface` detects loss, so it blocks Phase 2. |
| 5 | Does a model-lock feature exist (e.g. Model Match) and what is its wrong-model failure mode? | Firmware docs for whatever #0 found, then bench test: wrong model selected, does the boat move? |
| 6 | Available packet rates, and telemetry scheduling per rate? | Module config UI, then measure telemetry frame arrival rates at the chosen setting. |
| 7 | Why did `0x21` FM not appear? | Byte-dump the frame; check null-termination and length; re-discover sensors with only FM being sent. |
| 8 | Real telemetry downlink budget at the chosen rate? | Send battery+GPS at 1 Hz each, measure loss/latency on the handset. |

Everything below that depends on an OPEN item names the item it waits
on.

---

## Design that does not depend on the radio

These choices are radio-independent, which is exactly why they carry to
other projects.

### Semantic variables, not channel numbers

The vehicle publishes `RC_THRUST`, `RC_STEER`, `RC_ARM`, `RC_MODE`,
`RC_DEADMAN_EN`, … — nothing downstream ever sees a channel index. One
config file (`profile.yaml`, below) owns the channel↔semantic mapping.
When a future radio, protocol, or handset changes the map, consumers
don't move.

This also retires the v1 sign-convention wart: declare **+ = starboard,
+ = ahead** in the spec, put any inversion in the handset mix, and
delete the `mixer = -m_rc_channels[0]` negation in the same commit the
convention lands.

### State 1 is the operator-absent default

`iRCInterface` publishes switches → state 1 on link loss, so every
discrete channel is defined with state 1 = **the state the vehicle
should hold when no operator is intervening**. Lab policy (2026-08-11):
the vehicle is **armed by default** — an over-the-horizon boat must
not stop because the RC link went away. So state 1 means AUTO for
MODE, RUNNING for KILL, ENABLED for the deadman. The fallback is
correct by construction instead of by comment. This is an invariant
of the *contract*, not of any radio.

Guard-band decoding qualifier: for the KILL channel, a dead-band /
invalid value **latches the last valid state** instead of defaulting —
a config fault must never silently kill an autonomous boat, nor
un-kill a killed one.

### Kill is separate from mode

v1's single "RC mode" boolean conflated "outputs enabled" with "who is
commanding". Contract v2 keeps `KILL` (operator stop, armed-by-default
per lab policy) and `MODE` (command source) as separate variables
regardless of which physical controls they land on.

### The provisioning pipeline

EdgeTX models are YAML files [VERIFIED], so handset configuration can
be a build artifact:

```
rc/
  profile.yaml        # channel map, telemetry map, conventions
  models/_template.yml  # golden model, built once in Companion
  models/<boat>.yml     # generated per boat
  images/<boat>.png     # 192x114, name ≤ 9 chars
  scripts/gen_models.py # profile.yaml + template -> models
  scripts/gen_header.py # profile.yaml -> rc_profile.h
```

`profile.yaml` generates both the handset models and the C++ channel
constants, so handset and vehicle cannot disagree. Adapting to a new
project = new `profile.yaml`, same pipeline. Hazards: the YAML schema
is firmware-version specific (pin 2.12, re-verify on upgrade), and the
template must be born in Companion, not a text editor.

---

## Channel map (frozen 2026-08-11)

The census removed the fast/slow anxiety: under 16ch Rate/2 at 100 Hz
Full, **all 16 channels carry 10-bit values at ~50 Hz effective** — so
block boundaries below are organizational, not rate-driven. Two
protocol-derived constraints are load-bearing: **CH5 = ARM** (reserved
as the arm channel by Hybrid/Wide OTA modes, so the map survives a
range-fallback mode change), and **discrete channels must be decoded
with guard bands** — an unmixed channel emits 992 (centre), which
naive midpoint decoding maps to the ARMED edge of every 2-pos
channel. Guard-band decode: state 1 < ~600, state 2 > ~1400, dead
bands → invalid → safe state + config warning.

### Block A — axes (CH1–4), lab-wide

| CH | Name | Source | Range |
|---|---|---|---|
| 1 | `STEER` | right stick L/R | ±100, + = starboard |
| 2 | `AUX_AXIS_1` | right stick F/B | reserved |
| 3 | `THRUST` | left stick F/B | ±100, + = ahead |
| 4 | `AUX_AXIS_2` | left stick L/R | reserved |

Spring-centred throttle means stick release commands zero thrust —
enable the throttle-not-centred startup warning and centre beep on Thr.

### Block B — safety & mode (CH5–8), lab-wide

Finalized against lab policy and the YIP build
([rc_model_build_yip.md](rc_model_build_yip.md)):

| CH | Name | States | Control | 1 (default) | 2 | 3 |
|---|---|---|---|---|---|---|
| 5 | `KILL` | 2 | SW1 button, LED red = killed | RUNNING | KILLED | — |
| 6 | `MODE` | 2 | SF (up/toward operator = RC; mix weight −100) | AUTO | RC | — |
| 7 | `ACTION` | 3 | SB | HOLD | RESUME | RETURN |
| 8 | `DEADMAN_EN` | 2 | SW2 button, LED amber = disabled | ENABLED | DISABLED | — |

`MODE` default = AUTO preserves the existing latching intent: on link
loss nobody is driving, so the boat must not silently become manual.
Keep the "ignore MODE while disconnected" latch; apply the same latch
to `KILL`. SF carries MODE (operator preference — it is the frequent
switch); the ELRS arm-flag caveat of putting KILL on CH5 is noted in
the build card and is cosmetic in this system.

### Block C — vehicle class (CH9–12), per project

| CH | Name | Type | Proposed control |
|---|---|---|---|
| 9 | `PAYLOAD_PWR` | 2 states | SW2 |
| 10 | `PAYLOAD_MODE` | 3 states | SC |
| 11 | `THRUST_LIMIT` | analog, 25–100 % | S1 pot |
| 12 | `MARK` | momentary | SH |

`THRUST_LIMIT` is the best QOL/effort ratio on the page: dial authority
down for shallow water, new operators, first splash. Floor at 25 % so
"why won't it move" can't happen. `MARK` stamps an event token into the
MOOS log from the handset.

### CH13–16

Unassigned until OPEN #1/#2 say what they actually do. If they turn
out live and fast, Block C grows; if stale or slow, they stay reserved.

---

## Telemetry plan

All three designed frame types are [VERIFIED] end-to-end on 4.1.0
(2026-08-11): battery, GPS (all five sensors discovered, values live
from the `rc_telem_test --gps` simulated track), and the flight-mode
string (the earlier 0x21 failure did not recur after the firmware move
to 4.1.0). Downlink LQ reads 0% even with telemetry arriving —
structural, not firmware-version-specific: ELRS receivers populate
the `0x14` downlink fields only under the `DEBUG_BF_LINK_STATS`
debug build flag. Downlink quality is measured at the TX module and
surfaces only as the handset `TQly`/`TRSS`/`TSNR` sensors; the
handset is the truth.

| Frame | Handset sensors | Source | Status |
|---|---|---|---|
| `0x08` battery | `RxBt` `Curr` `Capa` `Bat%` | `NVGR_ROLLING_VOLTAGE/CURRENT` | verified |
| `0x02` GPS | `GPS` `GSpd` `Hdg` `Sats` `Alt` | `NAV_LAT/LONG/SPEED/HEADING` | verified (synthetic); wire real NAV_* next |
| `0x21` flight mode | `FM` text (≤13 chars) | MOOS mode string | verified |
| `0x1E` attitude | `Ptch` `Roll` `Yaw` | `NAV_ROLL/PITCH` | off unless needed |

**GPS first once gated** — EdgeTX derives distance-to-vehicle and a map
widget from it, which for a boat past easy sight range is worth more
than everything else combined. Rates configurable in the plug; write
the measured budget from #8 next to them.

**Numeric fault channel.** EdgeTX alarms only on numeric sensors, so
the FM string (even once fixed) can't drive audio alerts. The battery
frame's `Capa` field is currently zeroed; repurposing it as a fault
bitmask (leak, ESC fault, telemetry stale…) gives spoken fault callouts
via logical switches for three bytes already being sent. Non-standard
use of a standard field — documented in `profile.yaml` or not at all.

**Handset-side, no vehicle code:** three telemetry screens (OPS: bat,
LQ, dist, speed; LINK: full `0x14` stats — the screen that would have
caught the `zoe` failsafe in seconds; NAV: map widget), plus logical-
switch alarms for low LQ, two voltage steps, telemetry lost, `FLT > 0`.

---

## Identity & QOL

- **Name + image:** model name = boat name; image = photo with a
  high-contrast colour band and huge text (sunlight, dock, hurry);
  192×114 PNG, ≤ 9 chars. Label all models `BLUEBOAT` so the fleet
  groups in the model list.
- **Startup checklist:** `/MODELS/<model>.txt`, four lines — right
  boat confirmed, KILL dark, MODE = AUTO (SF down), thrust limit set.
- **Enforced pairing: ELRS Model Match** (resolved OPEN #5 — present
  on 4.1.0). Receiver No. = boat's fleet number, hull-based: **zoe 31,
  yip 32, xai 33, …** (Model Match range 0–63); latch with Model
  Match → On in the Lua per receiver. Wrong-model failure mode is a
  *half connection* — link up, no control data — which reads exactly
  like a wiring fault the first time; it's in the runbook.

---

## Sequencing

1. **Characterization afternoon** — answer OPEN #0–#6 with `rc_probe`
   and the bench rig. Cheap, and everything else stops guessing.
2. `profile.yaml` + `gen_header.py`; publish semantic `RC_*` alongside
   raw `RC_CH*` (additive).
3. Hand-build `_template.yml` in Companion; verify every channel in
   the Channels Monitor against expected CRSF raw values.
4. Migrate consumers to semantic variables; land the sign convention
   and delete the negation in one commit.
5. GPS telemetry (after #7/#8), then screens, alarms, images,
   checklists, model-lock.
6. `gen_models.py`, rest of the fleet.
