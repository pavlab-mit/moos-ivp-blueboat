# Characterization: TX16S MK3 + XR4 — bench results

Status: **complete (2026-08-11).** Every question from the pre-bench
version of this document is now answered by measurement. This is the
factual record the RC Contract v2 design freezes against; the design
itself is [rc_handset_architecture.md](rc_handset_architecture.md).

The framing that drove the session: what arrives at the Pi is the
product of three config layers — EdgeTX mixes → ELRS OTA packet mode →
receiver serial output — and each was characterized separately.

---

## Configuration under test

| Item | Value |
|---|---|
| Handset | TX16S MK3, internal dual-band Gemini (LR1121), EdgeTX 2.12 |
| Receiver | XR4 Gemini Xrossband |
| Firmware | **ExpressLRS v4.1.0 both ends** (FCC915/ISM2G4 build) |
| RF band | **915 MHz only** — deliberate: keeps 2.4 GHz clear for the primary comms channel |
| Packet rate | **100 Hz Full** (full-resolution mode) |
| Switch mode | **16ch Rate/2** |
| Telemetry ratio | 1:8 |
| Antenna mode | gemini |
| Arm | "Arm using CH5" |
| TX power (bench) | 100 mW |

Menu survey (this build): 915 rates K1000-Full/250/200-Full/200/
100-Full/100/50/25/D50; full-res switch modes (8ch, 16ch Rate/2,
12ch Mixed) offered on Full rates; on 2.4 GHz only Wide/Hybrid were
offered. Telem ratios Std(1:128)/Off/1:128…1:2/Race. Link modes
normal/mavlink.

---

## Channel census (measured 2026-08-11)

70 s of captures, 6,805 frames, **zero CRC failures, zero bad
lengths**, all sync 0xC8.

- **0x16 RC frames: 87.3 Hz** on the serial side = 100 Hz OTA × 7/8
  (every 8th slot is the 1:8 telemetry slot). 0x14 link stats: 9.9 Hz.
- **All 16 channel slots update at ~50 Hz effective** (min value-change
  interval 19.9–20 ms on every continuously-moved channel — sticks,
  pot, sliders alike). 16ch Rate/2 behaves exactly as documented: no
  fast/slow split, no round-robin quantization.
- **Full 10-bit resolution on every channel** (~180–230 distinct values
  observed on analog channels; ±1–2 ticks ADC noise on a resting pot).
- Range at full deflection: **172–1810**; 3-pos switches land
  172/992/1810.
- **0x16 payload is 22 bytes in every frame** — no ELRS 4.x arm byte on
  the wire with "Arm using CH5". `lib_crsf` accepts 23 B too, so a
  future change is absorbed.
- **Unmixed channels emit exactly 992 (centre), rock solid.** See
  hazard below — this is the census's most consequential number.

### Hazard: a missing mix line reads as the ARMED edge

992 through the v1 `MapSwitch` math resolves a 2-state channel to
**state 2** — in the v2 map that is ARMED / deadman-DISABLED. A model
with a forgotten mix does not produce safe states; it produces the
armed edge of every 2-pos boolean.

**Requirement for the v2 decoder:** discrete channels decode with
guard bands — state 1 below ~600, state 2 above ~1400 (3-state:
bands around 172/992/1810 with invalid gaps between) — and values in
the dead bands are treated as **invalid → safe state + config
warning**. "Forgot a mix" becomes a visible fault instead of a silent
arm.

---

## Failsafe signature (measured 2026-08-11)

XR4 failsafe = "no pulses". Two TX kills captured in one 20 s record:

| Time from TX kill | On the wire |
|---|---|
| 0 ms | **0x16 frames stop dead** — instantly, no fade, no fabricated values, no failsafe frames |
| 0 – 1.1 s | 0x14 continues at 10 Hz; uplink LQ decays linearly 100→0 |
| 1.1 – 3.5 s | 0x14 continues, LQ pinned 0 |
| ~3.5 s | **receiver goes completely silent** — link stats stop too |
| TX returns | 0x16 resumes within ~0.4 s at full rate, same values ±2 ticks; LQ ramps to 100 over ~1 s |

Zero CRC failures throughout — loss is presence/absence, never
corruption.

Consequences for `iRCInterface`:

- **The `zoe` failsafe bug class is structurally extinct** in this
  configuration. There are no fabricated channel frames to misread.
  Frame staleness (~3 missed periods ≈ 35 ms) is the correct and
  complete loss detector — exactly what the existing two-boolean
  model already implements.
- **LQ is a real early-warning channel, briefly**: the 1.1 s decay
  gives `RC_LINK_LQ < 80` about a second of lead time. Worth a
  handset voice alarm and a MOOS warning.
- **Do not treat 0x14 as long-term liveness**: 3.5 s after loss the
  port is silent. Total serial silence is an ordinary link-down
  state, not an error path. (A dead TX and a cut wire are
  indistinguishable after 3.5 s; same operator response, one runbook
  line.)
- Recovery needs no special handling.

---

## Telemetry (measured 2026-08-11)

All three designed frame types confirmed end-to-end on 4.1.0:

| Frame | Handset sensors | Result |
|---|---|---|
| 0x08 battery | `RxBt` `Curr` `Capa` `Bat%` | working (re-confirmed; first proven 2026-08-10) |
| 0x21 flight mode | `FM` | **works on 4.1.0** — the earlier failure did not recur after the firmware move to 4.1.0 |
| 0x02 GPS | `GPS` `GSpd` `Hdg` `Alt` `Sats` | **all five discovered**, values live (simulated 60 m circle track from `rc_telem_test --gps`) |

- **Downlink LQ reads 0% even with telemetry demonstrably arriving**
  (ratio 1:8, sensors live on the handset). Root cause found in the
  ELRS source (2026-08-11): the receiver writes the `0x14` downlink
  fields **only under the `DEBUG_BF_LINK_STATS` build flag**
  (`rx_main.cpp`), so they are structurally 0 on every normal build,
  at every telemetry ratio — the RX cannot measure telemetry
  reception; the TX module can, and reports it to the handset
  directly as `TQly`/`TRSS`/`TSNR`. Runbook: ignore Pi-side DLQ
  everywhere; **handset `TQly` is the downlink-health canary**.
- Sensor staleness on send-stop is subjectively fast (not precisely
  quantified; red-value latency is an EdgeTX-side display property).
- GPS encoder added to `lib_crsf` (`crsfEncodeGps`, 15 B big-endian
  payload, vehicle-native units in, clamping + heading normalisation
  inside); `rc_telem_test --gps` sends a simulated moving track.

Still open, non-blocking, telemetry-phase items: add a calculated
distance sensor on the handset; decide the `Capa`→`FLT` fault-bitmask
repurposing; wire real `NAV_*`/`NVGR_*` sources in `iRCInterface`.

---

## Anomaly log

- One early `rc_probe` run framed nothing at 464 B/s (vs 2410 B/s
  healthy) after a successful auto-detect. Signature matches two
  readers splitting one tty. Unconfirmed cause; if reproduced, check
  for a running MOOS RC app first and tighten `rc_probe`'s
  port-in-use refusal.

## Constraints now frozen for the v2 contract

1. **CH5 = ARM** — protocol-reserved in Hybrid/Wide modes, so the map
   is valid under every OTA mode including range-fallback.
2. **Guard-band decoding of discrete channels** (the 992 hazard).
3. **16ch Rate/2 @ 100 Hz Full on 915 MHz** as fleet config; all 16
   channels usable at 50 Hz / 10-bit.
4. Link-loss = frame staleness; LQ decay as early warning; silence is
   a normal state.
5. Telemetry: battery + GPS + mode string all viable at 1–2 Hz under
   ratio 1:8.
