# SBUS vs CRSF: protocol comparison and ELRS capability assessment

Status: **evaluation, no code written.** Decide from Part 4 before any
overhaul work starts.

Context: the fleet currently runs every receiver in SBUS mode so it
speaks to `iRCReader` / `lib_sbus`. That was a compatibility choice, not
a capability choice. This document establishes what SBUS costs us, what
the TX16S MK3 + XR4 hardware could do instead, and whether the delta
justifies an RC stack overhaul.

---

## Part 1 — What the two protocols actually are

### The wire

| | SBUS | CRSF |
|---|---|---|
| Baud | 100 000 | 420 000 (416 666 nominal) |
| Framing | 8E2 (**even parity, 2 stop**) | 8N1 |
| Inversion | Futaba inverted; RadioLink/ELRS non-inverted | non-inverted |
| Direction | receiver → FC only | **bidirectional on the same pair** |
| Frame | fixed 25 B, `0x0F` … flags … `0x00` | variable, `[sync][len][type][payload][crc8]`, ≤64 B |
| Integrity | **none** (start/end byte + range sanity only) | **CRC8, poly `0xD5`**, over type+payload |
| Channels | 16 × 11-bit packed + 2 digital | 16 × 11-bit packed (same packing) |
| Update rate | fixed ~7 ms (receiver-paced) | follows the link rate: 4 ms @ 250 Hz, 6.7 ms @ 150 Hz |

The 8E2 framing is the source of every UART headache in
[rc_troubleshooting.md](rc_troubleshooting.md) — it's why the Pi mini
UART (`ttyS0`) can't be used and why `disable-bt` is mandatory. **CRSF
is 8N1**, which any UART on the Pi can frame, including the mini UART.
That alone removes a whole failure class from the runbook.

Note the channel resolution is *identical*. Both pack 16 channels at
11 bits. The widely repeated "CRSF is higher resolution than SBUS" claim
is wrong at the protocol level — SBUS is 11-bit, not 10-bit. **We gain
nothing in stick resolution.** What we gain is latency, integrity, and
the return path.

### The three things CRSF has that SBUS structurally cannot

1. **A CRC.** SBUS validation is "start byte right, end byte right,
   values look plausible." That is exactly why `lib_sbus` needs the
   `validated_channels` range hack and 3-frame hysteresis — they are
   substitutes for a checksum. A corrupted SBUS frame whose channel
   values land in range is *undetectable* and gets published as real
   operator input. CRSF's CRC8 rejects it outright.

2. **A return path.** SBUS is simplex. There is no mechanism, at any
   layer, to send a byte back to the handset. Every bit of vehicle
   state we want the operator to see today has to travel over the
   shoreside MOOS link or not at all. CRSF is full-duplex on the same
   two wires and carries telemetry frames back on the same RF link.

3. **Link statistics as first-class data.** CRSF `LINK_STATISTICS`
   (type `0x14`) delivers uplink/downlink RSSI, SNR, link quality, RF
   mode and TX power *in band*, continuously. Today we infer link health
   from frame-loss heuristics. ELRS in SBUS mode smuggles a crude
   version of this onto CH14 — which is precisely why the mission plug
   sets `validated_channels = 12`.

### Frame types we would care about

| Type | Name | Use for us |
|---|---|---|
| `0x16` | RC_CHANNELS_PACKED | replaces SBUS channel decode |
| `0x14` | LINK_STATISTICS | RSSI/LQ/SNR → real link telemetry |
| `0x02` | GPS | lat/lon/speed/heading/sats **up to the handset** |
| `0x08` | BATTERY_SENSOR | voltage/current/mAh/percent |
| `0x1E` | ATTITUDE | pitch/roll/yaw |
| `0x21` | FLIGHT_MODE | **arbitrary string** — our MOOS mode, cheaply |

`FLIGHT_MODE` is the sleeper here: it's a free-text string field that
EdgeTX displays natively. Publishing `RC_MODE`, `DEPLOY`, or the active
behaviour name into it costs no custom Lua at all.

Frame types `0x28`+ use the extended header form
(`[sync][len][type][dest][src][payload][crc8]`), which is how
device-addressed and passthrough traffic works.

---

## Part 2 — What the TX16S MK3 + XR4 give us

### Currently unused, because SBUS mode disables it

Putting the XR4 in SBUS mode **turns telemetry off entirely.** The
ExpressLRS docs are explicit that SBUS "only has control packets and
does not have Flight Controller Telemetry," and recommend it for legacy
hardware only. So today we are paying for a dual-band Gemini receiver
and using roughly the control half of it.

Switching the XR4 to CRSF output unlocks:

- **Downlink telemetry to the handset.** Vehicle state visible on the
  TX16S screen with no shoreside link, no laptop, no network. This works
  at RF range, which for the XR4 Gemini (dual-band 2.4 GHz + 900 MHz,
  Xrossband) is far beyond where our WiFi/shoreside link dies.
- **Audible/haptic alarms.** EdgeTX triggers voice callouts and stick
  shake off telemetry sensor thresholds. "Battery low," "RC failsafe,"
  "EKF diverged" spoken to the operator, configured on the handset with
  no vehicle code.
- **Real link quality on the handset.** LQ/RSSI/SNR with EdgeTX's own
  low-LQ warnings — a genuine safety gain for over-the-horizon work.
- **Telemetry logging on the TX.** EdgeTX logs sensors to SD card,
  timestamped. An independent record of the RC link, separate from the
  MOOS alog.
- **Lua scripts on the handset.** The TX16S MK3 runs full EdgeTX with a
  colour screen; custom telemetry pages are possible.
- **Parameter editing over the link.** The ELRS Lua script configures
  packet rate, TX power and telemetry ratio in the field without
  re-flashing or re-binding.

### The bandwidth reality — read this before promising dashboards

Downlink is **narrow**. From the ExpressLRS telemetry-bandwidth tables,
the best case is 500 Hz air rate at 1:2 telemetry ratio giving 250
telemetry packets/sec ≈ **9922 bps with burst** — under 1.2 kB/s. And
that best case costs range, because high packet rates have worse
receiver sensitivity (1000 Hz FSK ≈ −101 dBm vs 25 Hz ≈ −123 dBm).

At settings we'd actually pick for a boat that wants range, expect
**hundreds of bps, not kilobits.** Practical consequences:

- A handful of scalar sensors at 1–5 Hz: comfortable.
- Native CRSF telemetry (GPS + battery + attitude + mode): fine.
- Streaming MOOS variables generically: **no.** Budget explicitly.
- MAVLink mode exists and is fully bidirectional, but forces telemetry
  ratio to 1:2 and wants F1000/333 Hz Full for usable throughput — it
  buys nothing for a MOOS boat and costs range.

Custom telemetry is real but not free: the CRSF passthrough mechanism
(as used by ArduPilot/Yaapu) needs a serial port and **deliberately
slows native CRSF telemetry to make room.** Custom sensors trade against
the built-in ones; they don't add capacity.

**Recommendation: start with native CRSF telemetry frames, not custom
ones.** GPS/battery/attitude/flight-mode cover most of what an operator
needs, are decoded by EdgeTX with zero handset-side work, and cost no
bandwidth negotiation. Reach for custom passthrough only when a specific
number can't be expressed in a native frame.

---

## Part 3 — What an overhaul actually costs us

### The fleet is mixed, and this is the dominant constraint

- `via` → RadioLink R12DS: **SBUS only.** No CRSF firmware path exists.
- `zoe` → RadioMaster XR4: CRSF-capable.

So this is **not** a migration. SBUS support cannot be deleted; `via`
would be bricked as an RC platform. The correct framing is *add a second
protocol backend*, with SBUS remaining a first-class supported path.

### What we keep

The MOOS-facing contract is the valuable, hard-won part and it should
not change:

- `RC_CH1..16`, `RC_CONNECTED`, `RC_FRAME_VALID`, `RC_FAILSAFE`
- the two-boolean model (`isFrameValid` per-frame safety gate vs
  `isControllerConnected` debounced state)
- the disconnected-fallback publish contract in
  [RCReader.cpp:245](../src/iRCReader/RCReader.cpp#L245), which
  `iBBNavigatorInterface` and its `RC_DEADMAN` watchdog depend on
- consumers: `iBBNavigatorInterface`, `pBB_Status`, `iFrontSeatBroker`

Everything downstream of `iRCReader` should be untouched by this work.
That is the test of a correct design.

### Where the work is

1. **Decoder abstraction.** Extract the interface `lib_sbus` already
   implicitly defines (channels, flags, validity, connection hysteresis)
   into a common base; `lib_sbus` and a new `lib_crsf` both implement it.
   `iRCReader` gains a `protocol = sbus|crsf` config param and picks a
   backend. `via` keeps `protocol = sbus`, `zoe` gets `protocol = crsf`.
   The hysteresis/validity logic is protocol-independent and should be
   lifted, not duplicated.

2. **CRSF decode.** Byte-stream framing on `[sync][len][type]…[crc8]`,
   CRC8 poly `0xD5`, then `0x16` unpack (identical 11-bit packing to
   SBUS — that code is reusable) and `0x14` link stats. Modest, and
   *simpler* than the SBUS path because the CRC replaces all the
   range-check and hysteresis guesswork. `validated_channels` disappears
   for CRSF.

3. **UART, now bidirectional.** Today the RC UART is RX-only. Telemetry
   needs TX wired back to the receiver — a **physical wiring change** on
   each boat, plus confirming the chosen Navigator UART has a free,
   routed TX pin. Worth checking early; it can gate the whole telemetry
   half. Upside: 8N1 means the mini UART constraint and the `disable-bt`
   requirement go away for CRSF boats.

4. **Telemetry publisher.** A new path that subscribes to MOOS state and
   emits CRSF telemetry frames. Cleanest as a separate app
   (`iRCTelemetry` or similar) so `iRCReader` stays a decoder — but note
   two processes cannot share one tty, so realistically the TX side
   belongs *inside* `iRCReader` even though that muddies its role. Worth
   a deliberate decision; the tty-ownership constraint probably wins.

5. **Tooling.** `sbus_probe` is genuinely excellent and the reason RC
   problems are now diagnosable. A CRSF equivalent (or `--protocol crsf`
   in the existing tool) is not optional — without it we lose the
   layer-by-layer diagnosis the runbook is built on, and the runbook
   needs a parallel CRSF branch.

Item 5 is the one most likely to be underestimated. The debugging
tooling is a large part of what makes the current stack trustworthy.

---

## Part 4 — Recommendation

**The effort is worth it.** See [rc_crsf_design.md](rc_crsf_design.md)
for the agreed design. Summary of the decision:

- Fleet moves to CRSF wholesale once hardware is in hand, so the
  dual-backend concern in Part 3 is **resolved**: SBUS stays only as a
  transitional/diagnostic path, not a permanent parallel product.
- `iRCReader` is replaced by `iRCInterface` (it is no longer read-only).
- `lib_crsf` is new; `rc_probe` supersedes `sbus_probe` and handles both
  protocols.
- First-pass telemetry: **battery voltage and connectivity only.**

The integrity argument alone justifies the control-path work: today an
in-range corrupted SBUS frame is published as genuine operator input,
and no amount of hysteresis fixes that.

---

## Sources

- [CRSF Packet Types (crsf-wg wiki)](https://github.com/crsf-wg/crsf/wiki/Packet-Types)
- [CRSF Message Format (crsf-wg wiki)](https://github.com/crsf-wg/crsf/wiki/Message-Format)
- [ExpressLRS — Receiver Serial Protocols](https://www.expresslrs.org/software/serial-protocols/)
- [ExpressLRS — Telemetry Bandwidth](https://www.expresslrs.org/info/telem-bandwidth/)
- [ExpressLRS — Signal Health](https://www.expresslrs.org/info/signal-health/)
- [ExpressLRS — MAVLink](https://www.expresslrs.org/software/mavlink/)
- [Yaapu — Passthrough over CRSF and ExpressLRS](https://github.com/yaapu/FrskyTelemetryScript/wiki/Passthrough-over-CRSF-and-ExpressLRS)
- [RadioMaster XR4 Gemini product page](https://radiomasterrc.com/products/xr4-gemini-xrossband-dual-band-expresslrs-receiver)
