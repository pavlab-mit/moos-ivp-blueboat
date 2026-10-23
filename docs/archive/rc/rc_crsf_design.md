# RC stack overhaul: iRCInterface + lib_crsf + rc_probe

Status: **bench phase complete (2026-08-11)** — `lib_crsf` (unit-tested),
`rc_probe`, and `rc_telem_test` built and hardware-verified on the bench;
all three telemetry frame types confirmed end-to-end and the downlink-LQ
question resolved (structural ELRS, see
[rc_characterization.md](rc_characterization.md)). **`iRCInterface`
implemented (CRSF-only; `iRCReader` kept as the SBUS fallback rather
than refactored behind `RcDecoder`) — MOOS HIL test pending**, see
[rc_hil_test.md](rc_hil_test.md). Field parity and fleet migration
remain.
Background and protocol justification: [rc_crsf_evaluation.md](rc_crsf_evaluation.md).

## Decisions taken

1. `iRCReader` → **`iRCInterface`**. The app is no longer read-only; it
   owns the RC serial port in both directions.
2. New **`lib_crsf`** alongside the existing `lib_sbus`.
3. New **`rc_probe`** applet, superseding `sbus_probe`, covering both
   protocols.
4. **The whole fleet moves to CRSF** once hardware is in hand. SBUS is a
   transitional and diagnostic path, not a permanent parallel product.
5. First-pass telemetry: **battery voltage** and **connectivity**.

Decision 4 is the one that simplifies everything. The evaluation doc
worried about maintaining two first-class backends forever; that concern
is now scoped to a migration window instead.

---

## Findings that change the design

### The XR4 wiring is confirmed usable — verify the pin function anyway

`ttyAMA1` is Navigator SERIAL 3 on GPIO 4/5 (documented in
[rc_troubleshooting.md:69](rc_troubleshooting.md)), and GPIO 4 = TXD3,
GPIO 5 = RXD3 under `dtoverlay=uart3`. Both directions are routed, so
Phase B is **not blocked**. Confirm on the boat before writing TX code:

```bash
raspi-gpio get 4
raspi-gpio get 5
```

Expect `func=TXD3` / `func=RXD3`. If GPIO 4 reads as an input or GPIO
function, the overlay is enabling RX only and the plumbing needs fixing
first. Also confirm the XR4's TX pad is physically wired back — RX-only
harnesses are common since nothing needed the return path until now.

### `zoe` is currently flying in ELRS failsafe and nobody noticed

From `LOG_SHORESIDE_27_7_2026_____10_59_10`, sustained across hundreds
of iterations:

```
Failsafe Active: YES
Frame Valid: NO     RC Connected: NO
Time Since Last Frame: 6.1 ms      Consecutive Frame Losses: 0
```

Frames arrive steadily every ~6 ms with zero losses, but the receiver
asserts the SBUS failsafe flag. This is **not** a link-quality problem —
it is the receiver reporting it has no valid uplink while still emitting
frames. Note CH11 reads 978 in the failsafe capture vs 1004 in the
healthy one, i.e. the receiver is emitting its configured failsafe
positions.

This matters for the overhaul in two ways:

- It is a **live open bug**, independent of CRSF. Worth diagnosing
  before the migration so we don't mistake it for a regression
  afterwards. Likely candidates: ELRS failsafe configured to
  "last position" vs "no pulses", or a bind/model-match issue.
- It is exactly the failure the current stack diagnoses *poorly*. The
  operator sees `RC_CONNECTED=false` with a healthy-looking frame rate
  and no explanation. CRSF `LINK_STATISTICS` would state the uplink
  quality directly rather than leaving us to infer it.

### `validated_channels` disappears, and so does a latent hazard

CH13/15/16 read 0 and CH14 carries ELRS link stats — the reason for
`validated_channels = 12`. Under CRSF the CRC does this job properly,
`validated_channels` is deleted from config, and link stats arrive in a
real frame instead of being smuggled through a channel slot.

---

## Architecture

```
                    ┌───────────────────────────────┐
   XR4 ─ ttyAMA1 ───┤ iRCInterface                  │
     (RX + TX)      │   owns the tty, both ways     │
                    │                               │
                    │  ┌─────────────────────────┐  │
                    │  │ RcDecoder (interface)   │  │
                    │  │   lib_sbus | lib_crsf   │  │
                    │  └─────────────────────────┘  │
                    │  ┌─────────────────────────┐  │
                    │  │ CrsfTelemetryEncoder    │  │
                    │  └─────────────────────────┘  │
                    └───────────────────────────────┘
                       ▲ subscribes          │ publishes
                       │                     ▼
              NVGR_ROLLING_VOLTAGE      RC_CH1..16, RC_CONNECTED,
                                        RC_FRAME_VALID, RC_FAILSAFE,
                                        RC_LINK_*  (new)
```

**One app owns the port in both directions.** The evaluation doc floated
a separate telemetry app; that is not viable, because two processes
cannot share one tty. Naming it `iRCInterface` rather than `iRCReader`
is what makes the combined role honest.

### What must not change

The MOOS-facing contract is load-bearing and stays byte-identical:

- `RC_CH1..16`, `RC_CONNECTED`, `RC_FRAME_VALID`, `RC_FAILSAFE`
- the two-boolean model: `isFrameValid` (per-frame safety gate) vs
  `isControllerConnected` (debounced state)
- the disconnected-fallback publish block at
  [RCReader.cpp:245](../src/iRCReader/RCReader.cpp#L245) — the
  `RC_DEADMAN` watchdog in `iBBNavigatorInterface` depends on its exact
  semantics, including that `RC_CONNECTED` is published *before* the
  safe-default `RC_CH*` values within an iterate
- consumers untouched: `iBBNavigatorInterface`, `pBB_Status`,
  `iFrontSeatBroker`

**This is the acceptance test for the whole overhaul:** nothing
downstream of the RC app changes. Channel scaling and switch mapping
must be lifted verbatim, including the CH5–CH9 mapping table and the
`SBUS_MIN/MID/MAX` → ±100 scaling. CRSF uses the same 11-bit packing, so
raw values land in the same 172/992/1812 range and the existing scaling
math carries over unchanged.

### New variables (additive only)

| Variable | Source | Note |
|---|---|---|
| `RC_LINK_LQ` | CRSF `0x14` | uplink link quality, % |
| `RC_LINK_RSSI` | CRSF `0x14` | dBm |
| `RC_LINK_SNR` | CRSF `0x14` | dB |
| `RC_PROTOCOL` | config | `sbus` or `crsf`, for log forensics |

`RC_LINK_LQ` is the one that would have made the `zoe` failsafe obvious.

---

## Decoder interface

Lift the protocol-independent logic — hysteresis, staleness, the
two-boolean connection model — into the shared layer. Only framing and
validation differ per protocol.

```cpp
class RcDecoder {
public:
  virtual ~RcDecoder() = default;

  virtual bool initialize() = 0;
  virtual void close() = 0;
  virtual bool update() = 0;              // pump bytes, true on new frame

  virtual uint16_t getChannel(uint8_t ch) const = 0;
  virtual bool     isFailsafe() const = 0;
  virtual bool     isFrameLost() const = 0;

  // Shared, non-virtual: identical policy across protocols.
  bool     isFrameValid() const;
  bool     isControllerConnected() const;
  uint64_t getTimeSinceLastFrame() const;

  virtual bool hasLinkStats() const { return false; }
  virtual LinkStats getLinkStats() const { return {}; }

  virtual bool sendTelemetry(const TelemetryFrame&) { return false; }
};
```

`lib_sbus` keeps its range-check validation; `lib_crsf` uses the CRC and
needs no equivalent.

### CRSF specifics

- 420 000 baud, **8N1** — no parity, so the mini-UART constraint and the
  `disable-bt` requirement do not apply. Do not carry that boot-config
  advice forward into the CRSF runbook.
- Frame: `[sync][len][type][payload][crc8]`, sync `0xC8` (accept `0xEE`),
  `len` counts type+payload+CRC, max 64 B total.
- CRC8 poly `0xD5` over type+payload.
- Decode `0x16` RC_CHANNELS_PACKED (16 × 11-bit) and `0x14`
  LINK_STATISTICS.

  The channel packing is the same scheme SBUS uses, but **write the
  unpack fresh against the spec rather than lifting it from
  `lib_sbus`.** That code is a private step inside `decodeFrame()`
  hardcoded to `frame_[1..22]` of a 25-byte SBUS frame and entangled
  with SBUS header/footer validation; it is also untested, so copying
  it would propagate any latent bit-order bug into the library that is
  meant to replace it. The shared property is worth knowing — it means
  the *scaling* math downstream carries over unchanged — but the
  decode itself should be independent.
- Ignore unknown frame types silently; the receiver emits several.

Resync rule: on CRC failure, discard one byte and rescan for sync rather
than dropping the whole buffer. Cheap, and avoids cascading loss.

---

## Telemetry: battery voltage and connectivity

Deliberately narrow, matching the bandwidth reality in the evaluation
doc (hundreds of bps at range-friendly settings).

**Battery** — CRSF `0x08` BATTERY_SENSOR, from `NVGR_ROLLING_VOLTAGE`
(already published; `pBB_Health` consumes it at
[BB_Health.cpp:399](../src/pBB_Health/BB_Health.cpp#L399)). Payload is
voltage (dV, big-endian), current (dA), capacity (3 B mAh), remaining
(%). Send voltage and current from `NVGR_ROLLING_CURRENT`; zero the
fields we don't have rather than inventing them.

**Connectivity** — mostly free: `LINK_STATISTICS` is receiver-generated,
so the handset already shows LQ/RSSI without us sending anything. What
we add is *vehicle-side* liveness, so the operator can distinguish "RF
link fine, MOOS dead" from "RF link lost". CRSF `0x21` FLIGHT_MODE is a
free-text string EdgeTX displays natively — publish the MOOS mode there
(`RC`, `AUTO`, `MANUAL`, `DEADMAN`). Costs one small frame and no
handset-side code.

Rate: **1–2 Hz is plenty** for both. Do not send on every iterate.

EdgeTX voice alarms then come for free on the handset: low-voltage
callouts and LQ warnings configured entirely on the TX16S, no vehicle
code. That is most of the operator-facing value of this phase.

### The one real hazard

The telemetry TX path must never stall the RC decode path. A blocking
write on a busy port delays control input. Guard it: non-blocking
writes, drop telemetry frames rather than queue them unboundedly, and
keep the encoder off the read thread's critical section. **RC control
latency is the safety-critical property; telemetry is not.**

---

## rc_probe

`sbus_probe` is why RC faults are diagnosable today, and its verdict
model is the part worth preserving. `rc_probe` keeps the structure and
generalises it:

- `--protocol sbus|crsf|auto` — auto-detect by trying 100 000 8E2 and
  420 000 8N1 and seeing which frames up. Auto-detect is the headline
  feature: "which protocol is this receiver actually emitting" is the
  first question in the field during a mixed-fleet migration.
- `--scan` across ttyS0 + ttyAMA*, both protocols.
- CRSF verdicts: CRC pass rate, frame-type histogram, decoded link
  stats. The CRC pass rate replaces most SBUS-era guesswork — a bad line
  shows up as a CRC failure rate rather than "values out of range".
- Keep `--record` / `--replay`; add frame-type tagging.
- Keep `--lib` (run the real decoder class) — the step that proves the
  app will agree with the probe.
- Keep the port-in-use refusal.

Then fork [rc_troubleshooting.md](rc_troubleshooting.md) into a CRSF
branch. Much of Step 1 (parity, mini UART, `disable-bt`) becomes
inapplicable; Step 3's verdict table needs CRSF rows.

---

## Sequencing

1. **Diagnose the `zoe` failsafe** on the existing stack. Don't carry an
   unexplained fault into the migration.
2. **Extract `RcDecoder`**, reimplement `lib_sbus` behind it, rename to
   `iRCInterface`. No behaviour change — `RC_*` output must be
   bit-identical. Verify against a replayed capture.
3. **`lib_crsf`** decode + `rc_probe` with auto-detect. Bench-test on
   `zoe` with the XR4 switched to CRSF output.
4. **Field-verify control parity** before any TX code. RC drive on CRSF
   must feel identical and the deadman must behave identically.
5. **Telemetry TX**: battery + flight-mode string, 1–2 Hz, with the
   non-blocking guard above.
6. **Fleet migration** + runbook fork.

Steps 2 and 4 are the risk-reduction steps and the ones most likely to
get skipped under time pressure. Step 2 landing separately means that if
CRSF misbehaves in the field, the fallback is a config change rather
than a revert.

## Open items

- Confirm `raspi-gpio get 4` / `get 5` show `TXD3`/`RXD3` on `zoe`, and that
  the XR4 TX pad is physically wired.
- Decide the ELRS packet rate / telemetry ratio for boat use — favour
  range over rate; 50–150 Hz is likely right for an ASV, not 500 Hz.
- Confirm whether any boat needs `RC_LINK_*` forwarded shoreside via
  `iFrontSeatBroker` `tx_vars`.
