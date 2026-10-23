# RC overhaul: library-first test plan

Status: **library + applet + HIL phases complete (2026-08-11)**;
the MOOS app (`iRCInterface`) is the remaining step.
Design: [rc_crsf_design.md](rc_crsf_design.md) ·
Background: [rc_crsf_evaluation.md](rc_crsf_evaluation.md)

Order of work: **unit-tested library + applet → hardware-in-the-loop
validation → MOOS app.** The app comes last because by then the decoder
is already proven; `iRCInterface` becomes a thin MOOS wrapper over
known-good code rather than the place bugs are discovered.

---

## Why this order is right, concretely

A CRSF decoder is a pure function: bytes in, frames out. No MOOS, no
hardware, no timing dependency except a clock we can inject. That makes
it the most testable component in the RC stack — and historically the
one where the bugs live (framing, resync, CRC, bit-packing).

The current stack demonstrates the cost of skipping this. `lib_sbus`
calls `read(fd_)` directly inside `update()`
([sbus_handler.cpp:372](../src/01_libraries/lib_sbus/sbus_handler.cpp#L372)),
so parsing and I/O are inseparable — there is no way to feed it bytes
without a real serial port. That is why it has zero unit tests, and why
`sbus_probe` had to **reimplement the parser** to get a testable seam
(its `--replay` path exists for exactly this reason). We now maintain
two SBUS parsers that can disagree.

**Do not repeat that in `lib_crsf`.** The single most important design
decision in this phase:

> **Separate byte-stream parsing from serial I/O.**

Everything else in this plan follows from that.

---

## Structure

```
lib_crsf/
  crsf_parser.h/.cpp      # pure: feed(bytes) -> frames.  NO I/O.
  crsf_frames.h/.cpp      # typed decode/encode of payloads
  crsf_handler.h/.cpp     # I/O + RcDecoder impl; owns a CrsfParser
  test/
    test_crsf_parser.cpp
    test_crsf_frames.cpp
    test_crsf_telemetry.cpp
```

The parser interface — no file descriptor anywhere:

```cpp
class CrsfParser {
public:
  // Feed arbitrary byte chunks; split anywhere, any size.
  // Returns number of complete, CRC-valid frames appended to `out`.
  size_t feed(const uint8_t* data, size_t len, std::vector<CrsfFrame>& out);

  // Counters -- the probe and unit tests read these.
  uint64_t framesValid()   const;
  uint64_t framesCrcFail() const;
  uint64_t bytesDiscarded() const;
  void reset();
};
```

`CrsfHandler` does the `read()`/`write()` and hands buffers to
`CrsfParser`. Tests exercise the parser directly; hardware exercises the
handler. `rc_probe --replay` runs the *same* parser as the app — no
second implementation.

Timing (staleness, hysteresis) must take an **injectable clock**, so
tests can drive timeouts deterministically instead of sleeping:

```cpp
using ClockFn = std::function<uint64_t()>;   // micros
explicit RcDecoder(ClockFn clock = defaultMicros);
```

This also lets us unit-test the shared hysteresis policy lifted out of
`lib_sbus` — which today is only verifiable on a boat.

---

## Test framework

No test infrastructure exists in this repo yet, so this phase
establishes it. Recommendation: **GoogleTest via CMake
`FetchContent`**, with `enable_testing()` + `add_test()` so `ctest`
runs everything.

Rationale: no system dependency for people who just want to build the
apps (gate it behind `option(BUILD_TESTING "" OFF)` so the normal
`build.sh` path is unaffected), it's conventional enough that anyone can
read the tests, and `ctest` gives a single command for CI later.

If pulling a dependency is unwelcome, a hand-rolled assert-and-count
`main()` per test file is genuinely adequate here and keeps `build.sh`
dependency-free. The important thing is the **seam**, not the
framework — decide, then move on.

---

## What to unit test

### 1. Parser framing and resync — the highest-value tests

These are where real bugs live, and none of them need hardware:

- **Chunk-boundary independence.** Feed one known-good frame split at
  every possible byte offset (1..N) and assert identical output. Then
  feed it one byte at a time. Framing bugs hide here and are painful to
  reproduce on hardware, because a real UART's chunk boundaries are
  nondeterministic.
- **Multiple frames in one chunk**, including a chunk ending mid-frame
  with the remainder in the next chunk.
- **CRC rejection**: flip one bit in the payload → frame rejected,
  `framesCrcFail()` increments.
- **Resync after corruption**: prepend garbage, inject a truncated
  frame, then a valid one → the valid frame is still recovered. Assert
  the discard-one-byte-and-rescan rule holds, not
  drop-the-whole-buffer.
- **Sync byte variants**: `0xC8` and `0xEE` both accepted.
- **Malformed length fields**: `len` = 0, 1, 255, and values implying a
  frame past the 64-byte max → rejected without over-reading the
  buffer. Fuzz-adjacent and worth explicit cases.
- **Unknown frame types** are skipped cleanly, not treated as errors.

### 2. Payload decode

- `0x16` RC_CHANNELS_PACKED: the 11-bit unpack. Test against an
  **independent bit-level reference** written in the test file — a
  straightforward "read 11 bits starting at bit offset 11*n from the
  LSB-first bitstream" loop, which is the protocol definition
  transcribed directly, not a second copy of the shift-chain. Then:
  all-zero, all-max (2047), a known vector with a distinct value per
  channel so a shift-order or off-by-one bug cannot pass, and
  single-channel isolation (channel N set, all others zero) for each
  of the 16 channels.

  **Do not cross-check against `lib_sbus`.** It is untested code being
  retired, so agreement with it proves only that both implementations
  share the same behaviour — a long-standing bit-order bug there would
  be silently ratified into `lib_crsf` behind a passing test. It is
  also not reusable: the unpack is a private step inside
  `decodeFrame()` hardcoded to `frame_[1..22]` of a 25-byte SBUS frame
  and welded to SBUS header/footer validation, so using it would mean
  either exposing internals or synthesizing fake SBUS frames. A
  bit-level reference derived from the spec is both a stronger oracle
  and less coupling.

  The one genuinely useful comparison against the existing stack is at
  a higher level: an **end-to-end value check on hardware**, where the
  same physical stick position is read once over SBUS and once over
  CRSF and the resulting `RC_CH*` values must match. That validates
  the whole chain against physical reality rather than against
  untested code, and it belongs in the hardware phase below.
- `0x14` LINK_STATISTICS: field offsets, signed SNR, RSSI sign
  convention (CRSF sends positive, meaning is negative dBm — easy to
  get wrong, worth an explicit assertion).
- Channel-value → `RC_CH*` scaling: assert bit-identical results to the
  current `ScaleJoystick` / `MapSwitch`
  ([RCReader.cpp:142](../src/iRCReader/RCReader.cpp#L142)) across the
  full 172–1812 range. **This is the regression gate for "nothing
  downstream changes."**

### 3. Telemetry encode

- `0x08` BATTERY_SENSOR: known voltage/current → exact byte sequence,
  big-endian, correct CRC. Round-trip through the parser.
- `0x21` FLIGHT_MODE: string payload, null termination, and
  over-long-string truncation without overflowing the 60-byte payload
  cap.
- Round-trip property: `encode(x)` → `feed()` → `decode()` == `x`.

### 4. Connection-state policy (shared layer)

With an injected clock, assert the two-boolean model exactly as
documented in
[sbus_handler.h:98](../src/01_libraries/lib_sbus/sbus_handler.h#L98):
`isFrameValid` flips per-frame; `isControllerConnected` needs N
consecutive good frames to rise and one bad event to fall; staleness
trips at the timeout. Today this is only testable on a boat.

### Test vectors

Capture real XR4 CRSF output early with `rc_probe --record` and commit
a short capture as a fixture. Golden-file tests against real receiver
bytes catch things synthetic vectors miss — and the capture from the
current failsafe condition on `zoe` is worth keeping as a fixture too.

---

## Then: hardware-in-the-loop

`rc_probe` is built and replay-validated against synthetic captures
(see below). It is the HIL tool — the same binary, driving the same
`CrsfParser` the app will use, against a real XR4.

Replay-validated scenarios, all producing the correct verdict:

| Capture | Verdict |
|---|---|
| CRSF healthy, 165 Hz | HEALTHY, 0 CRC failures |
| CRSF uplink LQ 0 | RECEIVER REPORTS FAILSAFE (100% of link-stat frames) |
| CRSF LQ 42%, SNR −7 | LINK QUALITY BELOW 100% |
| CRSF corrupt 1-in-5 | CRC FAILURES (20%) |
| Random bytes | DATA BUT NEVER FRAMES UP |
| SBUS healthy, 72 Hz | HEALTHY |
| SBUS failsafe flag | RECEIVER REPORTS FAILSAFE (100% of frames) |

Both protocols decode a swept CH1 correctly, so the channel path is
proven end to end before touching hardware.

1. `rc_probe --protocol auto -d /dev/ttyAMA1` → confirm CRSF detected,
   CRC pass rate ~100%, expected frame-type histogram.
2. **Compare against SBUS on the same receiver**: record both modes,
   confirm identical channel values for identical stick positions. This
   is the cross-check that proves the CRSF path before it drives
   anything.
3. Stress: power-cycle the receiver, walk it out of range, unplug mid-
   frame. Assert clean resync and correct failsafe reporting — and use
   this to characterise the `zoe` failsafe anomaly with real link
   stats.
4. **TX direction**: send battery telemetry, confirm it appears on the
   TX16S. First real test of the return path and of the GPIO 4 TXD3
   wiring.
5. Soak: run for an hour, confirm zero CRC failures on a good link and
   no counter overflow or drift.

Only after step 5 does `iRCInterface` work start.

---

## Suggested first commit

Small and self-contained, so the pattern is established before the
volume of code grows:

1. `enable_testing()` + test framework wiring, gated behind
   `BUILD_TESTING=OFF` by default.
2. `lib_crsf/crsf_parser.{h,cpp}` — framing + CRC only, no payload
   decode.
3. `test_crsf_parser.cpp` — the framing/resync cases above.

That is a complete, reviewable unit: it proves the seam, proves the test
harness, and decodes nothing yet. Payload decode lands next.

## Open items

- Pick the test framework (GoogleTest vs hand-rolled) and commit to it.
- Confirm `BUILD_TESTING=OFF` default keeps `build.sh` unchanged for
  everyone not running tests.
- Capture and commit a real XR4 CRSF fixture once one boat is switched
  to CRSF output.
