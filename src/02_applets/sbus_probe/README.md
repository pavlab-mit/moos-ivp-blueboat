# sbus_probe — RC / SBUS link diagnostics

Standalone CLI tool (no MOOS required) for debugging the
RadioLink AT9S Pro → receiver → UART → `iRCReader` chain when the
transmitter shows a healthy RF link but `RC_CONNECTED` stays false.

The transmitter's "connected" indicator only proves the RF bind.
Between the receiver and `RC_CONNECTED` there are five places the
chain can break, and the iRCReader appcast can't tell them apart.
This tool can.

## Quick start (on the vehicle)

```
# 1. System-level checks (UART mapping, port holders, serial console):
./scripts/sbus_diag.sh /dev/ttyS0

# 2. Stop MOOS / iRCReader (concurrent tty readers split the stream!)

# 3. Automatic diagnosis with verdict (5 s capture):
sbus_probe                      # default device /dev/ttyS0
sbus_probe -d /dev/ttyAMA0 -t 10
```

## Modes

| Mode | What it shows |
|---|---|
| *(default)* | Timed capture, then an automatic verdict |
| `--raw` | Live hex dump; start bytes begin new lines so frame cadence is visible |
| `--frames` | Live channel table with min/max, out-of-range marks, reject counters |
| `--lib` | Runs the **real `SbusHandler`** — reproduces exactly what iRCReader computes |
| `--record F` | Capture timestamped raw bytes to a file |
| `--replay F` | Re-run the verdict on a recorded capture (works on any machine) |

## Decision tree

```
port won't open ──────────► wrong device / permissions / UART disabled
        │
zero bytes ───────────────► receiver not outputting SBUS:
        │                    - RadioLink R9DS/R12DS in PWM mode (LED red).
        │                      Short-press ID SET twice within 1 s → blue = SBUS.
        │                    - wiring (signal pin, common ground, RX power)
        │                    - wrong UART device
        │
bytes but no frames ──────► line-level problem:
        │                    - byte mix dominated by 0x00/0xFF = inverted
        │                      signal (SBUS is inverted UART; needs inverter)
        │                    - Pi mini UART (ttyS0) has no parity HW; SBUS
        │                      is 8E2 → use a PL011 (ttyAMA*)
        │
frames but all rejected ──► decode-level problem:
        │                    - footer histogram wrong = misalignment
        │                      (baud/parity) or non-SBUS protocol (i-BUS?)
        │                    - per-channel range violations: undriven
        │                      channels at 0 make the library's all-16-
        │                      channel range check discard every frame →
        │                      enable the channels on the TX or relax check
        │
valid but flagged ────────► RF-side problem, receiver says so itself:
        │                    failsafe/frame_lost set → rebind, antennas,
        │                    interference. Intermittent flags also hold
        │                    RC_CONNECTED false (1 bad frame resets the
        │                    3-consecutive-good hysteresis).
        │
all clean ────────────────► link is fine; problem is app-side:
                             - iRCReader hardcodes /dev/ttyS0 (no config
                               option) — is SBUS actually on that UART?
                             - check appcast for "Failed to initialize
                               SBUS handler"
                             - another process stealing bytes from the tty
```

## Notes

- `iRCReader` publishes `RC_CONNECTED` (debounced, 3 consecutive
  clean frames to set, 1 bad frame to clear) and `RC_FRAME_VALID`
  (instantaneous). The probe's "library-equivalent RC_CONNECTED"
  replicates that state machine; `--lib` runs the genuine class.
- Healthy RadioLink SBUS: ~14 ms frame interval (~71 Hz, ~1790 B/s).
- Record on the boat, replay anywhere:
  `sbus_probe --record cap.txt -t 30` → copy off →
  `sbus_probe --replay cap.txt`.
