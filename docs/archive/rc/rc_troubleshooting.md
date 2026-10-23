# RC / SBUS troubleshooting runbook

Symptom this covers: transmitter (RadioLink AT9S Pro) shows a healthy
RF link, but iRCReader publishes `RC_CONNECTED = false`.

Remember: the transmitter's "connected" indicator only proves the RF
bind. The serial chain — receiver output mode, wiring, UART hardware,
line config, frame decode, receiver flags, app config — is invisible
to it and each layer fails differently. Work the layers in order;
each step's tool output tells you whether to stop or continue.

Tools (built from this repo; `sbus_probe` lands in `bin/`):
- `scripts/sbus_diag.sh` — system layer (UARTs, holders, console)
- `sbus_probe` — byte/frame layer with automatic verdicts

**Stop MOOS first.** Two processes reading one tty split the byte
stream and both see garbage. `sbus_probe` refuses to run if the port
is held (override with `--force`, but don't).

---

## Step 1 — system layer

```
./scripts/sbus_diag.sh /dev/ttyAMA0     # or the device in the plug
ls -l /dev/serial0 /dev/serial1
```

**Decision point — which UART owns the RC pins (GPIO 14/15)?**

| `/dev/serial0 ->` | Meaning | Action |
|---|---|---|
| `ttyAMA0` | PL011 (parity-capable) owns the pins | proceed to Step 2 |
| `ttyS0` | mini UART owns the pins | **fix boot config first, below** |

The Pi mini UART has no parity hardware. SBUS is 100000 baud 8E2.
No software setting fixes this. The failure is boat-dependent because
the mini UART's clock tracks the core clock (varies with board
revision, firmware, temperature): some boats decode nothing, others
limp along with ~50% wire-level frame loss and look like they "work"
until RC_CONNECTED starts flapping in the field. Either way, move to
a PL011. The tells in `sbus_probe` output: `even-parity=MISSING
2-stop=MISSING` in the config readback, plus either total garbage or
the "HEAVY WIRE-LEVEL FRAME LOSS" verdict.

**Boot config fix** (in `/boot/config.txt`):
1. Add `dtoverlay=disable-bt`
2. Remove `dtoverlay=uart1` (keep `enable_uart=1`)
3. `sudo systemctl disable hciuart`
4. Reboot, then verify:
   - `ls -l /dev/serial0` → `ttyAMA0`
   - `raspi-gpio get 14 15` → `func=TXD0` / `func=RXD0` (ALT0)

## Step 2 — find the receiver's port

```
sbus_probe --scan        # sweeps ttyS0 + all ttyAMA*, 3 s each
```

The port carrying SBUS shows valid frames. If a port shows bytes but
no valid frames, run the full diag there (Step 3). If everything is
silent, the receiver isn't outputting serial at all:

- **Receiver output mode** (most common): RadioLink R9DS/R12DS bind
  fine in PWM mode, where the SBUS pin is silent. Short-press the
  ID SET button twice within 1 s; LED blue = SBUS, red = PWM.
- Wiring: signal pin on the UART RX pin, common ground, receiver power.
- Receiver on a connector whose UART overlay isn't enabled in
  `/boot/config.txt` (Navigator set: uart3→ttyAMA1 GPIO4/5,
  uart4→ttyAMA2 GPIO8/9, uart5→ttyAMA3 GPIO12/13).

## Step 3 — full diagnosis on the live port

```
sbus_probe -d /dev/ttyAMA0
```

Follow the verdict:

| Verdict | Layer | Action |
|---|---|---|
| NO DATA | receiver | Step 2 checklist (mode, wiring, power) |
| DATA BUT NEVER FRAMES UP | line | readback shows parity MISSING → wrong UART (Step 1). Byte mix ≫50% `0x00`/`0xFF` → suspect inverted line; verify polarity below. NOTE: RadioLink SBUS is NON-inverted (unlike Futaba) — normal Pi UARTs read it directly, no inverter. |
| CHANNEL VALUES OUT OF RANGE | decode | verdict lists the violating channels. Undriven channels stuck at 0 → enable them on the transmitter (or relax the all-16-channel range check in sbus_handler.cpp) |
| FOOTER FAILURES | decode | misalignment: wrong baud/parity, or the pin carries a different protocol (i-BUS/PPM) |
| FAILSAFE / FRAME-LOST FLAGS | RF | the receiver itself says the link is bad: re-bind, antennas, distance, 2.4 GHz interference |
| DROPS FRAMES INTERMITTENTLY | RF | same, milder; note one flagged frame resets the 3-good-frame hysteresis, so moderate drop rates still read as disconnected |
| HEALTHY | app | Step 4 |

**Polarity check** (no scope needed — samples the RX pin level; SBUS
idles ~75% of the time, so idle level dominates):

```
for i in $(seq 1 30); do raspi-gpio get 15; sleep 0.1; done \
  | grep -o 'level=.' | sort | uniq -c
```

Mostly `level=1` = normal polarity (expected for RadioLink). Mostly
`level=0` = inverted → hardware inverter needed (Pi UARTs can't
invert RX).

**Transient after reboot:** the first probe run after a boot-config
change can show low-rate garbage (hciuart or similar briefly attached
to the port). Re-run before believing a bad result.

**Capture for offline analysis** (analyze on any machine):

```
sbus_probe --record /tmp/cap.txt -t 10 -d /dev/ttyAMA0
sbus_probe --replay /tmp/cap.txt
```

Burst cadence in the record identifies the source: SBUS is a ~3 ms
burst every ~14 ms (RadioLink ≈ 13.4 ms). ASCII-looking hex = some
other device (GPS/telemetry) is on that UART.

## Step 4 — app layer

```
sbus_probe --lib -d /dev/ttyAMA0    # runs the real SbusHandler class
```

`isControllerConnected: TRUE` here means iRCReader will agree if and
only if it opens the same device. Then:

1. Mission plug: `device = /dev/ttyAMA0` in the iRCReader
   ProcessConfig block (default is /dev/ttyS0 if unset).
2. Launch, open the iRCReader appcast: check the `Device:` line,
   `RC Connected (debounced): YES`, and live channel values.
3. Still failing? Check appcast run warnings for
   "Failed to initialize SBUS handler on <dev>" and re-run
   `scripts/sbus_diag.sh` to see if another process holds the port.
