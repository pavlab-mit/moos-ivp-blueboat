# CRSF bench setup: pre-flight checklist

Status: **pre-bench prep.** Work through this before wiring the XR4 to
the rig. Design: [rc_crsf_design.md](rc_crsf_design.md) ·
Tool: `rc_probe`.

Assumes a bound TX16S MK3 ↔ XR4 pair. Everything below is what to do
*after* binding and *before* the receiver reaches the Pi.

---

## 1. Set the receiver output protocol to CRSF

The single change that makes this whole effort possible. Two routes:

**ELRS Lua on the handset** (faster, no WiFi):
`SYS` → `Tools` → `ExpressLRS` → scroll to the receiver section.

**Receiver WebUI** (more visibility, needed for pin remapping):
Power the RX and let it enter WiFi mode, connect to its AP, then the
**Model tab** → output protocol → `CRSF`.

Reported defaults conflict across sources — some say the XR4 ships
CRSF, and your boats were deliberately set to SBUS anyway. **Verify
rather than assume**; that is the first thing `rc_probe --scan` will
tell you.

## 2. Identify which pads are Serial TX and Serial RX

This is the step most likely to cost you bench time, because on ELRS
PWM receivers the serial pins are **channel output pads reassigned to
serial duty**, not dedicated ports. On many receivers that is CH2/CH3;
the XR4's assignment is set in the WebUI **Model tab**, where each pin
gets a role.

Two specific traps:

- **Search results conflate the receiver with flight-controller
  UARTs.** Guidance about "UART1/UART2 at 115200, Betaflight Ports
  tab" describes an FC, not the XR4's own pads. Ignore it — 115200 is
  not CRSF's 420000, and there is no Betaflight here.
- **Serial RX (the pad that receives) is the one you need for
  telemetry.** Uplink-only wiring leaves the receiver unable to hear
  the Pi, which is precisely the Phase B path.

Open `hardware.html` in the receiver WebUI if you need to remap serial
to specific pins.

## 3. Confirm the TX direction exists on both ends

Telemetry is the reason for the overhaul, and it is the half most
likely to be silently missing.

**Receiver side:** a pad assigned to **Serial RX** in the Model tab.

**Pi side:** `ttyAMA1` is Navigator SERIAL 3 on GPIO 4/5. Under
`dtoverlay=uart3`, GPIO 4 = TXD3 and GPIO 5 = RXD3. Verify on the boat:

```bash
raspi-gpio get 4
raspi-gpio get 5
```

Expect `func=TXD3` / `func=RXD3`. If GPIO 4 shows as an input or plain
GPIO, the overlay is giving you RX only and that must be fixed before
any telemetry work.

**Harness:** the existing cable is very likely RX-only, since nothing
has needed the return path until now. Check before assuming; add the
Pi-TX → RX-Serial-RX wire if absent.

Cross-connected, as always: Pi TX (GPIO 4) → receiver Serial RX, Pi RX
(GPIO 5) → receiver Serial TX, plus common ground.

## 4. Pick packet rate and telemetry ratio

Set in the ELRS Lua script. Defaults are tuned for racing drones and
are wrong for an ASV.

**Packet rate — favour range over rate.** Higher rates degrade receiver
sensitivity badly (1000 Hz FSK ≈ −101 dBm vs 25 Hz ≈ −123 dBm; that is
a large real-world range difference). A boat does not need 500 Hz
control. **50–150 Hz is the sensible band**; 100 Hz is a good starting
point.

**Telemetry ratio — `Std` is fine to start.** Ratio trades downlink
bandwidth against uplink packets. Battery voltage plus a mode string at
1–2 Hz is a tiny load, so there is no reason to starve the control link
for it. Only revisit if telemetry demonstrably lags.

**TX power:** keep it low on the bench (10–25 mW). Full power into an
antenna a foot away can desensitise the receiver and produce
*worse*-looking link stats than a real deployment — a genuinely
confusing failure mode.

## 5. Configure failsafe deliberately

Directly relevant: `zoe` has been flying in sustained ELRS failsafe
with a healthy frame rate
([rc_crsf_design.md](rc_crsf_design.md#findings-that-change-the-design)).
While you are in the menus, set the failsafe behaviour explicitly
rather than inheriting a default, and note what you chose. For a boat,
"no pulses" (receiver stops emitting) is usually more honest than "last
position" — it makes link loss unambiguous downstream instead of
looking like a valid stationary command.

Worth reproducing on the bench on purpose: power the TX off mid-capture
and confirm `rc_probe` reports FAILSAFE with uplink LQ 0. That both
validates the tool and characterises the `zoe` symptom in a controlled
setting.

## 6. Note the channel map before changing anything

The MOOS contract depends on the existing switch assignments — CH5–CH9
map to specific switches, and **CH6 is the RC-mode switch** that
`iBBNavigatorInterface` gates on. The current appcast layout:

| CH | Function |
|---|---|
| 1–4 | sticks (R L/R, R F/B, L F/B, L L/R) |
| 5 | 3-pos switch |
| 6 | **2-pos, RC mode** |
| 7 | 2-pos |
| 8 | 3-pos |
| 9 | 2-pos |

Keep the model's channel order identical when switching to CRSF. If it
changes, the boat will still arm and drive — just on the wrong switch,
which is the kind of bug that is obvious on the water and invisible at
the bench.

Under CRSF, CH13/15/16 stop reading 0 and CH14 stops carrying link
stats (that was an SBUS-mode workaround), so `validated_channels`
becomes irrelevant.

## 7. Firmware version

Note the ELRS version on both TX and RX — they should match on the
major version or the link may refuse to bind. Also relevant to us:
**ELRS 4.0+ can send a 23-byte RC payload** with an appended arm-status
byte instead of the usual 22. `lib_crsf` handles both, but if you see
unexpected payload-length behaviour, firmware version is the first
thing to check.

---

## Bench sequence once wired

```bash
rc_probe --scan                  # which port, which protocol
rc_probe -d /dev/ttyAMA1         # auto-detect + full verdict
rc_probe -p crsf --frames -d /dev/ttyAMA1    # live stick values
rc_probe -p crsf --link  -d /dev/ttyAMA1     # live LQ / RSSI / SNR
```

Run with the MOOS RC app **stopped** — two readers on one tty split the
byte stream and both see corruption. `rc_probe` refuses to run if the
port is held.

Expected on a healthy bench link: ~100 Hz RC frames, **0 CRC
failures**, uplink LQ 100%, frame types `0x16` and `0x14` present.

Anything unexpected — record it and it can be analysed off-vehicle:

```bash
rc_probe -p crsf --record /tmp/cap.txt -t 10 -d /dev/ttyAMA1
rc_probe --replay /tmp/cap.txt -p crsf
```

## Telemetry downlink test

`rc_probe` on the bench showed **uplink LQ 100%, downlink LQ 0%** —
which at the time looked like the receiver reporting no return path
(it wasn't; see below). `rc_telem_test` sends real telemetry so the
return path can be judged:

```bash
./bin/rc_telem_test -d /dev/ttyAMA1 --sweep
```

It transmits battery (`0x08`) and flight-mode (`0x21`) frames at 2 Hz
while reading the receiver's link stats, printing a 1 Hz status line.
`--sweep` ramps the reported voltage 20→30 V so the handset value is
visibly live rather than a stale first reading.

**Success is measured on the handset, not by downlink LQ.**

> **CONFIRMED WORKING on the bench (2026-08-10).** `RxBt`, `Curr`,
> `Capa` and `Bat%` appear on the TX16S telemetry page while the tool
> runs and go stale when it stops. Those are exactly the four fields of
> our `0x08` battery frame, so the full path — Pi → GPIO 4 → receiver
> Serial RX → RF downlink → handset — works.
>
> **Downlink LQ read 0% the entire time.** That is not a fault. See
> below.

### Downlink LQ 0% does NOT mean telemetry failed

Pi-side downlink LQ is **always 0 on ELRS**, at every telemetry
ratio: receiver firmware writes the `0x14` downlink fields only under
the `DEBUG_BF_LINK_STATS` build flag (`rx_main.cpp`). The RX cannot
measure telemetry reception — the TX module does, and reports it
directly to the handset as the `TQly`/`TRSS`/`TSNR` sensors.

This tripped us twice before the source settled it: the first version
of `rc_telem_test` reported "telemetry is not reaching the RX" and
sent us to check wiring that was already fine, and a later theory
blamed the telemetry ratio (no ratio setting changes it — 1:8 with
live sensors still reads 0). The tool no longer reports downlink LQ
at all; judge the return path on the handset — sensors live plus
`TQly` ~100.

### Viewing telemetry on the TX16S

`MDL` → `PAGE` to **TELEMETRY** → scroll to **"Discover new sensors"**,
with `rc_telem_test` running. Sensors populate live.

Expect `RxBt` (voltage), `Curr` (current), `Capa` (capacity), `Bat%`
(remaining) from our battery frame, plus receiver-generated `RQly`,
`RSNR`, `TPWR`, `RFMD`. If sensors do not appear, delete all and
re-discover — stale entries block new ones.

**Open question: the flight-mode string (`0x21`) was not observed as a
sensor.** The four battery fields appeared; `FM` did not. Either EdgeTX
does not auto-discover it the same way, or ELRS is not forwarding it.
Worth resolving before relying on a mode string for operator feedback —
the design doc treats `0x21` as the cheap way to surface MOOS mode, and
that assumption is now unverified.

**If the handset shows NO sensors at all**, then it is a wiring or
pin-assignment fault: GPIO 4 = TXD3, receiver pad assigned to Serial RX
in the WebUI Model tab, common ground.

Useful variants:

```bash
./bin/rc_telem_test -v 12.4 -a 1.5 -m RC     # fixed values
./bin/rc_telem_test --no-mode                # battery only
./bin/rc_telem_test -r 5 -t 0                # 5 Hz until Ctrl-C
```

### One hazard worth knowing about

The serial port must have **output post-processing disabled**
(`OPOST`/`ONLCR` cleared). With it on, the tty driver rewrites any
`0x0A` byte to `0x0D 0x0A`. The battery frame's length byte is `0x0A`,
so **every battery frame would be silently corrupted while flight-mode
frames passed** — per-frame-type corruption with no error anywhere.

Both tools clear these flags explicitly. This was hit for real in a
loopback harness that omitted it, which is exactly how it would have
presented on hardware: "telemetry works but battery never shows up."

## The comparison worth doing while both modes are available

Before committing the fleet to CRSF, capture the **same physical stick
positions** in both SBUS and CRSF mode and confirm the channel values
match. That validates the new decode path against physical reality
rather than against `lib_sbus` (which is untested code being retired).
It is only easy to do while the receiver can still be flipped between
modes — so do it at the bench, not later.
