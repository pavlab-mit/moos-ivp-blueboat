# Companion build card: YIP

Model for boat **YIP**, built against RC Contract v2
([rc_handset_architecture.md](rc_handset_architecture.md)) and the
frozen link config ([rc_characterization.md](rc_characterization.md)).
This is the **golden template** — every field deliberate; the fleet's
generated models derive from it.

Lab policy encoded here: **the vehicle is armed by default.** SW1 is a
KILL, not an arm switch — unlit means running, lit red means an
operator has actively stopped the vehicle. Channel state 1 throughout
this model is the *operator-absent default* (AUTO, running, deadman
enabled), not "off".

Before starting: Companion → Read Models and Settings from Radio, work
on the real radio document, not a blank one. Copy `yip.png` to the SD
card's `/IMAGES/` first so it's selectable.

---

## Physical control map (what your hands learn)

| Control | Meaning | Feedback |
|---|---|---|
| Left stick F/B | thrust (spring-centred, release = stop) | — |
| Right stick L/R | steer | — |
| **SF** (2-pos toggle) | **MODE: up (toward you) = RC, down = AUTO** — "closer to you = you're in control" | screen banner, voice callout, FM telemetry echo |
| **SW1** (lit button) | **KILL: unlit = running, lit red = KILLED** | button LED |
| SB (3-pos) | ACTION: HOLD / RESUME / RETURN | — |
| SW2 (lit button) | DEADMAN: lit amber = deadman DISABLED | button LED |
| SW3 (lit button) | PAYLOAD_PWR | button LED |
| SC (3-pos) | PAYLOAD_MODE | — |
| S1 (pot) | THRUST_LIMIT (floor applied vehicle-side) | — |
| SH (momentary) | MARK — stamp event into MOOS log | beep |

The lit-button rule stays coherent: **a glowing button is an operator
intervention** — vehicle killed, watchdog off, payload energized.

Free for later: SA, SD, SE, SG, S2, LS, RS, SW4–SW6.

---

## Setup tab

- **Model name:** `YIP`. **Model image:** `yip`.
- **Internal Radio System → Protocol: CRSF** (the MK3's internal
  Gemini module). **External module: OFF.** ⚠ MODEL02 had this
  backwards.
- **Receiver No.: 32** — fleet numbering is hull-based: zoe 31,
  **yip 32**, xai 33, … (Model Match allows 0–63). The Model Match
  latch and the image's "RX 32" both reference this number.
- **Arm using: CH5** — with a caveat. ELRS's own convention is
  high = armed, so with KILL on CH5 (high = killed) ELRS's arm flag
  reads inverted, and the Lua script's config lockout engages while
  the boat is *killed* — exactly when you might want to fiddle
  settings. If that annoys in practice, un-kill to configure, or
  check whether the dropdown offers a value that can be pointed at an
  unused source. Cosmetic either way: no FC, no OSD.
- **Throttle Source: THR** (left stick). **Reverse Throttle: OFF** —
  the stick is a bidirectional ±100 axis, not an RC-plane throttle.
- **Custom Throttle Warning: value 0** — with a centre-sprung stick
  the correct at-rest position is centre, so warn if it's *not* at 0
  at power-on (catches a fouled or ratcheted stick). The default
  −100-idle warning would nag on every boot.
- **Center beep: Thr** — audible tick when thrust crosses zero.
- **Switch Warnings:** SF = **down** (AUTO) and SW1 = off (not killed)
  at power-on. Everything else unchecked. Powering on in "RC" or with
  a stale kill left engaged from transport gets flagged before
  anything moves.

## Customizable Switches section (same tab)

| | SW1 | SW2 | SW3 |
|---|---|---|---|
| Name | KIL | DMN | PAY |

(Names are capped at 3 characters.)
| Type | 2POS | 2POS | 2POS |
| Off color | dim/black | dim/black | dim/black |
| On color | **red** | **amber** | white |

## Flight Modes tab

- FM0: name `AUTO` (default — active with SF down)
- FM1: name `RC`, switch `SF↑`

The active name shows on the main screen; it's also what the voice
calls out.

## Inputs tab

Four inputs, weight 100, **no expo, no curves** (linearity is part of
the contract — scaling/deadband live vehicle-side in one place):

`I1 Ail 100` · `I2 Ele 100` · `I3 Thr 100` · `I4 Rud 100`

## Mixes tab

| CH | Source | Weight | Note |
|---|---|---|---|
| 1 | I1 (Ail) | 100 | STEER, + = starboard — verify sign at bench, flip weight here if needed, never in the consumer |
| 2 | I2 (Ele) | 100 | AUX_AXIS_1 |
| 3 | I3 (Thr) | 100 | THRUST, + = ahead |
| 4 | I4 (Rud) | 100 | AUX_AXIS_2 |
| 5 | SW1 | 100 | KILL (172 = running/default, 1810 = killed) |
| 6 | SF | **−100** | MODE — weight is negative so SF↑ (toward you) = 1810 = RC and SF↓ = 172 = AUTO. The wire semantic (172 = AUTO = operator-absent default) is contract-fixed; operator polarity lives here, in the mix, only |
| 7 | SB | 100 | ACTION (3-pos) |
| 8 | SW2 | 100 | DEADMAN_EN (lit/1810 = disabled) |
| 9 | SW3 | 100 | PAYLOAD_PWR |
| 10 | SC | 100 | PAYLOAD_MODE (3-pos) |
| 11 | S1 | 100 | THRUST_LIMIT — full range on the wire; the 25 % floor is applied in iRCInterface so the semantics live in profile.yaml |
| 12 | SH | 100 | MARK (momentary) |
| 13–16 | — | — | leave unmixed; they emit 992 and the decoder ignores reserved channels |

Decoder note (vehicle-side, recorded here so the model and decoder
are read together): discrete channels use guard-band decoding; for
**KILL specifically, a dead-band/invalid value latches the last valid
state** rather than defaulting — a config fault must never silently
kill an autonomous boat, nor un-kill a killed one.

## Outputs tab

Untouched: ±100, no subtrim, no reverse, no curves. All shaping
happens in mixes (handset) or iRCInterface (vehicle) — never here,
so there's exactly one place per side to look.

## Special Functions tab

- SF1: `SF↑` → Play Track "RC mode" (or Beep ×2 + Haptic until voice
  files are on the SD)
- SF2: `SF↓` → Play Track "autonomous" (or Beep ×1)
- SF3: `SH↓` → Beep (confirms the MARK registered)

## Logical Switches + alarms (after first telemetry discovery)

With `rc_telem_test` or the boat running: Telemetry tab → delete all →
Discover new sensors. Expect `RxBt Curr Capa Bat%`, `FM`, `GPS GSpd
Hdg Alt Sats`, plus `RQly RSNR TPWR RFMD` (+ min/max variants). Then:

- L1: `RQly < 50` → SF: Play "signal low" every 10 s. (Bench-measured:
  LQ decays 100→0 in ~1.1 s on link loss — this is the early warning.)
- L2: `RxBt < 14.0` → low battery callout ·
  L3: `RxBt < 13.2` → critical callout
  (**4S**: 3.5 V/cell low, 3.3 V/cell critical — adjust per pack
  chemistry if the fleet mixes types)
- L4: telemetry lost (no `RxBt` update) → "telemetry lost" — this is
  the "RF fine, MOOS dead" detector.

## Custom Screens

1. **OPS** — `Bat%` gauge, `RxBt`, `RQly` bar, `FM` (the boat's
   *confirmed* mode), `GSpd`
2. **LINK** — `RQly`, `RSNR`, `TPWR`, `RFMD`, RSSI min/max
3. **NAV** — GPS map widget (if the build offers it), `Hdg`, `Sats`,
   `Dist` if configured

## Checklist file (SD: /MODELS/YIP.txt)

```
YIP pre-start
[ ] Model = YIP (image + RX 32 match the hull number)
[ ] SF down (AUTO), SW1 dark (running), SW2 dark (deadman ON)
[ ] S1 thrust limit set for today's op
[ ] Telemetry: FM shows boat mode, RxBt sane (4S: ~14.8-16.8)
```

## After the model is written to the radio

1. Bind / confirm link to YIP's XR4, then ELRS Lua → **Model Match →
   On** (latches Receiver No. 32 into that receiver).
2. Wrong-model drill once: select another model, confirm the
   half-connection (link up, no control), switch back.
3. `rc_probe --frames` bench pass: every channel lands 172/992/1810
   as tabled above; verify CH1 steer sign and CH6 polarity.
4. Export the model as the golden template (`_template.yml`) for the
   generator.
