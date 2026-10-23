# RC handset acceptance test — YIP (template for fleet)

Run top to bottom after any model change, firmware update, or new
handset provisioning. Every step has an expected result; anything else
is a fail — stop and diagnose before continuing. Bench rig: XR4 on
the Pi (`ttyAMA1`), boat MOOS apps **stopped**.

Contract: [rc_handset_architecture.md](rc_handset_architecture.md) ·
Measurements: [rc_characterization.md](rc_characterization.md) ·
Build: [rc_model_build_yip.md](rc_model_build_yip.md)

---

## A. Link configuration audit (radio only, 2 min)

In the ELRS Lua (`SYS → Tools → ExpressLRS`), confirm against declared
state — these are the settings that silently drift:

| Setting | Expected |
|---|---|
| TX/RX firmware | 4.1.0 both (major versions must match) |
| RF band | 915 |
| Packet rate | 100 Hz Full |
| Switch mode | 16ch Rate/2 |
| Telem Ratio | **1:8** (drift here caused the 2026-08-11 lag) |
| Model Match | On |
| TX power | bench: 10–25 mW · ops: per mission |

- [ ] All match. Any mismatch: fix in Lua **while linked**, back out
      of the menu so it commits, re-check.

## B. Power-on behavior (radio only, 2 min)

- [ ] Power on with SF **up** (RC) → switch warning blocks startup.
- [ ] Power on with KIL **lit** → switch warning blocks startup.
- [ ] Power on clean (SF down, KIL dark) → no warnings; main screen
      shows model **YIP**, image, flight mode **AUTO**.
- [ ] Hold throttle off-centre during power-on → throttle warning.
- [ ] Press KIL → button glows **red**; press again → dark.
      SW2 → **amber**; SW3 → white.
- [ ] KIL start behavior: kill lit, power-cycle radio → KIL **re-lights**
      (Restore); DMN and PAY come up dark.

## C. Channel contract (`rc_probe -p crsf --frames -d /dev/ttyAMA1`, 10 min)

Everything at rest first:

- [ ] CH1–4 = **992**, CH5 = **172** (running), CH6 = **172** (AUTO),
      CH7–10 = 172, CH11 = pot position, CH12 = 172,
      **CH13–16 = 992** (unmixed).

Then each control, watching the one channel:

| Test | Channel | Expect |
|---|---|---|
| Right stick full right / left | CH1 | 1810 / 172 (**+ = starboard**: confirm right = 1810) |
| Right stick full fwd / back | CH2 | 1810 / 172 |
| Left stick full fwd / back | CH3 | 1810 / 172 (**+ = ahead**: fwd = 1810) |
| Left stick full right / left | CH4 | 1810 / 172 |
| Left stick released | CH3 | snaps to 992 (spring centre) |
| KIL press / release | CH5 | 1810 lit / 172 dark |
| SF up / down | CH6 | **1810 up (RC) / 172 down (AUTO)** — the polarity check |
| SB three positions | CH7 | 172 / 992 / 1810 |
| SW2 lit / dark | CH8 | 1810 / 172 |
| SW3 lit / dark | CH9 | 1810 / 172 |
| SC three positions | CH10 | 172 / 992 / 1810 |
| S1 full sweep | CH11 | smooth 172 → 1810, no steps |
| SH held / released | CH12 | 1810 / 172, returns on release |

- [ ] Verdict run (`rc_probe -d /dev/ttyAMA1`): ~87 Hz 0x16,
      **0 CRC failures**, 22-byte payloads.

## D. Mode & callout behavior (radio + Pi, 5 min)

With `rc_telem_test -m AUTO -t 0` running:

- [ ] SF up → voice "R C mode", haptic, screen banner → RC.
- [ ] SF down → voice "auto mode", banner → AUTO.
- [ ] `FM` sensor shows the string the *vehicle* sends (AUTO), not the
      switch position — restart tool with `-m RC` and watch FM change.
      This is the confirmed-mode loop: banner = commanded, FM = actual.
- [ ] SH press → "mark" callout.
- [ ] KIL / SW2 callouts if configured.

## E. Telemetry & sensors (`rc_telem_test --gps --dist -r 1 -t 0`, 10 min)

- [ ] All sensors updating, none stale/red: `RxBt Curr Capa Bat%`,
      `FM`, `GPS GSpd Hdg Sats`, `Dst`, `RQly RSNR TPWR RFMD`, `TQly`.
- [ ] **TQly ~100** on the handset — the canary that downlink slots
      are scheduled. TQly missing/0 = Telem Ratio drifted → step A.
      (Pi-side downlink LQ in `0x14` is always 0 — ELRS never
      populates it outside debug builds. Not an indicator; ignore.)
- [ ] `GSpd` reads **1.5** (m/s — unit conversion proof; 5.4 = wrong).
- [ ] `Dst` sweeps 0 → ~120 → 0 m over ~4 min, tracking the lap.
- [ ] Lag spot-check: values move within ~1 s of tool start; no
      growing delay over 2+ min.
- [ ] Ctrl-C the tool → sensors go stale/red; within ~15 s the
      **"telemetry lost"** callout fires (!L4 path). Restart tool →
      recovers, callout stops.

## F. Alarm thresholds (Pi, 5 min)

- [ ] `rc_telem_test -v 13.5 -t 20` → **"battery low"** repeats.
- [ ] `rc_telem_test -v 13.0 -t 20` → **"battery critical"**.
- [ ] `rc_telem_test -v 15.0 -t 20` → silence.
- [ ] Signal-low path: temporarily set L1 to `RQly < 101` → "signal
      low" fires → **set back to 50**. (Bench LQ is pinned at 100, so
      the pathway is tested, not the threshold.)

## G. Link loss & Model Match (10 min)

- [ ] TX off mid-run → Pi side: 0x16 stops instantly, LQ decays
      100→0 over ~1.1 s, serial silent after ~3.5 s. Handset:
      telemetry-lost behavior. TX on → control resumes < 1 s.
- [ ] Model Match latched (Receiver No. **32**). Select another model
      (or bump receiver number): link LEDs up but **no 0x16 frames**
      reach the Pi — the half-connection. Restore → frames return.
      This is the wrong-boat signature; it looks like a dead wire.

## H. Close out

- [ ] Radio → Read Models and Settings into Companion; save.
- [ ] Export YIP as the **golden template** `.etx` (screens, sensors,
      Lua reference, units all captured).
- [ ] Archive any captures + this checklist result in the repo.
- [ ] Record deviations in the docs before they become folklore.
