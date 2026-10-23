# iRCInterface MOOS HIL test — testbench navigator

The gate between the bench applet phase (`rc_probe` /
`rc_telem_test`, no MOOS) and any on-water use of `iRCInterface`.
Everything here runs on the **testbench navigator** with the XR4 on
`/dev/ttyAMA1`, the real TX16S, and **no thrusters connected**.

Prereqs: [rc_acceptance_test.md](rc_acceptance_test.md) previously
passed on this rig (handset model + link config known good).
Design contract under test: [rc_crsf_design.md](rc_crsf_design.md).

Setup:

```bash
cd missions/rc_hil_bench
pAntler rc_hil_bench.moos          # MOOSDB + pLogger + iRCInterface
uXMS rc_hil_bench.moos             # second terminal
uMS                                # or scope the appcast via uMAC
```

Every step has an expected result; anything else is a fail — stop
and diagnose before continuing.

---

## A. Startup & port ownership (2 min)

- [ ] `iRCInterface` appcast: device `(open)`, frames counting up,
      **CRC failures 0**, uplink LQ ~100.
- [ ] `RC_PROTOCOL = "crsf"` appears once in uXMS / the alog.
- [ ] With the app running, `rc_probe -d /dev/ttyAMA1` **refuses**
      the port (port-in-use check) — proves single ownership.
- [ ] Kill and relaunch `iRCInterface` with the receiver briefly
      unpowered → appcast shows `(NOT OPEN, retrying)` or 0 frames,
      then recovers without a restart when the receiver returns.

## B. Channel contract through MOOS (10 min)

The MOOS-side mirror of acceptance step C — same physical inputs,
but now checking the *scaled* `RC_CH*` values in uXMS:

Everything at rest first:

- [ ] `RC_CH1–4` ≈ **0** (sticks centred; CH3 spring-centre),
      `RC_CH5 = 1` (KIL dark), `RC_CH6 = 1` (SF down / AUTO),
      `RC_CH11` = pot raw ticks.

Then each control:

| Test | Variable | Expect |
|---|---|---|
| Right stick full right / left | RC_CH1 | ~+100 / ~-100 (**+ = starboard**) |
| Right stick full fwd / back | RC_CH2 | ~+100 / ~-100 |
| Left stick full fwd / back | RC_CH3 | ~+100 / ~-100 (**+ = ahead**) |
| Left stick released | RC_CH3 | snaps to ~0 |
| KIL press / release | RC_CH5 | 3 / 1 (3-state map of 1810/172) |
| SF up / down | RC_CH6 | **2 up (RC) / 1 down (AUTO)** |
| SB three positions | RC_CH7 | 1 / 1 / 2 (2-state map: mid lands in state 1) |
| SW2 lit / dark | RC_CH8 | 3 / 1 |
| SW3 lit / dark | RC_CH9 | 2 / 1 |
| S1 full sweep | RC_CH11 | smooth 172 → 1810 raw |

- [ ] Scaled values match what `iRCReader` produced for the same
      stick positions in old alogs (spot-check one or two — this is
      the byte-identical-contract check).

## C. Connection model & failsafe (10 min)

- [ ] TX off mid-run → within ~0.5 s: `RC_CONNECTED = "false"`,
      `RC_FRAME_VALID = "false"`, and the **safe-default block**
      appears: CH1–4 = 0, CH5–9 = 1, CH10–16 = 992.
- [ ] `RC_LINK_LQ` decays 100 → 0 over ~1.1 s, then stops updating
      (stats go silent ~3.5 s; the stale-gate stops the publish).
- [ ] TX on → `RC_CONNECTED` returns after the 3-good-frame
      hysteresis (< 1 s); channels resume live values.
- [ ] In the alog (`aloggrep RC_CONNECTED RC_CH1`): on the
      disconnect iterate, `RC_CONNECTED=false` is logged **before**
      the safe-default `RC_CH*` values — the publish-order guarantee
      the `RC_DEADMAN` watchdog depends on.
- [ ] Model-match half-connection (select wrong model on the TX):
      link LEDs up but no frames → behaves exactly like TX off at
      the MOOS level.

## D. Telemetry from live MOOS variables (10 min)

Now the real sources, poked by hand:

```bash
uPokeDB rc_hil_bench.moos NVGR_ROLLING_VOLTAGE=24.6 NVGR_ROLLING_CURRENT=3.2
uPokeDB rc_hil_bench.moos NAV_LAT=42.3585 NAV_LONG=-71.0870 NAV_SPEED=1.5 NAV_HEADING=90
uPokeDB rc_hil_bench.moos NAV_X=30 NAV_Y=40
uPokeDB rc_hil_bench.moos NVGR_RC_MODE=true
```

- [ ] `RxBt` = 24.6, `Curr` = 3.2 on the handset within ~1 s.
- [ ] `GPS`/`GSpd`/`Hdg` live; `GSpd` = **1.5** (unit conversion).
- [ ] `Dst` = **50** (hypot 30,40 — proves the origin-distance path).
- [ ] `FM` flips `AUTO` → `RC` on the `NVGR_RC_MODE` poke; poke
      `NVGR_RC_DEADMAN_ACTIVE=true` → `FM` = `DEADMAN` (priority
      over RC); clear both → `AUTO`.
- [ ] **Staleness gate**: stop poking for ~10 s → handset sensors go
      stale/red (inputs older than 5 s are not forwarded); the
      "telemetry lost" callout fires. Resume poking → recovers.
- [ ] Appcast: telemetry `sent` counting, **dropped 0** throughout.
- [ ] `TQly` ~100 the whole session (downlink slot health).

## E. Load & endurance (15 min, passive)

- [ ] Leave everything running 15 min with sticks occasionally
      moved: CRC failures stay 0, no write drops, no MOOS run
      warnings, memory/CPU of `iRCInterface` flat (`top`).
- [ ] Ctrl-C `pAntler`, check the alog closed cleanly and covers
      the whole session.

## F. Phase 2 — full-stack loop (optional on this rig)

If the testbench navigator can run `iBBNavigatorInterface`
(**ESCs unpowered / props removed — non-negotiable**), add it to
the Antler block and re-run C and D:

- [ ] `NVGR_RC_MODE` / `NVGR_RC_DEADMAN_ACTIVE` now feed `FM`
      automatically — SF up shows `FM=RC` with no pokes.
- [ ] TX off in RC mode → `NVGR_RC_DEADMAN_ACTIVE=true` within the
      watchdog window → `FM=DEADMAN` on the handset. This is the
      confirmed-mode loop end-to-end.
- [ ] TX on → deadman clears per its designed recovery rule.

Otherwise these two checks move to the pre-water dockside test.

---

## Close out

- [ ] Archive the alog + this checklist result under `assets/rc/`.
- [ ] Update [rc_crsf_design.md](rc_crsf_design.md) status
      (HIL passed → field control-parity test is next).
- [ ] Only then: wire `plug_iRCInterface.moos` into
      `meta_vehicle.moos` in place of `plug_iRCReader.moos` for the
      water test.
