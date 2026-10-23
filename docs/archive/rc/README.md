# Archived RC working documents

Superseded 2026-08-14 by [`docs/rc_controllers.md`](../../rc_controllers.md),
which is the single source of truth for the RC system. These are kept as
the evidence chain — bench measurements, protocol comparison, decision
history — and are **out of date wherever they disagree** with that file.

Known drift already corrected in the consolidated doc:

- `rc_hil_test.md` expects `RC_CH5` and `RC_CH8` to decode 3/1 (an
  SBUS-era 3-state map). The implemented v2 decode publishes **1/2**.
- `rc_handset_architecture.md` specifies 192×114 model images; the fleet
  standard moved to **720×800** on 2026-08-12.
- `rc_crsf_bench_setup.md` treats the `0x21` flight-mode string and
  Pi-side downlink LQ as open questions. Both were resolved: FM works on
  ELRS 4.1.0, and Pi-side downlink LQ is structurally always 0.
- `rc_crsf_bench_setup.md` claims ELRS receiver serial pins are channel
  pads reassigned in the WebUI Model tab — not true of the XR4, whose
  serial pads are dedicated.
- Several docs describe a `profile.yaml` → generated-models pipeline
  that was never built.

| File | What it still has that's worth reading |
|---|---|
| `rc_characterization.md` | The raw bench measurements — channel census, failsafe timing, telemetry results |
| `rc_crsf_evaluation.md` | SBUS vs CRSF protocol comparison and the case for the overhaul |
| `rc_crsf_design.md` | Architecture rationale, the `zoe` failsafe investigation |
| `rc_crsf_test_plan.md` | Library-first test strategy and the unit-test case list |
| `rc_handset_architecture.md` | Contract v2 derivation — why each channel is where it is |
| `rc_model_build_yip.md` | Field-by-field Companion build card |
| `rc_acceptance_test.md` | Full handset acceptance checklist |
| `rc_hil_test.md` | Full MOOS HIL checklist |
| `rc_crsf_bench_setup.md` | Receiver wiring/pad assignment detail |
| `rc_troubleshooting.md` | **Still current for SBUS boats on `iRCReader`** — mini UART parity, `disable-bt`, polarity checks |
