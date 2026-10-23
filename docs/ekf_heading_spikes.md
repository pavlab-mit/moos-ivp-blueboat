# NAV_HEADING Spikes on BlueBoats: Diagnosis and Proposed Fixes

*2026-07-24 — investigation of operator-reported ~100° single-iteration heading jumps on `via`.*

## Summary

`pBB_DGPS_EKF` fuses dual-antenna GNSS (`FIX_STATE_DGNSS` from iUnicore) with a
level-frame gyro (`GYRO_Z_LVL_IMU` from the navigator interface). Log analysis
of the 7/22 calibration run and the 7/23 `pavilion_encircle` missions confirmed
**two independent failure modes**, both on `via` (yip was clean), both reaching
the helm because the filter has no plausibility checks on either its
measurement or its prediction inputs.

## Failure mode A: confident-but-wrong DGPS heading at RTK reacquisition

*Evidence: 7/22 log (`LOG_VIA_22_7_2026_____14_05_20.alog`), five events.*

When the receiver's heading solution recovers from an outage, its first "fixed"
epoch can carry a wrong integer-ambiguity solution: `HDG_ACC` collapses from
garbage (100s of degrees) or `HDG_VALID=false` to **sub-degree claimed
accuracy in a single 0.1 s epoch**, while the reported heading simultaneously
swings 25–165°. The EKF's acceptance rule (`HDG_VALID && HDG_ACC < 10°`) admits
it immediately; with measurement noise taken verbatim from `HDG_ACC`, the
Kalman gain is 0.6–0.85 and `NAV_HEADING` moves up to ±96° in one update.
Consequences observed in-log: rudder slams (−8.6→+91, +100→−100) and a
`FULL_STOPPING` 0.09 s after one event. Standard quality fields (FIX, NSATS,
HDOP, H_ACC) stay nominal throughout — they do not validate the heading.

## Failure mode B: gyro yaw-rate blowup at the cos(pitch) singularity

*Evidence: 7/23 via logs (16/16 heading jumps correlated) and 7/22 log.*

`GYRO_Z_LVL_IMU` is not the raw gyro: the navigator interface publishes the
Euler yaw rate `(q·sinφ + r·cosφ)/cos(θ)`. The Madgwick AHRS runs IMU-only
(mag is stubbed off due to a navigator-rs `DataNotReady` panic) and during
dynamic disturbances its attitude estimate transiently reports pitch near ±90°
and roll flipping through 180° — non-physical for a surface boat. The
projection then amplifies modest body rates into spikes of ±18 to ±120 rad/s
(the `cos θ < 1e-6` "guard" multiplies by 10⁶ instead of clamping). The EKF
integrates these blindly, producing 50–60° heading jumps per iterate. GPS
heading was steady and valid during every such event.

## Aggravating defects

- **Measurement re-application:** the EKF applies the latest GNSS message on
  every 16 Hz iterate until it is 5 s stale — no new-timestamp check. One bad
  epoch is ingested ~2–3×, dragging the state most of the way to the bogus
  value and artificially collapsing covariance (the filter then *shrinks*
  `EKF_STD_HEADING` after swallowing a bad heading).
- **Dead config:** `sigma_gps_pos` and `sigma_gps_hdg` are parsed but never
  used; position R comes from `H_ACC`, heading R from `HDG_ACC` directly.
- **Gyro bias cal disabled:** `imu_cal_file` is commented out in the frontseat
  plug, and the calibration script's output format (`bias_x/y/z`) doesn't match
  what `readImuCalFile()` parses (`gyro_bias = x,y,z`). (Causes drift, not
  spikes.)
- The frontseat→backseat UDP link has no checksum, but the frame parser fails
  closed — dropped/spliced frames are discarded, not mis-parsed.

## Proposed fixes

### pBB_DGPS_EKF (priority 1 — one file, guards against both modes)

1. **Innovation gate** on the heading update: reject > 3–4σ innovations, with
   an escape hatch (after ~2 s of consecutive rejections, accept and re-inflate
   heading covariance). Would have blocked every mode-A event (innovations were
   72–163° against ~1° claimed sigma).
2. **Reacquisition hysteresis:** after `HDG_VALID=false` or high `HDG_ACC`,
   require ~1–2 s of consecutive good epochs (with inflated R) before trusting
   sub-degree claimed accuracy.
3. **Consume each GNSS message once** (track `TS`) and raise the heading noise
   floor from 0.57° to ~2–3°.
   > *Update 2026-08-31: the consume-once half landed with the lever-arm /
   > signed-surge work (`BB_DGPS_EKF_Model::updateGPS` is TS-gated;
   > `timestamp == 0` keeps the old always-fuse behavior for TS-less
   > sources). The noise-floor raise is still open.*
4. **Gyro plausibility clamp** in the predict step (reject/clamp |ω| > ~2 rad/s).

### iBBNavigatorInterface (priority 2 — kills mode B at the source)

5. Use body-frame `r` directly when |roll| or |pitch| > ~30° (the projection
   only diverges from `r` at attitudes where the AHRS is untrustworthy anyway);
   clamp the published rate to a physical envelope. Root-cause work: restore the
   magnetometer and characterize why via's AHRS destabilizes (raw IMU logging,
   v1/v2 config audit).

### iUnicore (priority 3 — kills mode A at the source)

6. Forward carrier-solution type and baseline length in `FIX_STATE_DGNSS`;
   gate `HDG_VALID` on `NARROW_INT` and on baseline length matching the
   physical antenna separation, so wrong-ambiguity fixes never arrive flagged
   valid.

Housekeeping: wire up or delete the dead sigma params; fix the cal-file format
mismatch and re-enable `imu_cal_file` in the mission plug.
