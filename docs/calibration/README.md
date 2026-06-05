# BlueBoat Navigator Sensor Calibration

Calibration pipeline for the Navigator's gyroscope and magnetometer.

## Prerequisites

- `sample_navigator` applet 
- Python 3 with numpy, scipy, pandas, matplotlib

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r scripts/requirements.txt
```

## Calibration Order

1. Gyroscope bias (stationary)
2. Magnetometer hard/soft iron (in-motion)

Gyro must be calibrated first. The Madgwick filter used during magnetometer data collection depends on accurate gyro data for roll/pitch estimation.

## Step 1: Gyroscope Bias Calibration

Place the vehicle on a flat, stable surface. Ensure it will not be disturbed for 30 seconds.

```bash
sample_navigator --duration 30 -o gyro_capture.csv
python3 scripts/get_bb_nav_gyro_cal.py gyro_capture.csv
cp gyro_bias.dat ~/system_data/$(hostname)/nav/
```

The script validates stationarity and reports warnings if the vehicle moved during capture.

## Step 2: Magnetometer Calibration

With the gyro bias file in place, sample while rotating the vehicle through as many orientations as possible. Roll, pitch, and yaw all matter — try to cover the full sphere.

```bash
sample_navigator --duration 180 -o mag_capture.csv
python3 scripts/get_bb_nav_mag_cal.py mag_capture.csv
cp mag_cal_nav.dat ~/system_data/$(hostname)/nav/
```

The script produces `mag_cal_nav.dat` and a diagnostic plot `mag_cal_nav_diagnostics.png` showing raw vs calibrated sphere and error distribution.

## Calibration File Locations

All calibration files live in `~/system_data/{hostname}/nav/`:

| File | Description |
|------|-------------|
| `gyro_bias.dat` | Gyroscope bias offsets (rad/s) |
| `mag_cal_nav.dat` | Magnetometer hard/soft iron parameters |

`sample_navigator` automatically loads these files on startup if they exist. Use `--gyro-cal <path>` and `--mag-cal <path>` to override.

## Verification

After calibration, run a quick sample and check the terminal output:

```bash
sample_navigator --duration 10
```

Roll and pitch should be stable and accurate. Magnetometer magnitude (|B|) should be close to the expected EMFI for your location (~51.3 uT in Boston).

## Madgwick Filter Tuning

`sample_navigator` uses the Madgwick AHRS filter in IMU-only mode (no magnetometer in the fusion loop). Tuning parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--beta` | 0.1 | Filter gain. Lower values trust the gyro more. Range: 0.01-1.0 |
| `--tau` | 0.075 | LPF smoothing time constant for attitude output |
| `--rate` | 150 | Sample rate in Hz |

## Using MCC

If using MCC, pass the following arguments:

```bash
sample_navigator --mcc --mcc-addr <laptop_ip>
```
