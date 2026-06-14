# BlueBoat boot / auto-launch system

`bb_init.sh` runs once at boot and decides whether the boat should auto-launch
its front-seat mission or sit idle. It is the BlueBoat replacement for the old
SeaScout boot script and is built for the Navigator V2 (BMP390) migration.

## Flow

```
bb_init.sh
  1. (optional) git pull + rebuild if the repo changed
  2. battery gate   bb_adc        -> stand by if pack voltage < BOAT_VOLT_MIN
  3. pitch gate     bb_attitude   -> stand by if |pitch| >= BOAT_PITCH_LIMIT
  4. launch         systemctl start $BOAT_MISSION_SERVICE   (start-once)
```

"Stand by" is a normal outcome (boat on a cart, being carried, low battery),
not an error — the script signals idle and exits 0.

## Status signaling (no screen needed)

| Outcome        | PWM0 status LED (GPIO18) | NeoPixel strip (if enabled) | Status file `state=` |
|----------------|-------------------------|-----------------------------|----------------------|
| Mission launched | solid on              | nav lights (green/red)      | `LAUNCHED`           |
| Idle: bad pitch  | blinks `BOAT_IDLE_FLASH_SECS` | rainbow              | `IDLE_PITCH`         |
| Idle: low batt   | blinks                 | rainbow                     | `IDLE_LOWBATT`       |
| Error            | blinks                 | rainbow                     | `ERROR`              |

- LED driver: `scripts/bb_led.sh` — drives the Navigator PWM0 header, which is
  wired to **Pi GPIO18** (not the PCA9685/user LEDs), via `pinctrl`. No build
  needed, so the LED works even if the C++ apps fail to compile.
- Strip driver: `bb_neopixel` — off by default (`BOAT_NEOPIXEL=false`) since no
  boats currently have a strip; flip it to `true` when one is installed.
- Status file: `/run/bb_boot/status` (`cat` it over SSH). Full log:
  `/var/log/bb_boot/boot.log`.

## Helper binaries (built from `src/02_applets`)

| Helper        | Purpose                                   | Kind |
|---------------|-------------------------------------------|------|
| `bb_attitude` | IMU → roll/pitch (pitch gate)             | C++, navigator-lib (NAVOS v1/v2 gated) |
| `bb_adc`      | Navigator ADC → pack voltage (batt gate)  | C++, navigator-lib (NAVOS v1/v2 gated) |
| `bb_neopixel` | LED-strip status (future hardware)        | C++, navigator-lib (NAVOS v1/v2 gated) |
| `bb_led.sh`   | PWM0 status LED on/off/flash              | shell, Pi GPIO18 via pinctrl (no build) |

The three C++ helpers select the navigator-lib API at compile time from
`NAVOS_VERSION` / `RASPBERRY_PI_VERSION` (see top-level `CMakeLists.txt`). On a V2
board they call `set_navigator_version(NavigatorVersion::Version2)` before
`init()`; without it `init()` fails on BMP390 hardware. `bb_led.sh` is pure Pi
GPIO and uses neither.

## Install

```bash
sudo cp scripts/systemd/bb-init.service      /etc/systemd/system/
sudo cp scripts/systemd/fs-mission.service   /etc/systemd/system/  # if not yours
sudo systemctl daemon-reload
sudo systemctl enable bb-init.service        # do NOT enable fs-mission
```

## Configuration

Every `BOAT_*` tunable has a built-in default in the script, so it runs
standalone. Per-boat overrides go in **`Environment=` lines in
`bb-init.service`** (uncomment the ones you need) — no separate config file. The
one you'll almost always set is **`BOAT_VOLT_MIN`** (calibrate for your pack).

The build-time vars `NAVOS_VERSION` / `RASPBERRY_PI_VERSION` / `NAV_PLATFORM_TYPE`
are **not** here — they live in the pi login profile (where you set them to build
the repo), and the boot-time rebuild inherits them via `sudo -u pi -H bash -lc`.

## Testing without going through a reboot

```bash
# Bench test the decision logic; touches no hardware or systemd:
scripts/bb_init.sh --dry-run --no-lock --no-pull --debug

# Force a specific gate result:
scripts/bb_init.sh --dry-run --fake-pitch 35 --no-pull --debug      # -> IDLE_PITCH
scripts/bb_init.sh --dry-run --fake-voltage 11.5 --no-pull --debug  # -> IDLE_LOWBATT

# Exercise the real sensors/LEDs but skip pull/build:
sudo scripts/bb_init.sh --no-pull
```

Other flags: `--force-launch` (bypass both gates), `--no-build`, `--no-lock`.
Per-boot lock lives at `/run/bb_boot/lock`; the start-once flag at
`/run/bb_boot/mission_started` (both cleared on reboot since `/run` is tmpfs).
