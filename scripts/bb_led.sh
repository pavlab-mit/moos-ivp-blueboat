#!/usr/bin/env bash
#==============================================================================
# bb_led.sh -- drive the PWM0 status LED (Pi 4 & Pi 5)
#
# The Navigator's PWM0 / fan header is wired directly to the Raspberry Pi SOC
# GPIO18 (BCM), NOT to the PCA9685 PWM driver or the Navigator user LEDs. So we
# drive it as a plain GPIO via pinctrl (or raspi-gpio), not navigator-lib. This
# needs no compiled binary, so the status LED works even if the C++ build is
# broken.
#
# Used by bb_init.sh:   on = solid (launched) | flash = blinking (idle)
#
# Usage: bb_led.sh on|off|flash [-d <secs>] [--hz <f>] [--end on|off]
#
# Env overrides:
#   BOAT_LED_GPIO          BCM pin (default 18 = PWM0)
#   BOAT_LED_ACTIVE_HIGH   true if driving the pin HIGH lights the LED (default true)
#
# Note: this reconfigures GPIO18 to a plain output. If a dtoverlay has it set
# as the PWM0 alt-function (e.g. for a fan), that use and this one conflict --
# pick one. Typically needs root (or gpio-group access) for pinctrl.
#==============================================================================
set -euo pipefail

GPIO="${BOAT_LED_GPIO:-18}"
ACTIVE_HIGH="${BOAT_LED_ACTIVE_HIGH:-true}"

cmd=""
duration=30
hz=2
end="off"

# heartbeat-mode tunables (idle "still alive" signal)
intro=10        # intro slow-flash duration (s); 0 to skip
intro_hz=1      # intro flash rate (Hz)
period=30       # seconds between blink groups
pulses=2        # blinks per group (2 = double-blink)
blink=0.12      # on-time per pulse (s)
gap=0.18        # off-time between pulses (s)

usage() {
  cat <<USAGE
Usage: bb_led.sh on|off|flash|heartbeat [options]
  on                 drive the LED solid on
  off                drive the LED off
  flash              blink, then leave it in the --end state
  heartbeat          intro slow-flash, then a blink group every --period forever
  -d, --duration <s> flash duration seconds (default 30)
      --hz <f>       flash rate Hz (default 2)
      --end <on|off> LED state after a flash (default off)
  heartbeat options:
      --intro <s>    intro slow-flash duration (default 10; 0 to skip)
      --intro-hz <f> intro flash rate Hz (default 1)
      --period <s>   seconds between blink groups (default 30)
      --pulses <n>   blinks per group (default 2 = double-blink)
      --blink <s>    on-time per pulse (default 0.12)
      --gap <s>      off-time between pulses (default 0.18)
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    on|off|flash|heartbeat)  cmd="$1" ;;
    -d|--duration) duration="${2:?}"; shift ;;
    --hz)          hz="${2:?}"; shift ;;
    --end)         end="${2:?}"; shift ;;
    --intro)       intro="${2:?}"; shift ;;
    --intro-hz)    intro_hz="${2:?}"; shift ;;
    --period)      period="${2:?}"; shift ;;
    --pulses)      pulses="${2:?}"; shift ;;
    --blink)       blink="${2:?}"; shift ;;
    --gap)         gap="${2:?}"; shift ;;
    -h|--help)     usage; exit 0 ;;
    *) echo "bb_led.sh: bad arg '$1'" >&2; exit 2 ;;
  esac
  shift
done
[[ -n "$cmd" ]] || { usage >&2; exit 2; }

# Pick a GPIO tool. Both pinctrl and raspi-gpio set the hardware pad and leave
# the level latched after exit (needed for a persistent "solid").
if command -v pinctrl >/dev/null 2>&1; then
  _drive() { pinctrl set "$GPIO" op "$1"; }     # $1 = dh | dl
elif command -v raspi-gpio >/dev/null 2>&1; then
  _drive() { raspi-gpio set "$GPIO" op "$1"; }
else
  echo "bb_led.sh: no GPIO tool found (need pinctrl or raspi-gpio)" >&2
  exit 1
fi

# Map logical on/off to a drive level, honoring active-high vs active-low.
led() {  # $1 = on | off
  local on_lvl off_lvl
  if [[ "$ACTIVE_HIGH" == "true" ]]; then on_lvl=dh; off_lvl=dl; else on_lvl=dl; off_lvl=dh; fi
  [[ "$1" == "on" ]] && _drive "$on_lvl" || _drive "$off_lvl"
}

case "$cmd" in
  on)  led on ;;
  off) led off ;;
  flash)
    half=$(awk -v hz="$hz" 'BEGIN{ printf "%.3f", 1/(2*hz) }')
    cycles=$(awk -v d="$duration" -v hz="$hz" 'BEGIN{ printf "%d", d*hz }')
    for ((i = 0; i < cycles; i++)); do
      led on;  sleep "$half"
      led off; sleep "$half"
    done
    led "$end"
    ;;
  heartbeat)
    # Idle "still alive" signal. Runs forever, so it lives in its own systemd
    # unit (bb-led-idle.service) that bb_init starts on stand-by; that unit's
    # ExecStopPost drives the LED off when the heartbeat is stopped (e.g. on a
    # later launch).
    if awk -v s="$intro" 'BEGIN{ exit !(s > 0) }'; then   # intro slow flash
      ihalf=$(awk -v hz="$intro_hz" 'BEGIN{ printf "%.3f", 1/(2*hz) }')
      icycles=$(awk -v d="$intro" -v hz="$intro_hz" 'BEGIN{ printf "%d", d*hz }')
      for ((i = 0; i < icycles; i++)); do
        led on;  sleep "$ihalf"
        led off; sleep "$ihalf"
      done
    fi
    led off
    while :; do                                           # blink group / period
      for ((p = 0; p < pulses; p++)); do
        led on;  sleep "$blink"
        led off; sleep "$gap"
      done
      sleep "$period"
    done
    ;;
esac
