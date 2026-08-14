#!/bin/bash
#---------------------------------------------------------------
# bb_esc_failsafe.sh -- last-line ESC runaway guard for BlueBoat.
#
# Covers the one gap iBBNavigatorInterface's signal handlers cannot:
# an UNHANDLED process death (SIGKILL, kernel OOM). The PCA9685
# free-runs its last-written pulses forever (bench-verified
# 2026-08-14 -- releasing the OE GPIO line does not reset the pin),
# so a hard kill mid-drive leaves the props turning with nobody in
# charge, until this watcher or a power cut intervenes.
#
# Policy (docs/rc_controllers.md section 8.3): the fleet safe state
# is NEUTRAL-ON-THE-WIRE -- ESCs armed at 1500us, silent, props
# stopped -- not signal-cut. So the corrective action is to write
# neutral to the thruster channels; OE-high (signal-cut) is only the
# fallback when the chip state can't be trusted (unexpected PWM
# frequency).
#
# Loop, 1 Hz:
#   app process running                     -> do nothing
#   no app, OE already high                 -> safe (no signal), done
#   no app, OE low, thrusters neutral/off   -> safe, done (quiet)
#   no app, OE low, thrusters DRIVING       -> write neutral (or cut
#                                              OE if freq unexpected)
#
# Install (as root, on the boat):
#   cp scripts/bb_esc_failsafe.sh /usr/local/bin/
#   cp scripts/bb-esc-failsafe.service /etc/systemd/system/
#   systemctl daemon-reload
#   systemctl enable --now bb-esc-failsafe
#
# Needs: i2c-tools, raspi-gpio (or pinctrl), pgrep. Run as root.
#---------------------------------------------------------------
set -u

APP_NAME="iBBNavigatorInterface"
OE_PIN=26                 # PCA9685 output-enable, active low
ADDR=0x40                 # PCA9685 I2C address
LEFT_CH=13                # PWM ch14, 0-based (left thruster)
RIGHT_CH=15               # PWM ch16, 0-based (right thruster)
NEUTRAL_OFF=614           # 1500us @ 100 Hz: 0.15 * 4096
TOL=8                     # counts (~20us); anything further is "driving"
PRESCALE_100HZ=59         # 24.576 MHz ext clock @ 100 Hz

# --- find the PWM bus (Pi 4: i2c-4, Pi 5: i2c-3) ----------------
BUS=""
for b in 4 3; do
  if i2cget -y "$b" $ADDR 0x00 >/dev/null 2>&1; then BUS=$b; break; fi
done
if [ -z "$BUS" ]; then
  logger -t bb_esc_failsafe "FATAL: PCA9685 not found on i2c-4/i2c-3"
  exit 1
fi
logger -t bb_esc_failsafe "started: bus i2c-$BUS, guarding ch$((LEFT_CH+1))/ch$((RIGHT_CH+1))"

read_oe_level() {
  # Read GPIO level WITHOUT requesting the line (a request would
  # reconfigure the pin and change the OE state). raspi-gpio and
  # pinctrl both read registers directly.
  if command -v raspi-gpio >/dev/null 2>&1; then
    raspi-gpio get $OE_PIN | grep -o 'level=[01]' | cut -d= -f2
  elif command -v pinctrl >/dev/null 2>&1; then
    pinctrl get $OE_PIN | grep -q '\bhi\b' && echo 1 || echo 0
  else
    echo ""   # no tool: fail safe by treating OE as low (act path)
  fi
}

cut_oe() {
  if command -v raspi-gpio >/dev/null 2>&1; then
    raspi-gpio set $OE_PIN op dh
  elif command -v pinctrl >/dev/null 2>&1; then
    pinctrl set $OE_PIN op dh
  fi
}

# Return 0 (safe) if the channel is at neutral or full-off, 1 if it
# is emitting a non-neutral pulse (driving).
channel_dangerous() {
  local ch=$1 base on_l on_h off_l off_h on off
  base=$((6 + 4 * ch))
  on_l=$(i2cget -y "$BUS" $ADDR $base)            || return 0
  on_h=$(i2cget -y "$BUS" $ADDR $((base + 1)))    || return 0
  off_l=$(i2cget -y "$BUS" $ADDR $((base + 2)))   || return 0
  off_h=$(i2cget -y "$BUS" $ADDR $((base + 3)))   || return 0
  on=$(( (on_h & 0x0F) * 256 + on_l ))
  off=$(( (off_h & 0x0F) * 256 + off_l ))
  # Full-off bit (OFF_H bit 4): no pulses at all -> no thrust -> safe.
  if (( (off_h & 0x10) != 0 )); then return 0; fi
  if (( on != 0 )); then return 1; fi
  local d=$(( off - NEUTRAL_OFF )); d=${d#-}
  if (( d > TOL )); then return 1; fi
  return 0
}

write_neutral() {
  local ch=$1 base
  base=$((6 + 4 * ch))
  i2cset -y "$BUS" $ADDR $base 0x00
  i2cset -y "$BUS" $ADDR $((base + 1)) 0x00
  i2cset -y "$BUS" $ADDR $((base + 2)) $((NEUTRAL_OFF & 0xFF))
  i2cset -y "$BUS" $ADDR $((base + 3)) $((NEUTRAL_OFF >> 8))
}

was_safe=1
while true; do
  sleep 1

  # App alive: it owns the chip; never touch anything.
  if pgrep -x "$APP_NAME" >/dev/null 2>&1; then was_safe=1; continue; fi

  # OE high: outputs tri-stated, nothing reaches the ESCs.
  oe=$(read_oe_level)
  if [ "$oe" = "1" ]; then was_safe=1; continue; fi

  danger=0
  channel_dangerous $LEFT_CH  || danger=1
  channel_dangerous $RIGHT_CH || danger=1
  if (( danger == 0 )); then was_safe=1; continue; fi

  # Dead app, live outputs, non-neutral thrust: the runaway case.
  # Re-check the app immediately before acting (startup race).
  pgrep -x "$APP_NAME" >/dev/null 2>&1 && continue

  prescale=$(( $(i2cget -y "$BUS" $ADDR 0xFE) )) 2>/dev/null || prescale=-1
  if [ "$prescale" = "$PRESCALE_100HZ" ]; then
    write_neutral $LEFT_CH
    write_neutral $RIGHT_CH
    action="forced thrusters to neutral"
  else
    # Frequency not the expected 100 Hz: our neutral counts would be
    # a wrong pulse width. Use the big hammer instead.
    cut_oe
    action="unexpected prescale ($prescale); cut OE (signal-off)"
  fi

  if (( was_safe == 1 )); then
    logger -t bb_esc_failsafe "RUNAWAY GUARD TRIPPED: $APP_NAME dead with live non-neutral thrust -- $action"
    was_safe=0
  fi
done
