#!/bin/bash
#--------------------------------------------------------------
# sbus_diag.sh - system-level RC/SBUS diagnostics for the boat
#
# Checks everything OUTSIDE the SBUS byte stream that can make
# iRCReader report "not connected" while the transmitter shows a
# healthy RF link. Run ON THE VEHICLE (Raspberry Pi). Pair with
# the sbus_probe applet, which analyzes the byte stream itself.
#
# Usage: ./sbus_diag.sh [device]     (default: /dev/ttyS0)
#--------------------------------------------------------------

DEV="${1:-/dev/ttyS0}"

hr()   { echo "--------------------------------------------------------"; }
ok()   { echo "  [ OK ] $*"; }
warn() { echo "  [WARN] $*"; }
bad()  { echo "  [FAIL] $*"; }

echo "SBUS system diagnostics for $DEV"
hr

# --- 0. Platform ------------------------------------------------
if [ -r /proc/device-tree/model ]; then
    echo "Platform: $(tr -d '\0' < /proc/device-tree/model)"
else
    warn "Not a device-tree platform (not a Pi?) - some checks skipped"
fi
hr

# --- 1. Device node ---------------------------------------------
echo "[1] Device node"
if [ -c "$DEV" ]; then
    ok "$DEV exists: $(ls -l "$DEV" | awk '{print $1, $3, $4}')"
else
    bad "$DEV does not exist"
    echo "     Available serial candidates:"
    ls -l /dev/ttyS* /dev/ttyAMA* /dev/serial* 2>/dev/null | sed 's/^/       /'
    echo "     -> iRCReader is hardcoded to /dev/ttyS0 (sbus_handler.h);"
    echo "        if SBUS is wired to another UART it will never connect."
fi

# serial0/serial1 aliases tell you which HW UART each tty maps to
for a in serial0 serial1; do
    if [ -L "/dev/$a" ]; then
        echo "     /dev/$a -> $(readlink /dev/$a)"
    fi
done

# Identify mini UART vs PL011. The mini UART (bcm2835aux / ttyS0)
# has NO parity hardware; SBUS is 8E2 and cannot be framed on it.
base=$(basename "$DEV")
if [ -e "/sys/class/tty/$base/device/driver" ]; then
    drv=$(basename "$(readlink "/sys/class/tty/$base/device/driver")")
    echo "     Driver: $drv"
    case "$drv" in
        *aux*|*8250*|*serial8250*)
            warn "$DEV appears to be the MINI UART (driver: $drv)."
            echo "       The Pi mini UART has no parity support; SBUS (8E2)"
            echo "       will read as garbage. Use a PL011 (/dev/ttyAMA*)"
            echo "       or check that BlueOS maps this tty to a PL011." ;;
        *pl011*|*amba*|*uart-pl011*)
            ok "$DEV is a PL011 UART (parity capable)" ;;
    esac
fi
hr

# --- 2. Permissions ---------------------------------------------
echo "[2] Permissions"
if [ -c "$DEV" ]; then
    if [ -r "$DEV" ]; then
        ok "current user ($(whoami)) can read $DEV"
    else
        bad "current user ($(whoami)) cannot read $DEV"
        echo "     -> sudo usermod -aG dialout $(whoami)  (then re-login),"
        echo "        or run iRCReader as a user with access."
    fi
fi
hr

# --- 3. Who is holding the port? --------------------------------
echo "[3] Port holders"
if [ -c "$DEV" ]; then
    HOLDERS=$(sudo fuser -v "$DEV" 2>&1 | tail -n +2)
    if [ -n "$HOLDERS" ]; then
        warn "processes currently using $DEV:"
        echo "$HOLDERS" | sed 's/^/       /'
        echo "     -> If this is iRCReader, stop MOOS before running"
        echo "        sbus_probe (concurrent readers split the stream)."
        echo "     -> If this is a getty/console/mavlink-router process,"
        echo "        it is EATING the SBUS bytes and iRCReader will"
        echo "        starve even with a perfect link."
    else
        ok "no other process has $DEV open"
    fi
fi
hr

# --- 4. Serial console stealing the UART ------------------------
echo "[4] Kernel serial console"
for CMDLINE in /boot/cmdline.txt /boot/firmware/cmdline.txt /proc/cmdline; do
    [ -r "$CMDLINE" ] || continue
    if grep -q "console=serial0\|console=ttyS0\|console=ttyAMA0" "$CMDLINE"; then
        warn "serial console enabled in $CMDLINE:"
        grep -o "console=[^ ]*" "$CMDLINE" | sed 's/^/       /'
        echo "     -> The kernel/getty owns the UART. Disable with"
        echo "        raspi-config (Interface Options -> Serial Port ->"
        echo "        login shell over serial: No) and reboot."
    else
        ok "no serial console in $CMDLINE"
    fi
    break
done
if systemctl list-units --type=service 2>/dev/null | grep -q "serial-getty@$(basename "$DEV")"; then
    bad "serial-getty@$(basename "$DEV").service is running (login shell on the port)"
    echo "     -> sudo systemctl disable --now serial-getty@$(basename "$DEV").service"
else
    ok "no getty running on $(basename "$DEV")"
fi
hr

# --- 5. Boot config UART setup ----------------------------------
echo "[5] Boot config"
for CFG in /boot/config.txt /boot/firmware/config.txt; do
    [ -r "$CFG" ] || continue
    echo "     UART-related lines in $CFG:"
    grep -n "enable_uart\|dtoverlay=uart\|dtoverlay=disable-bt\|dtoverlay=miniuart" "$CFG" \
        | sed 's/^/       /' || echo "       (none found)"
    if ! grep -q "^enable_uart=1" "$CFG" && [ "$(basename "$DEV")" = "ttyS0" ]; then
        warn "enable_uart=1 not set; ttyS0 (mini UART) is disabled by default"
    fi
    break
done
hr

# --- 6. Is data flowing right now? ------------------------------
echo "[6] Quick data sniff (3 seconds)"
if [ -r "$DEV" ]; then
    # Configure as close to SBUS as stty allows just for the sniff.
    stty -F "$DEV" raw cs8 parenb -parodd cstopb -echo 2>/dev/null
    BYTES=$(timeout 3 dd if="$DEV" bs=64 2>/dev/null | wc -c)
    if [ "$BYTES" -gt 0 ]; then
        ok "$BYTES bytes in 3s (healthy SBUS: ~5000-11000)"
        echo "     -> Bytes are arriving. Use 'sbus_probe' for frame-level"
        echo "        analysis (footer/range/failsafe rejection reasons)."
    else
        bad "0 bytes in 3s - nothing on the wire"
        echo "     Most common cause with a RadioLink R9DS/R12DS: the"
        echo "     receiver is in PWM mode, not SBUS mode. The TX binds"
        echo "     fine either way, so the transmitter LED looks good."
        echo "     R9DS: short-press the ID SET button twice within 1s;"
        echo "     LED blue = SBUS output on the CH9/SBUS port, red = PWM."
        echo "     Also check: signal wire on the right pin, common ground,"
        echo "     receiver power, and that this is the right UART."
    fi
else
    warn "cannot read $DEV - skipping sniff"
fi
hr

echo "Next step: run 'sbus_probe -d $DEV' (build/bin) for byte- and"
echo "frame-level diagnosis with an automatic verdict, or"
echo "'sbus_probe --lib' to reproduce exactly what iRCReader computes."
