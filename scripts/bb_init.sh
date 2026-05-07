#!/usr/bin/env bash
set -euo pipefail

# ---------- Args / toggles ----------
DEBUG=false
DRY_RUN=false
FORCE_LAUNCH=false
NO_LOCK=false
NO_NET_WAIT=false
FAKE_PITCH=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --debug) DEBUG=true ;;
    --dry-run) DRY_RUN=true ;;
    --force-launch) FORCE_LAUNCH=true ;;
    --no-lock) NO_LOCK=true ;;
    --no-network-wait) NO_NET_WAIT=true ;;
    --fake-pitch) FAKE_PITCH="${2:-}"; shift ;;
    -h|--help)
      cat <<'USAGE'
Usage: boot.sh [options]
  --debug            verbose shell trace and keep artifacts for inspection
  --dry-run          print actions without executing
  --force-launch     bypass pitch gate (launch regardless of pitch)
  --no-lock          do not use lock file
  --no-network-wait  skip any network waiting (if present)
  --fake-pitch <deg> skip bb_attitude and use this pitch value
USAGE
      exit 0
      ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

$DEBUG && set -x

# ---------- Config ----------
RUN_AS_USER="pi"

REPO_DIR="/home/pi/moos-ivp-blueboat"
BIN_DIR="$REPO_DIR/bin"
MISSIONS_DIR="$REPO_DIR/missions/blueboat_frontseat"

MOOS_DIRS=(
  "/home/pi/moos-ivp/bin"
  "/home/pi/moos/moos-ivp/bin"
  "$REPO_DIR/../moos-ivp/bin"
  "$REPO_DIR/MOOS-IvP/bin"
)

TEST_ADC="$BIN_DIR/test_bb_adc"
BB_ATTITUDE="$BIN_DIR/bb_attitude"
BB_LED="$BIN_DIR/test_discoturtle"
LAUNCH_CMD="./launch_vehicle.sh -a -nc"

LOG_DIR="/var/log/bb_boot"
LOG_FILE="$LOG_DIR/boot.log"
LOCK_FILE="/run/bb_boot/lock"

# Thresholds / tunables (overridable via /etc/default/boat)
PITCH_LIMIT="${BOAT_PITCH_LIMIT:-20}"
ATT_DUR="${BOAT_ATT_DURATION:-5}"   # seconds
# bb_attitude flags: -d <s> is supported; -v optional
BB_VERBOSE=false                    # set true to prefer verbose parsing

export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

# ---------- Logging ----------
sudo mkdir -p "$LOG_DIR"
sudo chown "$RUN_AS_USER:$RUN_AS_USER" "$LOG_DIR"
touch "$LOG_FILE"
chmod 644 "$LOG_FILE"
if ! $DEBUG; then
  exec >>"$LOG_FILE" 2>&1
fi

ts() { date -Is; }

run() {
  if $DRY_RUN; then
    echo "[$(ts)] DRY-RUN:" "$@"
    return 0
  fi
  echo "[$(ts)] RUN:" "$@"
  if (($# == 1)); then
    # A single string: execute via a shell so it can parse it
    bash -lc "$1"
  else
    # argv: execute directly, no shell
    "$@"
  fi
}

echo "[$(ts)] ===== boat boot start (debug=$DEBUG dry_run=$DRY_RUN force=$FORCE_LAUNCH) ====="

# ---------- Locking ----------
if ! $NO_LOCK; then
  if [[ -f "$LOCK_FILE" ]]; then
    echo "[$(ts)] Lock exists ($LOCK_FILE); exiting."
    exit 0
  fi
  echo $$ > "$LOCK_FILE"
  trap 'rm -f "$LOCK_FILE"' EXIT
else
  echo "[$(ts)] NO_LOCK enabled (not creating lock)"
fi

# ---------- 1) test_adc ----------
if [[ -x "$TEST_ADC" ]]; then
  run sudo -u "$RUN_AS_USER" -H "$TEST_ADC"
else
  echo "[$(ts)] WARN: test_adc not found at $TEST_ADC"
fi

# ---------- 2) bb_attitude & pitch ----------
PITCH=""

if [[ -n "${FAKE_PITCH:-}" ]]; then
  echo "[$(ts)] Using FAKE_PITCH=$FAKE_PITCH"
  PITCH="$FAKE_PITCH"

elif [[ -x "$BB_ATTITUDE" ]]; then
  # Build argv as an array (no eval; safe quoting)
  if $BB_VERBOSE; then
    CMD=(sudo -u "$RUN_AS_USER" -H "$BB_ATTITUDE" -d "$ATT_DUR" -v)
  else
    CMD=(sudo -u "$RUN_AS_USER" -H "$BB_ATTITUDE" -d "$ATT_DUR")
  fi

  if $DRY_RUN; then
    echo "[$(ts)] DRY-RUN:" "${CMD[@]}"
    PITCH="0.0"  # simulate acceptable pitch in dry-run
  else
    echo "[$(ts)] Sampling attitude ($ATT_DUR s)…"
    BB_OUT="$("${CMD[@]}" 2>&1 | tr -d '\r')"
    echo "[$(ts)] bb_attitude output:"
    printf '%s\n' "$BB_OUT"

    # Parse pitch:
    #  - verbose: "roll_deg=... pitch_deg=..."
    #  - plain:   "roll pitch"
    if [[ "$BB_OUT" =~ pitch_deg=([+-]?[0-9]+([.][0-9]+)?) ]]; then
      PITCH="${BASH_REMATCH[1]}"
    else
      # fall back to second field from plain output
      read -r _ PITCH <<< "$BB_OUT"
    fi

    if [[ -z "${PITCH:-}" ]]; then
      echo "[$(ts)] ERROR: Could not parse pitch from bb_attitude output."
      exit 1
    fi
  fi

else
  echo "[$(ts)] WARN: bb_attitude not found/executable at $BB_ATTITUDE; assuming pitch=0"
  PITCH="0.0"
fi

echo "[$(ts)] Parsed pitch=$PITCH  limit=$PITCH_LIMIT"

# Gate (set -e safe): abort launch when pitch >= limit
if awk -v p="$PITCH" -v lim="$PITCH_LIMIT" 'BEGIN{ exit !(p >= lim) }'; then
  echo "[$(ts)] Pitch >= limit; running LED indicator and aborting launch."
  # Swallow any non-zero from the LED step.
  run timeout 60 sudo -u "$RUN_AS_USER" -H "$BB_LED" rainbow || true
  # If this file might ever be sourced, return instead of exiting the shell:
  return 0 2>/dev/null || exit 0
fi

# ---------- 4) launch mission via systemd (start-once, no restarts) ----------
START_FLAG="/run/bb_boot/mission_started"

# ensure runtime dir exists (or set RuntimeDirectory=bb_boot in the unit)
mkdir -p /run/bb_boot

if systemctl is-active --quiet fs-mission.service; then
  echo "[$(ts)] blueboat.service is already active; not touching it."
elif [[ -e "$START_FLAG" ]]; then
  echo "[$(ts)] Start already attempted this boot (flag $START_FLAG present); skipping."
elif systemctl is-failed --quiet fs-mission.service; then
  echo "[$(ts)] blueboat.service is in FAILED state; not auto-restarting. Investigate logs and start manually."
  exit 1
else
  echo "[$(ts)] Starting blueboat.service..."
  if systemctl start fs-mission.service; then
    # only mark as started if the start call succeeded
    date -Is > "$START_FLAG"
    echo "[$(ts)] blueboat.service start requested; flag written to $START_FLAG"
  else
    echo "[$(ts)] ERROR: systemctl start blueboat.service failed."
    exit 1
  fi
fi

echo "[$(ts)] ===== boat boot done ====="
