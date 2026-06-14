#!/usr/bin/env bash
#==============================================================================
# bb_init.sh -- BlueBoat front-seat auto-launch boot script
#
# Run once at boot (from a systemd oneshot, see scripts/systemd/). Decides
# whether the boat should auto-launch its front-seat mission or stand by idle:
#
#   1. (optional) Pull + rebuild the repo if it changed.
#   2. Low-battery gate  -- read pack voltage (bb_adc); stand by if too low.
#   3. Pitch gate        -- read attitude  (bb_attitude); stand by if the boat
#                           is not sitting roughly level (e.g. on a cart, being
#                           carried, or stowed).
#   4. Launch            -- start the mission systemd service (start-once).
#
# Status is signaled three ways so you can tell what happened without a screen:
#   - the PWM0 status LED (bb_led.sh, Pi GPIO18): solid = launched, blinking = idle
#   - an optional NeoPixel strip (bb_neopixel), off by default
#   - a status file at $STATUS_FILE and a log at $LOG_FILE
#
# Every tunable below is read from the environment with a built-in default, so
# the script runs standalone. Per-boat overrides (mainly BOAT_VOLT_MIN) are set
# as Environment= lines in the systemd unit -- see scripts/systemd/bb-init.service.
#==============================================================================

set -euo pipefail

#------------------------------------------------------------------------------
# Args / toggles
#------------------------------------------------------------------------------
DEBUG=false
DRY_RUN=false
FORCE_LAUNCH=false      # bypass battery + pitch gates
NO_LOCK=false
NO_PULL=false           # skip git pull/build for this run
NO_BUILD=false          # pull but do not rebuild
FAKE_PITCH=""
FAKE_VOLTAGE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --debug)         DEBUG=true ;;
    --dry-run)       DRY_RUN=true ;;
    --force-launch)  FORCE_LAUNCH=true ;;
    --no-lock)       NO_LOCK=true ;;
    --no-pull)       NO_PULL=true ;;
    --no-build)      NO_BUILD=true ;;
    --fake-pitch)    FAKE_PITCH="${2:-}"; shift ;;
    --fake-voltage)  FAKE_VOLTAGE="${2:-}"; shift ;;
    -h|--help)
      cat <<'USAGE'
Usage: bb_init.sh [options]
  --debug            verbose shell trace; log to stdout instead of the log file
  --dry-run          print actions without executing hardware/launch steps
  --force-launch     bypass the battery and pitch gates (launch regardless)
  --no-lock          do not use the boot lock file
  --no-pull          skip the git pull + rebuild step this run
  --no-build         pull but do not rebuild even if the repo changed
  --fake-pitch <d>   skip bb_attitude and use this pitch (deg) instead
  --fake-voltage <v> skip bb_adc and use this pack voltage (V) instead
USAGE
      exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

#------------------------------------------------------------------------------
# Config (override any BOAT_* var via Environment= in bb-init.service)
#------------------------------------------------------------------------------
RUN_AS_USER="${BOAT_USER:-pi}"

REPO_DIR="${BOAT_REPO_DIR:-/home/${RUN_AS_USER}/moos-ivp-blueboat}"
BIN_DIR="$REPO_DIR/bin"
BUILD_SCRIPT="${BOAT_BUILD_SCRIPT:-$REPO_DIR/build.sh}"

# systemd unit that actually launches the mission (you maintain this unit;
# a reference copy is in scripts/systemd/fs-mission.service).
MISSION_SERVICE="${BOAT_MISSION_SERVICE:-fs-mission.service}"

# Idle status-LED heartbeat unit -- started when the boat stands by, stopped
# when it launches. Runs as its own unit because bb_init is a oneshot and
# exits, while the heartbeat must keep blinking for as long as the boat is on.
LED_IDLE_SERVICE="${BOAT_LED_IDLE_SERVICE:-bb-led-idle.service}"

# Helpers. The sensor/strip helpers are compiled (src/02_applets); the PWM0
# status LED is a shell helper (Pi GPIO18 via pinctrl, no build needed).
BB_ATTITUDE="$BIN_DIR/bb_attitude"
BB_ADC="$BIN_DIR/bb_adc"
BB_NEOPIXEL="$BIN_DIR/bb_neopixel"
BB_LED="${BOAT_LED_SCRIPT:-$REPO_DIR/scripts/bb_led.sh}"

# Git pull / rebuild
DO_PULL="${BOAT_DO_PULL:-true}"          # master toggle for the pull+build step
DO_BUILD="${BOAT_DO_BUILD:-true}"        # rebuild when the pull changes HEAD
GIT_REMOTE="${BOAT_GIT_REMOTE:-origin}"
GIT_BRANCH="${BOAT_GIT_BRANCH:-}"        # empty -> current branch
PULL_TIMEOUT="${BOAT_PULL_TIMEOUT:-60}"  # seconds (per fetch attempt)
PULL_RETRIES="${BOAT_PULL_RETRIES:-3}"   # fetch attempts before giving up
PULL_RETRY_DELAY="${BOAT_PULL_RETRY_DELAY:-5}"  # seconds between attempts
BUILD_TIMEOUT="${BOAT_BUILD_TIMEOUT:-600}"

# Gates (tunables)
PITCH_LIMIT="${BOAT_PITCH_LIMIT:-15}"    # |pitch| >= this (deg) -> stand by
ATT_DUR="${BOAT_ATT_DURATION:-5}"        # attitude sample seconds
VOLT_MIN="${BOAT_VOLT_MIN:-13.0}"        # pack volts below this -> stand by
                                         #   NOTE: calibrate for your pack!

# Status signaling. The PWM0 LED pin/polarity are read by bb_led.sh itself
# (BOAT_LED_GPIO / BOAT_LED_ACTIVE_HIGH), inherited from this process's env.
IDLE_FLASH_SECS="${BOAT_IDLE_FLASH_SECS:-30}"
USE_NEOPIXEL="${BOAT_NEOPIXEL:-false}"   # drive an LED strip too (future hw)
NEOPIXEL_COUNT="${BOAT_NEOPIXEL_COUNT:-24}"

# Logging / lock / runtime state
LOG_DIR="${BOAT_LOG_DIR:-/var/log/bb_boot}"
LOG_FILE="$LOG_DIR/boot.log"
RUN_DIR="${BOAT_RUN_DIR:-/run/bb_boot}"
LOCK_FILE="$RUN_DIR/lock"
START_FLAG="$RUN_DIR/mission_started"
STATUS_FILE="$RUN_DIR/status"

export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

$DEBUG && set -x

#------------------------------------------------------------------------------
# Logging
#------------------------------------------------------------------------------
ts() { date -Is; }
log() { echo "[$(ts)] $*"; }

sudo mkdir -p "$LOG_DIR" "$RUN_DIR"
sudo chown "$RUN_AS_USER:$RUN_AS_USER" "$LOG_DIR" "$RUN_DIR" 2>/dev/null || true
touch "$LOG_FILE"; chmod 644 "$LOG_FILE"
if ! $DEBUG; then
  exec >>"$LOG_FILE" 2>&1
fi

# Run a command, honoring --dry-run. argv form (no shell parsing).
run() {
  if $DRY_RUN; then
    log "DRY-RUN: $*"
    return 0
  fi
  log "RUN: $*"
  "$@"
}

# Run a command as $RUN_AS_USER (only sudo if we are not already that user).
as_user() {
  if [[ "$(id -un)" == "$RUN_AS_USER" ]]; then
    "$@"
  else
    sudo -u "$RUN_AS_USER" -H "$@"
  fi
}

#------------------------------------------------------------------------------
# Status signaling -- write a status file and drive the LED(s)
#------------------------------------------------------------------------------
# write_status <STATE> <detail...>
write_status() {
  local state="$1"; shift
  local detail="$*"
  log "STATUS: $state ${detail:+- $detail}"
  $DRY_RUN && return 0
  {
    echo "state=$state"
    echo "detail=$detail"
    echo "time=$(ts)"
    echo "pid=$$"
  } > "$STATUS_FILE" 2>/dev/null || true
}

# Mission running: a 5 s fast strobe to announce the launch, then leave the LED
# solid on (the pin latches, so no daemon is needed for the running state).
# First stop any idle heartbeat so it cannot fight us for the pin. Optional
# nav-lights on the strip. The LED runs as root (GPIO/pinctrl); strip as pi.
signal_running() {
  run systemctl stop "$LED_IDLE_SERVICE" || true
  [[ -x "$BB_LED" ]] && run "$BB_LED" flash -d 5 --hz 10 --end on || true
  if [[ "$USE_NEOPIXEL" == "true" && -x "$BB_NEOPIXEL" ]]; then
    run as_user "$BB_NEOPIXEL" navlights --count "$NEOPIXEL_COUNT" || true
  fi
}

# Idle/standby: hand the LED to bb-led-idle.service, which does a 10 s slow
# flash then a double-blink every minute for as long as the boat stays on, so
# an idle-but-powered boat is distinguishable from a dead one. We restart (not
# start) it so re-entering idle replays the intro flash. Optional rainbow strip.
signal_idle() {
  if [[ "$USE_NEOPIXEL" == "true" && -x "$BB_NEOPIXEL" ]]; then
    # timeout goes *inside* as_user (it must exec a real binary, not the
    # as_user shell function). -d already self-limits; timeout is a backstop.
    run as_user timeout "$IDLE_FLASH_SECS" "$BB_NEOPIXEL" rainbow \
        --count "$NEOPIXEL_COUNT" -d "$IDLE_FLASH_SECS" || true
  fi
  run systemctl restart --no-block "$LED_IDLE_SERVICE" || true
}

# Called whenever we choose not to launch. Records status, signals idle, exits 0
# (standing by is a normal outcome, not a failure).
stand_by() {
  local state="$1"; shift
  write_status "$state" "$*"
  signal_idle
  log "===== boat standing by ($state) ====="
  exit 0
}

# Called on a real error (something broke). Signals idle and exits non-zero.
fail() {
  local detail="$*"
  write_status "ERROR" "$detail"
  log "ERROR: $detail"
  signal_idle
  exit 1
}

#------------------------------------------------------------------------------
# Start
#------------------------------------------------------------------------------
log "===== boat boot start (debug=$DEBUG dry_run=$DRY_RUN force=$FORCE_LAUNCH) ====="
write_status "BOOTING"

#------------------------------------------------------------------------------
# Locking -- one boot attempt per boot
#------------------------------------------------------------------------------
if ! $NO_LOCK; then
  if [[ -f "$LOCK_FILE" ]]; then
    log "Lock exists ($LOCK_FILE); another bb_init is running or already ran. Exiting."
    exit 0
  fi
  echo $$ > "$LOCK_FILE"
  trap 'rm -f "$LOCK_FILE"' EXIT
else
  log "NO_LOCK enabled (not creating lock)"
fi

#------------------------------------------------------------------------------
# 1) Battery gate
#------------------------------------------------------------------------------
VOLTAGE=""
if $FORCE_LAUNCH; then
  log "FORCE_LAUNCH set; skipping battery gate."
elif [[ -n "$FAKE_VOLTAGE" ]]; then
  log "Using FAKE_VOLTAGE=$FAKE_VOLTAGE"
  VOLTAGE="$FAKE_VOLTAGE"
elif [[ -x "$BB_ADC" ]]; then
  if $DRY_RUN; then
    log "DRY-RUN: $BB_ADC ; assuming healthy voltage."
    VOLTAGE="99.0"
  else
    ADC_OUT="$(as_user "$BB_ADC" 2>&1 | tr -d '\r' || true)"
    log "bb_adc output: $ADC_OUT"
    VOLTAGE="$(printf '%s\n' "$ADC_OUT" | grep -oE 'VOLTAGE=[0-9.]+' | head -1 | cut -d= -f2)"
    [[ -z "$VOLTAGE" ]] && fail "Could not parse voltage from bb_adc output."
  fi
else
  log "WARN: bb_adc not found at $BB_ADC; skipping battery gate."
fi

if [[ -n "$VOLTAGE" ]]; then
  log "Pack voltage=$VOLTAGE  min=$VOLT_MIN"
  if awk -v v="$VOLTAGE" -v lim="$VOLT_MIN" 'BEGIN{ exit !(v < lim) }'; then
    stand_by "IDLE_LOWBATT" "voltage $VOLTAGE < $VOLT_MIN"
  fi
fi

#------------------------------------------------------------------------------
# 2) Pitch gate
#------------------------------------------------------------------------------
PITCH=""
if $FORCE_LAUNCH; then
  log "FORCE_LAUNCH set; skipping pitch gate."
elif [[ -n "$FAKE_PITCH" ]]; then
  log "Using FAKE_PITCH=$FAKE_PITCH"
  PITCH="$FAKE_PITCH"
elif [[ -x "$BB_ATTITUDE" ]]; then
  if $DRY_RUN; then
    log "DRY-RUN: $BB_ATTITUDE -d $ATT_DUR -v ; assuming level."
    PITCH="0.0"
  else
    log "Sampling attitude (${ATT_DUR}s)..."
    BB_OUT="$(as_user "$BB_ATTITUDE" -d "$ATT_DUR" -v 2>&1 | tr -d '\r' || true)"
    log "bb_attitude output: $BB_OUT"
    if [[ "$BB_OUT" =~ pitch_deg=([+-]?[0-9]+([.][0-9]+)?) ]]; then
      PITCH="${BASH_REMATCH[1]}"
    else
      read -r _ PITCH <<< "$BB_OUT"   # fall back to "roll pitch" plain output
    fi
    [[ -z "${PITCH:-}" ]] && fail "Could not parse pitch from bb_attitude output."
  fi
else
  log "WARN: bb_attitude not found at $BB_ATTITUDE; assuming level (pitch=0)."
  PITCH="0.0"
fi

if [[ -n "$PITCH" ]]; then
  log "Pitch=$PITCH  limit=+/-$PITCH_LIMIT"
  # Gate on absolute pitch -- nose-up or nose-down both mean "not in the water."
  if awk -v p="$PITCH" -v lim="$PITCH_LIMIT" 'BEGIN{ if (p<0) p=-p; exit !(p >= lim) }'; then
    stand_by "IDLE_PITCH" "|pitch| $PITCH >= $PITCH_LIMIT"
  fi
fi

#------------------------------------------------------------------------------
# 3) Pull + rebuild -- only reached once the gates have PASSED, so a boat that
# is standing by (elevated / low battery) never waits on a pull or build, and
# the network gets extra seconds to come up before we fetch. Best-effort: an
# offline pull just continues with the existing build, but a failed *build*
# aborts the launch (a half-built tree is unsafe).
#------------------------------------------------------------------------------
if $NO_PULL || [[ "$DO_PULL" != "true" ]]; then
  log "Skipping git pull (NO_PULL=$NO_PULL DO_PULL=$DO_PULL)"
elif [[ ! -d "$REPO_DIR/.git" ]]; then
  log "WARN: $REPO_DIR is not a git repo; skipping pull."
else
  branch="$GIT_BRANCH"
  [[ -z "$branch" ]] && branch="$(as_user git -C "$REPO_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || echo '')"
  before="$(as_user git -C "$REPO_DIR" rev-parse HEAD 2>/dev/null || echo '')"

  log "Pulling $GIT_REMOTE/$branch (timeout ${PULL_TIMEOUT}s, up to ${PULL_RETRIES} tries)..."
  # Boot-time networking (especially over the radio link) is often not ready the
  # instant bb-init runs, even with network-online.target -- the fetch fails for
  # a few seconds and then works. Retry with a short backoff before giving up.
  pull_ok=false
  if $DRY_RUN; then
    log "DRY-RUN: git fetch + ff-only merge $GIT_REMOTE/$branch"
    pull_ok=true
  else
    for ((attempt=1; attempt<=PULL_RETRIES; attempt++)); do
      if as_user timeout "$PULL_TIMEOUT" git -C "$REPO_DIR" fetch --quiet "$GIT_REMOTE" "$branch" \
         && as_user git -C "$REPO_DIR" merge --ff-only --quiet "$GIT_REMOTE/$branch"; then
        pull_ok=true
        break
      fi
      if [[ "$attempt" -lt "$PULL_RETRIES" ]]; then
        log "WARN: git pull failed (attempt $attempt/$PULL_RETRIES); sleeping ${PULL_RETRY_DELAY}s..."
        sleep "$PULL_RETRY_DELAY"
      fi
    done
  fi

  if ! $pull_ok; then
    # Offline / unreachable remote is fine -- launch with what we have.
    log "WARN: pull failed after ${PULL_RETRIES} attempts (offline?); continuing with existing build."
  elif ! $DRY_RUN; then
    after="$(as_user git -C "$REPO_DIR" rev-parse HEAD 2>/dev/null || echo '')"
    if [[ "$before" != "$after" ]]; then
      log "Repo updated: ${before:0:8} -> ${after:0:8}"
      if $NO_BUILD || [[ "$DO_BUILD" != "true" ]]; then
        log "Repo changed but rebuild disabled (NO_BUILD=$NO_BUILD DO_BUILD=$DO_BUILD)"
      else
        log "Rebuilding (timeout ${BUILD_TIMEOUT}s)..."
        if as_user timeout "$BUILD_TIMEOUT" bash -lc "cd '$REPO_DIR' && '$BUILD_SCRIPT'"; then
          log "Rebuild OK."
        else
          # A half-built tree is unsafe to launch -> stand by.
          fail "Rebuild failed after pull; not launching with a partial build."
        fi
      fi
    else
      log "Repo already up to date; no rebuild needed."
    fi
  fi
fi

#------------------------------------------------------------------------------
# 4) Launch the mission via systemd (start-once, no auto-restart loop)
#------------------------------------------------------------------------------
if $DRY_RUN; then
  log "DRY-RUN: systemctl start $MISSION_SERVICE"
  write_status "LAUNCHED" "dry-run"
  log "===== boat boot done (dry-run) ====="
  exit 0
fi

if systemctl is-active --quiet "$MISSION_SERVICE"; then
  log "$MISSION_SERVICE already active; leaving it alone."
  write_status "LAUNCHED" "already active"
  signal_running
elif [[ -e "$START_FLAG" ]]; then
  log "Start already attempted this boot (flag $START_FLAG present); skipping."
elif systemctl is-failed --quiet "$MISSION_SERVICE"; then
  fail "$MISSION_SERVICE is in FAILED state; not auto-restarting. Investigate logs."
else
  log "Starting $MISSION_SERVICE..."
  if systemctl start "$MISSION_SERVICE"; then
    date -Is > "$START_FLAG"
    log "$MISSION_SERVICE start requested; flag written to $START_FLAG"
    write_status "LAUNCHED" "pitch=$PITCH voltage=${VOLTAGE:-NA}"
    signal_running
  else
    fail "systemctl start $MISSION_SERVICE failed."
  fi
fi

log "===== boat boot done ====="
