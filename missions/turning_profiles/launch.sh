#!/bin/bash
#------------------------------------------------------------
# launch.sh -- turning_profiles SIMULATION: shoreside + one boat.
# Every argument except the ones below is passed to launch_vehicle.sh
# (so --profile=, --speeds=, --trials=, --dir=, ... all work here).
#   --nogui, -n     headless shoreside
#   [time_warp]     final numeric arg
#------------------------------------------------------------
on_exit() { echo; echo "$ME: Halting all apps"; kill -- -$$ 2>/dev/null || true; }
trap on_exit SIGINT
ME=$(basename "$0")
TIME_WARP=1
GUI_ARG=""
VARGS=""
for ARGI; do
    if [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--nogui" -o "${ARGI}" = "-n" ]; then GUI_ARG="--nogui"
    elif [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
        echo "$ME --profile=agile|slow [vehicle options] [--nogui] [time_warp]"; exit 0
    else VARGS+=" ${ARGI}"
    fi
done

./launch_shoreside.sh --auto --sim $GUI_ARG $TIME_WARP || exit 1
./launch_vehicle.sh --sim --auto $VARGS $TIME_WARP || { ktm >/dev/null 2>&1; exit 1; }

uMAC targs/targ_shoreside.moos
kill -- -$$ 2>/dev/null || true
