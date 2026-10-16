#!/bin/bash
#------------------------------------------------------------
# sim_test.sh -- headless end-to-end shakeout of one profile.
#
# Launches shoreside + vehicle in sim, pokes DEPLOY, waits until
# the vehicle log holds the expected number of FT_REPORTs and the
# mission has flipped to RETURN, tears down, then scores the log
# with analysis/turn_radius.py. Exit code 0 only if every turn
# reported and the scorer parsed them all.
#
#   ./sim_test.sh --profile=agile [--speeds=1.0,1.5] [--trials=1] [warp]
#------------------------------------------------------------
ME=$(basename "$0")
HERE=$(cd "$(dirname "$0")" && pwd)
cd "$HERE"
export PATH="$PATH:$HERE/../../bin"

TIME_WARP=10
PROFILE=""
SPEEDS="1.0,1.5"
TRIALS="1"
EXTRA=""
MAX_WAIT=900     # real seconds
FORCE_KTM="no"
for ARGI; do
    if [ "${ARGI//[^0-9]/}" = "$ARGI" ]; then TIME_WARP=$ARGI
    elif [ "${ARGI:0:10}" = "--profile=" ]; then PROFILE="${ARGI#--profile=*}"
    elif [ "${ARGI:0:9}" = "--speeds=" ]; then SPEEDS="${ARGI#--speeds=*}"
    elif [ "${ARGI:0:9}" = "--trials=" ]; then TRIALS="${ARGI#--trials=*}"
    elif [ "${ARGI:0:11}" = "--max_wait=" ]; then MAX_WAIT="${ARGI#--max_wait=*}"
    elif [ "${ARGI}" = "--force_ktm" ]; then FORCE_KTM="yes"
    else EXTRA+=" $ARGI"
    fi
done
[ -z "$PROFILE" ] && { echo "$ME: --profile=agile|slow required"; exit 1; }
command -v pBBPID >/dev/null || { echo "$ME: pBBPID not on PATH (build moos-ivp-blueboat)"; exit 1; }

NSPD=$(echo "$SPEEDS" | tr ',' '\n' | grep -c .)
EXPECT=$((NSPD * TRIALS))

# Teardown uses ktm, which kills EVERY MOOS process on this machine.
if pgrep -x MOOSDB >/dev/null 2>&1 && [ "$FORCE_KTM" != "yes" ]; then
    echo "$ME: a MOOSDB is already running on this machine; refusing to ktm it."
    echo "$ME: stop it yourself or pass --force_ktm."; pgrep -fl MOOSDB; exit 1
fi
ktm >/dev/null 2>&1; sleep 1
rm -rf logs/LOG_ABE_SIM_* logs/LOG_SHORESIDE_SIM_*
./launch_shoreside.sh --auto --sim --nogui $TIME_WARP || exit 2
./launch_vehicle.sh --sim --auto --profile=$PROFILE --speeds=$SPEEDS --trials=$TRIALS $EXTRA $TIME_WARP \
    || { ktm >/dev/null 2>&1; exit 2; }
sleep 6
uPokeDB targs/targ_shoreside.moos DEPLOY_ALL=true MOOS_MANUAL_OVERRIDE_ALL=false RETURN_ALL=false >/dev/null 2>&1

echo "$ME: $PROFILE  speeds=$SPEEDS trials=$TRIALS  expecting $EXPECT FT_REPORTs (warp $TIME_WARP)"
T0=$(date +%s); STATUS=1
while :; do
    ALOG=$(ls -t logs/LOG_ABE_SIM_*/*.alog 2>/dev/null | head -1)
    if [ -n "$ALOG" ]; then
        N=$(grep -cE '^\S+\s+FT_REPORT\s' "$ALOG")
        RET=$(grep -E '^\S+\s+RETURN\s' "$ALOG" | awk '$4=="true"' | wc -l | tr -d ' ')
        MODE=$(grep -E '^\S+\s+MODE\s' "$ALOG" | tail -1 | awk '{print $4}')
        printf "\r  t+%4ds  FT_REPORT %d/%d  RETURN=true posts %s  MODE %-28s" $(( $(date +%s) - T0 )) "$N" "$EXPECT" "$RET" "$MODE"
        if [ "$N" -ge "$EXPECT" ] && [ "$RET" -ge 1 ]; then STATUS=0; echo; break; fi
    fi
    if [ $(( $(date +%s) - T0 )) -ge "$MAX_WAIT" ]; then echo; echo "$ME: TIMEOUT waiting for turns"; break; fi
    sleep 3
done
sleep 2
ktm >/dev/null 2>&1

[ -z "$ALOG" ] && { echo "$ME: no vehicle alog produced"; exit 3; }
echo "$ME: log $ALOG"
grep -E '^\S+\s+FT_REPORT\s' "$ALOG" | awk '{printf "  %8.1f  %s\n", $1, $4}'
echo
python3 ../../analysis/turn_radius.py "$ALOG" --sim || STATUS=4
[ $STATUS -eq 0 ] && echo "$ME: PASS ($PROFILE)" || echo "$ME: FAIL ($PROFILE) status $STATUS"
exit $STATUS
