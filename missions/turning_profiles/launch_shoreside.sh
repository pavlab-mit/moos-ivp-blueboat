#!/bin/bash
#------------------------------------------------------------
# launch_shoreside.sh -- turning_profiles shoreside (viewer + logs).
#------------------------------------------------------------
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }
on_exit() { echo; echo "$ME: Halting all apps"; kill -- -$$; }
trap on_exit SIGINT

ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE="no"
VERBOSE=""
CONFIRM="yes"
AUTO_LAUNCHED="no"
LAUNCH_GUI="yes"
IP_ADDR="localhost"
MOOS_PORT="9000"
PSHARE_PORT="9200"
XMODE="SHORE"
FORCE_IP=""

for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
        echo "$ME [OPTIONS] [time_warp]"
        echo "  --just_make, -j   --verbose, -v   --noconfirm, -nc   --auto, -a"
        echo "  --nogui, -n       headless (no pMarineViewer)"
        echo "  --sim, -s         sim log file names"
        echo "  --ip=<addr>  --mport=<9000>  --pshare=<9200>"
        exit 0
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then JUST_MAKE="yes"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then VERBOSE="yes"
    elif [ "${ARGI}" = "--noconfirm" -o "${ARGI}" = "-nc" ]; then CONFIRM="no"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ]; then AUTO_LAUNCHED="yes"; CONFIRM="no"
    elif [ "${ARGI}" = "--nogui" -o "${ARGI}" = "-n" ]; then LAUNCH_GUI="no"
    elif [ "${ARGI}" = "--sim" -o "${ARGI}" = "-s" ]; then XMODE="SIM"
    elif [ "${ARGI:0:5}" = "--ip=" ]; then IP_ADDR="${ARGI#--ip=*}"; FORCE_IP="yes"
    elif [ "${ARGI:0:8}" = "--mport=" ]; then MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then PSHARE_PORT="${ARGI#--pshare=*}"
    else
        echo "$ME: Bad Arg:[$ARGI]. Exit Code 1."; exit 1
    fi
done

if [ "${AUTO_LAUNCHED}" = "no" -a "${FORCE_IP}" != "yes" ]; then
    MAYBE_IP_ADDR=`ipaddrs.sh --blunt 2>/dev/null`
    [ $? = 0 ] && [ -n "$MAYBE_IP_ADDR" ] && IP_ADDR=$MAYBE_IP_ADDR
fi

if [ "${CONFIRM}" = "yes" ]; then
    echo "========================================"
    echo " turning_profiles shoreside   IP_ADDR=$IP_ADDR  MOOS_PORT=$MOOS_PORT"
    echo "   PSHARE_PORT=$PSHARE_PORT  GUI=$LAUNCH_GUI  WARP=$TIME_WARP"
    echo -n "Hit any key to continue"
    read ANSWER
fi

mkdir -p targs logs
nsplug meta_shoreside.moos targs/targ_shoreside.moos --strict --force WARP=$TIME_WARP \
       IP_ADDR=$IP_ADDR  MOOS_PORT=$MOOS_PORT  PSHARE_PORT=$PSHARE_PORT \
       LAUNCH_GUI=$LAUNCH_GUI  XMODE=$XMODE  FORCE_IP=$FORCE_IP || exit 5

if [ "${JUST_MAKE}" = "yes" ]; then
    echo "$ME: Targ files made; exiting without launch."; exit 0
fi

echo "Launching Shoreside MOOS Community. WARP=$TIME_WARP"
pAntler targs/targ_shoreside.moos >& /dev/null &
[ "${AUTO_LAUNCHED}" = "yes" ] && exit 0
uMAC targs/targ_shoreside.moos
trap "" SIGINT
kill -- -$$
