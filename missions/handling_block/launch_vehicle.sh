#!/bin/bash
#------------------------------------------------------------
# launch_vehicle.sh -- handling_block back-seat mission.
#
# Mirrors alpha_pavlab_asv's conventions (get_robot_info.sh for
# identity on a real boat, same nsplug macro names) but with no
# helm: uTimerScript drives pBBPID directly.
#------------------------------------------------------------
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE="no"
CONFIRM="yes"

IP_ADDR="localhost"
MOOS_PORT="9000"
PSHARE_PORT="9201"
SHORE_IP="localhost"
SHORE_PSHARE="9200"
VNAME=""
COLOR="yellow"
FSEAT_IP=""
BASE_HDG="150"

for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
        echo "$ME [OPTIONS]"
        echo "  --base=<150>      base heading for the block (regenerates script)"
        echo "  --vname=<name>    community name (default: from get_robot_info.sh)"
        echo "  --fseat=<ip>      front-seat IP (default: from get_robot_info.sh)"
        echo "  --ip=<addr>  --mport=<9000>  --shore=<ip>"
        echo "  --just_make, -j   targ files only"
        echo "  --noconfirm, -nc"
        exit 0
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then JUST_MAKE="yes"
    elif [ "${ARGI}" = "--noconfirm" -o "${ARGI}" = "-nc" ]; then CONFIRM="no"
    elif [ "${ARGI:0:7}" = "--base=" ];  then BASE_HDG="${ARGI#--base=*}"
    elif [ "${ARGI:0:8}" = "--vname=" ]; then VNAME="${ARGI#--vname=*}"
    elif [ "${ARGI:0:8}" = "--fseat=" ]; then FSEAT_IP="${ARGI#--fseat=*}"
    elif [ "${ARGI:0:5}" = "--ip=" ];    then IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:8}" = "--mport=" ]; then MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:8}" = "--shore=" ]; then SHORE_IP="${ARGI#--shore=*}"
    else
        echo "$ME: Bad Arg: [$ARGI]. Exit Code 1."; exit 1
    fi
done

# Boat identity, same source alpha uses.
if command -v get_robot_info.sh >/dev/null 2>&1; then
    [ -z "$VNAME" ]    && VNAME=`get_robot_info.sh --name`
    [ -z "$FSEAT_IP" ] && FSEAT_IP=`get_robot_info.sh --fseat`
    [ "$IP_ADDR" = "localhost" ] && IP_ADDR=`get_robot_info.sh --ip`
fi
if [ -z "$VNAME" ]; then
    echo "$ME: no VNAME (no get_robot_info.sh?); use --vname=. Exit 2."; exit 2
fi
if [ -z "$FSEAT_IP" ]; then
    echo "$ME: no FSEAT_IP; use --fseat=. Exit 2."; exit 2
fi

# Regenerate the maneuver script for the requested base heading.
mkdir -p plugs targs logs
python3 gen_maneuver.py --base $BASE_HDG --out plugs/plug_uTimerScript.moos || exit 3

if [ "${CONFIRM}" = "yes" ]; then
    echo "=========================================="
    echo " handling_block          $VNAME"
    echo "   BASE_HDG  = $BASE_HDG"
    echo "   IP_ADDR   = $IP_ADDR   FSEAT_IP = $FSEAT_IP"
    echo "   MOOS_PORT = $MOOS_PORT SHORE_IP = $SHORE_IP"
    echo " No helm: uTimerScript drives pBBPID. The block only"
    echo " advances while BB_CMD_AUTHORITY=AUTONOMY (CH6 = AUTO)."
    echo "=========================================="
    echo -n "Hit any key to continue"
    read ANSWER
fi

nsplug meta_vehicle.moos targs/targ_$VNAME.moos --strict --force \
       WARP=$TIME_WARP            IP_ADDR=$IP_ADDR       \
       MOOS_PORT=$MOOS_PORT       PSHARE_PORT=$PSHARE_PORT \
       SHORE_IP=$SHORE_IP         SHORE_PSHARE=$SHORE_PSHARE \
       VNAME=$VNAME               COLOR=$COLOR           \
       XMODE=BBOAT                VEHICLE_TYPE=kayak     \
       FSEAT_IP=$FSEAT_IP         MISSION_NAME=handling_block

if [ "${JUST_MAKE}" = "yes" ]; then
    echo "$ME: targ files made; exiting without launch."; exit 0
fi

echo "Launching $VNAME handling_block."
pAntler targs/targ_$VNAME.moos >& /dev/null &
uMAC targs/targ_$VNAME.moos
kill -- -$$
