#!/bin/bash
#------------------------------------------------------------
# launch_vehicle.sh -- turning_profiles back-seat mission.
#
# Boat identity from get_robot_info.sh as in alpha; --sim for the
# actuator-mode simulation on a laptop. --profile is REQUIRED and
# selects the pBBPID capability profile (agile | slow); everything
# else about the schedule has a default that fits one Friday.
#------------------------------------------------------------
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE="no"
CONFIRM="yes"
AUTO_LAUNCHED="no"
XMODE="BBOAT"
SYNC_PLUG="no"

IP_ADDR="localhost"
MOOS_PORT="9001"
PSHARE_PORT="9201"
SHORE_IP="localhost"
SHORE_PSHARE="9200"
VNAME=""
COLOR="yellow"
FSEAT_IP=""
START_POS="15,-47,65"       # sim: SW of the leg's p1, pointing up the leg
RETURN_POS=""
MAX_SPD="3"

PROFILE=""
SPEEDS="0.75,1.0,1.25,1.5"
TRIALS="2"
DIR="star"
MOD_HDG="90"
LEG="45,-40,40,65"
TURN_RAD=""      # per-profile default below
TIMEOUT=""       # per-profile default below
CHECK_R=""       # per-profile default below
MARGIN="12"
GEN_FORCE=""

for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
        echo "$ME --profile=agile|slow [OPTIONS] [time_warp]"
        echo "  --profile=<agile|slow>  REQUIRED. pBBPID capability profile:"
        echo "                    agile = max_yawrate 3.0 rad/s (uncapped in practice)"
        echo "                    slow  = max_yawrate 0.10 rad/s (5.7 deg/s)"
        echo "  --speeds=<csv>    trial speeds m/s          (default $SPEEDS)"
        echo "  --trials=<n>      turns per speed           (default $TRIALS)"
        echo "  --dir=<star|port> fixed-turn direction      (default $DIR)"
        echo "  --mod_hdg=<deg>   heading offset in turn    (default $MOD_HDG)"
        echo "  --leg=<x,y,len,ang> racetrack leg           (default $LEG)"
        echo "  --turn_rad=<m>    leg-run end-turn radius   (default agile 7 / slow 18)"
        echo "  --timeout=<s>     per-turn timeout          (default agile 75 / slow 180)"
        echo "  --check_r=<m>     radius for the op-region clearance check"
        echo "                                              (default agile 6 / slow 16)"
        echo "  --margin=<m>      required clearance        (default $MARGIN)"
        echo "  --gen_force       write the schedule even if the geometry check fails"
        echo "  --sim, -s         laptop simulation (uSimMarineV22 actuator mode)"
        echo "  --start=<X,Y,HDG> sim start pos             (default $START_POS)"
        echo "  --return=<X,Y>    return point              (default: boat info / start)"
        echo "  --vname=<name>  --fseat=<ip>  --ip=<addr>  --mport=<$MOOS_PORT>"
        echo "  --pshare=<$PSHARE_PORT>  --shore=<ip>  --shore_pshare=<$SHORE_PSHARE>"
        echo "  --sync-plug       re-copy alpha's pBBPID block into plugs/plug_pBBPID.moos"
        echo "  --just_make, -j   targ files only     --noconfirm, -nc     --auto, -a"
        exit 0
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then JUST_MAKE="yes"
    elif [ "${ARGI}" = "--noconfirm" -o "${ARGI}" = "-nc" ]; then CONFIRM="no"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ]; then AUTO_LAUNCHED="yes"; CONFIRM="no"
    elif [ "${ARGI}" = "--sim" -o "${ARGI}" = "-s" ]; then XMODE="SIM"
    elif [ "${ARGI}" = "--sync-plug" ]; then SYNC_PLUG="yes"
    elif [ "${ARGI}" = "--gen_force" ]; then GEN_FORCE="--force"
    elif [ "${ARGI:0:10}" = "--profile=" ]; then PROFILE="${ARGI#--profile=*}"
    elif [ "${ARGI:0:9}" = "--speeds=" ]; then SPEEDS="${ARGI#--speeds=*}"
    elif [ "${ARGI:0:9}" = "--trials=" ]; then TRIALS="${ARGI#--trials=*}"
    elif [ "${ARGI:0:6}" = "--dir=" ]; then DIR="${ARGI#--dir=*}"
    elif [ "${ARGI:0:10}" = "--mod_hdg=" ]; then MOD_HDG="${ARGI#--mod_hdg=*}"
    elif [ "${ARGI:0:6}" = "--leg=" ]; then LEG="${ARGI#--leg=*}"
    elif [ "${ARGI:0:11}" = "--turn_rad=" ]; then TURN_RAD="${ARGI#--turn_rad=*}"
    elif [ "${ARGI:0:10}" = "--timeout=" ]; then TIMEOUT="${ARGI#--timeout=*}"
    elif [ "${ARGI:0:10}" = "--check_r=" ]; then CHECK_R="${ARGI#--check_r=*}"
    elif [ "${ARGI:0:9}" = "--margin=" ]; then MARGIN="${ARGI#--margin=*}"
    elif [ "${ARGI:0:8}" = "--start=" ]; then START_POS="${ARGI#--start=*}"
    elif [ "${ARGI:0:9}" = "--return=" ]; then RETURN_POS="${ARGI#--return=*}"
    elif [ "${ARGI:0:8}" = "--vname=" ]; then VNAME="${ARGI#--vname=*}"
    elif [ "${ARGI:0:8}" = "--fseat=" ]; then FSEAT_IP="${ARGI#--fseat=*}"
    elif [ "${ARGI:0:5}" = "--ip=" ]; then IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:8}" = "--mport=" ]; then MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then PSHARE_PORT="${ARGI#--pshare=*}"
    elif [ "${ARGI:0:8}" = "--shore=" ]; then SHORE_IP="${ARGI#--shore=*}"
    elif [ "${ARGI:0:15}" = "--shore_pshare=" ]; then SHORE_PSHARE="${ARGI#--shore_pshare=*}"
    else
        echo "$ME: Bad Arg: [$ARGI]. Exit Code 1."; exit 1
    fi
done

#------------------------------------------------------------
# Profile: required, and sets the geometry defaults.
#------------------------------------------------------------
case "$PROFILE" in
    agile) [ -z "$TURN_RAD" ] && TURN_RAD=7;  [ -z "$TIMEOUT" ] && TIMEOUT=75;  [ -z "$CHECK_R" ] && CHECK_R=6 ;;
    slow)  [ -z "$TURN_RAD" ] && TURN_RAD=18; [ -z "$TIMEOUT" ] && TIMEOUT=180; [ -z "$CHECK_R" ] && CHECK_R=16 ;;
    *) echo "$ME: --profile=agile|slow is required. Exit 1."; exit 1 ;;
esac

#------------------------------------------------------------
# pBBPID plug: the verbatim section must match alpha's plug.
#------------------------------------------------------------
ALPHA_PLUG=../alpha_pavlab_asv/plugs/blueboat/plug_pBBPID.moos
LOCAL_PLUG=plugs/plug_pBBPID.moos
# alpha body = the lines inside alpha's braces; local verbatim = between markers.
alpha_body() { awk 'f&&/^}/{exit} f{print} /^ProcessConfig *= *pBBPID/{getline; f=1}' $ALPHA_PLUG; }
verbatim()   { sed -n '/=== BEGIN ALPHA VERBATIM ===/,/=== END ALPHA VERBATIM ===/p' $LOCAL_PLUG | sed '1d;$d'; }
if [ "$SYNC_PLUG" = "yes" ]; then
    HEAD=$(sed -n '1,/=== BEGIN ALPHA VERBATIM ===/p' $LOCAL_PLUG)
    TAIL=$(sed -n '/=== END ALPHA VERBATIM ===/,$p' $LOCAL_PLUG)
    { echo "$HEAD"; alpha_body; echo "$TAIL"; } > $LOCAL_PLUG.new && mv $LOCAL_PLUG.new $LOCAL_PLUG
    echo "$ME: re-synced $LOCAL_PLUG from $ALPHA_PLUG"
fi
if ! diff -q <(alpha_body) <(verbatim) >/dev/null; then
    echo "$ME: $LOCAL_PLUG has DRIFTED from $ALPHA_PLUG:"
    diff <(alpha_body) <(verbatim) | head -20
    echo "$ME: run '$ME --sync-plug ...' to re-copy. Exit 4."
    exit 4
fi

#------------------------------------------------------------
# Identity
#------------------------------------------------------------
if [ "$XMODE" = "BBOAT" ]; then
    if command -v get_robot_info.sh >/dev/null 2>&1; then
        [ -z "$VNAME" ]      && VNAME=`get_robot_info.sh --name`
        [ -z "$FSEAT_IP" ]   && FSEAT_IP=`get_robot_info.sh --fseat`
        [ "$IP_ADDR" = "localhost" ] && IP_ADDR=`get_robot_info.sh --ip`
        [ -z "$RETURN_POS" ] && RETURN_POS=`get_robot_info.sh --return 2>/dev/null`
        COLOR=`get_robot_info.sh --color 2>/dev/null || echo $COLOR`
    fi
    [ -z "$VNAME" ]    && { echo "$ME: no VNAME (no get_robot_info.sh?); use --vname=. Exit 2."; exit 2; }
    [ -z "$FSEAT_IP" ] && { echo "$ME: no FSEAT_IP; use --fseat=. Exit 2."; exit 2; }
    [ -z "$RETURN_POS" ] && RETURN_POS="10,-10"
else
    [ -z "$VNAME" ] && VNAME="abe"
    FSEAT_IP="localhost"
    [ -z "$RETURN_POS" ] && RETURN_POS="${START_POS%,*}"
fi

#------------------------------------------------------------
# Generate the schedule + op region (geometry-checked).
#------------------------------------------------------------
mkdir -p plugs targs logs
python3 gen_turn_schedule.py --speeds "$SPEEDS" --trials "$TRIALS" --dir "$DIR" \
    --mod-hdg "$MOD_HDG" --timeout "$TIMEOUT" --leg "$LEG" --turn-rad "$TURN_RAD" \
    --check-radius "$CHECK_R" --margin "$MARGIN" $GEN_FORCE \
    --out plugs/plug_turn_schedule.bhv --out-opregion plugs/plug_opregion.bhv || exit 3

if [ "${CONFIRM}" = "yes" ]; then
    echo "=========================================="
    echo " turning_profiles       $VNAME   PROFILE = $PROFILE   XMODE = $XMODE"
    echo "   speeds $SPEEDS  trials $TRIALS  dir $DIR  mod_hdg $MOD_HDG"
    echo "   leg $LEG  turn_rad $TURN_RAD  timeout $TIMEOUT"
    echo "   IP_ADDR $IP_ADDR  FSEAT_IP $FSEAT_IP  SHORE_IP $SHORE_IP  RETURN $RETURN_POS"
    echo " Start the boat SW of the leg (near p1), pointing up the leg,"
    echo " BEFORE pressing DEPLOY. Reposition with CH6, not the kill switch."
    echo "=========================================="
    echo -n "Hit any key to continue"
    read ANSWER
fi

NSFLAGS="--strict --force"
nsplug meta_vehicle.moos targs/targ_$VNAME.moos $NSFLAGS WARP=$TIME_WARP \
       IP_ADDR=$IP_ADDR         MOOS_PORT=$MOOS_PORT       PSHARE_PORT=$PSHARE_PORT \
       SHORE_IP=$SHORE_IP       SHORE_PSHARE=$SHORE_PSHARE VNAME=$VNAME     \
       COLOR=$COLOR             XMODE=$XMODE               FSEAT_IP=$FSEAT_IP \
       START_POS=$START_POS     MAX_SPD=$MAX_SPD           PROFILE=$PROFILE \
       MISSION_NAME=turning_profiles || exit 5

nsplug meta_vehicle.bhv targs/targ_$VNAME.bhv $NSFLAGS \
       VNAME=$VNAME  RETURN_POS=$RETURN_POS  PROFILE=$PROFILE  XMODE=$XMODE || exit 5

if [ "${JUST_MAKE}" = "yes" ]; then
    echo "$ME: targ files made; exiting without launch."; exit 0
fi

echo "Launching $VNAME turning_profiles ($PROFILE). WARP=$TIME_WARP"
pAntler targs/targ_$VNAME.moos >& /dev/null &
[ "${AUTO_LAUNCHED}" = "yes" ] && exit 0
uMAC targs/targ_$VNAME.moos
kill -- -$$
