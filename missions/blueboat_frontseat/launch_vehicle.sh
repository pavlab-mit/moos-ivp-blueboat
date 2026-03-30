#!/bin/bash
#--------------------------------------------------------------
#   <TEMPLATE>
#   Script: launch_vehicle.sh
#   Author: Raymond Turrisi
#   LastEd: October 2024
#    Brief: 
#         Launches a single heron on the vehicle or in 
#         simulation
#--------------------------------------------------------------
#  Part 1: Declare global var defaults
#--------------------------------------------------------------
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE="no"
VERBOSE="no"
CONFIRM="yes"
AUTO_LAUNCHED="no"
CMD_ARGS=""

IP_ADDR="localhost"
MOOS_PORT="8999"

SHORE_IP="localhost"
SHORE_PSHARE="9200"
PSHARE_PORT="9201"

DEBUG="false"
LOG="false"

# VNAME will be set from hostname only
BSEAT_IP=""  # Will be set based on hostname

MISSION_NAME=""

#--------------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#--------------------------------------------------------------
for ARGI; do
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME [OPTIONS] [time_warp]                        "
	echo "                                                 " 
	echo "Options:                                         "
	echo "  --help, -h                                     " 
	echo "    Print this help message and exit             "
	echo "  --just_make, -j                                " 
	echo "    Just make targ files, but do not launch      "
	echo "  --verbose, -v                                  " 
	echo "    Verbose output, confirm before launching     "
	echo "  --noconfirm, -nc                               " 
	echo "    No confirmation before launching             "
        echo "  --auto, -a                                     "
        echo "     Auto-launched by a script.                  "
        echo "     Will not launch uMAC as the final step.     "
	echo "                                                 "
	echo "  --ip=<localhost>                               " 
	echo "    Force pHostInfo to use this IP Address       "
	echo "  --mport=<8999>                                 "
	echo "    Port number of this vehicle's MOOSDB port    "
	echo "                                                 "
	exit 0;
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
        JUST_MAKE="yes"
    elif [ "${ARGI}" = "--log" -o "${ARGI}" = "-l" ]; then
        LOG="true"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="yes"
    elif [ "${ARGI}" = "--noconfirm" -o "${ARGI}" = "-nc" ]; then
        CONFIRM="no"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ]; then
        AUTO_LAUNCHED="yes"
    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:7}" = "--mport" ]; then
        MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:8}" = "--mname=" ]; then
        MISSION_NAME="${ARGI#--mname=*}"
    elif [ "${ARGI:0:8}" = "--shore=" ]; then
        SHORE_IP="${ARGI#--shore=*}"
        DEBUG="true"
    else
        echo "$ME: Bad Arg:[$ARGI]. Exit Code 1."
        exit 1
    fi
done

#--------------------------------------------------------------
#  Part 3: Get hostname and set IP addresses
#--------------------------------------------------------------
# Get the hostname
HOSTNAME=$(hostname)

# Map hostname to vehicle name and IP addresses
case "${HOSTNAME}" in
        "zoe-bb")
            VNAME="zoe"
            IP_ADDR="10.31.1.1"
            BSEAT_IP="10.31.1.100"
            BCAST_IP="10.31.1.255"
            ;;
        "yip-bb")
            VNAME="yip"
            IP_ADDR="10.32.1.1"
            BSEAT_IP="10.32.1.100"
            BCAST_IP="10.32.1.255"
            ;;
        "xai-bb")
            VNAME="xai"
            IP_ADDR="10.33.1.1"
            BSEAT_IP="10.33.1.100"
            BCAST_IP="10.33.1.100"
            ;;
        "wes-bb")
            VNAME="wes"
            IP_ADDR="192.168.34.1"
            BSEAT_IP="192.168.34.100"
            BCAST_IP="192.168.34.255"
            ;;
        "via")
            VNAME="via"
            IP_ADDR="192.168.35.1"
            BSEAT_IP="192.168.35.100"
            BCAST_IP="192.168.35.255"
            ;;
        "uma")
            VNAME="uma"
            IP_ADDR="192.168.36.1"
            BSEAT_IP="192.168.36.100"
            BCAST_IP="192.168.36.255"
            ;;
        "t--")
            VNAME="t--"
            IP_ADDR="192.168.37.1"
            BSEAT_IP="192.168.37.100"
            BCAST_IP="192.168.37.255"
            ;;
        "s--")
            VNAME="s--"
            IP_ADDR="192.168.38.1"
            BSEAT_IP="192.168.38.100"
            BCAST_IP="192.168.38.255"
            ;;
        "r--")
            VNAME="r--"
            IP_ADDR="192.168.39.1"
            BSEAT_IP="192.168.39.100"
            BCAST_IP="192.168.39.255"
            ;;
        "q--")
            VNAME="q--"
            IP_ADDR="192.168.40.1"
            BSEAT_IP="192.168.40.100"
            BCAST_IP="192.168.40.255"
            ;;
        *)
            echo "$ME: Unknown hostname '${HOSTNAME}'. Exit Code 2"
            echo "Valid hostnames: zoe, yan, xai, wil, via, uma, t--, s--, r--, q--"
            exit 2
            ;;
esac

#---------------------------------------------------------------
#  Part 4: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" = "yes" -o "${CONFIRM}" = "yes" ]; then 
    echo "$ME"
    echo "CMD_ARGS =      [${CMD_ARGS}]     "
    echo "TIME_WARP =     [${TIME_WARP}]    "
    echo "AUTO_LAUNCHED = [${AUTO_LAUNCHED}]"
    echo "----------------------------------"
    echo "MOOS_PORT =     [${MOOS_PORT}]    "
    echo "IP_ADDR =       [${IP_ADDR}]      "
    echo "----------------------------------"
    echo "VNAME =         [${VNAME}]        "
    echo "BSEAT_IP =      [${BSEAT_IP}]     "
    echo "----------------------------------"
    echo -n "Hit any key to continue with launching"
    read ANSWER
fi


#--------------------------------------------------------------
#  Part 5: Create the .moos and .bhv files. 
#--------------------------------------------------------------

if [ "$MISSION_NAME" = "" ]; then
    MISSION_NAME=$(mhash_gen)
    mkdir logs/$MISSION_NAME
fi

NSFLAGS="-s -f"
if [ "${AUTO_LAUNCHED}" = "no" ]; then
    NSFLAGS="-i -f"
fi

nsplug meta_vehicle.moos targs/targ_$VNAME.moos $NSFLAGS \
    VNAME=$VNAME \
    IP_ADDR=$IP_ADDR \
    BSEAT_IP=$BSEAT_IP \
    BCAST_IP=$BCAST_IP \
    MOOS_PORT=$MOOS_PORT \
    MISSION_NAME=$MISSION_NAME\
    DEBUG=$DEBUG\
    LOG=$LOG\
    SHORE_IP=$SHORE_IP\
    SHORE_PSHARE=$SHORE_PSHARE\
    PSHARE_PORT=$PSHARE_PORT

if [ ${JUST_MAKE} = "yes" ] ; then
    echo "Files assembled; nothing launched; exiting per request."
    exit 0
fi


#--------------------------------------------------------------
#  Part 6: Launch the processes
#--------------------------------------------------------------

echo "Launching $VNAME MOOS Community. WARP="$TIME_WARP
if [ "${AUTO_LAUNCHED}" = "yes" ]; then
  exec pAntler targs/targ_${VNAME}.moos
else
  pAntler targs/targ_${VNAME}.moos >& /dev/null &
fi
echo "Done Launching $VNAME MOOS Community"

#---------------------------------------------------------------
#  Part 7: If launched from script, we're done, exit now
#---------------------------------------------------------------
if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

#---------------------------------------------------------------
# Part 8: Launch uMAC until the mission is quit
#---------------------------------------------------------------
uMAC targs/targ_$VNAME.moos
kill -- -$$
