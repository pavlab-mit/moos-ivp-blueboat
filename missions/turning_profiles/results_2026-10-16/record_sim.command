#!/bin/zsh -l
# Records pMarineViewer while the turning_profiles sim runs, one profile after the other.
# Output: results_2026-10-16/sim_agile.mov and sim_slow.mov (full-screen captures, cropped later).
cd ~/moos/moos-ivp-blueboat/missions/turning_profiles || exit 1
OUT=results_2026-10-16
WARP=4
echo "record_sim: killing any MOOS processes"; ktm >/dev/null 2>&1; sleep 1
run() {   # profile  seconds_to_record
  echo "record_sim: launching $1 (warp $WARP)"
  ./launch_shoreside.sh --auto --sim $WARP || exit 2
  ./launch_vehicle.sh --sim --auto --profile=$1 --speeds=1.0,1.5 --trials=1 $WARP || { ktm; exit 2; }
  sleep 12
  uPokeDB targs/targ_shoreside.moos DEPLOY_ALL=true MOOS_MANUAL_OVERRIDE_ALL=false RETURN_ALL=false >/dev/null 2>&1
  echo "record_sim: DEPLOYED, recording $2 s -> $OUT/sim_$1.mov"
  rm -f $OUT/sim_$1.mov
  screencapture -v -V $2 -x $OUT/sim_$1.mov
  echo "record_sim: done $1"; ktm >/dev/null 2>&1; sleep 3
}
run agile 100
run slow 150
echo "record_sim: ALL DONE"; ls -la $OUT/*.mov
