#!/bin/bash
# Remove logs and generated targets. Generated plugs are rebuilt by the launcher.
rm -rf logs/LOG_* logs/C*
rm -f  targs/targ_* *~ .LastOpenedMOOSLogDirectory
rm -f  plugs/plug_turn_schedule.bhv plugs/plug_opregion.bhv
