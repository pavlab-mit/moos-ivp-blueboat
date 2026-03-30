#!/bin/bash
#--------------------------------------------------------------
#   <TEMPLATE>
#   Script: post_process.sh
#   Author: Raymond Turrisi
#   LastEd: October 2024
#    Brief: 
#        A template mission post processing script
#        1) gets rid of irrelevant log files 
#        2) uses thin_logdirs as defined in 
#           moos-ivp-pavlab/scripts to get rid of all the 
#           variables which are typically not used in analysis,
#        3) uses mdm and the wrapper script to extract all the 
#           data from all the subdirectories and get them into 
#           csv's and jsons
#        4) uses an example plotting script after using the data
#           the data extracted from the alogs with mdm
#--------------------------------------------------------------


# Strip variables worth ignoring from a mission

mission_directory=$1

rm -rf $mission_directory/*/*.blog $mission_directory/*/*.ylog $mission_directory/*/*.slog

eval thin_logdirs $mission_directory/

mdm_tree_root=$mission_directory/

eval ./mdm/mw_directory_conversion.sh $mdm_tree_root

python3 plot.py ${mdm_tree_root}