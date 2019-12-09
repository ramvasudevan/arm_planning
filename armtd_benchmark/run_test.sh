#!/bin/bash

# bash script for running CHOMP test and collecting data

# Takes in a directory name containing the tests you want to run, or number of obstacles?
# Runs Chomp in one terminal, benchmark in other terminal
# Outputs raw data to file
# terminates programs -- when?? 

echo Running test number $2 for $1 obstacles

cd ~/ws_moveit/
source devel/setup.bash
roslaunch fetch_moveit_config demo_chomp.launch &> ~/ws_moveit/src/armtd_benchmark/chomp_processing/raw/raw_output_0$1_0$2.txt &

child_pid=$!
sleep 7 
rosrun armtd_benchmark armtd_benchmark.py ~/ws_moveit/src/armtd_benchmark/tests/scene_0$1_0$2.csv

kill $child_pid
