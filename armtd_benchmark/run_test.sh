#!/bin/bash

# bash script for running CHOMP tests and collecting data

# usage: ./run_test.sh <num_obstables> <test_number>

# Runs Chomp in one terminal, armtd_benchmark.py in other terminal

# Outputs raw data to file. If you want to change the name of the file,
# adjust file_out accordingly

# If the naming scheme of file storage location of test cases changes,
# adjust file_in accordingly

obstacles=$1
test=$2

echo Running test number $test for $obstacles obstacles

# Adjust numbers to work with file naming scheme
if ((obstacles<10))
then
	obstacles=00$obstacles
else
	obstacles=0$obstacles
fi

if((test<10))
then
	test=00$test
else
	test=0$test
fi

file_out=~/ws_moveit/src/armtd_benchmark/chomp_processing/raw/raw_output_$obstacles\_$test.txt
file_in=~/ws_moveit/src/armtd_benchmark/tests/20191205/scene_$obstacles\_$test.csv

# export environment variable because python cannot parse bash args as is
export FILE_IN=$file_in

# source ROS environment if not done yet
cd ~/ws_moveit/
source devel/setup.bash

# launch rviz 
roslaunch fetch_moveit_config demo_chomp.launch &> $file_out &
child_pid=$!
sleep 7 

# Run benchmark
rosrun armtd_benchmark armtd_benchmark.py FILE_IN

# Adjust this sleep for however long you want to view the robot executing the trajectory
# Do not go under about 2 seconds
sleep 2
kill $child_pid
