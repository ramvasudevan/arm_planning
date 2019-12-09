#!/bin/bash

num_obstacles=$1

for test in {1..3}
do
	./run_test.sh $num_obstacles $test
done