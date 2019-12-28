#!/bin/bash

# Run an entire batch of tests for x obstacles
# If you have more than 10 tests, adjust the number
# range in the for loop

# Usage: ./run_batch_tests.sh <num_obstacles>

num_obstacles=$1

for test in {1..10}
do
	./run_test.sh $num_obstacles $test
	sleep 2
done


