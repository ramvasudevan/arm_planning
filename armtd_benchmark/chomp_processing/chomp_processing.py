#!/usr/bin/env python
import sys
import csv

def main():

	file_path = "~/ws_moveit/src/armtd_benchmark/chomp_processing/processed/"

	filename_in = sys.argv[1]
	test_data = open(filename_in, 'r')

	num_obstacles = filename_in[15] + filename_in[16]
	print num_obstacles

	world = filename_in[18] + filename_in[19]
	print world 


	filename_out = "chomp_test_results_" + num_obstacles + world + ".txt"

	print filename_out

	output_data = open(filename_out, 'w+')

	#reader = csv.reader(test_data)

	lines_in = test_data.readlines()
	

	for line in lines_in:

		if "Chomp path is collision free" in line:
			print "SUCCESS!"
			output_data.write("SUCCESS")



		elif "Chomp path is not collision free!" in line:
			print "FAILURE!"
			output_data.write("FAILURE")

		if "Optimization core" in line:
			words = line.split("finished in")
			time = words[1]
			print time
			output_data.write(time)

		if "Time per iteration" in line:
			words = line.split("iteration")
			time = words[1]
			print time 
			output_data.write(time)


	return

if __name__ == '__main__':
	main()