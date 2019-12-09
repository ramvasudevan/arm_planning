#!/usr/bin/env python
import sys
import csv

def main():

	file_path = "processed/"

	filename_in = sys.argv[1]
	test_data = open(filename_in, 'r')

	num_obstacles = filename_in[15] + filename_in[16]
	print num_obstacles

	world = filename_in[18] + filename_in[19]
	print world 


	#filename_out = "chomp_test_results_" + num_obstacles + world + ".txt"
	filename_out = file_path + "chomp_test_results_" + num_obstacles + world + ".csv"

	print filename_out

	output_data = open(filename_out, 'w+')
	output_csv = csv.writer(output_data)

	#reader = csv.reader(test_data)

	lines_in = test_data.readlines() 
	row = []
	

	for line in lines_in:

		if "Chomp path is collision free" in line:
			print "SUCCESS!"
			#output_data.write("SUCCESS")
			row.append(1)



		elif "Chomp path is not collision free!" in line:
			print "FAILURE!"
			#output_data.write("FAILURE")
			row.append(0)

		if "Optimization core" in line:
			words = line.split("finished in")
			time = words[1].split("sec")
			#time = time[:9]

			print time
			#output_data.write(time)
			row.append(time[0][1:9])

		if "Time per iteration" in line:
			words = line.split("iteration")
			#time = words[1].slice(6)

			time = words[1]

			time = time[1:9]

			print time 
			#output_data.write(time)
			row.append(time)
	print "ROW"
	print row
	output_csv.writerow(row)
	return

if __name__ == '__main__':
	main()