" 
Function to plot figures after each test  
"

import csv
import matplotlib.pyplot as plt
import numpy as np
import glob
import os


x_points = []
y_points_motor1 = []
y_points_motor2 = []

periods = []

# Get sorted list of all files in log
f_ls = sorted(glob.glob("/BrickPi/log/*"))

for i in range( len(f_ls) ):
  with open(f[i],'r') as f:





with open('log kp:300.0 ki:500.0 kd:8.0', 'r') as tsvin:
	tsvin = csv.reader(tsvin, delimiter='\t')

	i = 0
	init_y = 0
	init_x = 0
	final_x = 0
	sampled_times = 0
	first_time_val = 0
	last_time_val = 0
	for row in tsvin:
		filtered_row = filter(None, row)
		if i == 1:
			first_time_val = float(filtered_row[0])

		last_time_val = float(filtered_row[0])
		# tsvin2 = csv.reader(tsvin, delimiter='\t')
		# j = 0
		# seen_value = false
		# for row2 in tsvin2:
		# 	if j <= i: continue
		# 	filtered_row2 = filter(None, row)




		if i % 2 == 0:
			x_points.append(filtered_row[0])
			y_points_motor1.append(float(filtered_row[1]) - float(filtered_row[2]))
			y_points_motor2.append(float(filtered_row[3]) - float(filtered_row[4]))

		if i == 10:
			init_y = float(float(filtered_row[1]) - float(filtered_row[2]))
			init_x = float(filtered_row[0])
		if i > 10:
			if abs(float(filtered_row[1]) - float(filtered_row[2]) - init_y) < 0.00001 :
				sampled_times += 1
				final_x = float(filtered_row[0])
				if sampled_times == 2:
					break
