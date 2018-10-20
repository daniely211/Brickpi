# Function to plot figures after each test  

# time, angle_a, angle_aref, angle_b, angle_bref

import csv
import matplotlib.pyplot as plt
import numpy as np
#import glob
#import os


x_points = []
y_points_motor1 = []
y_points_motor2 = []

periods = []

# Get sorted list of all files in log
#f_ls = sorted(glob.glob("/BrickPi/log/*"))

#for i in range( len(f_ls) ):
#  with open(f[i],'r') as f:



f = ".txt"

with open(f, 'r') as tsvin:
	tsvin = csv.reader(tsvin, delimiter='\t')

	i = 0
	init_y = 0
	init_x = 0
	final_x = 0
	sampled_times = 0
	first_time_val = 0
	last_time_val = 0
	
        # base angle : b
        b = tsvin[0][1]

        for row in tsvin:
          # filtered_row : f_row      
          f_row = filter(None, row)
          x_points_motor1.append(float(f_row[0]))
  	  y_points_motor1.append(float(f_row[2]))
          y_points_motor2.append(float(f_row[4]))


# now to plot
plt.plot(y_points_motor1,y_points_motor2)
plt.ylabel('Angle')
plt.xlabel('Time')
plt.savefig('/BrickPi/log/' + f[:-3] +"png")
