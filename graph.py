import csv
import matplotlib.pyplot as plt
import numpy as np
import glob

f_ls = sorted(glob.glob("./log/*"))

for f in f_ls:
	periods = []
	x_points = []
	y_points_motor1 = []
	y_points_motor2 = []
	t = 0

	with open(f, 'r') as tsvin:
		tsvin = csv.reader(tsvin, delimiter='\t')
		i = True
		b1 = 0
		b2 = 0
		for row in tsvin:
			if i:
				b1 = float(filter(None, row)[2])
				b2 = float(filter(None, row)[4])
				t = float(filter(None, row)[0])
				i = False

			f_row = filter(None, row)
			x_points.append(float(f_row[0])-t)
			y_points_motor1.append(float(f_row[2])-b1)
			y_points_motor2.append(float(f_row[4])-b2)

	plt.figure()
	plt.plot(x_points,y_points_motor1)
	plt.plot(x_points,y_points_motor2)
	plt.ylabel('Angle')
	plt.xlabel('Time')
	plt.xlim((3,5))
	plt.ylim((19.8,20.2))
	# plt.show()
	plt.savefig("./graph/"+ f[5:-3] +"png")
