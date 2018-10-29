import csv
import matplotlib.pyplot as plt
import numpy as np
import glob

f_ls = sorted(glob.glob("./log/NewDesign/*"))

for f in f_ls:
	periods = []
	time = []

	acutal_angle_motor1 = []
	actual_angle_motor2 = []

	reference_angle_motor1 = []

	difference_angle_motor1 = []
	difference_angle_motor2 = []

	init_t = 0

	with open(f, 'r') as tsvin:
		tsvin = csv.reader(tsvin, delimiter='\t')
		i = True
		init_actual_motor1 = 0
		init_actual_motor2 = 0
		init_reference_motor1 = 0

		for row in tsvin:
			f_row = filter(None, row)
			if i:
				init_actual_motor1 = float(f_row[2])
				init_actual_motor2 = float(f_row[4])
				init_reference_motor1 = float(f_row[1])
				init_t = float(f_row[0])
				i = False

			time.append(float(f_row[0]) - init_t)

			acutal_angle_motor1.append(float(f_row[2]) - init_actual_motor1)
			actual_angle_motor2.append(float(f_row[4]) - init_actual_motor2)

			reference_angle_motor1.append(float(f_row[1]) - init_reference_motor1)

			difference_angle_motor1.append(float(f_row[1]) - float(f_row[2]))
			difference_angle_motor2.append(float(f_row[3]) - float(f_row[4]))


	plt.figure()
	plt.plot(time,acutal_angle_motor1, label="actual motor1")
	plt.plot(time,actual_angle_motor2, label="actual motor2")
	plt.plot(time,reference_angle_motor1, label="reference angle")

	plt.legend()
	plt.ylabel('Angle')
	plt.xlabel('Time')
	plt.xlim((6,6.4))
	plt.ylim((49.9,50))
	plt.savefig("./graph/NewDesign/PID_tuning/angle/"+ f[16:-3] +"png")

	plt.figure()
	plt.plot(time,difference_angle_motor1, label="error motor1")
	plt.plot(time,difference_angle_motor2, label="error motor2")
	plt.legend()
	plt.ylabel('Angle')
	plt.xlabel('Time')
	plt.xlim((3,5))
	plt.ylim((-0.05,0.1))

	plt.savefig("./graph/NewDesign/PID_tuning/error/"+ f[16:-3] +"png")
