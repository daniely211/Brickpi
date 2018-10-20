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
		# init_y = 0
		# init_x = 0
		# final_x = 0
		# sampled_times = 0
		# first_time_val = 0
		# last_time_val = 0
		b1 = 0
		b2 = 0
		for row in tsvin:
			if i:
				b1 = float(filter(None, row)[2])
				b2 = float(filter(None, row)[4])
				t = float(filter(None, row)[0])
				i = False

			# f_row = filter(None, row)
			# filtered_row : f_row
			f_row = filter(None, row)
			x_points.append(float(f_row[0])-t)
			y_points_motor1.append(float(f_row[2])-b1)
			y_points_motor2.append(float(f_row[4])-b2)

			# if i == 1:
			# 	first_time_val = float(filtered_row[0])
			#
			# last_time_val = float(filtered_row[0])
			# tsvin2 = csv.reader(tsvin, delimiter='\t')
			# j = 0
			# seen_value = false
			# for row2 in tsvin2:
			# 	if j <= i: continue
			# 	filtered_row2 = filter(None, row)




			# if i % 2 == 0:
			# 	x_points.append(filtered_row[0])
			# 	y_points_motor1.append(float(filtered_row[1]) - float(filtered_row[2]))
			# 	y_points_motor2.append(float(filtered_row[3]) - float(filtered_row[4]))
			#
			# if i == 10:
			# 	init_y = float(float(filtered_row[1]) - float(filtered_row[2]))
			# 	init_x = float(filtered_row[0])
			# if i > 10:
			# 	if abs(float(filtered_row[1]) - float(filtered_row[2]) - init_y) < 0.00001 :
			# 		sampled_times += 1
			# 		final_x = float(filtered_row[0])
			# 		if sampled_times == 2:
			# 			break
			# i += 1
		# print 'period:' + str((final_x - init_x))
	plt.figure()
	plt.plot(x_points,y_points_motor1)
	plt.plot(x_points,y_points_motor2)
	plt.ylabel('Angle')
	plt.xlabel('Time')
	plt.xlim((3,5))
	plt.ylim((19.5,20.5))
	# plt.show()
	plt.savefig("./graph/"+ f[5:-3] +"png")



	#
	#
	# x_linspace1 = np.linspace(first_time_val, last_time_val, num=len(y_points_motor1))
	# x_linspace2 = np.linspace(first_time_val, last_time_val, num=len(y_points_motor2))
	#
	# axes = plt.gca()
	#
	# ymin = -.2
	# ymax = .20
	#
	# axes.set_ylim([ymin, ymax])
	# plt.plot(x_linspace1, y_points_motor1)
	# plt.plot(x_linspace2, y_points_motor2)
	# plt.show()

	# py.iplot(data, filename='basic-line')
