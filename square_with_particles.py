import brickpi
import time
import math
from move import forward,left, right
import random

interface=brickpi.Interface()
interface.initialize()

motors = [0,3]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255

motorParams.pidParameters.k_p = 540
motorParams.pidParameters.k_i = 3000
motorParams.pidParameters.K_d = 34

current =  (0,0,0)

def square():
	#18.6046511628

      for i in range(0,4):
            for i in range(4):
                  forward(10)
                  new_points = []
                  previous_pos = current
                  for i in range(100):
                        e = random.gauss(0, 5)
                        f = random.gauss(0, 5)
                        new_point = (current[0] + (10 + e)*math.cos(current[2]), current[1] + (10 + e)*math.sin(current[2]), current[2] + f)
                        new_points.append(new_point)
                  avgX = sum([x for (x,y,theta) in new_points])/100
                  avgY = sum([y for (x,y,theta) in new_points])/100
                  avgTheta = sum([theta for (x,y,theta) in new_points])/100
                  current = (avgX, avgY,avgTheta)
                  line = (previous_pos[0], previous_pos[1], current[0], current[1])
                  
                  #plot the points here
                  print("drawLine:" + str(line))
                  print("drawParticles:" + str(new_points))

            right(90)
            new_points = []
            for i in range(100):
                  g = random.gauss(0,5)
                  new_point = (current[0], current[1], current[2] + 90 + g)
                  
            #plot the new points here
            print("drawParticles:" + str(new_points))

            time.sleep(0.1)

if __name__ == "__main__":
   square()
#  forward(40)
#  right90()
#  forward(40)

interface.terminate()
