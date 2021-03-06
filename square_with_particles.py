import brickpi
import time
from math import cos,sin 
from move import forward,left, right
import random

interface=brickpi.Interface()
interface.initialize()

motors = [0,2]

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


def generate_particles_from_movement(current, D):
      x,y,theta = current[0], current[1], current[2]
      new_points = []
      for i in range(100):
            e = random.gauss(0, 5)
            f = random.gauss(0, 5)
            new_point = (x + (D + e)*cos(theta), y + (D + e)*sin(theta), theta + f)
            new_points.append(new_point)
      return new_points
def generate_particles_from_turn(current, angle):
      x,y,theta = current[0], current[1], current[2]
      new_points = []
      for i in range(100):
            g = random.gauss(0,5)
            new_point = (x, y, theta + angle + g)
            new_points.append(new_point)
      return new_points

def square():
	#18.6046511628
    current =  (0,0,0)
    for i in range(0,4):
        for j in range(4):
                forward(10)
                previous_pos = current
                particles = generate_particles_from_movement(current, 10)
                avgX = sum([x for (x,y,theta) in particles])/100
                avgY = sum([y for (x,y,theta) in particles])/100
                avgTheta = sum([theta for (x,y,theta) in particles])/100
                current = (avgX, avgY,avgTheta)
                line = (previous_pos[0], previous_pos[1], avgX, avgY)
                #plot the points
                print("drawLine:" + str(line))
                print("drawParticles:" + str(particles))
	print("###########################WENT FORWARD#################")
	time.sleep(0.2)        
	left(90)
	print("##############TURNED LEFT######################")
        particles = generate_particles_from_turn(current, 90)
        #plot the new points
        print("drawParticles:" + str(particles))

        time.sleep(0.1)





if __name__ == "__main__":
   square()
#  forward(40)
#  right90()
#  forward(40)

interface.terminate()
