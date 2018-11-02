import brickpi 
import time 
from math import pi,cos,sin
import random 
interface = brickpi.Interface()
interface.initialize()

motors = [0,2]
left_touch_port = 1
right_touch_port = 2
speed = -6
interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255

motorParams.pidParameters.k_p = 250
motorParams.pidParameters.k_i = 400
motorParams.pidParameters.K_d = 32

# Units: cm
wheel_radius = 2.15
wheel_circ = 2 * pi * wheel_radius

# Distance between wheels
wheel_dist = 15.0

interface.setMotorAngleControllerParameters(motors[0], motorParams)
interface.setMotorAngleControllerParameters(motors[1], motorParams)

def distance_to_rads(distance):
    delta = 1.02
    return 2 * pi * (distance / wheel_circ) * delta

def rotate(angle, direction):
        full_circ = 2 * pi * (wheel_dist / 2)
        turn_circ = full_circ * (float(angle) / 360)
        angle_rads = distance_to_rads(turn_circ) * 1.178
        if direction == 'left':
            interface.increaseMotorAngleReferences(motors, [angle_rads, -angle_rads])

        elif direction == 'right':
            interface.increaseMotorAngleReferences(motors, [-angle_rads, angle_rads])

        while not interface.motorAngleReferencesReached(motors):
            time.sleep(0.1)

def left(angle):
    rotate(angle, 'left')

def right(angle):
    rotate(angle, 'right')

def forward(dist):
    #dist=dist+15.1 #add the length
    angle = 2*pi*( dist/wheel_circ )*1.07 # add 7% to calibrate
    #adding 0.05% on the left motor below, to make it go straight
    interface.increaseMotorAngleReferences(motors,[-angle*1.0005, -angle]) # offset left wheel to keep straight line
    while not interface.motorAngleReferencesReached(motors):
          
          time.sleep(0.1)



def generate_particles_from_movement(current, D, i):
      x,y,theta = current[0], current[1], current[2]
      new_points = []
      for i in range(100):
            e = random.gauss(0, 3)
            f = random.gauss(0, 0.05)
            if i == 0 or i == 2:
                print("cos:" + str(cos(theta+f)))
                new_point = ((x + 50.0*(D + e)*cos(theta+f)), (y + (D + e)*sin(theta+f)), theta + f)
            else:
                print("sin:" + str(sin(theta+f)))
                new_point = ((x + (D + e)*cos(theta+f)), (y + 5.0*(D + e)*sin(theta+f)), theta + f)
            new_points.append(new_point)
      return new_points
def generate_particles_from_turn(current, angle):
      x,y,theta = current[0], current[1], current[2]
      new_points = []
      for i in range(100):
            g = random.gauss(0,0.1)
            new_point = (x, y, theta + angle + g)
            new_points.append(new_point)
      return new_points

def square():
    #18.6046511628
    current =  (10,10,0)
    for i in range(0,4):
        for j in range(4):
                forward(10)
                particles = generate_particles_from_movement(current, 10.0, i)
                avgX = sum([x for (x,y,theta) in particles])/100
                avgY = sum([y for (x,y,theta) in particles])/100
                avgTheta = sum([theta for (x,y,theta) in particles])/100
                line = (current[0], current[1], avgX, avgY)
                
                current = (avgX, avgY,avgTheta)
                
                

                #plot the points
                print("drawLine:" + str(line))
                print("drawParticles:" + str(particles))
                time.sleep(0.4)        
        left(90)
        current = (current[0], current[1], current[2] + pi/2)
        particles = generate_particles_from_turn(current, pi/2)
        #plot the new points
        print("drawParticles:" + str(particles))

        time.sleep(0.1)


square()
interface.terminate()
