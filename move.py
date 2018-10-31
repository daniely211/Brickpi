import brickpi
import time
import math

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

motorParams.pidParameters.k_p = 540
motorParams.pidParameters.k_i = 2000
motorParams.pidParameters.K_d = 34

# Units: cm
wheel_radius = 2.15
wheel_circ = 2 * math.pi * wheel_radius

# Distance between wheels
wheel_dist = 15

interface.setMotorAngleControllerParameters(motors[0], motorParams)
interface.setMotorAngleControllerParameters(motors[1], motorParams)

def distance_to_rads(distance):
    delta = 1.02
    return 2 * math.pi * (distance / wheel_circ) * delta

def rotate(angle, direction):
    	full_circ = 2 * math.pi * (wheel_dist / 2)
    	turn_circ = full_circ * (angle / 360)
    	angle_rads = distance_to_rads(turn_circ)

    	if direction == 'left':
        	interface.increaseMotorAngleReferences(motors, [angle, -angle])
    	elif direction == 'right':
        	interface.increaseMotorAngleReferences(motors, [-angle, angle])

	while not interface.motorAngleReferencesReached(motors):
		time.sleep(0.1)

def left(angle):
    rotate(angle, 'left')

def right(angle):
    rotate(angle, 'right')

def forward(dist):
    #dist=dist+15.1 #add the length
    angle = 2*math.pi*( dist/wheel_circ )*1.02 #FIX LATER
    interface.increaseMotorAngleReferences(motors,[-angle+0.1, -angle]) # offset left wheel to keep straight line
    print("rotating " + str(angle))
    while not interface.motorAngleReferencesReached(motors):
          motorAngles = interface.getMotorAngles(motors)
          print(motorAngles[0][0])
          print(motorAngles[1][0])
          print(interface.getMotorAngleReferences(motors))
          time.sleep(0.1)

interface.terminate()
