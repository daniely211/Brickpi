import brickpi
import time
import math

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
motorParams.pidParameters.k_i = 2000
motorParams.pidParameters.K_d = 34

# 

# Wheel in cm
wheel_r = 2.15
wheel_c = 2.15*2*math.pi
# Distance between wheels
wheel_dist = 13.7
interface.setMotorAngleControllerParameters(motors[0],motorParams)

motorParams.pidParameters.k_p = 540
interface.setMotorAngleControllerParameters(motors[1],motorParams)

def left90():
	angle = 5.320895231#FIX LATER
	interface.increaseMotorAngleReferences(motors,[angle, -angle])

def right90():
    angle = 6.1
    #5.180895231
    interface.increaseMotorAngleReferences(motors,[-angle, angle])
    while not interface.motorAngleReferencesReached(motors):
      time.sleep(0.1)

def forward(dist):
    #dist=dist+15.1 #add the length of robot so start point is front rather than back wheels 
    angle = 2*math.pi*( dist/wheel_c )*1.02 #FIX LATER
    interface.increaseMotorAngleReferences(motors,[-angle, -angle])
    print("rotating " + str(angle))
    while not interface.motorAngleReferencesReached(motors):
      time.sleep(0.1)


def square():
	#18.6046511628
    for i in range(0,4):
        forward(40)
        right90()
  

if __name__ == "__main__":
 square()

	
#  forward(40)
#  right90()
#  forward(40)

interface.terminate()
