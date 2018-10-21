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
motorParams.pidParameters.k_i = 3000
motorParams.pidParameters.k_D = 34

# Wheel in cm
wheel_r = 2.15
wheel_c = 2.15*2*math.pi
# Distance between wheels
wheel_dist = 13.7
interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)

def left90():
	angle = 5.023 #FIX LATER
	interface.increaseMotorAngleReferences(motors,[angle, -angle])

def right90():
    angle = 5.023
    interface.increaseMotorAngleReferences(motors,[-angle, angle])
    while not interface.motorAngleReferencesReached(motors):
      time.sleep(0.1)

def forward(dist):
    angle = 2*math.pi*( dist/wheel_c ) #FIX LATER
    interface.increaseMotorAngleReferences(motors,[angle, angle])
	print("rotating " + angle)
    while not interface.motorAngleReferencesReached(motors):
      print("")


def square():
	#18.6046511628
    for i in range(0,4):
        forward(40)
        left90()

if __name__ == "__main__":

  forward(10)

  right90()
  print("Finished")

interface.terminate()
