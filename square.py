import brickpi
import time

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
motorParams.pidParameters.k_p = 300.0
motorParams.pidParameters.k_i = 500.0
motorParams.pidParameters.k_d = 11.0

interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)

def left90degrees():    
	angle = 5 #FIX LATER
	interface.increaseMotorAngleReferences(motors,[-angle, angle])

def right90degrees():
    angle = 5
    interface.increaseMotorAngleReferences(motors,[angle, -angle])

def forward(dist):
    angle = dist #FIX LATER
        interface.increaseMotorAngleReferences(motors,[angle, angle])

def square():
    for i in range(0,4):
        forward(40)
        left90degrees()