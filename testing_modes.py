import brickpi
import time
import math
from move import forward

interface = brickpi.Interface()
interface.initialize()
speed = -6.0
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

interface.setMotorAngleControllerParameters(motors[0], motorParams)
interface.setMotorAngleControllerParameters(motors[1], motorParams)

forward(30)

interface.setMotorRotationSpeedReferences(motors,[speed,speed])

while not interface.motorAngleReferencesReached(motors) :
    time.sleep(0.1)

