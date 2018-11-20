import brickpi
import time
from math import pi, cos, sin, sqrt, atan2, pow, exp
import random

motors = [3]
speed = -6
current = (0, 0, 0)

interface = brickpi.Interface()
interface.initialize()
interface.sensorEnable(sonar_port, brickpi.SensorType.SENSOR_ULTRASONIC)

interface.motorEnable(motors[0])

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

interface.setMotorAngleControllerParameters(motors[0], motorParams)

def rotateSensor(interface, angle, direction):
    if direction == 'left':
        interface.increaseMotorAngleReferences(motors, [-angle_rads])
    elif direction == 'right':
        interface.increaseMotorAngleReferences(motors, [angle_rads])

    while not interface.motorAngleReferencesReached(motors):
        time.sleep(0.1)

def left(angle, interface):
    rotateSensor(interface, angle, 'left')

def right(angle, interface):
    rotateSensor(interface, angle, 'right')

left(2*pi, interface)