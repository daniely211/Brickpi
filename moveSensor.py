import brickpi
import time
from math import pi, cos, sin, sqrt, atan2, pow, exp
import random

motors = [1]
speed = -6
sonar_port = 2
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

motorParams.pidParameters.k_p = 200
motorParams.pidParameters.k_i = 320
motorParams.pidParameters.K_d = 15

interface.setMotorAngleControllerParameters(motors[0], motorParams)

def rotateSensor(interface, angle, direction):
    if direction == 'left':
        interface.increaseMotorAngleReference(motors[0], -angle)
    elif direction == 'right':
        interface.increaseMotorAngleReference(motors[0], angle)

    while not interface.motorAngleReferenceReached(motors[0]):
        usReading = interface.getSensorValue(sonar_port)
        if usReading:
            print(usReading)
        else:
            print("Failed reading")
        time.sleep(0.1)

def left(angle, interface):
    rotateSensor(interface, angle, 'left')

def right(angle, interface):
    rotateSensor(interface, angle, 'right')

left(2*pi, interface)
right(2*pi, interface)
