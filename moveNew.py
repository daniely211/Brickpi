import brickpi
import time
import math

interface=brickpi.Interface()
interface.initialize()

motors = [0,3]
speed = -6.0
left_touch_port = 1
right_touch_port = 2

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])
interface.sensorEnable(left_touch_port, brickpi.SensorType.SENSOR_TOUCH)
interface.sensorEnable(right_touch_port, brickpi.SensorType.SENSOR_TOUCH)


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

kp = motorParams.pidParameters.k_p
ki = motorParams.pidParameters.k_i
kd = motorParams.pidParameters.K_d

# Units: cm
wheel_radius = 2.15
wheel_circ = 2 * math.pi * wheel_radius

# Distance between wheels
wheel_dist = 15

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
    interface.setMotorAngleControllerParameters(motors[0],motorParams)
    interface.setMotorAngleControllerParameters(motors[1],motorParams)

    angle = 2*math.pi*( dist/wheel_circ )*1.02 #FIX LATER

    interface.increaseMotorAngleReferences(motors,[-angle,-angle])

        # While we have not reached the the Angle reference on the motors
    while not interface.motorAngleReferencesReached(motors) :
        try:
            motorAngles = interface.getMotorAngles(motors)
            if motorAngles :
        		# print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
        		# pause execution for 0.1 seconds
              time.sleep(0.1)
        except Exception as e:
        	print("Exception!!!")
        	print("stop logging")
        	interface.stopLogging()
        	raise e
while True:
    time.sleep(0.1)
    left_result = interface.getSensorValue(left_touch_port)
    right_result = interface.getSensorValue(right_touch_port)
    if left_result and right_result:
        left_touched = left_result[0]
        right_touched = right_result[0]
    	if left_touched and right_touched:
            print("FRONT!!!!")
            forward(-10)
            right(90)


interface.terminate()
