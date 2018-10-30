import brickpi
import time
from move import left,right,forward

interface = brickpi.Interface()
interface.initialize()

motors = [0,3]
speed = -6.0
left_touch_port = 1
right_touch_port = 2

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255

motorParams.pidParameters.k_p = 100
motorParams.pidParameters.k_i = 0
motorParams.pidParameters.K_d = 0

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)


interface.sensorEnable(left_touch_port, brickpi.SensorType.SENSOR_TOUCH)
interface.sensorEnable(right_touch_port, brickpi.SensorType.SENSOR_TOUCH)

while True:
	interface.setMotorRotationSpeedReferences(motors,[speed,speed])
	time.sleep(0.1)
	left_result = interface.getSensorValue(left_touch_port)
	right_result = interface.getSensorValue(right_touch_port)

	if left_result and right_result:
       		left_touched = left_result[0]
     		right_touched = right_result[0]

            if left_touched and right_touched:
				print "front"

				interface.setMotorPwm(motors[0],0)
				interface.setMotorPwm(motors[1],0)	
				# while not interface.motorRotationSpeedReferenceReached(0):
				# 	time.sleep(0.1)
				time.sleep(1)
				print("stopped")
				forward(-10)
				time.sleep(0.5)
				left(90)
				time.sleep(0.5)
			elif left_touched:
				print "left"

			elif right_touched:
				print "right"
            	
interface.terminate()
