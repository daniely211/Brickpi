import brickpi
import time
from square import forward, left, right

interface = brickpi.Interface()
interface.initialize()

motors = [0,1]
speed = 6.0
left_touch_port = 2
right_touch_port = 3

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 100.0
motorParams.pidParameters.k_i = 0.0
motorParams.pidParameters.k_d = 0.0

interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)

interface.setMotorRotationSpeedReferences(motors,[speed,speed])

interface.sensorEnable(left_touch_port, brickpi.SensorType.SENSOR_TOUCH)
interface.sensorEnable(right_touch_port, brickpi.SensorType.SENSOR_TOUCH)

forward(40)
left(90)
right(180)

print "Press Ctrl+C to exit"
while True:
	time.sleep(1)


interface.terminate()
