import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [0,3]
speed = 6.0

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
interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)

port = 0 # port which ultrasoic sensor is plugged in to
interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC);

kp = 0.8
zdesired = 30
while True:
        (zactual, extra) = interface.getSensorValue(port)

	if zactual :
        # calculate the speed at which it needs to go at

          speed = -kp*(zdesired - zactual)
          interface.setMotorRotationSpeedReferences(motors,[-speed,-speed])

          print "speed"+ str(speed)
	  print zactual
	else:
	  print "Failed US reading"

	time.sleep(0.05)

interface.terminate()
