import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [0,3]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

# 2.09672 - 1.79468 = 0.30204

# ku = 500
# Pu = 0.30204

# kp = 0.6ku, ki = 2kp/Pu, and kd = kpPu/8.
motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 300
motorParams.pidParameters.k_i = 500
motorParams.pidParameters.K_d = 8

kp = motorParams.pidParameters.k_p
ki = motorParams.pidParameters.k_i
kd = motorParams.pidParameters.K_d

interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)


while True:
	angle = float(input("Enter a angle to rotate (in radians): "))

	interface.increaseMotorAngleReferences(motors,[angle,angle])
	interface.startLogging("log kp:"+str(kp)+" ki:"+str(ki)+ " kd:"+str(kd))
	while not interface.motorAngleReferencesReached(motors) :
		try:
			motoxrAngles = interface.getMotorAngles(motors)
			if motorAngles :
				print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
			time.sleep(0.1)
		except Exception as e:
			print("Exception!!!")
			print("stop logging")
			interface.stopLogging()
			raise e
	interface.stopLogging()
	print("stop logging")
	print "Destination reached!"


interface.terminate()
