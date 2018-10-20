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
# kp =300, ki=500, kp=8

# Initially should'nt have to touch
motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255

# Adjust PID parameters, Ziegler-Nicholls method
motorParams.pidParameters.k_p = 540
motorParams.pidParameters.k_i = 2160
motorParams.pidParameters.K_d = 33.75

kp = motorParams.pidParameters.k_p
ki = motorParams.pidParameters.k_i
kd = motorParams.pidParameters.K_d

interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)


while True:
	angle = float(input("Enter a angle to rotate (in radians): "))

	# Set both motors to reach angle input
        interface.increaseMotorAngleReferences(motors,[angle,angle])
	# Start the logging to output file...
        interface.startLogging("/home/pi/Brickpi/log/kp_"+str(kp)+"_ki_"+str(ki)+"_kd_"+str(kd)+".txt")
	
        # While we have not reached the the Angle reference on the motors
        while not interface.motorAngleReferencesReached(motors) :
		try:
			# get the motor angle
                        motorAngles = interface.getMotorAngles(motors)
			if motorAngles :
				print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
			# pause execution for 0.1 seconds
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
