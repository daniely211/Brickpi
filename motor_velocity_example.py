import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [0,2]
speed = -6.0

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 250.0
motorParams.pidParameters.k_i = 400.0
motorParams.pidParameters.K_d = 32.0
kp = motorParams.pidParameters.k_p
ki = motorParams.pidParameters.k_i
kd = motorParams.pidParameters.K_d
interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)

interface.setMotorRotationSpeedReferences(motors,[speed,speed])

print "Press Ctrl+C to exit"
interface.startLogging("/home/pi/Brickpi/log/NewDesign/kp_"+str(kp)+"_ki_"+str(ki)+"_kd_"+str(kd)+".txt")
while True:
	time.sleep(1)


interface.terminate()
