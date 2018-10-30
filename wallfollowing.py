import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

port = 0 # port which ultrasoic sensor is plugged in to

interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC);

kp = 100
zdesired = 30
while True:
        (zactual, extra) = interface.getSensorValue(port)

	if zactual :
        # calculate the speed at which it needs to go at

          speed = -kp*(zdesired - zactual)
          print "speed"+ str(speed)
	  print zactual
	else:
	  print "Failed US reading"

	time.sleep(0.05)

interface.terminate()
