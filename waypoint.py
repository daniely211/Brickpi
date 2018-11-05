import math
import brickpi
from move import right,forward 

interface = brickpi.Interface()
interface.initialize()

motors = [0,2]
left_touch_port = 1
right_touch_port = 2
speed = -6
interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

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
interface.setMotorAngleControllerParameters(motors[1], motorParams)

def navigateToWaypoint(X, Y, current):  #X is desired X,Y is desired Y
    #assuming we have access to our x,y,theta values (position and direction of robot)
    #take dY = Y-y;dX = X-x
    #we need to turn (phi - theta) degrees with phi = atan2(dY,dX).
    #then move forward a distance of sqrt(pow(dY,2)+pow(dX,2))
    dY = Y-current[1]
    dX = X-current[0]
    print(dY)
    print(dX)
    phi = math.atan2(dY,dX)
    dist = math.sqrt(math.pow(dY,2)+math.pow(dX,2))
    if dX>0:
        angle = phi - current[2] #align with point if dX +ve
    else:
        angle = phi - (current[2]-math.pi) #offset by pi if dX -ve
    print(angle)
    print(dist)
    right(angle)
    forward(dist) #idk if this is how it works in python
    current[0]=current[0]+dX
    current[1]=current[1]+dY
    current[3]=current[3]+phi


current = (0,0,0)
navigateToWaypoint(10,10,current)
