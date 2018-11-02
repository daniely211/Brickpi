import math
from move import right,forward 

def navigateToWaypoint(X, Y, current):  #X is desired X,Y is desired Y
    #assuming we have access to our x,y,theta values (position and direction of robot)
    #take dY = Y-y;dX = X-x
    #we need to turn (phi - theta) degrees with phi = atan2(dY,dX).
    #then move forward a distance of sqrt(pow(dY,2)+pow(dX,2))
    dY = Y-current[1]
    dX = X-current[0]
    phi = math.atan2(dY,dX)
    dist = math.sqrt(math.pow(dY,2)+math.pow(dX,2))
    if dX>0:
        angle = phi - current[2] #align with point if dX +ve
    else:
        angle = phi - (current[2]-math.pi) #offset by pi if dX -ve
    right(angle)
    forward(dist) #idk if this is how it works in python


current = (0,0,0)
navigateToWaypoint(10,10,current)