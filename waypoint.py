def navigateToWaypoint(X, Y):  #X is desired X,Y is desired Y
    #assuming we have access to our x,y,theta values (position and direction of robot)
    #take dY = Y-y;dX = X-x
    #we need to turn (phi - theta) degrees with phi = atan2(dY,dX).
    #then move forward a distance of sqrt(pow(dY,2)+pow(dX,2))
    dY = Y-y
    dX = X-x
    phi = atan2(dY,dX)
    dist = sqrt(pow(dY,2)+pow(dX,2))
    angle = phi - theta #align with point
    #turnright(angle)
    forward(dist) #idk if this is how it works in python
