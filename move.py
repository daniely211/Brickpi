import brickpi 
import time 
from math import pi,cos,sin,sqrt,atan2,pow
import random 
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

# Units: cm
wheel_radius = 2.15
wheel_circ = 2 * pi * wheel_radius

# Distance between wheels
wheel_dist = 15.0

interface.setMotorAngleControllerParameters(motors[0], motorParams)
interface.setMotorAngleControllerParameters(motors[1], motorParams)

current =  (10,10,0)

def distance_to_rads(distance):
    delta = 1.02
    return 2 * pi * (distance / wheel_circ) * delta

def rotate(angle, direction):
        full_circ = 2 * pi * (wheel_dist / 2)
        turn_circ = full_circ * (float(angle) / 360)
        angle_rads = distance_to_rads(turn_circ) * 1.178
        if direction == 'left':
            interface.increaseMotorAngleReferences(motors, [angle_rads, -angle_rads])

        elif direction == 'right':
            interface.increaseMotorAngleReferences(motors, [-angle_rads, angle_rads])

        while not interface.motorAngleReferencesReached(motors):
            time.sleep(0.1)

def left(angle):
    rotate(angle, 'left')

def right(angle):
    rotate(angle, 'right')

def forward(dist):
    #dist=dist+15.1 #add the length
    angle = 2*pi*( dist/wheel_circ )*1.07 # add 7% to calibrate
    #adding 0.05% on the left motor below, to make it go straight
    interface.increaseMotorAngleReferences(motors,[-angle*1.0005, -angle]) # offset left wheel to keep straight line
    while not interface.motorAngleReferencesReached(motors):
          
          time.sleep(0.1)

def generate_particles_from_movement(particles, D, direction):
    new_particles = []
    for particle in particles:
        e = random.gauss(0, 2)
        f = random.gauss(0, 0.05)
        x,y,theta = particle[0], particle[1], particle[2]
        if direction == 0 or direction == 2:
            # along x axis
            #print("cos:" + str(cos(theta+f)))
            new_particle = ((x + 5.0*(D + e)*cos(theta+f)), (y + 10.0*(D + e)*sin(theta+f)), theta + f)
        else:
            #along y axis
            #print("sin:" + str(sin(theta+f)))
            new_particle = ((x + 10.0*(D + e)*cos(theta+f)), (y + 5.0*(D + e)*sin(theta+f)), theta + f)
        new_particles.append(new_particle)
    return new_particles

def generate_particles_from_turn(particles, angle):
      new_particles = []
      for particle in particles:
            x,y,theta = particle[0],particle[1],particle[2]
            g = random.gauss(0,0.05)
            new_particle = (x, y, theta + angle + g)
            new_particles.append(new_particle)
      return new_particles

def square():
    particles = [(0,0,0) for i in range(100)]
    for i in range(0,4):
        for j in range(4):
                forward(10)
                particles = generate_particles_from_movement(particles, 10.0, i)
                avgX = sum([x for (x,y,theta) in particles])/100
                avgY = sum([y for (x,y,theta) in particles])/100
                avgTheta = sum([theta for (x,y,theta) in particles])/100
                line = (current[0], current[1], avgX, avgY)
                current = (avgX, avgY,avgTheta)
                #plot the points
                print("drawLine:" + str(line))
                print("drawParticles:" + str(particles))
                time.sleep(0.4)        
        left(90)
        current = (current[0], current[1], current[2] + pi/2)
        particles = generate_particles_from_turn(particles, pi/2)
        #plot the new points
        print("drawParticles:" + str(particles))

        time.sleep(0.1)

        

def navigateToWaypoint(X, Y):  #X is desired X,Y is desired Y
    #assuming we have access to our x,y,theta values (position and direction of robot)
    #take dY = Y-y;dX = X-x
    #we need to turn (phi - theta) degrees with phi = atan2(dY,dX).
    #then move forward a distance of sqrt(pow(dY,2)+pow(dX,2))
    dY = Y-current[1]
    dX = X-current[0]
    print(dY)
    print(dX)
    phi = atan2(dY,dX)
    dist = sqrt(pow(dY,2)+pow(dX,2))
    if dX>0:
        angle = phi - current[2] #align with point if dX +ve
    else:
        angle = phi - (current[2]) #offset by pi if dX -ve
    print(angle*180/pi)
    print(dist)
    right(angle*180/pi)
    forward(dist) #idk if this is how it works in python
    new_pos = current
    new_pos[0]=current[0]+dX
    new_pos[1]=current[1]+dY
    new_pos[3]=current[3]+phi
    return new_pos


current = navigateToWaypoint(70,80, current)
print(current)
interface.terminate()
