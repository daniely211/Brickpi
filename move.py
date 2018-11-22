import brickpi
import time
from math import pi, cos, sin, sqrt, atan2, pow, exp
import random
from monte_carlo_localisation import monte_carlo_localisation
import map_particles 

sonar_port = 2
motors = [0, 2, 1]
left_touch_port = 1
right_touch_port = 2
speed = -6
current = (0, 0, 0)
PARTICLES_SIZE = 100

# Units: cm
wheel_radius = 2.15
wheel_circ = 2 * pi * wheel_radius

# Distance between wheels
wheel_dist = 15.0

interface = brickpi.Interface()
interface.initialize()
interface.sensorEnable(sonar_port, brickpi.SensorType.SENSOR_ULTRASONIC)

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])
interface.motorEnable(motors[2])

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

interface.sensorEnable(sonar_port, brickpi.SensorType.SENSOR_ULTRASONIC)

interface.motorEnable(motors[0])

topMotorParams = interface.MotorAngleControllerParameters()
topMotorParams.maxRotationAcceleration = 3.0
topMotorParams.maxRotationSpeed = 6.0
topMotorParams.feedForwardGain = 255/20.0
topMotorParams.minPWM = 18.0 
topMotorParams.pidParameters.minOutput = -255
topMotorParams.pidParameters.maxOutput = 255

topMotorParams.pidParameters.k_p = 360
topMotorParams.pidParameters.k_i = 800
topMotorParams.pidParameters.K_d = 22

interface.setMotorAngleControllerParameters(motors[0], motorParams)
interface.setMotorAngleControllerParameters(motors[1], motorParams)
interface.setMotorAngleControllerParameters(motors[2], topMotorParams)

def distance_to_rads(distance):
    return 2 * pi * (distance / wheel_circ)

def rotate(interface, angle, direction, error = 1):
    full_circ = 2 * pi * (wheel_dist / 2)
    turn_circ = full_circ * (float(angle) / 360)
    angle_rads = distance_to_rads(turn_circ) * error

    if direction == 'left':
        interface.increaseMotorAngleReferences(motors, [angle_rads, -angle_rads])
    elif direction == 'right':
        interface.increaseMotorAngleReferences(motors, [-angle_rads, angle_rads])

    while not interface.motorAngleReferencesReached(motors):
        time.sleep(0.1)

def left(angle, interface):
    rotate(interface, angle, 'left', 1.163)

def right(angle, interface):
    rotate(interface, angle, 'right', 1.158)

def forward(dist, interface):
    angle = 2 * pi * (dist / wheel_circ) * 1.04 # add 7% to calibrate
    # adding 0.05% on the left motor below, to make it go straight
    interface.increaseMotorAngleReferences(motors, [-angle * 1.06, -angle])

    while not interface.motorAngleReferencesReached(motors):
        time.sleep(0.1)

def generate_particles_from_movement(particles, D):
    new_particles = []

    for (x, y, theta) in particles:
        e = random.gauss(0, D*0.01)  #0.02
        f = random.gauss(0, D*0.001) #0.0025
        new_particle = ((x + (D + e) * cos(theta + f)), (y + (D + e) * sin(theta + f)), theta + f)
        new_particles.append(new_particle)

    return new_particles

def generate_particles_from_turn(particles, angle):
    new_particles = []

    for (x, y, theta) in particles:
        g = random.gauss(0, angle*0.005)
        new_particle = (x, y, theta + angle + g)
        new_particles.append(new_particle)

    return new_particles

def square():
    global current
    particles = [current for i in range(PARTICLES_SIZE)]

    square_size = 40

    for i in range(0, 4):
        for j in range(4):
            forward(square_size / 4)
            particles = generate_particles_from_movement(particles, square_size / 4)
            avgX = sum([x for (x, y, theta) in particles]) / PARTICLES_SIZE
            avgY = sum([y for (x, y, theta) in particles]) / PARTICLES_SIZE
            avgTheta = sum([theta for (x, y, theta) in particles]) / PARTICLES_SIZE
            line = (current[0], current[1], avgX, avgY)
            current = (avgX, avgY, avgTheta)
            #plot the points
            print("drawLine:" + str(line))
            print("drawParticles:" + str(particles))
            time.sleep(0.4)

        left(90, interface)
        current = (current[0], current[1], current[2] + pi / 2)
        particles = generate_particles_from_turn(particles, pi / 2)
        #plot the new points
        print("drawParticles:" + str(particles))
        time.sleep(0.1)

def set_theta(theta):
    global current
    current[2] = theta

def set_current(x, y, theta):
    global current
    current = (x, y, theta)

def navigate_to_waypoint(waypoint):  #X is desired X,Y is desired Y
    #assuming we have access to our x,y,theta values (position and direction of robot)
    #take dY = Y-y;dX = X-x
    #we need to turn (phi - theta) degrees with phi = atan2(dY,dX).
    #then move forward a distance of sqrt(pow(dY,2)+pow(dX,2))
    global current
    particles = [current for i in range(PARTICLES_SIZE)]

    (x, y) = waypoint
    dY = y - current[1]
    dX = x - current[0]
    dist = sqrt(pow(dY, 2) + pow(dX, 2))
    
    delta = 3
    
    while (dist > delta):

        phi = atan2(dY, dX)
        angle = phi - current[2] # offset by pi if dX -ve

        if angle < -pi:
            angle += 2 * pi
        elif angle > pi:
            angle -= 2 * pi

        print("angle to turn: " + str(angle))
        left(angle * 180 / pi, interface)
        particles = generate_particles_from_turn(particles, angle)

        if dist > 20:
            forward(20, interface)
            particles = generate_particles_from_movement(particles, 20)
        else:
            forward(dist, interface)
            particles = generate_particles_from_movement(particles, dist)

        current = monte_carlo_localisation(particles, interface, sonar_port)
        (x, y) = waypoint
        dY = y - current[1]
        dX = x - current[0]

        dist = sqrt(pow(dY, 2) + pow(dX, 2)) # new distance based on partial movement
        print("new distance: " + str(dist))

        #for i in range (int(dist/20)):
         #   particles = generate_particles_from_movement(particles, 20)
         #   current = monte_carlo_localisation(particles, interface, sonar_port)
        #forward(dist % 20)
        #particles = generate_particles_from_movement(particles, dist % 20)
        #current = monte_carlo_localisation(particles, interface, sonar_port)
    #else:
     #   forward(dist)
     #   particles = generate_particles_from_movement(particles, dist)
     #   current = monte_carlo_localisation(particles, interface, sonar_port)

    return current

# if __name__ == "__main__":

#     list_waypoint = [(180,30), (180,54), (138,54), (138,168), (114,168), (114,84), (84,84), (84,30)]
#     #waypoint test Lab 5
#     current = (84,30,0)
#     #draw map
#     for waypoint in list_waypoint:
#         current = navigate_to_waypoint(waypoint)
#         print("current: " + str(current))
    
    #forward(100)   
        

        # update
    #particles = Particles();

    #t = 0;
    #while True:
    #    particles.update();
    #    particles.draw();
    #    t += 0.05;
    #    time.sleep(0.05);


    
    #current = navigate_to_waypoint((180,30))
    #current = navigate_to_waypoint((180,54))
    #current = navigate_to_waypoint((138,54))
    #current = navigate_to_waypoint((138,168))
    #current = navigate_to_waypoint((114,168))
    #current = navigate_to_waypoint((114,84))
    #current = navigate_to_waypoint((84,84))
    #current = navigate_to_waypoint((84,30))
    
    # 

#place = ((0,0))
#    while( True  ):
#        place = input("Enter Coordinates: ")
#        place = (place[0]), (place[1])
#        print(place)
#        current = navigate_to_waypoint(place)
#
    # interface.terminate()
