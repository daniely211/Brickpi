import brickpi
import time
from math import pi,cos,sin,sqrt,atan2,pow, exp
from statistics import median
import random
import numpy
from monte_carlo_localisation import resample_particle_set

interface = brickpi.Interface()
interface.initialize()
ultra_port = 0
interface.sensorEnable( ultra_port, brickpi.SensorType.SENSOR_ULTRASONIC)

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

current = (0,0,0)

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
    # dist=dist+15.1 #add the length
    angle = 2 * pi * (dist / wheel_circ) * 1.07 # add 7% to calibrate
    # adding 0.05% on the left motor below, to make it go straight
    interface.increaseMotorAngleReferences(motors, [-angle * 1.0005, -angle])

    while not interface.motorAngleReferencesReached(motors):
        time.sleep(0.1)

def generate_particles_from_movement(particles, D, direction):
    new_particles = []

    for particle in particles:
        e = random.gauss(0, 0.2)
        f = random.gauss(0, 0.01)
        x, y, theta = particle[0], particle[1], particle[2]

        if direction == 0 or direction == 2:
            # along x axis
            # multiply by 5 and 10 to scale the spread of particles
            new_particle = ((x + 5.0 * (D + e) * cos(theta + f)), (y + 10.0 * (D + e) * sin(theta + f)), theta + f)
        else:
            #along y axis
            new_particle = ((x + 10.0 * (D + e) * cos(theta + f)), (y + 5.0 * (D + e) * sin(theta + f)), theta + f)
        new_particles.append(new_particle)

    return new_particles

def generate_particles_from_turn(particles, angle):
    new_particles = []

    for particle in particles:
        x,y,theta = particle[0],particle[1],particle[2]
        g = random.gauss(0,0.03)
        new_particle = (x, y, theta + angle + g)
        new_particles.append(new_particle)

    return new_particles

def square():
    global current
    particles = [current for i in range(100)]

    square_size = 10

    for i in range(0,4):
        for j in range(4):
            forward(square_size)
            particles = generate_particles_from_movement(particles, square_size, i)
            avgX = sum([x for (x,y,theta) in particles]) / len(particles)
            avgY = sum([y for (x,y,theta) in particles]) / len(particles)
            avgTheta = sum([theta for (x,y,theta) in particles]) / len(particles)
            line = (current[0], current[1], avgX, avgY)
            current = (avgX, avgY, avgTheta)
            #plot the points
            print("drawLine:" + str(line))
            print("drawParticles:" + str(particles))
            time.sleep(0.4)

        left(90)
        current = (current[0], current[1], current[2] + pi / 2)
        particles = generate_particles_from_turn(particles, pi / 2)
        #plot the new points
        print("drawParticles:" + str(particles))
        time.sleep(0.1)


def navigateToWaypoint(waypoint):  #X is desired X,Y is desired Y
    #assuming we have access to our x,y,theta values (position and direction of robot)
    #take dY = Y-y;dX = X-x
    #we need to turn (phi - theta) degrees with phi = atan2(dY,dX).
    #then move forward a distance of sqrt(pow(dY,2)+pow(dX,2))
    global current
    particles = [current for i in range(100)]
    X,Y = waypoint[0], waypoint[1]
    dY = Y-current[1]
    dX = X-current[0]
    phi = atan2(dY,dX)
    dist = sqrt(pow(dY,2)+pow(dX,2))
    if dX>0:
        angle = phi - current[2] #align with point if dX +ve
    else:
        angle = phi - (current[2]) #offset by pi if dX -ve
    left(angle*180/pi)
    particles = generate_particles_from_turn(particles, angle)
    forward(dist) #idk if this is how it works in python
    particles = generate_particles_from_movement(particles, dist)
    # ^ requires particles from movement to have consistent behaivour (no i) to work.
    current = monte_carlo(particles)
    # ^ requires monte_carlo function to exist
    return current

def Distance_To_Wall(line, particle):
    point1, point2 = line[0], line[1]
    A = ((point2[1]-point1[0])*(point1[0]-particle[0]))-((point2[0]-point1[0])*(point1[1]-particle[1]))
    B = (point2[1]-point1[1])*math.cos(particle[2])-(point2[0]-point1[0]*math.sin(particle[2]))
    if B == 0:
        return float("inf")
    else:
        return (A/B)

def find_distance(particle):
    x,y,theta = particle[0],particle[1],particle[2]

    points = {
        'O':(0,0),
        'A':(0, 168),
        'B':(84,168),
        'C':(84,126),
        'D':(84,210),
        'E':(168,210),
        'F':(168,84),
        'G':(210,84),
        'H':(210,0)
    }

    line_segments = {
        'a'  : (points['A'], points['O']),
        'b'  : (points['B'],points['A']),
        'c'  : (points['C'],points['B'])
        'c2' : (points['D'],points['B']),
        'd'  : (points['E'],points['D']),
        'e'  : (points['F'],points['E']),
        'f'  : (points['G'],points['F']),
        'g'  : (points['H'],points['G']),
        'h'  : (points['O'],points['H'])
    }
    dis = []
    for l in line_segments:
        dis.append((Distance_To_Wall(l,particle), l))

    positive_dists = filter(lambda (m,l): m > 0, dis)
    positive_dists.sort(key = (lambda (m,l): m))

    #Figure out which is the correct distance in dis
    line_intersected = None
    for (m,l) in positive_dists:
        intersection = (x + m*cos(theta), y + m*sin(theta))
        vect_to_seg_endpoint1 = (l[0][0] - intersection[0], l[0][1] - intersection[1])
        vect_to_seg_endpoint2 = (l[1][0] - intersection[0], l[1][1] - intersection[1])

        product1 = vect_to_seg_endpoint1[0] * vect_to_seg_endpoint2[0]
        product2 = vect_to_seg_endpoint1[1] * vect_to_seg_endpoint2[1]
        if product1 <= 0 and product2 <= 0:
            line_intersected = (m,l)
            break
    if line_intersected == None:
        raise ValueError('line intersected was found to be None in monte carlo function!')

    return line_intersected

def measurement_update_from_sonar(weighted_set):
    sonar_reading = get_sonar_reading()
    reweighted_set = []

    for (w, p) in weighted_set:
        m, wall = find_distance(p)
        new_weight = likelihood(m, sonar_reading)
        reweighted_set.append((new_weight, p))

    # resample based on new weights
    resampled_set = resample_particle_set(reweighted_set)

    # update current position based on resampled partices
    current = calculate_weighted_position(resampled_set)

def calculate_weighted_position(weighted_set):
    x = 0
    y = 0
    theta = 0

    for (w, (x1, y1, theta1)) in weighted_set:
        x += w * x1
        y += w * y1
        theta += w * theta1

    print("x:" + x)
    print("y:" + y)
    print("theta:" + theta)

    return (x, y, theta)

def likelihood(m, z):
    sigma = 1
    return exp(-pow((z - m), 2) / (2 * pow(sigma, 2)))

def error_corrected_sonar(estimate):
    mL = 0.876923076923
    cL = 4.06153846154
    mH = 1.17297297297
    mH = 1.17297297297

    if (estimate > 145):
        return (mH * estimate + cH )
    elif (estimate < 20):
        return (mL * estimate + cL)
    else:
        return (estimate)

def get_sonar_reading():
    readings = []

    for i in range(10):
        readings.append(error_corrected_sonar(interface.getSensorValue(ultra_port)))

    return median(readings)

if __name__ == "__main__":


    # Get 10 readings from sonar
    # Find median of 10 readings
    # wall readings
    v = get_sonar_reading();


#waypoint test Lab 5
    #current = (84,30,0)
    #current = navigateToWaypoint((180,30))
    #current = navigateToWaypoint((180,54))
    #current = navigateToWaypoint((138,54))
    #current = navigateToWaypoint((138,168))
    #current = navigateToWaypoint((114,168))
    #current = navigateToWaypoint((114,84))
    #current = navigateToWaypoint((84,84))
    #current = navigateToWaypoint((84,30))


#place = ((0,0))
#    while( True  ):
#        place = input("Enter Coordinates: ")
#        place = (place[0]*100), (place[1]*100)
#        print(place)
#        current = navigateToWaypoint(place)
#


    # Test for waypoint:
    #current = navigateToWaypoint((-10,10))
    #print(current)
    #current = navigateToWaypoint((0,0))
    #current = navigateToWaypoint((40,100))
    #print(current)
    #current = navigateToWaypoint((90,60))
    #print(current)

    #Test square:
    #square()
    interface.terminate()
