import brickpi
import time
import math

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

motorParams.pidParameters.k_p = 540
motorParams.pidParameters.k_i = 2000
motorParams.pidParameters.K_d = 34

# Units: cm
wheel_radius = 2.15
wheel_circ = 2 * math.pi * wheel_radius

# Distance between wheels
wheel_dist = 15

interface.setMotorAngleControllerParameters(motors[0], motorParams)
interface.setMotorAngleControllerParameters(motors[1], motorParams)

def distance_to_rads(distance):
    delta = 1.02
    return 2 * math.pi * (distance / wheel_circ) * delta

def rotate(angle, direction):
    	full_circ = 2 * math.pi * (wheel_dist / 2)
    	turn_circ = full_circ * (angle / 360)
    	angle_rads = distance_to_rads(turn_circ)

    	if direction == 'left':
        	interface.increaseMotorAngleReferences(motors, [angle, -angle])
    	elif direction == 'right':
        	interface.increaseMotorAngleReferences(motors, [-angle, angle])

	while not interface.motorAngleReferencesReached(motors):
		time.sleep(0.1)

def left(angle):
    rotate(angle, 'left')

def right(angle):
    rotate(angle, 'right')

def forward(dist):
    #dist=dist+15.1 #add the length
    angle = 2*math.pi*( dist/wheel_circ )*1.02 #FIX LATER
    interface.increaseMotorAngleReferences(motors,[-angle+0.1, -angle]) # offset left wheel to keep straight line
    print("rotating " + str(angle))
    while not interface.motorAngleReferencesReached(motors):
          motorAngles = interface.getMotorAngles(motors)
          print(motorAngles[0][0])
          print(motorAngles[1][0])
          print(interface.getMotorAngleReferences(motors))
          time.sleep(0.1)



def generate_particles_from_movement(current, D):
      x,y,theta = current[0], current[1], current[2]
      new_points = []
      for i in range(100):
            e = random.gauss(0, 5)
            f = random.gauss(0, 5)
            new_point = (x + (D + e)*cos(theta), y + (D + e)*sin(theta), theta + f)
            new_points.append(new_point)
      return new_points
def generate_particles_from_turn(current, angle):
      x,y,theta = current[0], current[1], current[2]
      new_points = []
      for i in range(100):
            g = random.gauss(0,5)
            new_point = (x, y, theta + angle + g)
            new_points.append(new_point)
      return new_points

def square():
	#18.6046511628
    current =  (0,0,0)
    for i in range(0,4):
        for i in range(4):
                forward(10)
                previous_pos = current
                particles = generate_particles_from_movement(current, 10)
                avgX = sum([x for (x,y,theta) in particles])/100
                avgY = sum([y for (x,y,theta) in particles])/100
                avgTheta = sum([theta for (x,y,theta) in particles])/100
                current = (avgX, avgY,avgTheta)
                line = (previous_pos[0], previous_pos[1], avgX, avgY)
                #plot the points
                print("drawLine:" + str(line))
                print("drawParticles:" + str(particles))

        left(90)
        particles = generate_particles_from_turn(current, 90)
        #plot the new points
        print("drawParticles:" + str(particles))

        time.sleep(0.1)



square()
interface.terminate()
