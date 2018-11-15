import random
from sensors import get_sonar_reading
from math import cos, sin, exp
from map_particles import Particles


# make the sum of all particle weights to 1
def normalise_weights(particle_set):
    sum_weights = sum([weight for (weight, point) in particle_set])

    return [(1 / float(len(particle_set)) if sum_weights == 0 else float(weight) / sum_weights, point) for (weight, point) in particle_set]

# returns an array of cumulative probabilites of sampling a given particle from
# an ordered array
def get_cdf_array(particle_set):
    cdf_array = []

    for i in range(len(particle_set)):
        (weight, point) = particle_set[i]

        if i == 0:
            cdf_array.append(weight)
        else:
            cdf_array.append(cdf_array[i - 1] + weight)

    return cdf_array

def resample_particle_set(particle_set):
    new_particles = []

    cdf_array = get_cdf_array(particle_set)

    normalised_weight = 1 / float(len(particle_set))

    for i in range(len(particle_set)):
        rand = random.random()

        for j in range(len(cdf_array)):
            current_weight = cdf_array[j]

            if rand <= current_weight:
                (w, p) = particle_set[j]
                new_particles.append((normalised_weight, p))
                break

    return new_particles

def calculate2_weighted_position(weighted_set):
    x = 0
    y = 0
    theta = 0

    for (w, (x1, y1, theta1)) in weighted_set:
        x += w * x1
        y += w * y1
        theta += w * theta1

    print("x: " + str(x))
    print("y: " + str(y))
    print("theta: " + str(theta))

    return (x, y, theta)

def likelihood(m, z):
    sigma = 1
    return exp(-pow((z - m), 2) / (2 * pow(sigma, 2)))

def measurement_update_from_sonar(weighted_set, interface, sonar_port):
    sonar_reading = get_sonar_reading(interface, sonar_port)
    print("#####################################")
    print("sonar reading: " + str(sonar_reading))
    reweighted_set = []

    for (w, p) in weighted_set:
        m, wall = find_distance(p)
        new_weight = likelihood(m, sonar_reading)
        reweighted_set.append((new_weight, p))

    return reweighted_set

def distance_to_wall(line, particle):
    ((Ax, Ay), (Bx, By)) = line
    (x, y, theta) = particle

    num = ((By - Ay) * (Ax - x)) - ((Bx - Ax) * (Ay - y))
    denom = (By - Ay) * cos(theta) - (Bx - Ax) * sin(theta)

    if denom == 0:
        return float("inf")
    else:
        return (num/denom)

def find_distance(particle):
    (x, y, theta) = particle

    points = {
        'O': (0, 0),
        'A': (0, 168),
        'B': (84, 168),
        'C': (84, 126),
        'D': (84, 210),
        'E': (168, 210),
        'F': (168, 84),
        'G': (210, 84),
        'H': (210, 0)
    }

    line_segments = {
        'a': (points['A'], points['O']),
        'b': (points['B'], points['A']),
        'c': (points['C'], points['B']),
        'c2': (points['D'], points['B']),
        'd': (points['E'], points['D']),
        'e': (points['F'], points['E']),
        'f': (points['G'], points['F']),
        'g': (points['H'], points['G']),
        'h': (points['O'], points['H'])
    }

    dis = []
    for (k, l) in line_segments.items():
        dis.append((distance_to_wall(l, particle), l))

    positive_dists = filter(lambda (m, l): m > 0, dis)
    positive_dists.sort(key = (lambda (m, l): m))

    #Figure out which is the correct distance in dis
    line_intersected = None

    epsilon = 0.000000000000001

    for (m, l) in positive_dists:
        intersection = (x + m * cos(theta), y + m * sin(theta))
        ((x1, y1), (x2, y2)) = l
        #print("m: " + str(m))
        #print("line: " + str(l))
        vect_to_seg_endpoint1 = (x1 - intersection[0], y1 - intersection[1])
        vect_to_seg_endpoint2 = (x2 - intersection[0], y2 - intersection[1])

        product1 = vect_to_seg_endpoint1[0] * vect_to_seg_endpoint2[0]
        product2 = vect_to_seg_endpoint1[1] * vect_to_seg_endpoint2[1]

        #print("p1: " + str(product1))
        #print("p2: " + str(product2))

        if product1 <= epsilon and product2 <= epsilon:
            line_intersected = (m, l)
            #print("selected")
            break

    if line_intersected == None:
        raise ValueError('line intersected was found to be None in monte carlo function!')

    return line_intersected

def check_bounds(particle_set):
    for  for (w, (x, y, theta)) in weighted_set: in range particle_set:
        if(x<0 | x>210):
            w=0
        if(y>210 | y<0):
            w=0
        if(x<84 && y>168):
            w=0
        if(x>168 && y>84):    
            w=0
    return particle_set

def monte_carlo_localisation(particles, interface, sonar_port):
    # generate weighted set of particles where w = 1 / N for all particles
    w = 1 / float(len(particles))
    particle_set = [(w, p) for p in particles]

    # adjust weight using likelihood function
    particle_set = measurement_update_from_sonar(particle_set, interface, sonar_port)

    # normalise all weights
    particle_set = normalise_weights(particle_set)

    # resample points
    particle_set = resample_particle_set(particle_set)


    particle_set = check_bounds(particle_set)

    particles_map = Particles()
    particles_map.update(particle_set)
    particles_map.draw()

    return calculate_weighted_position(particle_set)
