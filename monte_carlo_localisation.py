import random
from sensors import get_sonar_reading
from math import cos, sin

# make the sum of all particle weights to 1
def normalise_weights(particle_set):
    sum_weights = sum([weight for (weight, point) in particle_set])

    return [(float(weight) / sum_weights, point) for (weight, point) in particle_set]

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
                new_particles.append((p, normalised_weight))
                break

    return new_particles

def calculate_weighted_position(weighted_set):
    x = 0
    y = 0
    theta = 0

    for (w, (x1, y1, theta1)) in weighted_set:
        x += w * x1
        y += w * y1
        theta += w * theta1

    print("x: " + x)
    print("y: " + y)
    print("theta: " + theta)

    return (x, y, theta)

def likelihood(m, z):
    sigma = 1
    return exp(-pow((z - m), 2) / (2 * pow(sigma, 2)))

def measurement_update_from_sonar(weighted_set, interface, sonar_port):
    sonar_reading = get_sonar_reading(interface, sonar_port)
    reweighted_set = []

    for (w, p) in weighted_set:
        m, wall = find_distance(p)
        new_weight = likelihood(m, sonar_reading)
        reweighted_set.append((new_weight, p))

    return reweighted_set

def distance_to_wall(line, particle):
    point1, point2 = line[0], line[1]
    A = ((point2[1]-point1[0])*(point1[0]-particle[0]))-((point2[0]-point1[0])*(point1[1]-particle[1]))
    B = (point2[1]-point1[1])*cos(particle[2])-(point2[0]-point1[0]*sin(particle[2]))

    if B == 0:
        return float("inf")
    else:
        return (A/B)

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
    for l in line_segments:
        dis.append((distance_to_wall(l, particle), l))

    positive_dists = filter(lambda (m,l): m > 0, dis)
    positive_dists.sort(key = (lambda (m,l): m))

    #Figure out which is the correct distance in dis
    line_intersected = None
    for (m, l) in positive_dists:
        intersection = (x + m * cos(theta), y + m * sin(theta))
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

    return calculate_weighted_position(particle_set)
