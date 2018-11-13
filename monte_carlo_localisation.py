import random

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

    normalised_set = normalise_weights(particle_set)
    cdf_array = get_cdf_array(normalised_set)

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
