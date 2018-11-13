import numpy

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

def get_sonar_reading(interface, sonar_port):
    readings = []

    for i in range(10):
        readings.append(error_corrected_sonar(interface.getSensorValue(sonar_port)))

    return numpy.median(readings)
