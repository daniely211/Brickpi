#!/usr/bin/env python
# By Jacek Zienkiewicz and Andrew Davison, Imperial College London, 2014
# Based on original C code by Adrien Angeli, 2009

import random
import os
import brickpi
import time
from move import left,right,forward,interface, distance_to_rads

#interface=brickpi.Interface()
# interface.initialize()

motors = [0, 2]

# interface.motorEnable(motors[0])

# motorParams = interface.MotorAngleControllerParameters()
# motorParams.maxRotationAcceleration = 6.0
# motorParams.maxRotationSpeed = 12.0
# motorParams.feedForwardGain = 255/20.0
# motorParams.minPWM = 18.0
# motorParams.pidParameters.minOutput = -255
# motorParams.pidParameters.maxOutput = 255

# # Adjust PID parameters, Ziegler-Nicholls method
# motorParams.pidParameters.k_p = 250.0
# motorParams.pidParameters.k_i = 400.
# motorParams.pidParameters.K_d = 32.0

# kp = motorParams.pidParameters.k_p
# ki = motorParams.pidParameters.k_i
# kd = motorParams.pidParameters.K_d

# interface.setMotorAngleControllerParameters(motors[0],motorParams)
port = 0 # port which ultrasoic sensor is plugged in to
# interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC);

# Location signature class: stores a signature characterizing one location
class LocationSignature:
    def __init__(self, no_bins = 255):
        self.sig = [0] * no_bins

    def print_signature(self):
        for i in range(len(self.sig)):
            print self.sig[i]

# --------------------- File management class ---------------
class SignatureContainer():
    def __init__(self, size = 5):
        self.size      = size; # max number of signatures that can be stored
        self.filenames = [];

        # Fills the filenames variable with names like loc_%%.dat
        # where %% are 2 digits (00, 01, 02...) indicating the location number.
        for i in range(self.size):
            self.filenames.append('loc_{0:02d}.dat'.format(i))

    # Get the index of a filename for the new signature. If all filenames are
    # used, it returns -1;
    def get_free_index(self):
        n = 0
        while n < self.size:
            if (os.path.isfile(self.filenames[n]) == False):
                break
            n += 1

        if (n >= self.size):
            return -1;
        else:
            return n;

    # Delete all loc_%%.dat files
    def delete_loc_files(self):
        print "STATUS:  All signature files removed."
        for n in range(self.size):
            if os.path.isfile(self.filenames[n]):
                os.remove(self.filenames[n])

    # Writes the signature to the file identified by index (e.g, if index is 1
    # it will be file loc_01.dat). If file already exists, it will be replaced.
    def save(self, signature, index):
        filename = self.filenames[index]
        if os.path.isfile(filename):
            os.remove(filename)

        f = open(filename, 'w')

        for i in range(len(signature.sig)):
            s = str(signature.sig[i]) + "\n"
            f.write(s)
        f.close();

    # Read signature file identified by index. If the file doesn't exist
    # it returns an empty signature.
    def read(self, index):
        ls = LocationSignature()
        filename = self.filenames[index]
        if os.path.isfile(filename):
            f = open(filename, 'r')
            for i in range(len(ls.sig)):
                s = f.readline()
                if (s != ''):
                    ls.sig[i] = int(s)
            f.close();
        else:
            print "WARNING: Signature does not exist."

        return ls

# FILL IN: spin robot or sonar to capture a signature and store it in ls
def characterize_location(ls):
    # print "TODO:    You should implement the function that captures a signature."
    # by default ls has 255 bins, for each possible depth measurement
    right_turn_error = 1.158
    angle = 360
    full_circ = 2 * pi * (wheel_dist / 2)
    turn_circ = full_circ * (float(angle) / 360)
    angle_rads = distance_to_rads(turn_circ) * right_turn_error
    interface.increaseMotorAngleReferences(motors, [-angle_rads, angle_rads])
    while not interface.motorAngleReferencesReached(motors):
          motorAngles = interface.getMotorAngles(motors)
          (reading, _) = interface.getSensorValue(port)
          ls.sig[reading] += 1
          time.sleep(0.1)
    # for i in range(72):
    #     right(TURNING_ANGLE, interface)
    #     (reading, _) = interface.getSensorValue(port)
    #     ls.sig[i] = reading
    # for i in range(len(ls.sig)):
    #     ls.sig[i] = random.randint(0, 255)
    return ls
# FILL IN: compare two signatures
def compare_signatures(ls1, ls2):
    dist = 0
    #print "TODO:    You should implement the function that compares two signatures."
    Hm = ls1.sig #Hm is the histogram generated from current point
    Hk = ls2.sig #Hk is histogram of saved point.
    for i in range(length(Hm)):
        dist += (Hm[i]-Hk[i])**2 #from lecture slides.
    return dist

# This function characterizes the current location, and stores the obtained
# signature into the next available file.
def learn_location():
    ls = LocationSignature()
    characterize_location(ls)
    idx = signatures.get_free_index();
    if (idx == -1): # run out of signature files
        print "\nWARNING:"
        print "No signature file is available. NOTHING NEW will be learned and stored."
        print "Please remove some loc_%%.dat files.\n"
        return

    signatures.save(ls,idx)
    print "STATUS:  Location " + str(idx) + " learned and saved."

# This function tries to recognize the current location.
# 1.   Characterize current location
# 2.   For every learned locations
# 2.1. Read signature of learned location from file
# 2.2. Compare signature to signature coming from actual characterization
# 3.   Retain the learned location whose minimum distance with
#      actual characterization is the smallest.
# 4.   Display the index of the recognized location on the screen
def recognize_location():
    ls_obs = LocationSignature();
    characterize_location(ls_obs);

    # FILL IN: COMPARE ls_read with ls_obs and find the best match
    minDist = compare_signatures(ls_obs, signatures.read(0))
    minIdx = 0
    for idx in range(1, signatures.size):
        print "STATUS:  Comparing signature " + str(idx) + " with the observed signature."
        ls_read = signatures.read(idx)
        dist    = compare_signatures(ls_obs, ls_read)
        if dist < minDist:
            minDist = dist
            minIdx = idx
    
    return minDist, minSig
# Prior to starting learning the locations, it should delete files from previous
# learning either manually or by calling signatures.delete_loc_files().
# Then, either learn a location, until all the locations are learned, or try to
# recognize one of them, if locations have already been learned.

signatures = SignatureContainer(5)
#signatures.delete_loc_files()
for i in range(5):
    learn_location()
    print("DONE WITH LOCATION")
    time.sleep(10)
#print(recognize_location())

