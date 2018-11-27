from place_rec_bits import LocationSignature, characterize_location
from move import set_current, navigate_to_waypoint, compare_signatures
import numpy as np
waypoints = [(83, 30), (180, 30), (180, 54), (138, 54), (138, 168)]

def navigate_course(current_point):
    index = waypoints.index(current_point)
    for i in range(len(waypoints)):
        j = (index + i) % len(waypoints)
        current = navigate_to_waypoint(waypoints[j])
 
# findout which waypoint we are at, 
# turn to the right orientation
# starting going through the waypoints. (DONE)

def get_lowest_point_index(arr):
    return arr.argsort()[0]

def roll(r_ls):
    r_ls = r_ls.append(r_ls[0])
    r_ls = r_ls[1:]
    return r_ls

def check_angle(idx, orientation_ls):
    f = open("location" + str(idx), 'r')
    
    min = 1000000000
    min_idx = 0
    for i in range(len(f)):
        orientation_ls = roll(orientation_ls)
        
        # do dot.product instead 
        ls_diff = compare_signatures(f, orientation_ls)
        if(ls_diff < min):
            min = ls_diff
            min_idx = i
    
    return min_idx


if __name__ == "__main__":
    # given we are at location 1
    (finDist, finW, location_sig) = recognize_location()
    current_location = waypoints[finW]
    orientation_signature = location_sig[1] # the second one is the orientation one
    new_theta = check_angle(finW, orientation_signature) 
    print("Angle needed "+str(new_theta))
    set_current(current_location, new_theta)
    navigate_course(current_location)




