from place_rec_bits import compare_signatures, recognize_location, LocationSignature
from move import set_current, navigate_to_waypoint
import numpy as np
import math
waypoints = [(83, 30), (180, 30), (180, 54), (138, 54), (138, 168)]

def navigate_course(current_point):
    index = waypoints.index(current_point)
    for i in range(1,len(waypoints)):
        j = (index + i) % len(waypoints)
        print("navigating to waypoint: " + str(j))
        current = navigate_to_waypoint(waypoints[j])
 
# findout which waypoint we are at, 
# turn to the right orientation
# starting going through the waypoints. (DONE)

def get_lowest_point_index(arr):
    return arr.argsort()[0]

def roll(r_ls): 
    r_ls.append(r_ls[0])
    r_ls = r_ls[1:]
    return r_ls

def check_angle(idx, orientation_ls):
    with open("location" + str(idx), 'r') as f:
        content = f.readlines()
    # you may also want to remove whitespace characters like `\n` at the end of each line
    content = [x.strip() for x in content]
    signature = [int(x.split()[1]) for x in content]
    location_ls = LocationSignature(360)
    location_ls.save_signature(signature)
    min = 1000000000
    min_idx = 0
    for i in range(len(content)):
        orientation_ls.sig = roll(orientation_ls.sig)
        # do dot.product instead
        ls_diff = compare_signatures(location_ls, orientation_ls)
        if(ls_diff < min):
            min = ls_diff
            min_idx = i
    return min_idx


if __name__ == "__main__":
    # given we are at location 1
    (finDist, finW, location_sig) = recognize_location()
    print("location recognised: " + str(finW+1))
    current_location = waypoints[finW]
    orientation_signature = location_sig[1] # the second one is the orientation one
    #for i in range(len(orientation_signature.sig)):
    #    print(str(i)+": " + str(orientation_signature.sig[i]))
    new_theta = check_angle(finW+1, orientation_signature)
    print("Current Orientation from x axis: " + str(new_theta))
    if new_theta > 180:
      new_theta -=360
    elif new_theta < -180:
      new_theta += 360
    print("Angle after correction for wrapping: "+str(new_theta))
    #subtract from 360 to get orientation in terms of ACW turn from x axis
     
    set_current(current_location, (new_theta * math.pi / 180.0))
    navigate_course(current_location)




