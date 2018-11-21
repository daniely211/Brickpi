# import place_rec_bits



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

# find orientation by comparing 2 graphs
# return the degrees away from x
def find_orientation(current_graph, location):
    current_graph = np.array(current_graph)
    turn_angle = get_lowest_point_index(current_graph)
    
    if (location == 1) {
        # the shortest wall is perpendicular to the wall
        turn_angle = turn_angle - 90
    }
    return turn_angle


# given we are at location 1

