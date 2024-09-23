import numpy as np

class STATE:
    def __init__(self):
        # Initialize all attributes with their default values
        self.prev_turn_angle = None
        self.prev_speed_diff = None
        self.prev_distance = None
        self.prev_speed = None
        self.prev_progress = 0
        self.prev_progress2 = 0

    # Optional: You could also define a reset method to reset all attributes
    def reset(self):
        self.prev_turn_angle = None
        self.prev_speed_diff = None
        self.prev_distance = None
        self.prev_speed = None
        self.prev_progress = 0
        self.prev_progress2 = 0
        
state = STATE()

class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        import math

        ################## HELPER FUNCTIONS ###################
        def reset_state(steps):
            if steps <= 2:
                state.reset()

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):
            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]
        
        def find_border_points(params):
            waypoints = params['waypoints']
            closest_waypoints = params['closest_waypoints']
            track_width = params['track_width']
            
            next_waypoint_index = closest_waypoints[1]
            prev_waypoint_index = closest_waypoints[0]
            next_waypoint = waypoints[next_waypoint_index]
            prev_waypoint = waypoints[prev_waypoint_index]
            
            # Calculate the direction vector from prev_waypoint to next_waypoint
            direction_vector = np.array([next_waypoint[0] - prev_waypoint[0], next_waypoint[1] - prev_waypoint[1]])
            
            # Calculate the perpendicular vector
            perpendicular_vector = np.array([-direction_vector[1], direction_vector[0]])
            
            # Normalize the perpendicular vector
            perpendicular_vector = perpendicular_vector / np.linalg.norm(perpendicular_vector)
            
            # Calculate the half-width of the track
            half_width = track_width / 2.0
            half_width += 0.4
            
            # Calculate the border points
            inner_border1 = np.array(prev_waypoint) - perpendicular_vector * half_width
            outer_border1 = np.array(prev_waypoint) + perpendicular_vector * half_width
            inner_border2 = np.array(next_waypoint) - perpendicular_vector * half_width
            outer_border2 = np.array(next_waypoint) + perpendicular_vector * half_width
            
            return inner_border1, outer_border1, inner_border2, outer_border2
        
        def find_min_max_heading(params, inner_border2, outer_border2):
            car_x = params['x']
            car_y = params['y']
            car_heading = params['heading']

            # Calculate the vector from the car to the inner border
            inner_vector_x = inner_border2[0] - car_x
            inner_vector_y = inner_border2[1] - car_y

            # Calculate the vector from the car to the outer border
            outer_vector_x = outer_border2[0] - car_x
            outer_vector_y = outer_border2[1] - car_y

            # Compute the angles in degrees
            inner_heading = math.degrees(math.atan2(inner_vector_y, inner_vector_x))
            outer_heading = math.degrees(math.atan2(outer_vector_y, outer_vector_x))

            # Normalize angles to be within 0 to 360 degrees
            inner_heading = (inner_heading + 360) % 360
            outer_heading = (outer_heading + 360) % 360

            # Normalize car heading to be within 0 to 360 degrees
            car_heading = (car_heading + 360) % 360

            # Get the min and max headings
            min_heading = min(inner_heading, outer_heading)
            max_heading = max(inner_heading, outer_heading)

            # Check if the car's heading is within the range considering circular nature
            if max_heading - min_heading <= 180:
                # Normal case where min_heading is less than max_heading and the angle difference is <= 180
                is_within_range = min_heading <= car_heading <= max_heading
            else:
                # Case where angles wrap around, e.g., min_heading=60, max_heading=270, car_heading=350
                is_within_range = car_heading >= max_heading or car_heading <= min_heading

            return min_heading, max_heading, is_within_range
        
        def scale_value(x, old_min=1, old_max=2.9, new_min=1, new_max=2):
            # Scale the value from the old range to the new range
            scaled_value = new_min + ((x - old_min) / (old_max - old_min)) * (new_max - new_min)
            return scaled_value

        #################### RACING LINE ######################

        # Optimal racing line
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[-0.37852, -5.40179, 4.0, 0.04288],
        [-0.50852, -5.40192, 4.0, 0.0325],
        [-0.63852, -5.40204, 4.0, 0.0325],
        [-0.81004, -5.4022, 4.0, 0.04288],
        [-1.11156, -5.40248, 4.0, 0.07538],
        [-1.41308, -5.40276, 4.0, 0.07538],
        [-1.71459, -5.40304, 4.0, 0.07538],
        [-2.01611, -5.40333, 4.0, 0.07538],
        [-2.31763, -5.40361, 4.0, 0.07538],
        [-2.61915, -5.40389, 4.0, 0.07538],
        [-2.92067, -5.40417, 4.0, 0.07538],
        [-3.22219, -5.40444, 3.51477, 0.08579],
        [-3.52371, -5.40475, 2.76264, 0.10914],
        [-3.82523, -5.40512, 2.34805, 0.12841],
        [-4.12675, -5.40529, 2.07692, 0.14518],
        [-4.42507, -5.39906, 1.88106, 0.15863],
        [-4.71489, -5.37705, 1.72525, 0.16847],
        [-4.9902, -5.3321, 1.60171, 0.17416],
        [-5.24501, -5.26007, 1.4, 0.18914],
        [-5.47393, -5.15974, 1.4, 0.17853],
        [-5.6723, -5.0322, 1.4, 0.16845],
        [-5.83587, -4.88001, 1.4, 0.15958],
        [-5.96069, -4.70701, 1.4, 0.15238],
        [-6.03633, -4.51695, 1.45035, 0.14104],
        [-6.06755, -4.31946, 1.50478, 0.13287],
        [-6.05832, -4.12084, 1.5416, 0.12898],
        [-6.01054, -3.92594, 1.61119, 0.12455],
        [-5.9268, -3.73877, 1.68956, 0.12136],
        [-5.80924, -3.56282, 1.7798, 0.1189],
        [-5.6598, -3.40118, 1.88133, 0.11701],
        [-5.48039, -3.25674, 2.00242, 0.11503],
        [-5.27326, -3.13198, 2.14792, 0.11258],
        [-5.04113, -3.02881, 2.32729, 0.10915],
        [-4.78736, -2.94822, 2.55634, 0.10416],
        [-4.51589, -2.88991, 2.86736, 0.09684],
        [-4.23102, -2.85197, 3.32172, 0.08652],
        [-3.93705, -2.83089, 4.0, 0.07368],
        [-3.63779, -2.82176, 4.0, 0.07485],
        [-3.33629, -2.81881, 4.0, 0.07538],
        [-3.03479, -2.81577, 4.0, 0.07538],
        [-2.73329, -2.81274, 4.0, 0.07538],
        [-2.43178, -2.80973, 4.0, 0.07538],
        [-2.13028, -2.80671, 4.0, 0.07538],
        [-1.82877, -2.80369, 4.0, 0.07538],
        [-1.52727, -2.80067, 4.0, 0.07538],
        [-1.22577, -2.79765, 4.0, 0.07538],
        [-0.92426, -2.79464, 4.0, 0.07538],
        [-0.62276, -2.79161, 3.54819, 0.08498],
        [-0.32202, -2.78696, 3.19953, 0.09401],
        [-0.02379, -2.77708, 2.93607, 0.10163],
        [0.27011, -2.75859, 2.7191, 0.1083],
        [0.5578, -2.72851, 2.54003, 0.11388],
        [0.83745, -2.68437, 2.39008, 0.11845],
        [1.10728, -2.62417, 2.26065, 0.12229],
        [1.36559, -2.54638, 2.14251, 0.12591],
        [1.61065, -2.44973, 2.14251, 0.12296],
        [1.84067, -2.33326, 2.14251, 0.12034],
        [2.05369, -2.19616, 2.14251, 0.11824],
        [2.2474, -2.03777, 2.14251, 0.11679],
        [2.4188, -1.85745, 2.17344, 0.11447],
        [2.56731, -1.65739, 2.20331, 0.11308],
        [2.69201, -1.43948, 2.23409, 0.11238],
        [2.79172, -1.20565, 2.26316, 0.11232],
        [2.86503, -0.95799, 2.29084, 0.11275],
        [2.91036, -0.69902, 2.31557, 0.11354],
        [2.92621, -0.43193, 2.34084, 0.1143],
        [2.91151, -0.16069, 2.36616, 0.1148],
        [2.86597, 0.11009, 2.38343, 0.1152],
        [2.79012, 0.37556, 2.40063, 0.11501],
        [2.68525, 0.63151, 2.31643, 0.11941],
        [2.55275, 0.87503, 2.23071, 0.12428],
        [2.39319, 1.10447, 2.15362, 0.12977],
        [2.20791, 1.31679, 2.07939, 0.13552],
        [2.00312, 1.5047, 2.01149, 0.13818],
        [1.78549, 1.66174, 1.94742, 0.13781],
        [1.56001, 1.78587, 1.88725, 0.13638],
        [1.32999, 1.87716, 1.82448, 0.13564],
        [1.09769, 1.93597, 1.82448, 0.13134],
        [0.86489, 1.96236, 1.82448, 0.12842],
        [0.63319, 1.95569, 1.82448, 0.12705],
        [0.40434, 1.91425, 1.82448, 0.12747],
        [0.18065, 1.83462, 2.11313, 0.11237],
        [-0.03762, 1.72475, 2.33402, 0.1047],
        [-0.2504, 1.58789, 2.63792, 0.09591],
        [-0.4582, 1.42764, 3.09814, 0.0847],
        [-0.66204, 1.2485, 3.92939, 0.06906],
        [-0.86331, 1.05643, 4.0, 0.06955],
        [-1.08458, 0.85947, 3.9202, 0.07556],
        [-1.31235, 0.67148, 3.83916, 0.07693],
        [-1.54668, 0.49291, 3.75315, 0.0785],
        [-1.78762, 0.32427, 3.67023, 0.08013],
        [-2.03524, 0.16612, 3.58928, 0.08186],
        [-2.28958, 0.01902, 3.50814, 0.08375],
        [-2.55065, -0.11636, 3.41411, 0.08614],
        [-2.81846, -0.23927, 3.17588, 0.09278],
        [-3.09296, -0.34883, 2.85877, 0.10338],
        [-3.37397, -0.44407, 2.61751, 0.11336],
        [-3.66117, -0.5239, 2.42783, 0.12278],
        [-3.95396, -0.58693, 2.27003, 0.13194],
        [-4.25131, -0.6299, 2.13881, 0.14047],
        [-4.54938, -0.64752, 2.0262, 0.14737],
        [-4.84095, -0.63577, 1.92638, 0.15148],
        [-5.11934, -0.59302, 1.83824, 0.15322],
        [-5.37929, -0.51944, 1.75792, 0.15368],
        [-5.61665, -0.41652, 1.68525, 0.15352],
        [-5.82806, -0.28647, 1.60349, 0.15479],
        [-6.01047, -0.13192, 1.60349, 0.1491],
        [-6.16098, 0.04416, 1.60349, 0.14446],
        [-6.27649, 0.23842, 1.60349, 0.14094],
        [-6.35359, 0.44677, 1.60349, 0.13855],
        [-6.38776, 0.66407, 1.67785, 0.1311],
        [-6.38245, 0.88379, 1.75833, 0.12499],
        [-6.34087, 1.10135, 1.84866, 0.11982],
        [-6.26599, 1.31343, 1.95274, 0.11518],
        [-6.16052, 1.51756, 2.07373, 0.1108],
        [-6.02707, 1.71193, 2.21699, 0.10635],
        [-5.8682, 1.89532, 2.39468, 0.10132],
        [-5.68669, 2.06713, 2.61751, 0.09549],
        [-5.48543, 2.22744, 2.91256, 0.08834],
        [-5.26747, 2.37712, 3.33316, 0.07932],
        [-5.03579, 2.51796, 4.0, 0.06778],
        [-4.79315, 2.65268, 4.0, 0.06938],
        [-4.54289, 2.78434, 4.0, 0.07069],
        [-4.28696, 2.92106, 4.0, 0.07254],
        [-4.03213, 3.05971, 4.0, 0.07253],
        [-3.77863, 3.20061, 4.0, 0.07251],
        [-3.52669, 3.34408, 4.0, 0.07248],
        [-3.27656, 3.49043, 4.0, 0.07245],
        [-3.02855, 3.63993, 4.0, 0.0724],
        [-2.78299, 3.79283, 4.0, 0.07232],
        [-2.54031, 3.94934, 3.95671, 0.07298],
        [-2.30121, 4.09781, 3.68418, 0.07639],
        [-2.06046, 4.2398, 3.45626, 0.08087],
        [-1.81759, 4.37379, 3.26512, 0.08495],
        [-1.57215, 4.49835, 3.0877, 0.08914],
        [-1.32369, 4.61208, 3.0877, 0.0885],
        [-1.07179, 4.71357, 3.0877, 0.08795],
        [-0.81608, 4.80137, 3.0877, 0.08756],
        [-0.55621, 4.87395, 3.0877, 0.08739],
        [-0.29178, 4.92951, 3.66397, 0.07374],
        [-0.02414, 4.97288, 3.96557, 0.06837],
        [0.2463, 5.00572, 4.0, 0.06811],
        [0.51919, 5.02966, 4.0, 0.06848],
        [0.79414, 5.04632, 4.0, 0.06886],
        [1.07076, 5.0574, 4.0, 0.06921],
        [1.34861, 5.06462, 4.0, 0.06949],
        [1.63171, 5.0698, 4.0, 0.07079],
        [1.91482, 5.07644, 3.89932, 0.07262],
        [2.19793, 5.08381, 3.38375, 0.0837],
        [2.47928, 5.09082, 3.02986, 0.09289],
        [2.75674, 5.09363, 2.76163, 0.10048],
        [3.02985, 5.08871, 2.55092, 0.10708],
        [3.29776, 5.07287, 2.38056, 0.11274],
        [3.55928, 5.04331, 2.2359, 0.11771],
        [3.8131, 4.99767, 2.10779, 0.12235],
        [4.05775, 4.93387, 1.99724, 0.12659],
        [4.29159, 4.85014, 1.99724, 0.12436],
        [4.51273, 4.7449, 1.99724, 0.12262],
        [4.71889, 4.61662, 1.99724, 0.12157],
        [4.9071, 4.46362, 1.99724, 0.12145],
        [5.07338, 4.28422, 2.17273, 0.11258],
        [5.21906, 4.08318, 2.28663, 0.10858],
        [5.34419, 3.86288, 2.41821, 0.10477],
        [5.44893, 3.62543, 2.57484, 0.10079],
        [5.53375, 3.37289, 2.75749, 0.09661],
        [5.59938, 3.10721, 2.9853, 0.09167],
        [5.64703, 2.83045, 3.27677, 0.08571],
        [5.67845, 2.54465, 3.66743, 0.0784],
        [5.69596, 2.25184, 4.0, 0.07333],
        [5.70247, 1.95395, 4.0, 0.07449],
        [5.71686, 1.65548, 4.0, 0.07471],
        [5.73868, 1.35841, 4.0, 0.07447],
        [5.76753, 1.06272, 4.0, 0.07427],
        [5.80306, 0.76836, 4.0, 0.07412],
        [5.84493, 0.4753, 4.0, 0.07401],
        [5.89283, 0.18349, 4.0, 0.07393],
        [5.94646, -0.10711, 4.0, 0.07388],
        [6.00557, -0.39653, 4.0, 0.07385],
        [6.06993, -0.68481, 3.54702, 0.08327],
        [6.13932, -0.97197, 3.17461, 0.09306],
        [6.21251, -1.25412, 2.89851, 0.10056],
        [6.27523, -1.52513, 2.67676, 0.10392],
        [6.32652, -1.7924, 2.49464, 0.10909],
        [6.36391, -2.05751, 2.17447, 0.12313],
        [6.38482, -2.32027, 2.17447, 0.12122],
        [6.38679, -2.58012, 2.17447, 0.1195],
        [6.36741, -2.8363, 2.17447, 0.11815],
        [6.32415, -3.08779, 2.17447, 0.11735],
        [6.25006, -3.33187, 2.24576, 0.11358],
        [6.1473, -3.56719, 2.31823, 0.11077],
        [6.01768, -3.79228, 2.39681, 0.10837],
        [5.86299, -4.00558, 2.4801, 0.10624],
        [5.68512, -4.20554, 2.56824, 0.1042],
        [5.48613, -4.39076, 2.66167, 0.10214],
        [5.26819, -4.56006, 2.76129, 0.09994],
        [5.03359, -4.71258, 2.87175, 0.09744],
        [4.78463, -4.84787, 2.99385, 0.09464],
        [4.52354, -4.96589, 3.12995, 0.09154],
        [4.25239, -5.06696, 3.28584, 0.08807],
        [3.97307, -5.15176, 3.46447, 0.08426],
        [3.68725, -5.22125, 3.6752, 0.08004],
        [3.39637, -5.27663, 3.92664, 0.07541],
        [3.1017, -5.31929, 4.0, 0.07444],
        [2.80429, -5.35078, 4.0, 0.07477],
        [2.50501, -5.37274, 4.0, 0.07502],
        [2.20457, -5.38686, 4.0, 0.07519],
        [1.90347, -5.39491, 4.0, 0.0753],
        [1.60208, -5.39865, 4.0, 0.07535],
        [1.30059, -5.39987, 4.0, 0.07537],
        [0.99907, -5.40033, 4.0, 0.07538],
        [0.69756, -5.40082, 4.0, 0.07538],
        [0.39604, -5.40109, 4.0, 0.07538],
        [0.09452, -5.40134, 4.0, 0.07538],
        [-0.207, -5.40163, 4.0, 0.07538]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        prev_waypoint_index = closest_waypoints[0]
        next_waypoint_index = closest_waypoints[1]
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################
        
        reset_state(steps)

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if self.first_racingpoint_index is None:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 0.1

        ## Reward if car goes close to optimal racing line ##
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        
        DISTANCE_PUNISHMENT = 1
        if dist > (track_width * 0.5):
            DISTANCE_PUNISHMENT = 0.5
            
        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        
        A = 4
        B = 2
        delta_progress_reward = 0
        delta_progress = ((progress - state.prev_progress) * A)**B
        delta_progress2 = ((progress - state.prev_progress2) * 0.5 * A) ** B
        if delta_progress < 0 or delta_progress2 < 0:
            print(f'progress: {progress}')
            print(f'prev_progress: {state.prev_progress}')
            print(f'prev_progress2: {state.prev_progress2}')
            print(f'Closest waypoint index: {closest_index}')
            print(f'steps: {steps}')
        if delta_progress >= 3 or delta_progress2 >= 3:
            delta_progress_reward == 1

        delta_progress_reward = max(0, delta_progress + delta_progress2)
        delta_progress_reward = min(100, delta_progress_reward)
        
        
        inner_border1, outer_border1, inner_border2, outer_border2 = find_border_points(params)
        min_heading, max_heading, is_within_range = find_min_max_heading(params, inner_border2, outer_border2)
                
        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        
        optimal_speed = optimals[2]
        speed_cap = optimal_speed + 0.75
            
        try:
            scaled_multiplier = scale_value(4/optimal_speed, 1, 2.9, 1, 4)
        except:
            print('Error with scaled_multiplier.')
            scaled_multiplier = 4/optimal_speed
        
        DISTANCE_MULTIPLE = scaled_multiplier
        DISTANCE_EXPONENT = scaled_multiplier
        SPEED_MULTIPLE = 3 - DISTANCE_MULTIPLE
                
        # Distance component
        DC = (distance_reward) * DISTANCE_MULTIPLE
        # Speed component
        SC = speed_reward * SPEED_MULTIPLE
        # Progress component
        DPC = delta_progress_reward
        
        total_prog_reward = (progress/100)
        
        try:
            if steps % 100 == 0:
                print(f'steps: {steps}')
                print(f'progress: {progress}')
                print(f'delta_progress reward: {DPC}')
                print(f'Total progress reward: {total_prog_reward}')
                print(f'DC: {DC}\nPC: {DPC}')
        except:
            print('Error in printing steps and delta_progress')
        reward += DPC
        
        if state.prev_turn_angle is not None and state.prev_speed_diff is not None and state.prev_distance is not None and state.prev_speed is not None:
            delta_turn_angle = abs(steering_angle - state.prev_turn_angle)
            delta_speed = abs(speed - state.prev_speed)
            # Erratic steering punishments
            if state.prev_turn_angle > 10 and steering_angle < -10:
                reward *= 0.5
            elif state.prev_turn_angle < -10 and steering_angle > 10:
                reward *= 0.5
            if delta_turn_angle > 30:
                reward *= 0.5
        
        # Punishing erratic steering or steering out of range of valid directions.
        if speed > 2.5 and (steering_angle >= 20 or steering_angle <= -20):
            reward *= 0.5
        if direction_diff > 30:
            reward = 1e-3
        
        reward *= DISTANCE_PUNISHMENT

        ## Zero reward if off track ##
        track_width = params['track_width']
        distance_from_center = params['distance_from_center']

        # Zero reward if the center of the car is off the track.

        if not all_wheels_on_track and distance_from_center >= (track_width/2)+0.05:
            reward = min(reward, 0.001)

        ####################### VERBOSE #######################

        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            # print("=== Finish reward: %f ===" % finish_reward)

        #################### RETURN REWARD ####################
        
        state.prev_turn_angle = steering_angle
        state.prev_speed_diff = speed_diff
        state.prev_distance = dist
        state.prev_speed = speed
        state.prev_progress = progress
        state.prev_progress2 = state.prev_progress

        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)