# Import package (needed for heading)
import math
import numpy as np

class STATE:
    prev_speed_diff = None
    prev_steering = None
    prev_direction_diff = None
    prev_normalized_distance_from_route = None
    intermediate_progress = {i: 0 for i in range(1, 11)}

class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        ################## HELPER FUNCTIONS ###################

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
            next_point, prev_point = next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading)

            # Calculate the direction in radians, the result is (-pi, pi) in radians
            track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degrees
            track_direction = math.degrees(track_direction)

            # Normalize angles to be within -180 to 180 degrees
            track_direction = (track_direction + 360) % 360
            heading = (heading + 360) % 360

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = track_direction - heading
            
            # Account for cyclical difference
            if direction_diff > 180:
                direction_diff -= 360
            elif direction_diff < -180:
                direction_diff += 360

            return abs(direction_diff)

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):
            print(start, end, array_len)
            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            if current_expected_time is not None and total_expected_time is not None:
                try:
                    projected_time = (current_actual_time/current_expected_time) * total_expected_time
                except ZeroDivisionError:
                    projected_time = 9999
            else:
                projected_time = 9999

            return projected_time
        
        def find_border_points(params):
            waypoints = params['waypoints']
            closest_waypoints = params['closest_waypoints']
            track_width = params['track_width']
            
            next_waypoint = waypoints[closest_waypoints[1]]
            prev_waypoint = waypoints[closest_waypoints[0]]
            
            # Calculate the direction vector from prev_waypoint to next_waypoint
            direction_vector = np.array([next_waypoint[0] - prev_waypoint[0], next_waypoint[1] - prev_waypoint[1]])
            
            # Calculate the perpendicular vector
            perpendicular_vector = np.array([-direction_vector[1], direction_vector[0]])
            
            # Normalize the perpendicular vector
            perpendicular_vector = perpendicular_vector / np.linalg.norm(perpendicular_vector)
            
            # Calculate the half-width of the track
            half_width = track_width / 2.0
            
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
            print(f'car_heading: {car_heading}')

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

        #################### RACING LINE ######################

        # Optimal racing line
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[5.03288, 0.85564, 4.0, 0.07447],
        [5.01964, 1.15157, 4.0, 0.07406],
        [5.00165, 1.44532, 4.0, 0.07357],
        [4.97821, 1.73646, 3.89081, 0.07507],
        [4.94839, 2.02446, 3.49189, 0.08292],
        [4.91133, 2.30879, 3.10891, 0.09223],
        [4.8662, 2.58891, 2.76219, 0.10272],
        [4.81083, 2.86348, 2.44627, 0.1145],
        [4.74332, 3.13117, 2.11927, 0.13027],
        [4.66127, 3.3902, 1.83353, 0.14819],
        [4.56186, 3.63823, 1.83353, 0.14574],
        [4.44172, 3.87211, 1.83353, 0.1434],
        [4.29589, 4.08657, 1.83353, 0.14145],
        [4.11865, 4.27333, 2.0357, 0.12648],
        [3.91788, 4.43579, 2.24976, 0.1148],
        [3.69872, 4.57696, 2.46557, 0.10573],
        [3.46468, 4.69918, 2.68924, 0.09818],
        [3.21845, 4.80439, 2.96225, 0.09039],
        [2.9625, 4.89466, 3.19042, 0.08507],
        [2.69861, 4.97126, 3.41585, 0.08044],
        [2.42843, 5.03527, 3.62983, 0.07649],
        [2.15353, 5.08758, 3.76913, 0.07424],
        [1.87532, 5.12861, 4.0, 0.0703],
        [1.59531, 5.15937, 4.0, 0.07042],
        [1.31471, 5.18034, 4.0, 0.07035],
        [1.03457, 5.1919, 4.0, 0.07009],
        [0.7558, 5.19431, 4.0, 0.0697],
        [0.47916, 5.18771, 4.0, 0.06918],
        [0.20532, 5.17222, 3.99827, 0.0686],
        [-0.06522, 5.14795, 3.99827, 0.06794],
        [-0.332, 5.11493, 3.99827, 0.06723],
        [-0.59468, 5.07329, 3.96628, 0.06706],
        [-0.85299, 5.02305, 3.83939, 0.06854],
        [-1.10687, 4.96465, 3.72593, 0.06992],
        [-1.35639, 4.89857, 3.34702, 0.07712],
        [-1.60125, 4.82462, 3.05015, 0.08386],
        [-1.8411, 4.74254, 2.72666, 0.09297],
        [-2.07558, 4.65213, 2.56795, 0.09786],
        [-2.30337, 4.55175, 2.52532, 0.09857],
        [-2.52296, 4.43988, 2.32256, 0.10611],
        [-2.73205, 4.31436, 2.08647, 0.11688],
        [-2.92881, 4.17442, 1.82337, 0.13242],
        [-3.11226, 4.02063, 1.59605, 0.14999],
        [-3.27926, 3.85181, 1.53, 0.15521],
        [-3.42475, 3.66642, 1.53, 0.15402],
        [-3.54018, 3.4632, 1.53, 0.15276],
        [-3.61372, 3.24425, 1.53, 0.15096],
        [-3.64036, 3.01838, 1.41519, 0.16071],
        [-3.6356, 2.79287, 1.41519, 0.15939],
        [-3.59681, 2.57108, 1.41519, 0.1591],
        [-3.51544, 2.35905, 1.41519, 0.16047],
        [-3.38247, 2.16829, 1.6794, 0.13846],
        [-3.21362, 1.99934, 1.87227, 0.12758],
        [-3.01589, 1.85221, 2.13512, 0.11544],
        [-2.79561, 1.72494, 2.41825, 0.1052],
        [-2.5574, 1.61545, 2.76846, 0.0947],
        [-2.30524, 1.52119, 3.25436, 0.08272],
        [-2.04281, 1.43897, 4.0, 0.06875],
        [-1.77359, 1.36512, 4.0, 0.06979],
        [-1.50051, 1.2962, 3.73031, 0.0755],
        [-1.22853, 1.22987, 3.24749, 0.08621],
        [-0.96011, 1.15948, 2.90404, 0.09555],
        [-0.69762, 1.08244, 2.53395, 0.10796],
        [-0.44338, 0.99635, 2.25186, 0.1192],
        [-0.19968, 0.89914, 1.95523, 0.13419],
        [0.03129, 0.78919, 1.71069, 0.14953],
        [0.24606, 0.66421, 1.71069, 0.14525],
        [0.4409, 0.52251, 1.63682, 0.14719],
        [0.60998, 0.36196, 1.45884, 0.15982],
        [0.7458, 0.18179, 1.3, 0.17357],
        [0.85168, -0.01206, 1.3, 0.16991],
        [0.92163, -0.2181, 1.3, 0.16738],
        [0.94582, -0.43295, 1.3, 0.16631],
        [0.9117, -0.64659, 1.44815, 0.14939],
        [0.83152, -0.84917, 1.57766, 0.1381],
        [0.71328, -1.03646, 1.71728, 0.12898],
        [0.56302, -1.20656, 1.8737, 0.12113],
        [0.38557, -1.35875, 2.02013, 0.11572],
        [0.18422, -1.49243, 2.18851, 0.11044],
        [-0.03799, -1.6075, 2.38894, 0.10475],
        [-0.27806, -1.70445, 2.63312, 0.09833],
        [-0.53299, -1.78439, 2.90818, 0.09187],
        [-0.8001, -1.84872, 3.29096, 0.08348],
        [-1.07658, -1.89968, 3.57147, 0.07872],
        [-1.3608, -1.93838, 3.70565, 0.07741],
        [-1.65117, -1.96628, 3.39598, 0.0859],
        [-1.94681, -1.9837, 3.1271, 0.09471],
        [-2.23701, -2.01239, 2.87889, 0.10129],
        [-2.5204, -2.05315, 2.64084, 0.10842],
        [-2.79541, -2.10737, 2.41219, 0.1162],
        [-3.06036, -2.17625, 2.20927, 0.12391],
        [-3.31335, -2.26095, 1.96054, 0.13608],
        [-3.55207, -2.36261, 1.7453, 0.14867],
        [-3.77374, -2.48233, 1.7453, 0.14435],
        [-3.97516, -2.62084, 1.7453, 0.14006],
        [-4.1509, -2.77951, 1.64376, 0.14405],
        [-4.29409, -2.95835, 1.5193, 0.15079],
        [-4.40657, -3.15159, 1.5193, 0.14717],
        [-4.48759, -3.35581, 1.5193, 0.14461],
        [-4.53224, -3.56819, 1.5193, 0.14284],
        [-4.53429, -3.78363, 1.56721, 0.13748],
        [-4.50057, -3.99652, 1.56721, 0.13753],
        [-4.4311, -4.20249, 1.56721, 0.1387],
        [-4.32517, -4.39661, 1.56721, 0.14111],
        [-4.18003, -4.57168, 1.73242, 0.13127],
        [-4.00297, -4.72578, 1.91855, 0.12234],
        [-3.79974, -4.85875, 2.11931, 0.1146],
        [-3.57476, -4.97116, 2.32964, 0.10796],
        [-3.33153, -5.06381, 2.57991, 0.10088],
        [-3.07334, -5.13812, 2.84665, 0.09438],
        [-2.8029, -5.19557, 3.20329, 0.08631],
        [-2.52298, -5.23841, 3.63931, 0.07781],
        [-2.23592, -5.26906, 4.0, 0.07217],
        [-1.94366, -5.28991, 4.0, 0.07325],
        [-1.64769, -5.30314, 4.0, 0.07407],
        [-1.34916, -5.31066, 4.0, 0.07465],
        [-1.049, -5.31424, 4.0, 0.07505],
        [-0.74784, -5.31525, 4.0, 0.07529],
        [-0.44644, -5.31426, 4.0, 0.07535],
        [-0.14587, -5.31101, 4.0, 0.07515],
        [0.15356, -5.30511, 4.0, 0.07487],
        [0.45157, -5.29604, 4.0, 0.07454],
        [0.74786, -5.28326, 4.0, 0.07414],
        [1.0421, -5.26623, 4.0, 0.07368],
        [1.334, -5.24442, 4.0, 0.07318],
        [1.62322, -5.21729, 4.0, 0.07262],
        [1.90935, -5.1842, 3.62696, 0.07941],
        [2.19188, -5.14438, 3.24731, 0.08786],
        [2.47004, -5.09662, 2.89788, 0.09739],
        [2.74282, -5.03943, 2.62753, 0.10607],
        [3.009, -4.97118, 2.28948, 0.12003],
        [3.26699, -4.88982, 1.97688, 0.13684],
        [3.5147, -4.79295, 1.75856, 0.15125],
        [3.7499, -4.67837, 1.75856, 0.14877],
        [3.96844, -4.54204, 1.75856, 0.14647],
        [4.16397, -4.3787, 1.75856, 0.14488],
        [4.32832, -4.18397, 2.09863, 0.12142],
        [4.46896, -3.96859, 2.33544, 0.11014],
        [4.58908, -3.73693, 2.54375, 0.10259],
        [4.69065, -3.49163, 2.7691, 0.09588],
        [4.77545, -3.23484, 3.01727, 0.08963],
        [4.84513, -2.96835, 3.29217, 0.08367],
        [4.9013, -2.69372, 3.60466, 0.07776],
        [4.94553, -2.41234, 3.98163, 0.07154],
        [4.97947, -2.1255, 4.0, 0.07221],
        [5.00464, -1.8343, 4.0, 0.07307],
        [5.02265, -1.53976, 4.0, 0.07377],
        [5.03499, -1.24275, 4.0, 0.07432],
        [5.04297, -0.94395, 4.0, 0.07473],
        [5.04773, -0.64389, 4.0, 0.07502],
        [5.0502, -0.343, 4.0, 0.07523],
        [5.05029, -0.04181, 4.0, 0.0753],
        [5.04772, 0.25863, 4.0, 0.07511],
        [5.04206, 0.55788, 4.0, 0.07483]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        track_width = params['track_width']
        is_crashed = params['is_crashed']
        track_width = params['track_width']
        distance_from_center = params['distance_from_center']
        closest_waypoints = params['closest_waypoints']
        is_left_of_center = params['is_left_of_center']
        prev_waypoint_index = closest_waypoints[0]
        next_waypoint_index = closest_waypoints[1]

        ############### OPTIMAL X,Y,SPEED,TIME ################

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
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        normalized_dist = dist/(track_width*0.5)
        distance_reward = max(1e-3, 1 - normalized_dist)
        reward += distance_reward * DISTANCE_MULTIPLE

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
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        STANDARD_TIME = 20
        FASTEST_TIME = 17
        times_list = [row[3] for row in racing_track]

        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        # Harshly punish if direction is obviously wrong
        if direction_diff > 30:
            reward *= 0.01
        elif direction_diff > 25:
            reward *= 0.5
        elif direction_diff > 20:
            reward *= 0.8
            
        inner_border1, outer_border1, inner_border2, outer_border2 = find_border_points(params)
        min_heading, max_heading, is_within_range = find_min_max_heading(params, inner_border2, outer_border2)
        
        # Sharp right turn punishments and bonus.
        if next_waypoint_index > 62 and next_waypoint_index < 73:
            # Punishment for not steering right during sharp right turn.
            if params['steering_angle'] > 0:
                reward *= 0.1
            # Reward slightly for being in correct lane during first half of turn.
            if not params['is_left_of_center']:
                reward += distance_reward
            else:
                reward *= 0.5
                
        # Second half of right turn punishment.
        if next_waypoint_index > 72 and next_waypoint_index < 77:
            # Harsh punishment for steering left during right turn.
            if params['steering_angle'] > 0:
                reward *= 0.1
            else:
                reward += distance_reward
                
        if next_waypoint_index > 76 and next_waypoint_index < 81:
            reward += distance_reward
            
        # Sharp left turn punishments and bonuses
        if (next_waypoint_index > 90 and next_waypoint_index < 103) or (next_waypoint_index > 36 and next_waypoint_index < 49):
            # Punish for steering right during sharp left turn.
            if params['steering_angle'] < 0:
                reward *= 0.1
            # Bonus reward for being in left lane during sharp left turn.
            if params['is_left_of_center']:
                reward += distance_reward
            else: # Punish for being in right lane during sharp left turn.
                reward *= 0.5
        # Straight sections punishments and bonuses.
        if (next_waypoint_index > 112 and next_waypoint_index < 124) or (next_waypoint_index > 143):
            # Harshly punish sharp steering on straight section.
            if params['steering_angle'] > 3 or params['steering_angle'] < -3:
                reward *= 0.1
            # Harshly punish being far off-center during straight seciton (Must be in 90% of track width).
            if params['distance_from_center'] > (track_width/2) * 0.9:
                reward *= 0.1
                
        # Gentle left turn bonuses and punishment. Currently same as sharp turn rewards.
        if (next_waypoint_index > 9 and next_waypoint_index < 15) or (next_waypoint_index > 129 and next_waypoint_index < 136):
            # Punish for steering right during sharp left turn.
            if params['steering_angle'] < 0:
                reward *= 0.1
            # Bonus reward for being in left lane during left turn.
            if params['is_left_of_center']:
                reward += distance_reward
            else: # Punish for being in right lane during left turn.
                reward *= 0.5                
        if (next_waypoint_index > 56 and next_waypoint_index < 62):
            if params['steering_angle'] > 10 or params['steering_angle'] < -10:
                reward *= 0.5
        
        ################ React to State Changes ################
        
        if STATE.prev_direction_diff != None and STATE.prev_normalized_distance_from_route != None and STATE.prev_speed_diff != None and STATE.prev_steering != None:
            steering_angle_change =  params['steering_angle'] != STATE.prev_steering
        
            steering_angle_maintain_bonus = 0
            # Not changing the steering angle is a good thing if heading in the right direction
            if direction_diff < 10 and not steering_angle_change:
                if abs(direction_diff) < 10:
                    steering_angle_maintain_bonus += 0.1
                if abs(direction_diff) < 5:
                    steering_angle_maintain_bonus += 0.25
                if STATE.prev_direction_diff is not None and abs(STATE.prev_direction_diff) > abs(direction_diff):
                    steering_angle_maintain_bonus += 0.1
                    
            reward += steering_angle_maintain_bonus
        
        # Punish harshly for 30+ degree turn angle change, punish moderately for 20+ degree angle change.
        if STATE.prev_steering is not None:
            delta_turn_angle = abs(params['steering_angle'] - STATE.prev_steering)
            if delta_turn_angle > 30:
                reward *= 0.5
            elif delta_turn_angle > 20:
                reward *= 0.8
        
        ################ END React to State Changes ################
        
        ###################### PROGRESS REWARD ######################
        
        # Reward every 10% progress to make lap completion more reliable. The goal of this model is to never crash.
        
        # progress_multiplier = 40 # This determines how much the car prioritizes lap completion.
        
        # pi = int(progress // 10)  # Calculate which 10% segment we're in
        
        # # Initialize intermediate_progress_bonus to 0 for safety
        # intermediate_progress_bonus = 0

        # # Check if this segment has been completed before
        # if pi != 0 and STATE.intermediate_progress[pi] == 0:
        #     cubed_progress_per_step = progress/steps**3
        #     # Reward is equal to the segment's progress (e.g., 10 points for 10%, 20 for 20%)
        #     intermediate_progress_bonus = pi * cubed_progress_per_step * progress_multiplier
        #     # Mark this segment as rewarded
        #     STATE.intermediate_progress[pi] = intermediate_progress_bonus

        # reward += intermediate_progress_bonus
        
        ###################### PROGRESS REWARD ######################
        
        # This gives a harsh penalty if the car is not steering in range.
        if not is_within_range:
            print('Penalizing the car for heading off track.')
            print('heading: ', heading)
            print('min_heading: ', min_heading)
            print('max_heading: ', max_heading )
            reward = 1e-3

        # Zero reward of obviously too slow. Exception made for sharp right turn waypoints.
        speed_diff_zero = optimals[2]-speed
        speed_threshold = 0.5
        if next_waypoint_index > 55 and next_waypoint_index < 79:
            speed_threshold = 1
        if speed_diff_zero > speed_threshold:
            reward = 1e-3

        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 20  # seconds (time that is easily done by model)
        FASTEST_TIME = 18  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
                
        # Zero reward if the center of the car is off the track.
        # This will add a margin of 

        if is_crashed:
            reward = min(reward, 0.001)
        # Cut reward in half if one wheel is off the track
        if not all_wheels_on_track:
            if distance_from_center >= 0.5 * track_width:
                reward = min(reward, 0.001)
                
        ##################### UPDATE STATE #####################
        
        STATE.prev_steering = params['steering_angle']
        STATE.prev_normalized_distance_from_route = normalized_dist
        STATE.prev_direction_diff = direction_diff
        STATE.prev_speed_diff = speed_diff

        ####################### VERBOSE #######################

        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
