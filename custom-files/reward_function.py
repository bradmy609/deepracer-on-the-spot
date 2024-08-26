import numpy as np

class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        import math

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
        racing_track = [[5.03377, 0.85599, 4.0, 0.07449],
        [5.02067, 1.15196, 4.0, 0.07406],
        [5.00262, 1.44563, 3.90501, 0.07535],
        [4.97882, 1.73654, 3.50897, 0.08318],
        [4.94841, 2.02418, 3.15031, 0.09181],
        [4.91038, 2.30792, 2.82465, 0.10135],
        [4.86378, 2.58712, 2.4678, 0.1147],
        [4.80672, 2.86056, 2.18136, 0.12806],
        [4.7373, 3.1269, 1.82795, 0.15057],
        [4.65336, 3.38444, 1.82795, 0.14818],
        [4.55238, 3.63101, 1.82795, 0.14576],
        [4.43051, 3.86306, 1.82795, 0.14339],
        [4.28379, 4.07603, 1.82795, 0.14148],
        [4.10475, 4.25995, 2.00728, 0.12787],
        [3.90112, 4.41773, 2.21803, 0.11614],
        [3.67841, 4.55248, 2.46615, 0.10555],
        [3.44099, 4.66714, 2.71802, 0.097],
        [3.19227, 4.76401, 2.96863, 0.08991],
        [2.93509, 4.84494, 3.21336, 0.0839],
        [2.67185, 4.91148, 3.49997, 0.07758],
        [2.40459, 4.96539, 3.72224, 0.07325],
        [2.13475, 5.00783, 3.93875, 0.06935],
        [1.86347, 5.03985, 4.0, 0.06829],
        [1.59163, 5.0624, 4.0, 0.06819],
        [1.31988, 5.07626, 4.0, 0.06803],
        [1.0487, 5.08214, 4.0, 0.06781],
        [0.77843, 5.08072, 4.0, 0.06757],
        [0.5093, 5.0726, 4.0, 0.06731],
        [0.24151, 5.05819, 4.0, 0.06704],
        [-0.02477, 5.03779, 4.0, 0.06677],
        [-0.28941, 5.01169, 4.0, 0.06648],
        [-0.55223, 4.97997, 3.63376, 0.07285],
        [-0.81288, 4.94222, 3.19388, 0.08246],
        [-1.07096, 4.898, 2.84419, 0.09206],
        [-1.32583, 4.84646, 2.84419, 0.09143],
        [-1.57668, 4.78649, 2.62052, 0.09842],
        [-1.82232, 4.71661, 2.62052, 0.09746],
        [-2.06087, 4.63449, 2.62052, 0.09628],
        [-2.29004, 4.5378, 2.34525, 0.10606],
        [-2.51014, 4.42824, 2.08948, 0.11766],
        [-2.71843, 4.30348, 1.85093, 0.13117],
        [-2.91446, 4.16465, 1.62715, 0.14762],
        [-3.09784, 4.01282, 1.53148, 0.15546],
        [-3.26462, 3.84591, 1.53148, 0.15407],
        [-3.40938, 3.66218, 1.53148, 0.15273],
        [-3.52429, 3.46085, 1.53148, 0.15137],
        [-3.59812, 3.24395, 1.43919, 0.1592],
        [-3.62419, 3.01991, 1.43919, 0.15672],
        [-3.61922, 2.79626, 1.43919, 0.15544],
        [-3.58043, 2.57634, 1.43919, 0.15517],
        [-3.50089, 2.36557, 1.43919, 0.15653],
        [-3.37012, 2.17529, 1.70454, 0.13546],
        [-3.20342, 2.00627, 1.90325, 0.12473],
        [-3.00776, 1.85857, 2.16745, 0.1131],
        [-2.78938, 1.73044, 2.44937, 0.10337],
        [-2.55279, 1.61995, 2.79304, 0.09349],
        [-2.30191, 1.52474, 3.28118, 0.08178],
        [-2.04047, 1.44168, 4.0, 0.06858],
        [-1.77193, 1.36725, 3.89228, 0.0716],
        [-1.4998, 1.29729, 3.30419, 0.08503],
        [-1.23019, 1.22832, 2.87796, 0.0967],
        [-0.96337, 1.15627, 2.53841, 0.10888],
        [-0.70187, 1.07838, 2.25119, 0.1212],
        [-0.44834, 0.99196, 2.00045, 0.1339],
        [-0.20544, 0.89453, 1.77434, 0.1475],
        [0.02392, 0.78383, 1.77434, 0.14353],
        [0.23648, 0.65792, 1.66013, 0.14881],
        [0.42833, 0.51518, 1.49202, 0.16027],
        [0.59466, 0.35447, 1.3, 0.17791],
        [0.72909, 0.17559, 1.3, 0.17213],
        [0.83464, -0.0162, 1.3, 0.16839],
        [0.90457, -0.21992, 1.3, 0.16569],
        [0.93007, -0.43233, 1.3, 0.16457],
        [0.89631, -0.64388, 1.46788, 0.14594],
        [0.81686, -0.84463, 1.60524, 0.1345],
        [0.69981, -1.03056, 1.76957, 0.12416],
        [0.55177, -1.20027, 1.92345, 0.11708],
        [0.37707, -1.35298, 2.05682, 0.11282],
        [0.17843, -1.4878, 2.21469, 0.1084],
        [-0.04144, -1.60433, 2.40278, 0.10356],
        [-0.27977, -1.70268, 2.64123, 0.09762],
        [-0.53357, -1.78376, 2.92627, 0.09105],
        [-0.79995, -1.84901, 3.24072, 0.08463],
        [-1.0764, -1.9, 3.57221, 0.07869],
        [-1.36087, -1.93828, 3.42522, 0.0838],
        [-1.65151, -1.9656, 3.05292, 0.09562],
        [-1.94715, -1.98298, 2.77079, 0.10688],
        [-2.23857, -2.00957, 2.53147, 0.1156],
        [-2.52369, -2.04781, 2.34148, 0.12286],
        [-2.80043, -2.09999, 2.18409, 0.12894],
        [-3.06634, -2.16834, 2.03245, 0.13508],
        [-3.3188, -2.25444, 1.90596, 0.13995],
        [-3.55497, -2.3594, 1.85162, 0.13958],
        [-3.77202, -2.48358, 1.79907, 0.13899],
        [-3.96716, -2.62658, 1.64799, 0.1468],
        [-4.13718, -2.78769, 1.51709, 0.15439],
        [-4.27892, -2.96536, 1.51709, 0.14982],
        [-4.39106, -3.1567, 1.51709, 0.14619],
        [-4.47215, -3.35881, 1.51709, 0.14354],
        [-4.51636, -3.56911, 1.51709, 0.14166],
        [-4.51714, -3.7823, 1.57067, 0.13573],
        [-4.48257, -3.9927, 1.57067, 0.13575],
        [-4.41372, -4.19643, 1.57067, 0.13692],
        [-4.31, -4.38916, 1.57067, 0.13935],
        [-4.16626, -4.56301, 1.76362, 0.1279],
        [-3.99082, -4.71645, 1.95448, 0.11925],
        [-3.78929, -4.84927, 2.15906, 0.11179],
        [-3.56601, -4.96198, 2.37193, 0.10545],
        [-3.32441, -5.05528, 2.62428, 0.09869],
        [-3.06769, -5.13048, 2.89644, 0.09236],
        [-2.79858, -5.18903, 3.27234, 0.08416],
        [-2.51989, -5.23324, 3.67995, 0.07668],
        [-2.23384, -5.26522, 4.0, 0.07196],
        [-1.9424, -5.28737, 4.0, 0.07307],
        [-1.64708, -5.3018, 4.0, 0.07392],
        [-1.34893, -5.31011, 4.0, 0.07456],
        [-1.04894, -5.31407, 4.0, 0.07501],
        [-0.74784, -5.31525, 4.0, 0.07528],
        [-0.4466, -5.31385, 4.0, 0.07531],
        [-0.14641, -5.30981, 4.0, 0.07505],
        [0.1525, -5.30287, 4.0, 0.07475],
        [0.44996, -5.29277, 4.0, 0.07441],
        [0.74573, -5.27911, 4.0, 0.07402],
        [1.03955, -5.26144, 4.0, 0.07359],
        [1.33109, -5.2392, 4.0, 0.0731],
        [1.61996, -5.21174, 3.66866, 0.0791],
        [1.90568, -5.17827, 3.3013, 0.08714],
        [2.18766, -5.13786, 2.95538, 0.09639],
        [2.46518, -5.08946, 2.62777, 0.10721],
        [2.73719, -5.03154, 2.36492, 0.1176],
        [3.0025, -4.96247, 2.02113, 0.13564],
        [3.25954, -4.88035, 1.80091, 0.14984],
        [3.50631, -4.78288, 1.80091, 0.14733],
        [3.74006, -4.66721, 1.80091, 0.14482],
        [3.95762, -4.53061, 1.80091, 0.14264],
        [4.15238, -4.3674, 1.80091, 0.1411],
        [4.31646, -4.17332, 2.14819, 0.11831],
        [4.45723, -3.95885, 2.38824, 0.10742],
        [4.5778, -3.72824, 2.59804, 0.10016],
        [4.6801, -3.48408, 2.82433, 0.09373],
        [4.76582, -3.22842, 3.07302, 0.08775],
        [4.83657, -2.96303, 3.3472, 0.08206],
        [4.89387, -2.68942, 3.65621, 0.07646],
        [4.93924, -2.40894, 4.0, 0.07103],
        [4.97426, -2.12287, 4.0, 0.07205],
        [5.00041, -1.8323, 4.0, 0.07294],
        [5.0193, -1.53828, 4.0, 0.07366],
        [5.03241, -1.24167, 4.0, 0.07422],
        [5.04118, -0.94324, 4.0, 0.07464],
        [5.0466, -0.64347, 4.0, 0.07495],
        [5.04965, -0.3428, 4.0, 0.07517],
        [5.05019, -0.04169, 4.0, 0.07528],
        [5.048, 0.25881, 4.0, 0.07513],
        [5.04269, 0.55815, 4.0, 0.07485]]

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
        next_waypoint_index = closest_waypoints[1]
        prev_waypoint_index = closest_waypoints[0]
        is_offtrack = params['is_offtrack']

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
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
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
        REWARD_PER_STEP_FOR_FASTEST_TIME = 2.5
        STANDARD_TIME = 21
        FASTEST_TIME = 15
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

        ################## DIRECTION PUNISHMENT ##################

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        # Harsh punishment if off by 30+ degrees.
        if direction_diff > 30:
            reward = 1e-3
        # Moderate punishment if off by 25+ degrees.
        elif direction_diff > 25:
            reward *= 0.5
        # Mild punishment if off by 20+ degrees.
        elif direction_diff > 20:
            reward *= 0.8
        
        ################## END DIRECTION PUNISHMENT ##################
        
        ################## HARDCODED WAYPOINT INCENTIVES ##################
        
        # # Reward for staying inside on sharp 180 degree right turn, punishment for being outside.
        # if next_waypoint_index > 63 and next_waypoint_index < 80:
        #     if params['is_left_of_center']:
        #         reward *= 0.5 # Half reward if in left during right turn.
        #     else:
        #         reward += 0.5 # Slight reward for being in right during right turn.
                
        # # Slight reward for staying inside on sharp 180 degree left turn, punishment for being outside.
        # if next_waypoint_index > 91 and next_waypoint_index < 104:
        #     if params['is_left_of_center']:
        #         reward += 0.5 # Slight reward for being in left during left turn.
        #     else:
        #         reward *= 0.5 # Half reward if in right during left turn.
        
        ################## END WAYPOINT INCENTIVES #####################

        inner_border1, outer_border1, inner_border2, outer_border2 = find_border_points(params)
        min_heading, max_heading, is_within_range = find_min_max_heading(params, inner_border2, outer_border2)
        
        # Steering limiter. If car isn't in range of triangle formed between car and next border 
        # waypoints, set reward to zero.
        # if not is_within_range:
        #     reward = 1e-3
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3

        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1000 # should be adapted to track length and other rewards
        STANDARD_TIME = 21  # seconds (time that is easily done by model)
        FASTEST_TIME = 17.5  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward

        ## Zero reward if off track ##
        is_crashed = params['is_crashed']
        track_width = params['track_width']
        distance_from_center = params['distance_from_center']

        # Zero reward if the center of the car is off the track.

        # Harsh punishment for half the car being off-track.
        if not all_wheels_on_track and abs(distance_from_center) > (track_width/2):
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
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)

