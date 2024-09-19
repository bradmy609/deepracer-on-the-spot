import numpy as np

class STATE:
    def __init__(self):
        # Initialize all attributes with their default values
        self.prev_turn_angle = None
        self.prev_speed_diff = None
        self.prev_distance = None
        self.prev_speed = None
        self.prev_wp_index = None
        self.steps_at_waypoint = 0
        self.prev_progress = 0
        self.first_waypoint_index = None
        
        self.progress_intervals = {10: None, 20: None, 30: None, 40: None, 50: None, 60: None, 70: None, 80: None, 90: None, 100: None}
        self.next_progress_interval = list(self.progress_intervals.keys())[0]  # Start with 10%
        self.steps_at_last_interval = 0
        
        self.wp_rewards = {i: None for i in range(213)}
        
    # Optional: You could also define a reset method to reset all attributes
    def reset(self):
        self.prev_turn_angle = None
        self.prev_speed_diff = None
        self.prev_distance = None
        self.prev_speed = None
        self.prev_wp_index = None
        self.steps_at_waypoint = 0
        self.prev_progress = 0
        self.first_waypoint_index = None
        
        self.progress_intervals = {10: None, 20: None, 30: None, 40: None, 50: None, 60: None, 70: None, 80: None, 90: None, 100: None}
        self.next_progress_interval = list(self.progress_intervals.keys())[0]  # Start with 10%
        self.steps_at_last_interval = 0
        
state = STATE()

class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        import math

        ################## HELPER FUNCTIONS ###################
        def reset_state(steps, prev_waypoint_index):
            if steps <= 2:
                state.first_waypoint_index = prev_waypoint_index
                print(f'Resetting state...')
                state.reset()
        
        def calculate_progress_reward(params, state):
            progress = params['progress']  # Get the current progress percentage
            steps = params['steps']  # Get the number of steps

            progress_interval_reward = 0

            # Iterate through the progress intervals and find the next one that hasn't been rewarded
            for interval in state.progress_intervals.keys():
                # Check if the progress is at or above the current interval and if the reward hasn't been given
                if progress >= interval and state.progress_intervals[interval] is None:
                    pi = int(progress // 10)
                    intermediate_progress_bonus = 0

                    if pi != 0 and state.progress_intervals.get(pi * 10) is None:
                        if progress/steps >= 0.37:
                            intermediate_progress_bonus = (progress/steps - 0.37) * 200 * pi
                        else:
                            intermediate_progress_bonus = 0

                        state.progress_intervals[pi * 10] = intermediate_progress_bonus

                    progress_interval_reward = max(0, intermediate_progress_bonus)  # Ensure no negative reward

                    # Mark the interval as rewarded
                    state.progress_intervals[interval] = progress_interval_reward

                    # Update steps_at_last_interval to track the current step
                    state.steps_at_last_interval = steps

                    # Break after rewarding one interval to avoid double rewards
                    break

            return progress_interval_reward

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
            half_width += 0.3
            
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

        #################### RACING LINE ######################

        # Optimal racing line
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[5.03377, 0.85599, 4.0, 0.07449],
        [5.02067, 1.15196, 4.0, 0.07406],
        [5.00262, 1.44563, 3.77889, 0.07786],
        [4.97882, 1.73654, 3.39264, 0.08603],
        [4.94841, 2.02418, 3.04193, 0.09509],
        [4.91038, 2.30792, 2.65763, 0.10772],
        [4.86378, 2.58712, 2.34916, 0.12049],
        [4.80672, 2.86056, 1.96857, 0.1419],
        [4.7373, 3.1269, 1.96857, 0.13982],
        [4.65336, 3.38444, 1.96857, 0.1376],
        [4.55238, 3.63101, 1.96857, 0.13535],
        [4.43051, 3.86306, 1.96857, 0.13315],
        [4.28379, 4.07603, 1.96857, 0.13137],
        [4.10475, 4.25995, 2.16169, 0.11874],
        [3.90112, 4.41773, 2.38864, 0.10785],
        [3.67841, 4.55248, 2.65585, 0.09801],
        [3.44099, 4.66714, 2.9271, 0.09007],
        [3.19227, 4.76401, 3.19699, 0.08349],
        [2.93509, 4.84494, 3.46054, 0.07791],
        [2.67185, 4.91148, 3.7692, 0.07204],
        [2.40459, 4.96539, 4.0, 0.06816],
        [2.13475, 5.00783, 4.0, 0.06829],
        [1.86347, 5.03985, 4.0, 0.06829],
        [1.59163, 5.0624, 4.0, 0.06819],
        [1.31988, 5.07626, 4.0, 0.06803],
        [1.0487, 5.08214, 4.0, 0.06781],
        [0.77843, 5.08072, 4.0, 0.06757],
        [0.5093, 5.0726, 4.0, 0.06731],
        [0.24151, 5.05819, 4.0, 0.06704],
        [-0.02477, 5.03779, 4.0, 0.06677],
        [-0.28941, 5.01169, 3.91328, 0.06795],
        [-0.55223, 4.97997, 3.43957, 0.07696],
        [-0.81288, 4.94222, 3.06297, 0.08599],
        [-1.07096, 4.898, 3.06297, 0.08549],
        [-1.32583, 4.84646, 2.8221, 0.09214],
        [-1.57668, 4.78649, 2.8221, 0.09139],
        [-1.82232, 4.71661, 2.8221, 0.09049],
        [-2.06087, 4.63449, 2.52566, 0.09989],
        [-2.29004, 4.5378, 2.2502, 0.11054],
        [-2.51014, 4.42824, 1.99331, 0.12334],
        [-2.71843, 4.30348, 1.75231, 0.13856],
        [-2.91446, 4.16465, 1.64929, 0.14564],
        [-3.09784, 4.01282, 1.64929, 0.14435],
        [-3.26462, 3.84591, 1.64929, 0.14307],
        [-3.40938, 3.66218, 1.64929, 0.14182],
        [-3.52429, 3.46085, 1.5499, 0.14957],
        [-3.59812, 3.24395, 1.5499, 0.14783],
        [-3.62419, 3.01991, 1.5499, 0.14553],
        [-3.61922, 2.79626, 1.5499, 0.14433],
        [-3.58043, 2.57634, 1.5499, 0.14409],
        [-3.50089, 2.36557, 1.5499, 0.14535],
        [-3.37012, 2.17529, 1.83566, 0.12578],
        [-3.20342, 2.00627, 2.04965, 0.11582],
        [-3.00776, 1.85857, 2.33418, 0.10502],
        [-2.78938, 1.73044, 2.63778, 0.09599],
        [-2.55279, 1.61995, 2.7, 0.08681],
        [-2.30191, 1.52474, 2.9, 0.07594],
        [-2.04047, 1.44168, 3.1, 0.06858],
        [-1.77193, 1.36725, 2.9, 0.07831],
        [-1.4998, 1.29729, 2.7, 0.09065],
        [-1.23019, 1.22832, 2.60, 0.1018],
        [-0.96337, 1.15627, 2.42436, 0.114],
        [-0.70187, 1.07838, 2.15433, 0.12665],
        [-0.44834, 0.99196, 1.91083, 0.14018],
        [-0.20544, 0.89453, 1.91083, 0.13696],
        [0.02392, 0.78383, 1.78783, 0.14245],
        [0.23648, 0.65792, 1.60679, 0.15375],
        [0.42833, 0.51518, 1.4, 0.17081],
        [0.59466, 0.35447, 1.4, 0.1652],
        [0.72909, 0.17559, 1.4, 0.15983],
        [0.83464, -0.0162, 1.4, 0.15636],
        [0.90457, -0.21992, 1.4, 0.15385],
        [0.93007, -0.43233, 1.4, 0.15282],
        [0.89631, -0.64388, 1.5808, 0.13551],
        [0.81686, -0.84463, 1.72872, 0.12489],
        [0.69981, -1.03056, 1.90569, 0.11529],
        [0.55177, -1.20027, 2.07141, 0.10872],
        [0.37707, -1.35298, 2.21504, 0.10476],
        [0.17843, -1.4878, 2.38505, 0.10066],
        [-0.04144, -1.60433, 2.58761, 0.09616],
        [-0.27977, -1.70268, 2.8444, 0.09064],
        [-0.53357, -1.78376, 3.05137, 0.08455],
        [-0.79995, -1.84901, 3.19001, 0.07858],
        [-1.0764, -1.9, 3.28869, 0.07621],
        [-1.36087, -1.93828, 3.18776, 0.0873],
        [-1.65151, -1.9656, 2.98393, 0.09783],
        [-1.94715, -1.98298, 2.72619, 0.10863],
        [-2.23857, -2.00957, 2.52159, 0.11605],
        [-2.52369, -2.04781, 2.3521, 0.1223],
        [-2.80043, -2.09999, 2.18879, 0.12867],
        [-3.06634, -2.16834, 2.05257, 0.13376],
        [-3.3188, -2.25444, 1.99405, 0.13377],
        [-3.55497, -2.3594, 1.93746, 0.13339],
        [-3.77202, -2.48358, 1.77475, 0.1409],
        [-3.96716, -2.62658, 1.63379, 0.14808],
        [-4.13718, -2.78769, 1.63379, 0.14336],
        [-4.27892, -2.96536, 1.63379, 0.13912],
        [-4.39106, -3.1567, 1.63379, 0.13574],
        [-4.47215, -3.35881, 1.63379, 0.13329],
        [-4.51636, -3.56911, 1.63379, 0.13154],
        [-4.51714, -3.7823, 1.6915, 0.12604],
        [-4.48257, -3.9927, 1.6915, 0.12606],
        [-4.41372, -4.19643, 1.6915, 0.12714],
        [-4.31, -4.38916, 1.6915, 0.12939],
        [-4.16626, -4.56301, 1.89929, 0.11877],
        [-3.99082, -4.71645, 2.10482, 0.11073],
        [-3.78929, -4.84927, 2.32514, 0.10381],
        [-3.56601, -4.96198, 2.55438, 0.09791],
        [-3.32441, -5.05528, 2.82615, 0.09164],
        [-3.06769, -5.13048, 3.11925, 0.08576],
        [-2.79858, -5.18903, 3.52406, 0.07815],
        [-2.51989, -5.23324, 3.96302, 0.0712],
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
        [1.33109, -5.2392, 3.95087, 0.07401],
        [1.61996, -5.21174, 3.55524, 0.08162],
        [1.90568, -5.17827, 3.18272, 0.09039],
        [2.18766, -5.13786, 2.8299, 0.10066],
        [2.46518, -5.08946, 2.54684, 0.11061],
        [2.73719, -5.03154, 2.1766, 0.12777],
        [3.0025, -4.96247, 1.93944, 0.14135],
        [3.25954, -4.88035, 1.93944, 0.13914],
        [3.50631, -4.78288, 1.93944, 0.1368],
        [3.74006, -4.66721, 1.93944, 0.13447],
        [3.95762, -4.53061, 1.93944, 0.13246],
        [4.15238, -4.3674, 1.93944, 0.13102],
        [4.31646, -4.17332, 2.31343, 0.10986],
        [4.45723, -3.95885, 2.57195, 0.09974],
        [4.5778, -3.72824, 2.79789, 0.09301],
        [4.6801, -3.48408, 3.04159, 0.08704],
        [4.76582, -3.22842, 3.3094, 0.08148],
        [4.83657, -2.96303, 3.60468, 0.0762],
        [4.89387, -2.68942, 3.93746, 0.071],
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
        
        def calculate_passed_waypoints(start_index, end_index, waypoint_count):
            passed_waypoints = []
            if start_index != None:
                # If end_index is greater than start_index (no wraparound)
                if end_index >= start_index:
                    passed_waypoints = list(range(start_index, end_index + 1))
                else:
                    # Handle wraparound (from, e.g., 153 back to 1)
                    passed_waypoints = list(range(start_index, waypoint_count)) + list(range(0, end_index + 1))
            else:
                print('Start index has not been initialized.')
            return passed_waypoints

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
        
        optimal_times = [racing_track[i][3] for i in range(len(racing_track))]
        total_time = sum(optimal_times)
        
        waypoint_count = len(optimal_times)  # 153 waypoints
        passed_waypoints = calculate_passed_waypoints(state.first_waypoint_index, prev_waypoint_index, waypoint_count)
        # Determine waypoints passed in the last 5 steps

        # Sum the times for the waypoints passed
        passed_time_sum = sum(optimal_times[i] for i in passed_waypoints)

        # Calculate the fraction of total time
        fraction_of_total_time = passed_time_sum / total_time
        global_prog_per_step = 0.37
        waypoints_passed = len(passed_waypoints)  # Number of waypoints passed in this segment
            
        global_prog_per_step = 0.37

        # Calculate the goal prog/step for the current segment
        goal_prog_per_step = 0.37
        if fraction_of_total_time > 0:  # Avoid division by zero
            goal_prog_per_step = global_prog_per_step / fraction_of_total_time * waypoints_passed/len(optimal_times)
        elif progress >= 99:
            goal_prog_per_step = .37
        else:
            goal_prog_per_step = .37  # Handle the edge case
        
        if progress/steps >= goal_prog_per_step and steps < 270 and steps % 5 == 0:
            step_interval_reward = ((progress/steps - goal_prog_per_step) * 2500) * (progress/15)
        else:
            step_interval_reward = 0
        
        reset_state(steps, prev_waypoint_index)

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
        
        delta_progress = progress - state.prev_progress
        if delta_progress < 0:
            print(f'progress: {progress}')
            print(f'prev_progress: {state.prev_progress}')
            print(f'steps: {steps}')
            # I think when resetting an episode on first step we are having issues while prev_progress is being reset.
            # This will check if we are on the first step and progress is in the expected value range, then it will just
            # use current progress as delta progress.
            if steps <= 1 and progress < 1:
                delta_progress = progress
        if delta_progress > 1:
            delta_progress == 1
        progress_reward = max(0, delta_progress)
        
        
        inner_border1, outer_border1, inner_border2, outer_border2 = find_border_points(params)
        min_heading, max_heading, is_within_range = find_min_max_heading(params, inner_border2, outer_border2)
                
        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        
        SPEED_THRESHOLD = 0.5
        SPEED_PUNISHMENT = 0.1
        SPEED_MULTIPLE = 2
        DISTANCE_MULTIPLE = 1
        DISTANCE_EXPONENT = 1
        SPEED_CAP = None
        SUPER_FAST_BONUS = 0
        STEERING_PUNISHMENT = 1
        straight_steering_bonus = 0
        # 90 degree left turns (half speed, half distance reward to tighten turns)
        if (next_waypoint_index >= 9 and next_waypoint_index <= 16) or (next_waypoint_index >= 130 and next_waypoint_index <= 139):
            DISTANCE_EXPONENT = 2
            DISTANCE_MULTIPLE = 1.5
            SPEED_MULTIPLE = 1.5
            SPEED_THRESHOLD = 0.5
            SPEED_PUNISHMENT = 0.1
            SPEED_CAP = None
        # Set dist multiplier to 2 and speed threshold to 1 for sharp turns.
        elif next_waypoint_index >= 62 and next_waypoint_index <= 78:
            DISTANCE_EXPONENT = 2
            DISTANCE_MULTIPLE = 2
            SPEED_THRESHOLD = 0.5
            SPEED_PUNISHMENT = 0.1
            SPEED_MULTIPLE = 1
            SPEED_CAP = 3
            if next_waypoint_index > 63 and next_waypoint_index < 76:
                SPEED_CAP = 2.5
            if steering_angle > 5:
                STEERING_PUNISHMENT = 0.1
        # Set distance multiplier to 2 and speed threshold to 1 for sharp turns.
        elif (next_waypoint_index >= 91 and next_waypoint_index <= 106) or (next_waypoint_index >= 34 and next_waypoint_index <= 54):
            DISTANCE_EXPONENT = 2
            DISTANCE_MULTIPLE = 2
            SPEED_THRESHOLD = 0.5
            SPEED_PUNISHMENT = 0.5
            SPEED_MULTIPLE = 1
            SPEED_CAP = 3.0
            if steering_angle < -5:
                STEERING_PUNISHMENT = 0.5
        # For sections going into turns or coming out of turns to allow the car to go unpunished while getting up to speed.
        elif (next_waypoint_index >= 0 and next_waypoint_index <= 8) or (next_waypoint_index >= 107 and next_waypoint_index <= 111)\
        or (next_waypoint_index >= 54 and next_waypoint_index <= 56) or (next_waypoint_index >= 79 and next_waypoint_index <= 82)\
        or (next_waypoint_index >= 17 and next_waypoint_index <= 21) or (next_waypoint_index >= 140 and next_waypoint_index <= 143)\
        or (next_waypoint_index >= 127 and next_waypoint_index <= 129) or (next_waypoint_index >= 88 and next_waypoint_index <= 90):
            DISTANCE_EXPONENT = 1.25
            DISTANCE_MULTIPLE = 1.25
            SPEED_MULTIPLE = 1.75
            SPEED_THRESHOLD = 0.75
            SPEED_PUNISHMENT = 0.5
        else: # Values for non-turning sections. Punish speed off by 0.5 harshly, reduce dist reward.
            if steering_angle > 5 or steering_angle < -5:
                STEERING_PUNISHMENT = 0.5
            else:
                STEERING_PUNISHMENT = 1
            straight_steering_bonus = max(0.001, .2 - (abs(steering_angle)/150))
            DISTANCE_EXPONENT = 1.0
            DISTANCE_MULTIPLE = 1.0
            SPEED_THRESHOLD = 0.5
            SPEED_PUNISHMENT = 0.5
            SPEED_MULTIPLE = 2.0
            SPEED_CAP = None
        if (21 <= next_waypoint_index < 30) or (112 <= next_waypoint_index <= 124) or (next_waypoint_index >= 140) or (next_waypoint_index <= 2):
            # Bonus reward if going 4 m/s or faster during optimal spots
            if speed >= 3.95:
                SUPER_FAST_BONUS = 1
                
        # reward = add_bonus_reward(next_waypoint_index, distance_reward, reward)
        def calculate_wp_reward(params, state):
            closest_wp_index = params['closest_waypoints'][1]
            wp_reward = 0

            # Check if previous waypoint index is set
            if state.prev_wp_index is not None:
                if state.prev_wp_index == closest_wp_index:
                    # Increment steps if still at the same waypoint
                    state.steps_at_waypoint += 1
                else:
                    # If we moved to a new waypoint
                    skipped_waypoints = []  # Initialize the skipped waypoints list

                    # Handle skipped waypoints (i.e., if we jumped more than 1 waypoint)
                    if closest_wp_index > state.prev_wp_index + 1:
                        skipped_waypoints = range(state.prev_wp_index + 1, closest_wp_index)
                    
                    # Check and reward the current waypoint if not already rewarded
                    if state.wp_rewards[closest_wp_index] is None:
                        wp_reward += max(0, (3 - state.steps_at_waypoint))
                        state.wp_rewards[state.prev_wp_index] = wp_reward

                    # Reward any skipped waypoints if they haven't been rewarded
                    for i in skipped_waypoints:
                        if state.wp_rewards[i] is None:
                            state.wp_rewards[i] = 3  # Full reward for skipped waypoints
                            wp_reward += 3

                    # Handle the case where the car wraps around from the last waypoint (212) to the first (0)
                    if closest_wp_index == 0 and state.prev_wp_index == 212 and state.wp_rewards[closest_wp_index] is None:
                        wp_reward += max(0, (3 - state.steps_at_waypoint))
                        state.wp_rewards[state.prev_wp_index] = wp_reward

                    # Update the previous waypoint index and reset steps for the new waypoint
                    state.prev_wp_index = closest_wp_index
                    state.steps_at_waypoint = 0  # Start with 0 steps for the new waypoint
            else:
                # Initialize steps if it's the first step
                state.steps_at_waypoint += 1
                state.prev_wp_index = closest_wp_index

            return wp_reward
        
        progress_interval_reward = calculate_progress_reward(params, state)
        if distance_reward > 0.9:
            step_interval_multiple = 1
        else:
            step_interval_multiple = distance_reward
        
        progress_multiplier = 4
        DC = (distance_reward**DISTANCE_EXPONENT) * DISTANCE_MULTIPLE
        SC = speed_reward * SPEED_MULTIPLE
        PC = (progress_reward) * progress_multiplier
        PIC = progress_interval_reward
        SIC = step_interval_reward * step_interval_multiple
        # distance component, speed component, and progress_component
        if SIC > 0:
            print(f'SIC: {SIC}')
        if steps % 100 == 0:
            print(f'steps: {steps}')
            print(f'delta_progress: {progress-state.prev_progress}')
            print(f'SIC: {SIC}')
            print(f'DC: {DC}\nPC: {PC}, SUPER_FAST_BONUS: {SUPER_FAST_BONUS}\nstraight_steering_bonus: {straight_steering_bonus}')
        reward += DC + SC + SIC + SUPER_FAST_BONUS + straight_steering_bonus
        
        if state.prev_turn_angle is not None and state.prev_speed_diff is not None and state.prev_distance is not None and state.prev_speed is not None:
            delta_turn_angle = abs(steering_angle - state.prev_turn_angle)
            delta_speed = abs(speed - state.prev_speed)
            # Speed maintain bonus if speed is close to optimal
            if delta_speed <= 0.1 and speed_diff <= 0.1:
                reward += 0.1
            # Bonus for small steering changes when close to racing line.
            if delta_turn_angle <= 3 and dist <= 0.1:
                reward += 0.1
            # Erratic steering punishments
            if state.prev_turn_angle > 10 and steering_angle < -10:
                reward *= 0.1
            elif state.prev_turn_angle < -10 and steering_angle > 10:
                reward *= 0.1
            elif delta_turn_angle >= 30:
                reward = min(reward, 0.001)
        
        # Punishing erratic steering or steering out of range of valid directions.
        if speed > 2.5 and (steering_angle >= 20 or steering_angle <= -20):
            reward *= 0.1
        if not is_within_range:
            reward *= 0.01
        if direction_diff > 30:
            reward = 1e-3
        
        # Punishing too fast or too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > SPEED_THRESHOLD:
            reward *= SPEED_PUNISHMENT
        if SPEED_CAP is not None and speed > SPEED_CAP:
            reward *= 0.01
        
        reward *= DISTANCE_PUNISHMENT
        reward *= STEERING_PUNISHMENT

        ## Zero reward if off track ##
        track_width = params['track_width']
        distance_from_center = params['distance_from_center']
        
        if progress == 100:
            # finish reward starts scaling up when the steps are below 300, or time is below 20s.
            finish_reward = ((1 - (steps/285)) * 5000) + 10
            # Don't let finish_reward fall below 10.
            if finish_reward < 10:
                finish_reward = 10
            reward += finish_reward
        else:
            finish_reward = 0

        # Zero reward if the center of the car is off the track.

        if not all_wheels_on_track and distance_from_center >= (track_width/2)+0.05:
            if steps % 5 == 0:
                print(f'state.wp_rewards: {state.wp_rewards}')
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

        #################### RETURN REWARD ####################
        
        state.prev_turn_angle = steering_angle
        state.prev_speed_diff = speed_diff
        state.prev_distance = dist
        state.prev_speed = speed
        state.prev_progress = progress
        state.first_waypoint_index = prev_waypoint_index

        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)