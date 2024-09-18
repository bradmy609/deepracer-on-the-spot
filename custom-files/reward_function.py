import numpy as np
import math

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
        
        self.progress_intervals = {10: None, 20: None, 30: None, 40: None, 50: None, 60: None, 70: None, 80: None, 90: None, 100: None}
        self.next_progress_interval = list(self.progress_intervals.keys())[0]  # Start with 10%
        self.steps_at_last_interval = 0
        
    # Optional: You could also define a reset method to reset all attributes
    def reset(self):
        self.prev_turn_angle = None
        self.prev_speed_diff = None
        self.prev_distance = None
        self.prev_speed = None
        self.prev_wp_index = None
        self.steps_at_waypoint = 0
        self.prev_progress = 0
        
        self.progress_intervals = {10: None, 20: None, 30: None, 40: None, 50: None, 60: None, 70: None, 80: None, 90: None, 100: None}
        self.next_progress_interval = list(self.progress_intervals.keys())[0]  # Start with 10%
        self.steps_at_last_interval = 0
        
state = STATE()

class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        ################## HELPER FUNCTIONS ###################
        def reset_state(steps):
            if steps == 2:
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
                    progress_reward = 10 * (progress / steps) if steps != 0 else 0
                    intermediate_progress_bonus = 0

                    if pi != 0 and state.progress_intervals.get(pi * 10) is None:
                        if progress/steps > 0.37:
                            intermediate_progress_bonus = (progress/steps - 0.37) * 1000

                        state.progress_intervals[pi * 10] = intermediate_progress_bonus
                        print(intermediate_progress_bonus)

                    progress_interval_reward = max(0, intermediate_progress_bonus)  # Ensure no negative reward

                    # Mark the interval as rewarded
                    state.progress_intervals[interval] = progress_interval_reward

                    # Update steps_at_last_interval to track the current step
                    state.steps_at_last_interval = steps

                    # Break after rewarding one interval to avoid double rewards
                    break

            return progress_interval_reward

        # def calculate_wp_reward(params, state):
        #     closest_wp_index = params['closest_waypoints'][1]
        #     wp_reward = 0

        #     # Check if previous waypoint index is set
        #     if state.prev_wp_index is not None:
        #         if state.prev_wp_index == closest_wp_index:
        #             # Increment steps if still at the same waypoint
        #             state.steps_at_waypoint += 1
        #         else:
        #             # If we moved to a new waypoint
        #             skipped_waypoints = []  # Initialize the skipped waypoints list

        #             # Handle skipped waypoints (i.e., if we jumped more than 1 waypoint)
        #             if closest_wp_index > state.prev_wp_index + 1:
        #                 skipped_waypoints = range(state.prev_wp_index + 1, closest_wp_index)
                    
        #             # Check and reward the current waypoint if not already rewarded
        #             if state.wp_rewards[closest_wp_index] is None:
        #                 wp_reward += max(0, (4 - state.steps_at_waypoint))
        #                 state.wp_rewards[state.prev_wp_index] = wp_reward

        #             # Reward any skipped waypoints if they haven't been rewarded
        #             for i in skipped_waypoints:
        #                 if state.wp_rewards[i] is None:
        #                     state.wp_rewards[i] = 4  # Full reward for skipped waypoints
        #                     wp_reward += 4

        #             # Handle the case where the car wraps around from the last waypoint (212) to the first (0)
        #             if closest_wp_index == 0 and state.prev_wp_index == 212 and state.wp_rewards[closest_wp_index] is None:
        #                 wp_reward += max(0, (4 - state.steps_at_waypoint))
        #                 state.wp_rewards[state.prev_wp_index] = wp_reward

        #             # Update the previous waypoint index and reset steps for the new waypoint
        #             state.prev_wp_index = closest_wp_index
        #             state.steps_at_waypoint = 0  # Start with 0 steps for the new waypoint
        #     else:
        #         # Initialize steps if it's the first step
        #         state.steps_at_waypoint += 1
        #         state.prev_wp_index = closest_wp_index

        #     return wp_reward


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
        
        def scale_value(x, old_min=1, old_max=2.9, new_min=1, new_max=2):
            # Scale the value from the old range to the new range
            scaled_value = new_min + ((x - old_min) / (old_max - old_min)) * (new_max - new_min)
            return scaled_value

        #################### RACING LINE ######################

        # Optimal racing line
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[-0.74955, -5.35735, 4.0, 0.06678],
        [-0.48316, -5.34517, 4.0, 0.06667],
        [-0.21709, -5.33113, 4.0, 0.06661],
        [0.04881, -5.31502, 4.0, 0.0666],
        [0.31469, -5.29667, 4.0, 0.06663],
        [0.58067, -5.2759, 4.0, 0.0667],
        [0.84685, -5.25257, 4.0, 0.0668],
        [1.11331, -5.22659, 4.0, 0.06693],
        [1.38011, -5.19793, 4.0, 0.06708],
        [1.64726, -5.1666, 4.0, 0.06725],
        [1.91476, -5.1326, 4.0, 0.06742],
        [2.18262, -5.09587, 4.0, 0.06759],
        [2.45082, -5.05628, 4.0, 0.06778],
        [2.71933, -5.01343, 4.0, 0.06798],
        [2.9881, -4.96686, 3.84552, 0.07093],
        [3.257, -4.91593, 3.34894, 0.08172],
        [3.52577, -4.85978, 3.20633, 0.08563],
        [3.79398, -4.79752, 3.20633, 0.08588],
        [4.06078, -4.72727, 3.20633, 0.08605],
        [4.32461, -4.64699, 3.20633, 0.08601],
        [4.58294, -4.5542, 2.81194, 0.09762],
        [4.83215, -4.44601, 2.50848, 0.10831],
        [5.0701, -4.32231, 2.22368, 0.1206],
        [5.29977, -4.18853, 1.96846, 0.13502],
        [5.51922, -4.0433, 1.71093, 0.15381],
        [5.72641, -3.88554, 1.71093, 0.15221],
        [5.91678, -3.71246, 1.71093, 0.15038],
        [6.08529, -3.52219, 1.71093, 0.14855],
        [6.22491, -3.31313, 1.71093, 0.14693],
        [6.32603, -3.08464, 1.71093, 0.14604],
        [6.37314, -2.8377, 2.34526, 0.10719],
        [6.38996, -2.5836, 2.65305, 0.09599],
        [6.38193, -2.32449, 3.08715, 0.08397],
        [6.35453, -2.06193, 3.60661, 0.0732],
        [6.31221, -1.79693, 4.0, 0.06709],
        [6.25969, -1.53037, 4.0, 0.06792],
        [6.20201, -1.26305, 4.0, 0.06837],
        [6.14399, -0.97325, 4.0, 0.07389],
        [6.08977, -0.68265, 4.0, 0.0739],
        [6.03868, -0.3914, 4.0, 0.07393],
        [5.99029, -0.09958, 4.0, 0.07395],
        [5.94426, 0.19274, 4.0, 0.07398],
        [5.90029, 0.48548, 4.0, 0.07401],
        [5.85809, 0.77859, 4.0, 0.07403],
        [5.81745, 1.07202, 4.0, 0.07406],
        [5.77826, 1.36576, 4.0, 0.07408],
        [5.74046, 1.65978, 4.0, 0.07411],
        [5.70403, 1.95409, 3.54043, 0.08376],
        [5.67039, 2.23786, 3.09327, 0.09238],
        [5.63395, 2.51992, 2.70914, 0.10498],
        [5.59199, 2.79855, 2.37139, 0.11882],
        [5.54196, 3.07205, 2.03216, 0.13682],
        [5.48115, 3.3385, 1.86197, 0.14678],
        [5.40672, 3.59563, 1.86197, 0.14376],
        [5.31576, 3.84076, 1.86197, 0.14042],
        [5.20513, 4.07043, 1.86197, 0.13692],
        [5.07141, 4.27999, 1.86197, 0.13351],
        [4.91006, 4.4616, 1.86197, 0.13047],
        [4.72047, 4.60846, 2.10852, 0.11374],
        [4.51137, 4.7269, 2.33263, 0.10303],
        [4.28726, 4.82092, 2.60229, 0.09339],
        [4.05142, 4.89413, 2.88938, 0.08546],
        [3.80609, 4.9494, 3.22248, 0.07804],
        [3.55292, 4.98937, 3.64287, 0.07036],
        [3.29316, 5.01668, 4.0, 0.0653],
        [3.02779, 5.03397, 4.0, 0.06648],
        [2.75785, 5.04408, 4.0, 0.06753],
        [2.48407, 5.04958, 4.0, 0.06846],
        [2.20106, 5.05286, 4.0, 0.07076],
        [1.91812, 5.0553, 4.0, 0.07074],
        [1.63619, 5.0561, 4.0, 0.07048],
        [1.35559, 5.05444, 4.0, 0.07015],
        [1.07652, 5.04945, 3.64498, 0.07657],
        [0.79925, 5.04006, 3.14099, 0.08833],
        [0.52399, 5.02502, 3.14099, 0.08776],
        [0.25099, 5.0029, 3.14099, 0.0872],
        [-0.01953, 4.97217, 3.14099, 0.08668],
        [-0.28706, 4.93031, 3.14099, 0.08621],
        [-0.55096, 4.87425, 3.14099, 0.08589],
        [-0.81003, 4.79919, 3.43771, 0.07846],
        [-1.06498, 4.7082, 3.78539, 0.07151],
        [-1.31643, 4.6039, 4.0, 0.06806],
        [-1.56497, 4.48854, 4.0, 0.0685],
        [-1.81125, 4.36406, 4.0, 0.06899],
        [-2.05592, 4.23197, 4.0, 0.06951],
        [-2.29967, 4.09373, 4.0, 0.07006],
        [-2.5431, 3.95093, 4.0, 0.07056],
        [-2.78655, 3.80527, 4.0, 0.07093],
        [-3.03507, 3.65527, 4.0, 0.07257],
        [-3.28377, 3.50559, 4.0, 0.07257],
        [-3.53268, 3.35627, 4.0, 0.07256],
        [-3.78185, 3.20742, 4.0, 0.07256],
        [-4.03134, 3.05912, 4.0, 0.07256],
        [-4.28123, 2.91151, 3.45093, 0.0841],
        [-4.53159, 2.76472, 2.92335, 0.09927],
        [-4.76717, 2.62793, 2.5357, 0.10743],
        [-4.99931, 2.4888, 2.22032, 0.12189],
        [-5.22448, 2.34503, 2.22032, 0.12032],
        [-5.43939, 2.19462, 2.22032, 0.11814],
        [-5.64056, 2.03574, 2.09863, 0.12215],
        [-5.8234, 1.86637, 1.92072, 0.12976],
        [-5.98298, 1.68521, 1.73977, 0.13876],
        [-6.11341, 1.49212, 1.52825, 0.15248],
        [-6.22138, 1.29188, 1.41123, 0.1612],
        [-6.30353, 1.08532, 1.41123, 0.15752],
        [-6.3565, 0.87399, 1.41123, 0.15438],
        [-6.3757, 0.66027, 1.41123, 0.15206],
        [-6.35472, 0.44808, 1.41123, 0.15109],
        [-6.28243, 0.2456, 1.41123, 0.15235],
        [-6.15139, 0.06773, 1.64329, 0.13445],
        [-5.97888, -0.08183, 1.81887, 0.12552],
        [-5.77397, -0.20106, 2.00724, 0.11811],
        [-5.54448, -0.2894, 2.23618, 0.10997],
        [-5.29788, -0.34832, 2.46937, 0.10267],
        [-5.04021, -0.38008, 2.69954, 0.09617],
        [-4.77599, -0.38723, 2.92655, 0.09032],
        [-4.5085, -0.37234, 3.19819, 0.08377],
        [-4.23999, -0.33843, 3.41953, 0.07915],
        [-3.9719, -0.28759, 3.64661, 0.07483],
        [-3.70522, -0.22172, 3.8789, 0.07082],
        [-3.44057, -0.1425, 4.0, 0.06906],
        [-3.17837, -0.05138, 4.0, 0.0694],
        [-2.91887, 0.05037, 4.0, 0.06968],
        [-2.66226, 0.16164, 4.0, 0.06993],
        [-2.4086, 0.2814, 4.0, 0.07013],
        [-2.15792, 0.40865, 4.0, 0.07028],
        [-1.91015, 0.54241, 4.0, 0.07039],
        [-1.66516, 0.68174, 4.0, 0.07046],
        [-1.42274, 0.82576, 4.0, 0.07049],
        [-1.1827, 0.97366, 2.8197, 0.09999],
        [-0.9449, 1.12493, 2.27563, 0.12385],
        [-0.70936, 1.2792, 1.88439, 0.14942],
        [-0.47634, 1.43645, 1.63369, 0.17207],
        [-0.26485, 1.58372, 1.63369, 0.15775],
        [-0.0516, 1.71918, 1.63369, 0.15465],
        [0.16511, 1.83276, 1.63369, 0.14976],
        [0.38658, 1.9159, 1.63369, 0.14481],
        [0.61333, 1.9581, 1.63369, 0.14118],
        [0.84294, 1.94756, 1.82519, 0.12594],
        [1.07035, 1.89405, 2.02175, 0.11555],
        [1.29174, 1.80436, 2.18284, 0.10943],
        [1.50366, 1.68308, 2.32863, 0.10485],
        [1.70297, 1.53448, 2.48448, 0.10007],
        [1.88752, 1.36307, 2.57024, 0.09799],
        [2.05553, 1.17257, 2.57024, 0.09883],
        [2.20621, 0.96667, 2.57024, 0.09927],
        [2.33909, 0.74842, 2.57024, 0.09941],
        [2.45223, 0.51966, 2.57024, 0.09929],
        [2.54087, 0.2819, 2.71058, 0.09361],
        [2.60781, 0.0392, 2.53583, 0.09928],
        [2.65432, -0.20615, 2.33474, 0.10696],
        [2.68168, -0.45245, 2.12994, 0.11635],
        [2.68967, -0.69831, 1.92391, 0.12786],
        [2.67802, -0.94225, 1.71297, 0.14257],
        [2.64518, -1.18251, 1.71297, 0.14156],
        [2.58891, -1.41672, 1.71297, 0.14062],
        [2.50609, -1.64148, 1.71297, 0.13983],
        [2.39292, -1.85196, 1.71297, 0.13951],
        [2.24454, -2.04092, 1.71297, 0.14026],
        [2.05457, -2.19602, 2.26427, 0.10831],
        [1.84185, -2.32823, 2.49196, 0.10051],
        [1.61029, -2.43936, 2.71742, 0.09452],
        [1.36269, -2.53076, 2.99613, 0.08809],
        [1.10172, -2.60423, 3.30597, 0.08201],
        [0.82968, -2.66153, 3.7339, 0.07446],
        [0.54899, -2.7051, 4.0, 0.07101],
        [0.2617, -2.73733, 4.0, 0.07227],
        [-0.03052, -2.76041, 4.0, 0.07328],
        [-0.32634, -2.77631, 4.0, 0.07406],
        [-0.62469, -2.78681, 4.0, 0.07463],
        [-0.92472, -2.79346, 4.0, 0.07503],
        [-1.22577, -2.79765, 4.0, 0.07527],
        [-1.52727, -2.80067, 4.0, 0.07538],
        [-1.82877, -2.80369, 4.0, 0.07538],
        [-2.13028, -2.80671, 4.0, 0.07538],
        [-2.43178, -2.80973, 4.0, 0.07538],
        [-2.73314, -2.81306, 3.9016, 0.07725],
        [-3.03343, -2.81873, 3.53155, 0.08504],
        [-3.33143, -2.82908, 3.2107, 0.09287],
        [-3.62575, -2.84635, 2.91047, 0.1013],
        [-3.91486, -2.87262, 2.61397, 0.11106],
        [-4.19705, -2.9098, 2.35869, 0.12067],
        [-4.47045, -2.95958, 2.1216, 0.13098],
        [-4.733, -3.02343, 1.8911, 0.14288],
        [-4.98246, -3.10256, 1.68055, 0.15573],
        [-5.21618, -3.19805, 1.46998, 0.17175],
        [-5.43083, -3.31096, 1.46998, 0.16499],
        [-5.62281, -3.44167, 1.46998, 0.158],
        [-5.78793, -3.59003, 1.46351, 0.15168],
        [-5.92083, -3.75529, 1.3, 0.16313],
        [-6.01495, -3.93542, 1.3, 0.15634],
        [-6.06063, -4.12599, 1.3, 0.15075],
        [-6.0716, -4.31915, 1.3, 0.14882],
        [-6.04291, -4.51111, 1.3, 0.1493],
        [-5.96643, -4.69542, 1.3, 0.1535],
        [-5.82589, -4.85584, 1.77483, 0.12016],
        [-5.64927, -4.99469, 1.98384, 0.11325],
        [-5.44195, -5.11064, 2.24486, 0.10582],
        [-5.2093, -5.20385, 2.55389, 0.09813],
        [-4.95634, -5.2756, 2.91661, 0.09016],
        [-4.68749, -5.32799, 3.37466, 0.08116],
        [-4.4068, -5.36395, 3.93341, 0.07194],
        [-4.11756, -5.38661, 4.0, 0.07253],
        [-3.82225, -5.39872, 4.0, 0.07389],
        [-3.52319, -5.40362, 4.0, 0.07478],
        [-3.22219, -5.40444, 4.0, 0.07525],
        [-2.92552, -5.40386, 4.0, 0.07417],
        [-2.64322, -5.40214, 4.0, 0.07058],
        [-2.36666, -5.39935, 4.0, 0.06915],
        [-2.09335, -5.39545, 4.0, 0.06833],
        [-1.82214, -5.39045, 4.0, 0.06781],
        [-1.55248, -5.38425, 4.0, 0.06743],
        [-1.28401, -5.37674, 4.0, 0.06714],
        [-1.01646, -5.36778, 4.0, 0.06693]]

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
        
        delta_progress = progress - state.prev_progress
        try:
            if delta_progress < 0:
                if steps <= 2 and progress < 1:
                    delta_progress = progress
        except:
            print('Error with steps.')
        progress_reward = max(0, delta_progress)
        
        
        inner_border1, outer_border1, inner_border2, outer_border2 = find_border_points(params)
        min_heading, max_heading, is_within_range = find_min_max_heading(params, inner_border2, outer_border2)
                
        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        
        optimal_speed = optimals[2]
        
        # Bonus reward if going 4 m/s or faster during optimal spots. Also small bonus for small steering angles.
        if speed >= 3.95 and optimal_speed >= 3.95:
            SUPER_FAST_BONUS = 1
            STRAIGHT_STEERING_BONUS = max(0.001, .2 - (abs(steering_angle)/150))
        else:
            SUPER_FAST_BONUS = 0
            STRAIGHT_STEERING_BONUS = 0
            
        try:
            scaled_multiplier = scale_value(4/optimal_speed, 1, 3.1, 1, 2)
        except:
            print('Error with scaled_multiplier.')
            scaled_multiplier = 4/optimal_speed
        
        DISTANCE_MULTIPLE = scaled_multiplier
        DISTANCE_EXPONENT = scaled_multiplier
        SPEED_MULTIPLE = 3 - DISTANCE_MULTIPLE
        heading_reward = math.cos( abs(direction_diff ) * ( math.pi / 180 ) ) ** 10
        if abs(direction_diff) <= 20:
            heading_reward = math.cos( abs(direction_diff ) * ( math.pi / 180 ) ) ** 4
        HEADING_MULTIPLE = -1.4 + DISTANCE_MULTIPLE
        if HEADING_MULTIPLE < 0:
            HEADING_MULTIPLE = 0
                
        progress_multiplier = 4
        # Distance component
        DC = (distance_reward**DISTANCE_EXPONENT) * DISTANCE_MULTIPLE
        # Speed component
        SC = speed_reward * SPEED_MULTIPLE
        # Progress component
        PC = progress_reward * progress_multiplier
        # Progress interval
        PI = calculate_progress_reward(params, state)
        # Heading component, will be given if distance multiplier is greater than 1.5 (to help with steering on turning sections)
        HC = heading_reward * HEADING_MULTIPLE
        try:
            if steps % 100 == 0:
                print(f'steps: {steps}')
                print(f'delta_progress: {progress-state.prev_progress}')
                print(f'DC: {DC}\nPC: {PC}, SUPER_FAST_BONUS: {SUPER_FAST_BONUS}\nstraight_steering_bonus: {STRAIGHT_STEERING_BONUS}')
        except:
            print('Error in printing steps and delta_progress')
        reward += DC + SC + PC + HC + SUPER_FAST_BONUS + STRAIGHT_STEERING_BONUS
        
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
        if speed_diff_zero > 0.5:
            reward *= 0.5
        
        reward *= DISTANCE_PUNISHMENT

        ## Zero reward if off track ##
        track_width = params['track_width']
        distance_from_center = params['distance_from_center']

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
            print("Predicted time: %f" % projected_time)
            # print("=== Finish reward: %f ===" % finish_reward)

        #################### RETURN REWARD ####################
        
        state.prev_turn_angle = steering_angle
        state.prev_speed_diff = speed_diff
        state.prev_distance = dist
        state.prev_speed = speed
        state.prev_progress = progress

        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)