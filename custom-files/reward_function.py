import numpy as np
import math

class STATE:
    def __init__(self):
        # Initialize all attributes with their default values
        self.prev_turn_angle = None
        self.prev_speed_diff = None
        self.prev_distance = None
        self.prev_speed = None
        self.prev_progress = 0
        self.prev_progress2 = 0
        self.prev_progress3 = 0
        self.prev_progress4 = 0
        self.prev_progress5 = 0
        self.prev_progress6 = 0
        
    # Optional: You could also define a reset method to reset all attributes
    def reset(self):
        self.prev_turn_angle = None
        self.prev_speed_diff = None
        self.prev_distance = None
        self.prev_speed = None
        self.prev_progress = 0
        self.prev_progress2 = 0
        self.prev_progress3 = 0
        self.prev_progress4 = 0
        self.prev_progress5 = 0
        self.prev_progress6 = 0
        
state = STATE()

class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):
        try:
            def update_and_calculate_reward(new_delta_progress, delta_progress_list):
                # FILO: Add new delta-progress value to the end and remove the oldest one
                delta_progress_list.append(new_delta_progress)  # Add new value
                delta_progress_list.pop(0)  # Remove the oldest value (first in the list)

                # Check if the list contains any zeros
                if 0 in delta_progress_list:
                    return 0  # If any zero values, return 0 as reward

                # Calculate the average of the non-zero values
                avg_delta_progress = sum(delta_progress_list) / len(delta_progress_list)

                # Return the average as the reward
                return avg_delta_progress
            
            def normalize_delta_angle(angle, old_min=0, old_max=18.4, new_min=0.1, new_max=1):
                # Apply min-max normalization formula
                normalized_angle = ((angle - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min
                return normalized_angle
            
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
            racing_track = [[ 5.05151105,  0.86354104],
            [ 5.05148792,  1.16529298],
            [ 5.04943011,  1.46632064],
            [ 5.04225552,  1.7654497 ],
            [ 5.02703862,  2.06141768],
            [ 5.00109957,  2.35288184],
            [ 4.96205768,  2.63844094],
            [ 4.90784367,  2.9166432 ],
            [ 4.83667096,  3.18597059],
            [ 4.74701285,  3.44481661],
            [ 4.63755862,  3.69143857],
            [ 4.50711271,  3.92384138],
            [ 4.35456621,  4.13964153],
            [ 4.17840702,  4.33536107],
            [ 3.98171583,  4.51120912],
            [ 3.76702145,  4.66749506],
            [ 3.53650168,  4.80462572],
            [ 3.29209748,  4.92311329],
            [ 3.03557032,  5.02357281],
            [ 2.7685916 ,  5.10681566],
            [ 2.49272773,  5.17381656],
            [ 2.20948345,  5.22579603],
            [ 1.92029036,  5.26423132],
            [ 1.6264881 ,  5.29085464],
            [ 1.32930014,  5.30764007],
            [ 1.02980077,  5.31675208],
            [ 0.72888828,  5.32047283],
            [ 0.42727886,  5.32116318],
            [ 0.12552685,  5.321208  ],
            [-0.17411827,  5.32076643],
            [-0.46223148,  5.31755914],
            [-0.73628549,  5.30984539],
            [-0.99792703,  5.29584824],
            [-1.25160349,  5.27370246],
            [-1.50238552,  5.24104674],
            [-1.75452193,  5.19481895],
            [-2.01022488,  5.13117383],
            [-2.26859452,  5.04602501],
            [-2.52517914,  4.9359219 ],
            [-2.7729465 ,  4.79928009],
            [-3.00432637,  4.63667052],
            [-3.21275174,  4.45037371],
            [-3.39324854,  4.2437705 ],
            [-3.54249217,  4.02083822],
            [-3.65838904,  3.78568883],
            [-3.73951499,  3.54235144],
            [-3.78505636,  3.29480519],
            [-3.7941943 ,  3.04699302],
            [-3.766238  ,  2.80302056],
            [-3.7004905 ,  2.56734712],
            [-3.59622323,  2.34509808],
            [-3.45283201,  2.14260512],
            [-3.27765075,  1.96154934],
            [-3.07532862,  1.80315926],
            [-2.85000946,  1.66777161],
            [-2.60550992,  1.55489082],
            [-2.34543957,  1.46309933],
            [-2.0731898 ,  1.39004186],
            [-1.79192757,  1.33243709],
            [-1.50457883,  1.28620778],
            [-1.21382046,  1.24675039],
            [-0.92828677,  1.20266841],
            [-0.6487455 ,  1.14848258],
            [-0.37854424,  1.07976005],
            [-0.12111547,  0.99304188],
            [ 0.12000738,  0.88591192],
            [ 0.34122355,  0.75705849],
            [ 0.53879947,  0.60618548],
            [ 0.70878333,  0.43395438],
            [ 0.84690818,  0.24200542],
            [ 0.94867155,  0.03315712],
            [ 1.00909334, -0.18842896],
            [ 1.02299983, -0.41663616],
            [ 0.98538588, -0.6424394 ],
            [ 0.90656137, -0.85896116],
            [ 0.79064749, -1.06174597],
            [ 0.641356  , -1.24745159],
            [ 0.46221904, -1.4135778 ],
            [ 0.25662308, -1.55831173],
            [ 0.02788046, -1.68049074],
            [-0.22067294, -1.77969891],
            [-0.48568125, -1.85635976],
            [-0.76377829, -1.91189197],
            [-1.05173534, -1.94868454],
            [-1.34659681, -1.97000644],
            [-1.64577087, -1.97991681],
            [-1.94715148, -1.98297602],
            [-2.24551151, -1.99412115],
            [-2.53856806, -2.01758489],
            [-2.82373709, -2.05695146],
            [-3.09824491, -2.1150024 ],
            [-3.3592271 , -2.19363159],
            [-3.60373103, -2.29394243],
            [-3.82880715, -2.41621624],
            [-4.03145429, -2.56005091],
            [-4.20863173, -2.72436931],
            [-4.35705826, -2.90752253],
            [-4.47489066, -3.10648407],
            [-4.55921208, -3.31839072],
            [-4.60679499, -3.53964753],
            [-4.61420206, -3.76552572],
            [-4.5818928 , -3.99034481],
            [-4.51214585, -4.20918508],
            [-4.40404898, -4.41664445],
            [-4.2569994 , -4.60609962],
            [-4.07575279, -4.77321249],
            [-3.86760416, -4.91769619],
            [-3.63626981, -5.03842139],
            [-3.38538416, -5.1351634 ],
            [-3.11846432, -5.20860837],
            [-2.83888353, -5.26039258],
            [-2.54983989, -5.29317401],
            [-2.25425225, -5.31055558],
            [-1.95467111, -5.31693865],
            [-1.65320396, -5.31725001],
            [-1.35145247, -5.31670094],
            [-1.04970199, -5.31613708],
            [-0.74795032, -5.3155601 ],
            [-0.4461989 , -5.31498694],
            [-0.14444755, -5.3144145 ],
            [ 0.1573038 , -5.3138411 ],
            [ 0.4590552 , -5.31327009],
            [ 0.76080677, -5.31270194],
            [ 1.06255746, -5.31211805],
            [ 1.36372253, -5.30988212],
            [ 1.66308001, -5.30274151],
            [ 1.95927884, -5.28761226],
            [ 2.25083804, -5.26167724],
            [ 2.53619786, -5.22246112],
            [ 2.81375617, -5.16786776],
            [ 3.08187897, -5.0961669 ],
            [ 3.33886578, -5.00591622],
            [ 3.5829323 , -4.89594097],
            [ 3.81210654, -4.76521489],
            [ 4.02412695, -4.61281641],
            [ 4.21628654, -4.4379024 ],
            [ 4.38497734, -4.23957884],
            [ 4.53202858, -4.02225504],
            [ 4.65808841, -3.78839484],
            [ 4.76391965, -3.54013442],
            [ 4.8504634 , -3.27942012],
            [ 4.91882484, -3.00804283],
            [ 4.9704296 , -2.72774427],
            [ 5.00705762, -2.44021775],
            [ 5.03085025, -2.14708995],
            [ 5.04431964, -1.84990029],
            [ 5.05025211, -1.55004061],
            [ 5.05165505, -1.24872249],
            [ 5.05164599, -0.9469707 ],
            [ 5.05162096, -0.6452187 ],
            [ 5.05159712, -0.34346674],
            [ 5.05157495, -0.04171484],
            [ 5.05155206,  0.26003705],
            [ 5.05152893,  0.56178895],
            [ 5.05151105,  0.86354104]]

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
            
            try:
                reset_state(steps)
            except:
                print('Error with reset_state.')

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
            
            inner_border1, outer_border1, inner_border2, outer_border2 = find_border_points(params)
            min_heading, max_heading, is_within_range = find_min_max_heading(params, inner_border2, outer_border2)
                    
            # Zero reward if obviously wrong direction (e.g. spin)
            direction_diff = racing_direction_diff(
                optimals[0:2], optimals_second[0:2], [x, y], heading)
            
            optimal_speed = optimals[2]
            STEERING_PUNISHMENT = 1
            SPEED_PUNISHMENT = 1
            LANE_REWARD = 0
            
            is_in_turn = False
            delta_p_multiple = 8
            capstone_multiple = 1
            
                
            delta_p1 = (progress - state.prev_progress)
            delta_p2 = (progress - state.prev_progress2) / 2
            delta_p3 = (progress - state.prev_progress3) / 3
            delta_p4 = (progress - state.prev_progress4) / 4
            delta_p5 = (progress - state.prev_progress4) / 5
            delta_p6 = (progress - state.prev_progress4) / 6
            
            if delta_p1 > 1.0:
                delta_p1 = 1.0
            if delta_p2 > 1.5:
                delta_p2 = 1.5
            if delta_p3 > 2.0:
                delta_p3 = 2.0
            if delta_p4 > 2.5:
                delta_p4 = 2.5
            if delta_p5 > 3.0:
                delta_p5 = 3.0
            if delta_p6 > 3.5:
                delta_p6 = 3.5
                
            delta_p_reward = ((delta_p1 * 2) + delta_p2 + delta_p3 + delta_p4 + delta_p5 + delta_p6) / 6
            avg_delta_p = ((delta_p_reward * delta_p_multiple) ** 2)
            
            try:
                scaled_multiplier = scale_value(4/optimal_speed, 1, 2.9, 1, 1.5)
                SPEED_BONUS = scale_value(4/optimal_speed, 1, 2.9, 1, 2.9)
            except:
                print('Error with scaled_multiplier.')
                scaled_multiplier = 4/optimal_speed
            
            DISTANCE_MULTIPLE = scaled_multiplier
            DISTANCE_EXPONENT = scaled_multiplier
            SPEED_MULTIPLE = 3 - DISTANCE_MULTIPLE
                    
            # Distance component
            DC = (distance_reward) * DISTANCE_MULTIPLE
            SQDC = distance_reward ** DISTANCE_EXPONENT
            # Speed component
            SC = (speed_reward ** 2) * SPEED_MULTIPLE
            # Progress component
            DISTANCE_PUNISHMENT = 1
            
            if is_in_turn:
                reward = (avg_delta_p) + (capstone_multiple * (SPEED_BONUS * speed_reward * SPEED_MULTIPLE + (0.5 * distance_reward * DISTANCE_MULTIPLE) + (0.5 * (distance_reward ** 2) * DISTANCE_MULTIPLE)))
                if dist > (track_width * 0.5):
                    DISTANCE_PUNISHMENT = 0.5
            else:
                if dist > (track_width * 0.25):
                    DISTANCE_PUNISHMENT = 0.5
                reward = (avg_delta_p) + (SPEED_BONUS * speed_reward * SPEED_MULTIPLE + (0.5 * distance_reward * DISTANCE_MULTIPLE) + (0.5 * (distance_reward ** 2) * DISTANCE_MULTIPLE))
            
            if optimal_speed >= 3.2 and speed >= optimal_speed:
                reward += (2 * distance_reward)
                
            # No more additions to rewards after this point.
            
            if state.prev_turn_angle is not None and state.prev_speed_diff is not None and state.prev_distance is not None and state.prev_speed is not None:
                # Erratic steering punishments
                delta_turn_angle = abs(steering_angle - state.prev_turn_angle)
                delta_speed = abs(speed - state.prev_speed)
                if state.prev_turn_angle > 10 and steering_angle < -10:
                    reward *= 0.1
                elif state.prev_turn_angle < -10 and steering_angle > 10:
                    reward *= 0.1
                if delta_turn_angle > 30:
                    reward *= 0.1
            
            if prev_waypoint_index >= 18 and prev_waypoint_index <= 27:
                if speed > 2.5:
                    SPEED_PUNISHMENT = 0.5
                if steering_angle > 0:
                    STEERING_PUNISHMENT *= 0.5
            
            # Punishing erratic steering or steering out of range of valid directions.
            if speed > 2.5 and (steering_angle >= 20 or steering_angle <= -20):
                reward *= 0.5
            if not is_within_range:
                reward *= 0.8
                
            if direction_diff > 30:
                reward *= 0.75
            elif direction_diff >= 25:
                reward *= 0.8
            elif direction_diff >= 20:
                reward *= 0.85
            elif direction_diff >= 15:
                reward *= 0.9
            
            # Punishing too fast or too slow
            speed_diff_zero = optimals[2]-speed
            if speed_diff_zero > 0.6:
                reward *= 0.5
            elif speed_diff_zero < -0.6:
                reward *= 0.5
            
            reward *= DISTANCE_PUNISHMENT
            reward *= STEERING_PUNISHMENT
            reward *= SPEED_PUNISHMENT

            ## Zero reward if off track ##
            track_width = params['track_width']
            distance_from_center = params['distance_from_center']

            # Zero reward if the center of the car is off the track.
            reward += LANE_REWARD
        except Exception as e:
            print(f'Error in reward calculation: {e}')
            if distance_from_center <= track_width/2:
                reward += 1

        if not all_wheels_on_track and distance_from_center >= (track_width/2)+0.05:
            reward = min(reward, 0.001)

        #################### RETURN REWARD ####################
        
        state.prev_turn_angle = steering_angle
        state.prev_speed_diff = speed_diff
        state.prev_distance = dist
        state.prev_speed = speed
        state.prev_progress = progress
        state.prev_progress2 = state.prev_progress
        state.prev_progress3 = state.prev_progress2
        state.prev_progress4 = state.prev_progress3
        state.prev_progress5 = state.prev_progress4
        state.prev_progress6 = state.prev_progress5

        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)