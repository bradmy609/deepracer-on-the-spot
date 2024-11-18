import numpy as np # type: ignore
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
            racing_track = [[-0.19566, -5.38263, 4.0, 0.06564],
[-0.4589, -5.38735, 4.0, 0.06582],
[-0.72369, -5.39101, 4.0, 0.0662],
[-0.99064, -5.39383, 4.0, 0.06674],
[-1.25974, -5.39611, 4.0, 0.06728],
[-1.53109, -5.398, 4.0, 0.06784],
[-1.80444, -5.39963, 4.0, 0.06834],
[-2.08056, -5.401, 4.0, 0.06903],
[-2.35997, -5.40215, 4.0, 0.06985],
[-2.64347, -5.40309, 4.0, 0.07088],
[-2.93012, -5.40387, 4.0, 0.07166],
[-3.22219, -5.40444, 3.80802, 0.0767],
[-3.52371, -5.40475, 3.27788, 0.09199],
[-3.82482, -5.40431, 2.87899, 0.10459],
[-4.1228, -5.39767, 2.56388, 0.11625],
[-4.41447, -5.37981, 2.29916, 0.12709],
[-4.69638, -5.34649, 2.04923, 0.13853],
[-4.96497, -5.29445, 1.8034, 0.15171],
[-5.21642, -5.22125, 1.42123, 0.18427],
[-5.44682, -5.12548, 1.42123, 0.17556],
[-5.65205, -5.00664, 1.42123, 0.16687],
[-5.82702, -4.86469, 1.42123, 0.15853],
[-5.96457, -4.70029, 1.42123, 0.15082],
[-6.0438, -4.51414, 1.51961, 0.13314],
[-6.07438, -4.32021, 1.62969, 0.12046],
[-6.06398, -4.12559, 1.64527, 0.11846],
[-6.01553, -3.93483, 1.64527, 0.11963],
[-5.92598, -3.7535, 1.78211, 0.11348],
[-5.79992, -3.58543, 1.95442, 0.1075],
[-5.64174, -3.43291, 2.12172, 0.10356],
[-5.45456, -3.29793, 2.31456, 0.09971],
[-5.2416, -3.18184, 2.53951, 0.09551],
[-5.00635, -3.08528, 2.84248, 0.08946],
[-4.75298, -3.00743, 3.20561, 0.08268],
[-4.48543, -2.94662, 3.61973, 0.0758],
[-4.207, -2.90081, 4.0, 0.07054],
[-3.92056, -2.86752, 4.0, 0.07209],
[-3.6284, -2.84431, 4.0, 0.07327],
[-3.33227, -2.82887, 4.0, 0.07413],
[-3.0335, -2.81914, 4.0, 0.07473],
[-2.73306, -2.81336, 4.0, 0.07512],
[-2.43178, -2.80973, 4.0, 0.07532],
[-2.13028, -2.80671, 4.0, 0.07538],
[-1.82877, -2.80369, 4.0, 0.07538],
[-1.52727, -2.80067, 4.0, 0.07538],
[-1.22592, -2.79734, 4.0, 0.07534],
[-0.9252, -2.79265, 4.0, 0.07519],
[-0.62582, -2.7852, 4.0, 0.07487],
[-0.32853, -2.77355, 4.0, 0.07438],
[-0.03415, -2.7563, 3.59479, 0.08203],
[0.25639, -2.73195, 3.14922, 0.09258],
[0.54201, -2.69892, 2.81132, 0.10227],
[0.82137, -2.65535, 2.49047, 0.11353],
[1.09296, -2.59941, 2.22176, 0.12481],
[1.35476, -2.52879, 1.8956, 0.14305],
[1.60395, -2.44057, 1.8956, 0.13945],
[1.83749, -2.33229, 1.8956, 0.1358],
[2.05124, -2.20104, 1.8956, 0.13232],
[2.2399, -2.04414, 1.8956, 0.12945],
[2.39255, -1.85698, 2.04394, 0.11816],
[2.51208, -1.64817, 2.22828, 0.10798],
[2.60199, -1.42426, 2.36742, 0.10192],
[2.66417, -1.18959, 2.52611, 0.0961],
[2.70081, -0.94783, 2.65555, 0.09208],
[2.71346, -0.70179, 2.76695, 0.08904],
[2.70343, -0.45384, 2.61777, 0.09479],
[2.6717, -0.20603, 2.61777, 0.09544],
[2.6195, 0.03983, 2.61777, 0.09601],
[2.54712, 0.28192, 2.61777, 0.09652],
[2.45288, 0.51758, 2.61777, 0.09696],
[2.33419, 0.74232, 2.67481, 0.09502],
[2.19786, 0.95615, 2.49365, 0.10169],
[2.04526, 1.15726, 2.2927, 0.11011],
[1.8777, 1.34375, 2.08222, 0.1204],
[1.69651, 1.51362, 1.82811, 0.13586],
[1.50205, 1.66301, 1.61371, 0.15196],
[1.29556, 1.78785, 1.61371, 0.14952],
[1.07884, 1.88319, 1.61371, 0.14672],
[0.85438, 1.94301, 1.61371, 0.14395],
[0.6257, 1.95764, 1.61371, 0.142],
[0.39893, 1.91452, 1.95925, 0.11782],
[0.17714, 1.83069, 2.29892, 0.10314],
[-0.03952, 1.71474, 2.84862, 0.08627],
[-0.25226, 1.57567, 4.0, 0.06354],
[-0.46303, 1.42495, 4.0, 0.06478],
[-0.69491, 1.26608, 4.0, 0.07027],
[-0.93025, 1.11166, 4.0, 0.07037],
[-1.16875, 0.96205, 4.0, 0.07038],
[-1.41022, 0.81765, 4.0, 0.07034],
[-1.65465, 0.67915, 4.0, 0.07024],
[-1.90194, 0.5471, 4.0, 0.07008],
[-2.15196, 0.42203, 4.0, 0.06989],
[-2.40469, 0.30481, 4.0, 0.06965],
[-2.66, 0.19625, 4.0, 0.06936],
[-2.91766, 0.09694, 4.0, 0.06903],
[-3.17732, 0.00719, 3.92242, 0.07004],
[-3.43855, -0.07322, 3.6433, 0.07502],
[-3.70101, -0.14357, 3.35246, 0.08106],
[-3.96431, -0.20312, 3.00097, 0.08995],
[-4.22792, -0.25073, 2.69353, 0.09945],
[-4.49113, -0.28498, 2.39072, 0.11103],
[-4.75297, -0.30409, 2.10417, 0.12477],
[-5.01202, -0.30588, 1.84092, 0.14072],
[-5.26606, -0.28698, 1.60354, 0.15886],
[-5.51182, -0.24401, 1.4, 0.17821],
[-5.74454, -0.17311, 1.4, 0.17377],
[-5.95728, -0.07039, 1.4, 0.16874],
[-6.1402, 0.06731, 1.4, 0.16354],
[-6.27928, 0.24097, 1.4, 0.15893],
[-6.35472, 0.44431, 1.6131, 0.13445],
[-6.38019, 0.65743, 1.77912, 0.12064],
[-6.36417, 0.87215, 1.93384, 0.11134],
[-6.31247, 1.08402, 2.10433, 0.10364],
[-6.22982, 1.29041, 2.31101, 0.0962],
[-6.12062, 1.48998, 2.38468, 0.09539],
[-5.98629, 1.68123, 2.38468, 0.09801],
[-5.82482, 1.86121, 2.71559, 0.08904],
[-5.64147, 2.03025, 3.05395, 0.08166],
[-5.44006, 2.18907, 3.50058, 0.07327],
[-5.22431, 2.33904, 4.0, 0.06569],
[-4.9979, 2.482, 4.0, 0.06694],
[-4.76429, 2.62003, 4.0, 0.06784],
[-4.52703, 2.75551, 4.0, 0.0683],
[-4.2766, 2.90215, 4.0, 0.07255],
[-4.0274, 3.05099, 4.0, 0.07257],
[-3.7791, 3.20143, 4.0, 0.07258],
[-3.5314, 3.35292, 4.0, 0.07259],
[-3.28399, 3.50492, 4.0, 0.07259],
[-3.03826, 3.6549, 4.0, 0.07197],
[-2.79312, 3.80277, 4.0, 0.07157],
[-2.54839, 3.94777, 4.0, 0.07112],
[-2.30378, 4.08909, 4.0, 0.07062],
[-2.05891, 4.22589, 4.0, 0.07012],
[-1.81333, 4.35726, 3.51132, 0.07932],
[-1.56657, 4.48206, 3.10418, 0.08908],
[-1.31797, 4.59848, 3.10418, 0.08843],
[-1.06683, 4.70448, 3.10418, 0.08782],
[-0.81239, 4.79768, 3.10418, 0.08729],
[-0.55368, 4.87453, 3.10418, 0.08694],
[-0.28959, 4.93056, 3.93984, 0.06852],
[-0.02212, 4.9736, 4.0, 0.06773],
[0.24822, 5.00595, 4.0, 0.06807],
[0.52105, 5.02944, 4.0, 0.06846],
[0.79606, 5.04576, 4.0, 0.06887],
[1.07306, 5.05632, 4.0, 0.0693],
[1.35189, 5.06244, 4.0, 0.06972],
[1.63253, 5.06524, 4.0, 0.07016],
[1.91558, 5.06554, 4.0, 0.07076],
[2.19631, 5.06409, 4.0, 0.07018],
[2.47353, 5.05967, 4.0, 0.06931],
[2.74688, 5.05086, 3.55773, 0.07687],
[3.01604, 5.03624, 3.21449, 0.08385],
[3.2805, 5.01414, 2.81826, 0.09417],
[3.53967, 4.98279, 2.45386, 0.10639],
[3.79276, 4.94029, 2.14877, 0.11943],
[4.03864, 4.88439, 1.87821, 0.13425],
[4.27602, 4.81295, 1.87821, 0.13199],
[4.50252, 4.72238, 1.87821, 0.12988],
[4.71445, 4.608, 1.87821, 0.12822],
[4.90635, 4.46422, 1.87821, 0.12767],
[5.06869, 4.28394, 2.16866, 0.11187],
[5.20544, 4.07736, 2.4336, 0.1018],
[5.31923, 3.85009, 2.7124, 0.09371],
[5.41237, 3.60603, 3.05088, 0.08562],
[5.48744, 3.34854, 3.4768, 0.07714],
[5.54732, 3.0806, 4.0, 0.06864],
[5.59529, 2.80493, 4.0, 0.06995],
[5.63535, 2.52434, 4.0, 0.07086],
[5.67116, 2.24117, 4.0, 0.07136],
[5.70772, 1.94919, 4.0, 0.07356],
[5.74518, 1.65718, 4.0, 0.0736],
[5.78401, 1.36514, 4.0, 0.07365],
[5.82443, 1.07313, 4.0, 0.0737],
[5.86655, 0.78122, 4.0, 0.07373],
[5.9103, 0.48947, 4.0, 0.07375],
[5.95561, 0.19791, 4.0, 0.07376],
[6.00261, -0.09336, 4.0, 0.07376],
[6.05149, -0.38428, 4.0, 0.07375],
[6.10246, -0.67472, 4.0, 0.07372],
[6.15586, -0.96457, 3.86268, 0.0763],
[6.21217, -1.25364, 3.20087, 0.09201],
[6.26788, -1.52233, 2.74196, 0.10007],
[6.31867, -1.78975, 2.37974, 0.11438],
[6.3599, -2.05499, 1.9284, 0.1392],
[6.38717, -2.3175, 1.9284, 0.13686],
[6.39518, -2.57644, 1.9284, 0.13434],
[6.37833, -2.83046, 1.9284, 0.13202],
[6.33002, -3.07723, 1.9284, 0.1304],
[6.23568, -3.30962, 2.1407, 0.11716],
[6.10537, -3.52604, 2.35989, 0.10705],
[5.94644, -3.72594, 2.62843, 0.09716],
[5.76522, -3.91005, 2.85358, 0.09053],
[5.56573, -4.07895, 3.08333, 0.08477],
[5.35116, -4.2334, 3.31561, 0.07974],
[5.12403, -4.37427, 3.54818, 0.07532],
[4.88637, -4.5024, 3.783, 0.07137],
[4.63986, -4.61863, 4.0, 0.06813],
[4.38591, -4.72373, 4.0, 0.06871],
[4.12569, -4.81834, 4.0, 0.06922],
[3.86025, -4.90304, 4.0, 0.06966],
[3.59059, -4.97837, 4.0, 0.07],
[3.31769, -5.04489, 4.0, 0.07022],
[3.04251, -5.10318, 4.0, 0.07032],
[2.76602, -5.15384, 4.0, 0.07027],
[2.48915, -5.1973, 4.0, 0.07007],
[2.21277, -5.23444, 4.0, 0.06971],
[1.93763, -5.2659, 4.0, 0.06923],
[1.66427, -5.29231, 4.0, 0.06866],
[1.39307, -5.31428, 4.0, 0.06802],
[1.12417, -5.3324, 4.0, 0.06738],
[0.85749, -5.34721, 4.0, 0.06677],
[0.59272, -5.3592, 4.0, 0.06626],
[0.32938, -5.36884, 4.0, 0.06588],
[0.06683, -5.37652, 4.0, 0.06567]]

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
            reward = 1.0

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
            speed_cap = optimal_speed + 0.75
            STEERING_PUNISHMENT = 1
            SPEED_PUNISHMENT = 1
            LANE_REWARD = 0
            
            delta_p = progress - state.prev_progress
            if delta_p > 0.8:
                print(f'Error with delta-p calculation: {delta_p} at waypoint: {prev_waypoint_index}')
                delta_p = 0.8
            
            is_in_turn = False
            delta_p_multiple = 8
            capstone_multiple = 1
            
                
            delta_p1 = (progress - state.prev_progress)
            delta_p2 = (progress - state.prev_progress2) / 2
            delta_p3 = (progress - state.prev_progress3) / 3
            delta_p4 = (progress - state.prev_progress4) / 4
            delta_p5 = (progress - state.prev_progress5) / 5
            delta_p6 = (progress - state.prev_progress6) / 6
            
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
            delta_p_reward = ((delta_p1 * 2) + delta_p2 + delta_p3 + delta_p4 + delta_p5 + delta_p6) / 7
            avg_delta_p = ((delta_p_reward * delta_p_multiple) ** 2) + ((delta_p_reward * (delta_p_multiple/2)) ** 3)
            
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
            
            if prev_waypoint_index >= 55 and optimal_speed >= 3.2 and speed >= optimal_speed:
                reward += (3 * distance_reward)
                
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
            if speed_diff_zero > 0.7:
                reward *= 0.5
            elif speed_diff_zero < -0.7:
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