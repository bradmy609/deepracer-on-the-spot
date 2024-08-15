import math

def set_optimized_waypoints(params):
    
    params['optimized_waypoints'] = [[ 5.03288116,  0.85563751],
       [ 5.01963634,  1.15157374],
       [ 5.00165461,  1.44531754],
       [ 4.97821213,  1.73645655],
       [ 4.94839165,  2.02446102],
       [ 4.91132605,  2.30879297],
       [ 4.8661957 ,  2.58890823],
       [ 4.81082942,  2.86347602],
       [ 4.74332007,  3.13116875],
       [ 4.6612713 ,  3.39019872],
       [ 4.5618616 ,  3.63823265],
       [ 4.44171828,  3.87210827],
       [ 4.2958937 ,  4.08657427],
       [ 4.11865037,  4.27332933],
       [ 3.91787946,  4.4357903 ],
       [ 3.69871569,  4.57696266],
       [ 3.46468029,  4.6991846 ],
       [ 3.21845415,  4.80438974],
       [ 2.96250267,  4.89465569],
       [ 2.69861103,  4.97125707],
       [ 2.42843201,  5.03526673],
       [ 2.15352916,  5.08757935],
       [ 1.87532074,  5.12861317],
       [ 1.59531104,  5.15936551],
       [ 1.31470903,  5.1803387 ],
       [ 1.03456805,  5.19190472],
       [ 0.75579522,  5.19431047],
       [ 0.47916445,  5.18770806],
       [ 0.20531779,  5.17221808],
       [-0.06522142,  5.14795159],
       [-0.33199746,  5.11493301],
       [-0.59468479,  5.07328803],
       [-0.85298501,  5.02304998],
       [-1.10686945,  4.96465235],
       [-1.3563925 ,  4.89856999],
       [-1.60125356,  4.82461711],
       [-1.84109932,  4.74254211],
       [-2.07558405,  4.65212796],
       [-2.30336729,  4.55175111],
       [-2.52296374,  4.43988046],
       [-2.73204863,  4.31436062],
       [-2.92880586,  4.17442373],
       [-3.11225924,  4.02063311],
       [-3.27926159,  3.85180591],
       [-3.42475126,  3.66642214],
       [-3.54017692,  3.4631976 ],
       [-3.61372101,  3.24425314],
       [-3.64035923,  3.01838203],
       [-3.6356006 ,  2.79286974],
       [-3.59680659,  2.57107573],
       [-3.51544424,  2.35904857],
       [-3.38246852,  2.16829349],
       [-3.21362497,  1.9993442 ],
       [-3.01588778,  1.85220594],
       [-2.79560764,  1.72493691],
       [-2.55739733,  1.61545436],
       [-2.30523826,  1.5211862 ],
       [-2.04280874,  1.43896638],
       [-1.77359437,  1.36511907],
       [-1.50051123,  1.29619752],
       [-1.22852638,  1.22987085],
       [-0.96011434,  1.15947874],
       [-0.69762154,  1.08243955],
       [-0.44338358,  0.99634689],
       [-0.1996773 ,  0.89913744],
       [ 0.03129199,  0.78919071],
       [ 0.24605709,  0.6642066 ],
       [ 0.44090426,  0.52250766],
       [ 0.6099831 ,  0.36196495],
       [ 0.74580398,  0.18178754],
       [ 0.85167977, -0.01206114],
       [ 0.92162769, -0.21810428],
       [ 0.94581785, -0.43295463],
       [ 0.91170399, -0.64658953],
       [ 0.83151787, -0.84916613],
       [ 0.71327503, -1.03645658],
       [ 0.56302427, -1.20655673],
       [ 0.38557196, -1.35874712],
       [ 0.18421784, -1.49242836],
       [-0.0379894 , -1.60750161],
       [-0.27805698, -1.70445149],
       [-0.53299459, -1.7843917 ],
       [-0.80010312, -1.84872234],
       [-1.07657978, -1.89967887],
       [-1.36079849, -1.93838348],
       [-1.65116628, -1.96627972],
       [-1.94680737, -1.98369783],
       [-2.23700789, -2.01238524],
       [-2.52039931, -2.05315272],
       [-2.79541212, -2.10736799],
       [-3.06036401, -2.17624806],
       [-3.31334653, -2.26094525],
       [-3.5520738 , -2.36260515],
       [-3.7737427 , -2.48232816],
       [-3.97515941, -2.62083597],
       [-4.15090472, -2.7795096 ],
       [-4.29408588, -2.95835436],
       [-4.40656956, -3.15159074],
       [-4.48759291, -3.35581188],
       [-4.53223935, -3.56818576],
       [-4.53428707, -3.78362998],
       [-4.50057314, -3.99651737],
       [-4.43110346, -4.20249434],
       [-4.32517416, -4.39661412],
       [-4.18003017, -4.57168046],
       [-4.00297452, -4.72577691],
       [-3.79973848, -4.85875   ],
       [-3.57475535, -4.9711589 ],
       [-3.33153222, -5.06381257],
       [-3.07333693, -5.1381199 ],
       [-2.80290298, -5.1955727 ],
       [-2.52297925, -5.23841367],
       [-2.23592241, -5.26906497],
       [-1.94365752, -5.28991377],
       [-1.64768715, -5.30313701],
       [-1.34916309, -5.31065548],
       [-1.049004  , -5.31424414],
       [-0.747836  , -5.31524743],
       [-0.44644019, -5.31425765],
       [-0.14587295, -5.31100577],
       [ 0.15356335, -5.3051061 ],
       [ 0.45157381, -5.29603999],
       [ 0.74785679, -5.28326168],
       [ 1.04210292, -5.2662265 ],
       [ 1.33400497, -5.24442112],
       [ 1.62322151, -5.2172943 ],
       [ 1.90934596, -5.18420492],
       [ 2.19187657, -5.14437984],
       [ 2.47003922, -5.09662156],
       [ 2.74281569, -5.03943158],
       [ 3.0090024 , -4.97118036],
       [ 3.26699146, -4.88982247],
       [ 3.51470226, -4.79294583],
       [ 3.74989511, -4.67836705],
       [ 3.96844008, -4.54204078],
       [ 4.16396538, -4.37869913],
       [ 4.32832176, -4.18397296],
       [ 4.46896009, -3.96859157],
       [ 4.58907727, -3.73692541],
       [ 4.6906516 , -3.49162983],
       [ 4.77544963, -3.23483629],
       [ 4.84513479, -2.96834527],
       [ 4.90130169, -2.69371582],
       [ 4.94553069, -2.41233747],
       [ 4.97947337, -2.12550059],
       [ 5.00463766, -1.83429725],
       [ 5.02265061, -1.53976199],
       [ 5.03498994, -1.24274818],
       [ 5.04297388, -0.94394533],
       [ 5.04772615, -0.64389013],
       [ 5.05020163, -0.34299873],
       [ 5.05028613, -0.04181366],
       [ 5.04771686,  0.25863085],
       [ 5.04205797,  0.55788288],
       [ 5.03288116,  0.85563751]]

def dist(point1, point2):
    return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5

# thanks to https://stackoverflow.com/questions/20924085/python-conversion-between-coordinates
def rect(r, theta):
    """
    theta in degrees

    returns tuple; (float, float); (x,y)
    """

    x = r * math.cos(math.radians(theta))
    y = r * math.sin(math.radians(theta))
    return x, y


# thanks to https://stackoverflow.com/questions/20924085/python-conversion-between-coordinates
def polar(x, y):
    """
    returns r, theta(degrees)
    """

    r = (x ** 2 + y ** 2) ** .5
    theta = math.degrees(math.atan2(y,x))
    return r, theta


def angle_mod_360(angle):
    """
    Maps an angle to the interval -180, +180.

    Examples:
    angle_mod_360(362) == 2
    angle_mod_360(270) == -90

    :param angle: angle in degree
    :return: angle in degree. Between -180 and +180
    """

    n = math.floor(angle/360.0)

    angle_between_0_and_360 = angle - n*360.0

    if angle_between_0_and_360 <= 180.0:
        return angle_between_0_and_360
    else:
        return angle_between_0_and_360 - 360


def get_optimized_waypoints_ordered_in_driving_direction(params):
    # waypoints are always provided in counter clock wise order
    if params['is_reversed']: # driving clock wise.
        return list(reversed(params['optimized_waypoints']))
    else: # driving counter clock wise.
        return params['optimized_waypoints']
    
def get_waypoints_ordered_in_driving_direction(params):
    # waypoints are always provided in counter clock wise order
    if params['is_reversed']: # driving clock wise.
        return list(reversed(params['waypoints']))
    else: # driving counter clock wise.
        return params['waypoints']


def up_sample(waypoints, factor):
    """
    Adds extra waypoints in between provided waypoints

    :param waypoints:
    :param factor: integer. E.g. 3 means that the resulting list has 3 times as many points.
    :return:
    """
    p = waypoints
    n = len(p)

    return [[i / factor * p[(j+1) % n][0] + (1 - i / factor) * p[j][0],
             i / factor * p[(j+1) % n][1] + (1 - i / factor) * p[j][1]] for j in range(n) for i in range(factor)]


def get_target_point(params):
    waypoints = up_sample(get_waypoints_ordered_in_driving_direction(params), 10)

    car = [params['x'], params['y']]

    distances = [dist(p, car) for p in waypoints]
    min_dist = min(distances)
    i_closest = distances.index(min_dist)

    n = len(waypoints)

    waypoints_starting_with_closest = [waypoints[(i+i_closest) % n] for i in range(n)]

    r = params['track_width'] * 0.9

    is_inside = [dist(p, car) < r for p in waypoints_starting_with_closest]
    i_first_outside = is_inside.index(False)

    if i_first_outside < 0:  # this can only happen if we choose r as big as the entire track
        return waypoints[i_closest]

    return waypoints_starting_with_closest[i_first_outside]

def get_closest_optimized_waypoint(params):
    waypoints = up_sample(get_optimized_waypoints_ordered_in_driving_direction(params), 10)

    car = [params['x'], params['y']]

    distances = [dist(p, car) for p in waypoints]
    min_dist = min(distances)
    i_closest = distances.index(min_dist)

    n = len(waypoints)

    waypoints_starting_with_closest = [waypoints[(i+i_closest) % n] for i in range(n)]
    return waypoints_starting_with_closest[0]


def get_target_steering_degree(params):
    tx, ty = get_target_point(params)
    car_x = params['x']
    car_y = params['y']
    dx = tx-car_x
    dy = ty-car_y
    heading = params['heading']

    _, target_angle = polar(dx, dy)

    steering_angle = target_angle - heading

    return angle_mod_360(steering_angle)

def calculate_progress_per_step_reward(params):
    progress = params['progress']
    steps = params['steps']
    
    x = progress/steps
    x1, y1 = 0.1, 0.1
    x2, y2 = 0.5, 2
    y = y1 + (x - x1) * ((y2 - y1) / (x2 - x1))
    return y

def calculate_position_reward(params):
    # This multiplier determines how much the car prioritizes adhering to racing line
    multiplier = 1
    # Get the closest waypoint
    closest_waypoint = get_closest_optimized_waypoint(params)

    # Calculate the distance to the closest waypoint
    distance_to_closest_waypoint = dist([params['x'], params['y']], closest_waypoint)

    # Normalize the distance to a value between 0 and 1
    normalized_distance = distance_to_closest_waypoint / (params['track_width'])

    base_reward = calculate_progress_per_step_reward(params)

    # Define the thresholds
    thresholds = [0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]

    # Calculate the reward based on the thresholds
    if normalized_distance <= thresholds[0]:
        reward = base_reward
    elif normalized_distance <= thresholds[1]:
        reward = base_reward * 0.9
    elif normalized_distance <= thresholds[2]:
        reward = base_reward * 0.8
    elif normalized_distance <= thresholds[3]:
        reward = base_reward * 0.7
    elif normalized_distance <= thresholds[4]:
        reward = base_reward * 0.6
    elif normalized_distance <= thresholds[5]:
        reward = base_reward * 0.5
    elif normalized_distance <= thresholds[6]:
        reward = base_reward * 0.4
    elif normalized_distance <= thresholds[7]:
        reward = base_reward * 0.3
    elif normalized_distance <= thresholds[8]:
        reward = base_reward * 0.2
    elif normalized_distance <= thresholds[9]:
        reward = base_reward * 0.15
    elif normalized_distance <= thresholds[10]:
        reward = base_reward * 0.1
    elif normalized_distance <= thresholds[11]:
        reward = base_reward * 0.05
    else:
        reward = base_reward * 0.01

    # Clamp the reward between 0.01 and 1
    reward = max(min(reward, 1), 0.01) * multiplier

    return reward

def calculate_progress_reward(params):
    # This multiplier determines how much the car prioritizes staying on track w/o crashing
    multiplier = 0.2
    progress = params['progress']
    steps = params['steps']
    progress_reward = progress/100 * multiplier
    if progress == 100:
        actual_progress_per_step = progress/steps
        progress_reward += calculate_progress_per_step_reward(actual_progress_per_step)
        progress_reward *= 10
        progress_reward += 10
        
    return max(progress_reward, 0.01)

def reward_function(params):
    # Sets optimized waypoints to be equal to precalced list of waypoints from optimal racing line.
    set_optimized_waypoints(params)
    
    position_reward = float(calculate_position_reward(params))
    print(f'position_reward: {position_reward}')
    
    progress_reward = float(calculate_progress_reward(params))
    print(f'progress_reward: {progress_reward}')
    
    final_reward = position_reward + progress_reward
    print(f'final_reward: {final_reward}')
    
    is_crashed = params['is_crashed']
    all_wheels_on_track = params['all_wheels_on_track']
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']

    # Zeros out reward if the entire car is off the track. 
    # That means the distance from the center is greater than half the track width + car length of 0.1 meters.
    if is_crashed:
        final_reward = min(final_reward, 0.01)
    elif not all_wheels_on_track and distance_from_center >= (track_width/2):
        final_reward = min(final_reward, 0.01)
    
    return final_reward