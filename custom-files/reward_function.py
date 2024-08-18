import math

# Define constant weights
STEP_DIFFERENCE_WEIGHT = 0.75
GOAL_PASSING_BOOST = 1000

# Initialize a list to keep track of the last 5 steps
step_list = []

def calculate_direction_diff(params):
    '''
    Calculate the difference between the track direction and the heading direction of the car.
    '''
    next_waypoint = params['waypoints'][params['closest_waypoints'][1]]
    prev_waypoint = params['waypoints'][params['closest_waypoints'][0]]
    
    track_direction = math.atan2(next_waypoint[1] - prev_waypoint[1], next_waypoint[0] - prev_waypoint[0])
    track_direction = math.degrees(track_direction)
    
    direction_diff = abs(track_direction - params['heading'])
    
    # Normalize direction difference
    if direction_diff > 180:
        direction_diff = 360 - direction_diff
    
    return direction_diff, track_direction

def calculate_turn_reward(params, direction_diff, track_direction):
    '''
    Reward based on the turn direction and car's position relative to the track center.
    '''
    distance_from_center_normalized = params['distance_from_center'] / (params['track_width'] / 2)
    reward = 1.0  # Base reward

    # Determine the turn direction
    is_left_turn = track_direction > params['heading'] if track_direction - params['heading'] > 0 else False
    is_right_turn = params['heading'] > track_direction if params['heading'] - track_direction > 0 else False

    if is_left_turn:
        # Left turn: Reward for being on the left side
        reward += 1.0 if distance_from_center_normalized < 0.5 else -1.0
    elif is_right_turn:
        # Right turn: Reward for being on the right side
        reward += 1.0 if distance_from_center_normalized > 0.5 else -1.0
    else:
        # Straightaway: Reward for being close to the center
        reward += 1.0 if distance_from_center_normalized < 0.1 else -0.5

    return reward

def calculate_step_difference_bonus(step_list, current_steps):
    '''
    Calculate the bonus based on the difference in steps over the last 5 cycles.
    '''
    step_list.append(current_steps)
    
    # Maintain the list to only keep the last 5 steps
    if len(step_list) > 5:
        step_list.pop(0)
    
    if len(step_list) == 5:
        step_difference = step_list[-1] - step_list[0]
        return step_difference * STEP_DIFFERENCE_WEIGHT
    
    return 0.0

def calculate_goal_passing_bonus(progress):
    '''
    Calculate the bonus when the car passes the goal.
    '''
    if progress >= 100.0:
        return GOAL_PASSING_BOOST
    
    return 0.0

def progress_bonus(params):

    global step_list

    # Calculate the direction difference and track direction
    direction_diff, track_direction = calculate_direction_diff(params)
    
    # Calculate rewards
    reward = calculate_turn_reward(params, direction_diff, track_direction)
    reward += calculate_step_difference_bonus(step_list, params['steps'])
    reward += calculate_goal_passing_bonus(params['progress'])

    # Ensure reward is within bounds
    reward = max(reward, 0.1)
    
    return float(reward)


def reward_function(params):
    '''
    Reward function for AWS DeepRacer.
    '''

    # Read input parameters
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    steps = params['steps']
    progress = params['progress']

    # Calculate the next direction
    next_waypoint = waypoints[closest_waypoints[1]]
    prev_waypoint = waypoints[closest_waypoints[0]]
    
    track_direction = math.atan2(next_waypoint[1] - prev_waypoint[1], next_waypoint[0] - prev_waypoint[0])
    track_direction = math.degrees(track_direction)
    
    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - heading)
    
    # Normalize direction difference
    if direction_diff > 180:
        direction_diff = 360 - direction_diff
    
    # Determine the turn direction
    is_left_turn = track_direction > heading if track_direction - heading > 0 else False
    is_right_turn = heading > track_direction if heading - track_direction > 0 else False

    # Reward calculation
    reward = 1.0  # Start with a base reward

    # Penalize if the car is too far from the center line
    distance_from_center_normalized = distance_from_center / (track_width / 2)

    # Left turn: Reward for being on the left side
    if is_left_turn:
        if distance_from_center_normalized < 0.5:  # Left side
            reward += 1.0
        else:  # Right side
            reward -= 1.0
    
    # Right turn: Reward for being on the right side
    elif is_right_turn:
        if distance_from_center_normalized > 0.5:  # Right side
            reward += 1.0
        else:  # Left side
            reward -= 1.0

    # Straightaway: Reward for being close to the center
    else:
        if distance_from_center_normalized < 0.1:  # Close to center
            reward += 1.0
        else:  # Far from center
            reward -= 0.5
    
    # Append the current step to the list
    reward += progress_bonus(params)

    # Ensure reward is within bounds
    reward = max(reward, 0.1)  # Minimum reward
    
    return float(reward)
