import math

prev_progress = 0.0

PROGRESS_MULTIPLIER = 20
def reward_function(params):
    global prev_progress
    reward = 0.1

    progress = params['progress']
    # Call the function to check future checkpoints
    if params["all_wheels_on_track"] and params["steps"] > 0:
        reward = (progress - prev_progress) ** 2
    prev_progress = progress

    return max(0.001, reward)