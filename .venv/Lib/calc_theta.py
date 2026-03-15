import json
import math
import os
from pathlib import Path
import matplotlib.pyplot as plt
from collections import defaultdict

def calc_deltas(trajectories, dist_goal, vel_init):
    # x = trajectories
    my_dict = defaultdict(dict)
    for x in trajectories:
        my_dict[x["velocity"]] [x["distance"]] = x["theta"]
    closest_velocity = min(my_dict, key=lambda k: (abs(k - vel_init), k))
    closest_dist = min(my_dict[closest_velocity], key=lambda k: (abs(k - dist_goal), k))
    print(dist_goal,vel_init)
    print(closest_dist, closest_velocity)
    print(my_dict[closest_velocity][closest_dist])
    return


directory = Path(__file__).parent
with open(os.path.join(directory, "trajectories.json"), "r") as t:
    trajectories = json.load(t)

distance = 1.2 #in meeters
# distance_feet = 7.1 #in feet
# distance = distance_feet/3.281

velocity =8.1 #in m/s
# rpm = 2#rpm shooter
# velocity= rpm * 6.6/2800

deltas = calc_deltas(trajectories, distance, velocity)