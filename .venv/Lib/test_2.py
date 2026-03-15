import json
import math
import os
from pathlib import Path
import matplotlib.pyplot as plt

directory = Path(__file__).parent
with open(os.path.join(directory, "trajectories.json"), "r") as t:
    trajectories = json.load(t)

def calc_deltas(trajectories):
    sorted_trajectories = sorted(
        trajectories, key=lambda x: (x["velocity"], x["distance"])
    )
    deltas = []
    prev_theta = 0
    prev_x = 0
    prev_vel = 0
    for val in sorted_trajectories:
        if prev_vel == val["velocity"] and not math.isclose(
            prev_x + 0.01, val["distance"], rel_tol=0.001
        ):
            delta_distance = val["distance"] - prev_x
            delta_theta = val["theta"] - prev_theta
            delta = {
                "delta_distance": delta_distance,
                "delta_theta": delta_theta,
                "proportion": delta_distance / delta_theta,
                "velocity": val["velocity"],
                "current_theta": val["theta"],
                "previous_distance": prev_x,
                "distance": val["distance"],
            }
            deltas.append(delta)

        prev_vel = val["velocity"]
        prev_theta = val["theta"]
        prev_x = val["distance"]

    return deltas


deltas = calc_deltas(trajectories)

list_delta = calc_deltas(trajectories)
# list_delta.append({
#             "delta_distance": 0,
#             "delta_theta": 0,
#             "proportion": 0,
#             "velocity": 10,
#             "current_theta": 70,
#             "previous_distance": 0,
#             "distance": 0,
#         })

angles = []
dist = []
vi = []
d_dist= []
proportion= []
d_theta= []
p_dist= []
for each in list_delta:
    angles.append(each['current_theta'])
    dist.append(each['distance'])
    vi.append(each['velocity'])
    d_theta.append(each["delta_theta"])
    d_dist.append(each["delta_distance"])
    proportion.append(each["proportion"])
    p_dist.append(each["previous_distance"])

print(max(dist))

# with open(os.path.join(directory, "deltas.json"), "w") as file:
#     file.write(json.dumps(deltas, indent=2))
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.plot_trisurf(angles,vi,d_dist)
ax.set_xlabel("Theta_i")
ax.set_ylabel("Velocity_i")
ax.set_zlabel("Distance")
# ax.set_zlim(0.15,0.41)
plt.show()