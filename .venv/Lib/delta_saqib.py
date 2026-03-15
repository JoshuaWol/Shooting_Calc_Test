import json
import math
import os
from pathlib import Path
import matplotlib.pyplot as plt

def calc_deltas(trajectories):
    sorted_trajectories = sorted(
        trajectories, key=lambda x: (x["theta"], x["distance"])
    )
    deltas = []
    prev_theta = 0
    prev_x = 0
    prev_vel = 0
    for val in sorted_trajectories:
        if True == True :#math.isclose(prev_vel , val["velocity"], rel_tol = 0.1) and not math.isclose(
            #prev_x , val["distance"], rel_tol=0.02
        #):
            # print(val)
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
            if delta_distance < 10 and delta_theta < 10:
                deltas.append(delta)

        prev_vel = val["velocity"]
        prev_theta = val["theta"]
        prev_x = val["distance"]
    print(deltas)
    return deltas


directory = Path(__file__).parent
with open(os.path.join(directory, "trajectories.json"), "r") as t:
    trajectories = json.load(t)

deltas = calc_deltas(trajectories)
# print(deltas)
with open(os.path.join(directory, "deltas.json"), "w") as file:
    file.write(json.dumps(deltas, indent=2))


angles = []
dist = []
vi = []
d_dist= []
d_dist_cm = []
proportion= []
d_theta= []
p_dist= []
for each in deltas:
    angles.append(each['current_theta'])
    dist.append(each['distance'])
    vi.append(each['velocity'])
    d_theta.append(each["delta_theta"])
    d_dist.append(each["delta_distance"])
    d_dist_cm.append(each["delta_distance"]*100)
    proportion.append(each["proportion"])
    p_dist.append(each["previous_distance"])

print(max(dist))
#
# # with open(os.path.join(directory, "deltas.json"), "w") as file:
# #     file.write(json.dumps(deltas, indent=2))
# print(len(angles),len(vi),len(d_dist))
# fig = plt.figure()
# ax = fig.add_subplot(111, projection="3d")
# ax.plot_trisurf(angles,vi,d_dist)
# ax.set_xlabel("Theta_i")
# ax.set_ylabel("Velocity_i")
# ax.set_zlabel("Delta_Distance")
# # ax.set_zlim(0.15,0.41)
# print(min(d_dist),max(d_dist))
# plt.show()
plt.figure()
plt.scatter(angles, d_dist_cm)
plt.xlabel("Theta (deg)")
plt.ylabel("delta_Distance (cm)")
plt.title("Distance Change vs Angle")
plt.show()
