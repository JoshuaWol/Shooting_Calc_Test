import json
import math
import os
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

slope = 38.227
slope = 22.17
servo_values = [0.2, 0.4, 0.62, 0.8, 1]
dist_values = [158, 203, 232, 259, 284]
angle_delta_values = [round(slope * (servo_values[i]-servo_values[0]),1) for i in range(len(servo_values))]
print(angle_delta_values)






directory = Path(__file__).parent
with open(os.path.join(directory, "trajectories_ground.json"), "r") as t:
    trajectories = json.load(t)



sorted_trajectories = sorted(
    trajectories, key=lambda x: (x["velocity"], -x["theta"])
)
with open(os.path.join(directory, "deltas_sorted.json"), "w") as file:
    file.write(json.dumps(sorted_trajectories, indent=2))


vi_true = []
for i in range(len(sorted_trajectories)):
    each = sorted_trajectories[i]
    dist = each['distance']
    if math.isclose(dist,dist_values[0], abs_tol=6):
        angle = each['theta']
        vi = each['velocity']
        checks_out = True
        max_delta = 0
        angle_set = [angle]
        for j in range(1,len(angle_delta_values)):
            angle_2 = angle_delta_values[j]
            dist_2 = dist_values[j]
            if i+int(angle_2*10) >= len(sorted_trajectories):
                checks_out = False
            else:
                delta = sorted_trajectories[i+int(angle_2*10)]["distance"] - dist_2
                angle_set.append(sorted_trajectories[i + int(angle_2 * 10)]["theta"])
                if abs(delta) > max_delta:
                    max_delta = abs(delta)
                    info = (j, vi, angle, delta)
        if checks_out and vi not in vi_true and max_delta < 4:
            print(info)
            vi_true.append((vi, round(info[3],2), angle))#, angle_set))

x = 73.2 + 0.2*22.17
print(x)
print(vi_true)












# angles = []
# dist = []
# vi = []
# d_dist= []
# d_dist_cm = []
# proportion= []
# d_theta= []
# p_dist= []
# for each in deltas:
#     angles.append(each['current_theta'])
#     dist.append(each['distance'])
#     vi.append(each['velocity'])
#     d_theta.append(each["delta_theta"])
#     d_dist.append(each["delta_distance"])
#     d_dist_cm.append(each["delta_distance"]*100)
#     proportion.append(each["proportion"])
#     p_dist.append(each["previous_distance"])
#
# print(max(dist))
# #
# # # with open(os.path.join(directory, "deltas.json"), "w") as file:
# # #     file.write(json.dumps(deltas, indent=2))
# # print(len(angles),len(vi),len(d_dist))
# # fig = plt.figure()
# # ax = fig.add_subplot(111, projection="3d")
# # ax.plot_trisurf(angles,vi,d_dist)
# # ax.set_xlabel("Theta_i")
# # ax.set_ylabel("Velocity_i")
# # ax.set_zlabel("Delta_Distance")
# # # ax.set_zlim(0.15,0.41)
# # print(min(d_dist),max(d_dist))
# # plt.show()
# plt.figure()
# plt.scatter(angles, d_dist_cm)
# plt.xlabel("Theta (deg)")
# plt.ylabel("delta_Distance (cm)")
# plt.title("Distance Change vs Angle")
# plt.show()
