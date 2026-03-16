import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib.patches as patches
import scipy
import time


#constants
#Coord system = cartesian is Z=height, x=radial, y = tangential relative to hub, spherical is radial from robot to goal
rad = 3.14159/180
vi= 9.4 #m/s
theta_i = 60 *rad #deg -> rad
phi_i = 0 *rad # deg -> rad
m = 0.475 /2.205 #lb -> kg  [0.448 - 0.5]
ball_Diam = 5.91 *(1/12/3.281) #in -> m
ball_radius = ball_Diam/2
x_Area = 3.14159 * ball_radius**2 #cross sectional area
rho = 1.225 #kg/m^3
xi= 0 #m
zi = 18 *(1/12/3.281) #in -> m
zf = 72 *(1/12/3.281) #in -> m
x_goal = 6.12 # m  the max distance is 6.12m
x_goal_radius = 20.85 * 1/12/3.281 #in to m using inscribed circle
    # x_goal_max = 12 *(1/12/3.281) #in -> m
    # x_goal_min = 20 *(1/12/3.281) #in -> m
    # y_goal_left = 21 *(1/12/3.281) #in -> m
    # y_goal_right = 21 *(1/12/3.281) #in -> m
Cd = 0.47 #drag coeff sphere in 10^4 - 10^5 reynolds num
g = -9.81 #m/s^2

#ERROR values in std using ranges (assumed to be gaussian)

err_phi = 3 * rad/3   #* 0.0001  #deg -> rad
err_theta = 3 * rad/3  #* 0.0001 #deg -> rad

big_data = []
starting_Vi = [9,10,11,12,13,14]

#inputs - Starting Coord, Final Coord, final error, Cd, g, Ball, Shooting Initial, error, dt, N

for vi in starting_Vi:
    # err_vi = (0.1 * vi) / 3  # *0.0001 #range of intial velocity ~10%
    err_vi = 1 / 3  # *0.0001 #range of intial velocity 1m/s

    #Calculating the Theta Angles Results 0.1deg definition
    start_Angle = 30
    end_Angle = 90
    N = (end_Angle - start_Angle)*10
    theta = np.linspace(start_Angle*rad,end_Angle*rad,N)
    dt = np.array([1e-4 for i in range(N)])
    t = 0
    z=[zi for i in range(N)]
    x=0
    y=0
    Cd1 = Cd

    ## To be used with the graphing below for drag comparison
    # theta = np.linspace(start_Angle*rad,end_Angle*rad,N//2)
    # theta2 = np.concatenate([theta,theta])
    # theta = np.array(theta2)
    # CD_yes = np.array([Cd for i in range(N//2)])
    # CD_no = np.array([1e-6 for i in range(N//2)])
    # Cd1 = np.concatenate([CD_yes,CD_no])


    v = vi
    phi = phi_i
    theta_list = list(theta)

    v_x = v *np.cos(theta)*np.cos(phi)
    v_z = v * np.sin(theta)
    v_y = v * np.cos(theta) * np.sin(phi)

    #Angle calculation loop
    alive = np.ones(N, dtype = bool)
    while (max(z) > zf or  max(v_z) >0) or max(z)<=0:
        accel_x = -Cd1 * rho * x_Area * v ** 2 * np.cos(theta) * np.cos(phi) / (2*m)
        accel_y = -Cd1 * rho * x_Area * v ** 2 * np.cos(theta) * np.sin(phi) / (2*m)
        accel_z = g - Cd1*rho*x_Area*v**2*np.sin(theta)/(2*m)
        v_x += accel_x * dt
        v_z += accel_z * dt
        v_y += accel_y * dt
        v = np.sqrt(v_x**2 + v_z**2 + v_y**2)
        theta = np.arctan(v_z/np.sqrt(v_x**2+v_y**2))
        phi = np.arctan(v_y/v_x)
        zprev = np.array(z)
        z += v_z * dt
        x += v_x * dt
        y += v_y * dt
        t += dt

        hit = alive & (((z <= zf) & (v_z <= 0) & (zprev > zf)) | (z<= 0))


        dt[hit] = 0

    failed = alive & (z<= 0)
    x[failed] = 0



    if np.max(x)<x_goal:
        print(f"for all angles the max x = {np.max(x)} which is lower than goal distance of {x_goal}")
        quit

    #taking list only from the max distance angle and above
    x_dist = [[x[i],theta_list[i]] for i in range(N)]
    x_dist = x_dist[np.argmax(x):]
    x_target_list = np.array(list(x[np.argmax(x):]))

    # print(f"the max distance angle is {theta_list[np.argmax(x)]/rad:.2f}")
    #
    # x_check = x[N//2:]
    # theta_check = np.array(theta_list)[N//2:]/rad
    # print(f"the distance angle for furthest field shot is {theta_list[np.abs(x-24/3.281).argmin()]/rad:.2f} for vi {vi}")


    ##To be used with the Cd1 change above
    # plt.figure()
    # plt.title("Angle vs Distance")
    # plt.scatter(np.array(theta_list)[N//2:]/rad,x[N//2:],color = 'red' , label = 'Coef_Drag = 0')
    # plt.scatter(np.array(theta_list)[:N//2]/rad,x[:N//2],color = 'blue' , label = 'Coef_Drag = 0.47')
    # plt.legend()
    # plt.xlabel("Initial Shooting Angle (deg)")
    # plt.ylabel("Distance (m)")
    # plt.title(f"Drag Effect on Shooting Distance {vi}")
    # plt.grid(True, which = "major")
    # plt.show()









    #intialization equations
    start = time.perf_counter()
    distances_to_calc = np.linspace(1.25,6.5,int((6.5-1.25)/0.1+1))
    perc_score = []
    avg_score = []
    scatter_record_x = []
    scatter_record_y = []
    score_record = []
    v_initial_record = []
    theta_initial_record = []
    phi_initial_record = []
    goal_angle = []

    for i in range(len(distances_to_calc)):
        x_goal = distances_to_calc[i]
        N = 1000
        dt_val = 1e-3
        dt = np.array([dt_val for i in range(N)]) #s
        rng = np.random.default_rng(488)
        theta_target = x_dist[np.abs(x_target_list - x_goal).argmin()][1]
        goal_angle.append([x_goal,theta_target / rad])
        t = 0
        z=[zi for i in range(N)]
        x=0
        y=0


        v = rng.normal(loc = vi, scale = err_vi, size=N)
        theta = rng.normal(loc = theta_target, scale = err_theta, size=N)
        phi = rng.normal(loc = phi_i, scale = err_phi, size=N)
        theta_initial = list(theta /rad) # rad->deg
        v_initial = list(v)
        v_initial_record.extend(v)
        theta_initial_record.extend(theta)
        phi_initial_record.extend(phi)

        v_x = v *np.cos(theta)*np.cos(phi)
        v_z = v * np.sin(theta)
        v_y = v * np.cos(theta) * np.sin(phi)



        #calculation loop
        alive = np.ones(N, dtype = bool)
        while (max(z) > zf or  max(v_z) >0) or max(z)<=0:
            accel_x = -Cd * rho * x_Area * v ** 2 * np.cos(theta) * np.cos(phi) / (2*m)
            accel_y = -Cd * rho * x_Area * v ** 2 * np.cos(theta) * np.sin(phi) / (2*m)
            accel_z = g - Cd*rho*x_Area*v**2*np.sin(theta)/(2*m)
            v_x += accel_x * dt
            v_z += accel_z * dt
            v_y += accel_y * dt
            v = np.sqrt(v_x**2 + v_z**2 + v_y**2)
            theta = np.arctan(v_z/np.sqrt(v_x**2+v_y**2))
            phi = np.arctan(v_y/v_x)
            zprev = np.array(z)
            z += v_z * dt
            x += v_x * dt
            y += v_y * dt
            t += dt

            hit = alive & (((z <= zf) & (v_z <= 0)&(zprev > zf)) | (z<= 0))


            dt[hit] = 0

        failed = alive & (z <= 0)

        x[failed] = 0
        score = np.sqrt((x_goal - x) ** 2 + y**2) < x_goal_radius
        score_record.append(score)
        perc_score.append([ x_goal, np.count_nonzero(score)/np.size(score,0)])
        avg_score=np.count_nonzero(score)/np.size(score,0)
        scatter_record_x.append(x-x_goal)
        scatter_record_y.append(y)

    #need to record score%, distance, vi, and theta_targwt

        big_data.append([np.mean(avg_score),x_goal,vi,theta_target])

plt.figure()
big_data = np.array(big_data)

for each in starting_Vi:
    mask = big_data[:,2] == each
    plt.scatter(big_data[mask,1],big_data[mask,0], label =f"Vi = {each} m/s, Theta [{np.min(big_data[mask,3])/rad:.0f}, {np.max(big_data[mask,3])/rad:.0f}] deg")
plt.xlabel("Distance from Goal (m)")
plt.ylabel("Scoring Percent (%)")
plt.title("Initial Velocity Investigation")
plt.grid(True,which = "major")
plt.legend()

            # plt.figure()
            # perc_score = np.array(perc_score)
            # plt.scatter(perc_score[:,0],perc_score[:,1]*100)
            # plt.ylabel('Chance to Score (%)')
            # plt.xlabel('Distance to Hub (m)')
            # plt.title('Success vs Distance')
            #
            #
            #
            #
            #
            # fig, ax = plt.subplots()
            # ax.scatter(scatter_record_x, scatter_record_y)
            # circ = Circle((0, 0), x_goal_radius, fill=False, linewidth=2, edgecolor = 'red')  # add edgecolor=... if you want
            # ax.add_patch(circ)
            # ax.text(x_goal_radius*0.9, x_goal_radius *0.9, "Goal", ha="center", va="center", color = 'red', size = 20 )
            # ax.set_aspect('equal', adjustable='box')  # makes it look like a true circle
            # ax.set_xlabel("X Distance (m)")
            # ax.set_ylabel("Y Distance (m)")
            # ax.set_title("Monte Carlo of Shots")
            #
            #
            # # --- Dimensions (inches) ---
            # W = 317.7   # overall width (x)
            # D = 158.6   # overall depth (y)
            # point_calc = 300
            #
            # radius_shoot = perc_score[:,0] * 3.28 * 12
            # chance_score = perc_score[:,1]
            # x_points_graph = np.linspace(0,W,point_calc)
            # y_points_graph = np.linspace(0,D,point_calc)
            # points = np.array([[i,j] for i in x_points_graph for j in y_points_graph])
            # points_score_percent = []
            # rmax = 0
            #
            #
            #
            # for each in points:
            #     x = np.abs(each[0] - W/2)
            #     y = each[1] + 47/2
            #     radius_calc = np.sqrt(y**2 + x**2)
            #     rmax = max(rmax,radius_calc)
            #     idmin = np.abs(radius_calc - radius_shoot).argmin()
            #     if radius_calc > radius_shoot[idmin]:
            #         left_id = idmin
            #         right_id = idmin+1
            #     else:
            #         left_id = idmin-1
            #         right_id = idmin
            #     points_score_percent.append(100*(chance_score[left_id] + (chance_score[left_id] - chance_score[right_id])*(radius_calc-radius_shoot[left_id])/ (radius_shoot[left_id] -radius_shoot[right_id])))
            #
            #
            # print(rmax)
            # plt.figure()
            # goal_angle = np.array(goal_angle)
            # plt.scatter(goal_angle[:,0],goal_angle[:,1])
            # plt.show
            #
            # square_w = 47.0
            # hex_across_flats = 41.7  # flat-to-flat distance
            # R = hex_across_flats /(2* np.cos(np.pi / 6))  # circumradius
            # cx = W / 2.0
            # sq_left = cx - square_w / 2.0
            # sq_top = 0.0
            # sq_bottom = sq_top - square_w
            # hex_center_y = sq_top - R
            # angles = np.deg2rad([30, 90, 150, 210, 270, 330])
            # hex_xy = np.column_stack([
            #     cx + R * np.cos(angles),
            #     hex_center_y + R * np.sin(angles)
            # ])
            #
            # # --- Plot ---
            # fig, ax = plt.subplots()
            #
            # # cntr = ax.tricontourf(points[:,0],points[:,1],points_score_percent, levels = 256)
            # # cbar = fig.colorbar(cntr, ax=ax)
            # # cbar.set_label("Chance to Score")
            #
            # cntr = ax.tripcolor(points[:,0],points[:,1],points_score_percent,  shading="gouraud")
            # cbar = fig.colorbar(cntr, ax=ax)
            # cbar.set_label("Chance to Score")
            #
            # boundary = patches.Rectangle((0, 0), W, D, fill=False, linewidth=2)
            # ax.add_patch(boundary)
            #
            # sq = patches.Rectangle((sq_left, sq_bottom), square_w, square_w, fill=True, linewidth=2, color = 'black')
            # ax.add_patch(sq)
            #
            # # Hexagon
            # hex_patch = patches.Polygon(hex_xy, closed=True, fill=True, linewidth=2, color = 'blue')
            # ax.add_patch(hex_patch)
            #
            # # Labels / view
            # ax.set_title("2D Contour Layout")
            # ax.set_xlabel("Distance(x)")
            # ax.set_ylabel("Distance (y)")
            # ax.set_aspect("equal", adjustable="box")
            # ax.set_xlim(-5, W + 5)
            # ax.set_ylim(-50, D + 5)
            #
            #
            # end = time.perf_counter()
            # print(f"it took {end-start} seconds")
            #
            # v_initial_record = np.array(v_initial_record)
            # theta_initial_record =np.array(theta_initial_record)
            # phi_initial_record =np.array(phi_initial_record)
            # X = np.column_stack([v_initial_record,theta_initial_record,phi_initial_record])
            # print(v_initial_record.shape, theta_initial_record.shape, phi_initial_record.shape,X.shape)
            #
            # y = np.asarray(score_record).astype(int)
            # X = np.asarray(X, dtype = float)
            #
            # cov_all = np.cov(X, rowvar=False)
            # # cov_fail = np.cov(X[y == 0], rowvar=False)   # covariance among inputs when failing
            # # cov_succ = np.cov(X[y == 1], rowvar=False)
            #
            # print(cov_all)





# if max(z)<0:
#     print(f"for dt = {dt} there was no solution found")

#
# #out put printing
# print(f"Theta_initial mean = {np.mean(theta_initial):.4f} and std = {np.std(theta_initial):.4f}")
# # print(f"For phi mean = {np.mean(phi)} and std = {np.std(phi)}")
# print(f"V_initial mean = {np.mean(vi):.4f} and std = {np.std(vi):.4f}")
# print(f"Distance mean = {np.mean(x):.4f} and std = {np.std(x):.4f}, max = {np.max(x):.4f}")
#
# score = np.sqrt((x_goal-x)**2) < x_goal_radius
# print(f"you scored {100*np.count_nonzero(score)/np.size(score,0):.2f}%")
#
# #output plotting
# plt.figure()
# plt.tricontourf(v_initial,theta_initial,x)
# plt.colorbar(label="Distance")
# plt.xlabel("Initial Velocity (m/s)")
# plt.ylabel("Intial Theta (Deg)")
# plt.title('Distance vs Initial Conditions')
# plt.scatter(v_initial, theta_initial, s=1, c="k")
#
# fig, ax = plt.subplots()
# ax.scatter(x, y)
# circ = Circle((x_goal, 0), x_goal_radius, fill=False, linewidth=2, edgecolor = 'red')  # add edgecolor=... if you want
# ax.add_patch(circ)
# ax.text(x_goal + x_goal_radius*0.9, x_goal_radius *0.9, "Goal", ha="center", va="center", color = 'red', size = 20 )
# ax.set_aspect('equal', adjustable='box')  # makes it look like a true circle
# ax.set_xlabel("X Distance (m)")
# ax.set_ylabel("Y Distance (m)")
# ax.set_title("Monte Carlo of Shots")

# plt.figure()
# plt.scatter(theta_initial,x)
# plt.grid(True,which='major')


plt.show()
