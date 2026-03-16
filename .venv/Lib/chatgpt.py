import math
import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# USER INPUTS
# ============================================================
v0 = 11.0                 # m/s (launch speed)
angles_deg = np.linspace(1, 89, 177)  # angles to sweep for plotting

z0_in = 18.0              # launch height (in)
zT_in = 72.0              # target height (in)

diam_in = 6.0             # sphere diameter (in)
weight_lb = 0.475         # assumed lbf (force) unless mass_mode="lbm"

mass_mode = "lbf"         # "lbf" = treat 0.475 as weight (lbf); "lbm" = treat as mass (lbm)
Cd = 0.47                 # smooth sphere (typical)
rho = 1.225               # kg/m^3
g = 9.80665               # m/s^2

dt = 0.001                # s
tmax = 10.0               # s

# Example trajectory to plot (optional)
example_angle_deg = 45.0


# ============================================================
# UNIT CONVERSIONS + BALL PARAMS
# ============================================================
IN_TO_M = 0.0254
LBF_TO_N = 4.4482216152605
LBM_TO_KG = 0.45359237
M_TO_FT = 3.280839895

z0 = z0_in * IN_TO_M
zT = zT_in * IN_TO_M

d = diam_in * IN_TO_M
A = math.pi * d**2 / 4.0

if mass_mode.lower() == "lbf":
    W_N = weight_lb * LBF_TO_N
    m = W_N / g
elif mass_mode.lower() == "lbm":
    m = weight_lb * LBM_TO_KG
else:
    raise ValueError("mass_mode must be 'lbf' or 'lbm'")

# Quadratic drag acceleration: a_drag = -k*|v|*v_vec
k = 0.5 * rho * Cd * A / m


# ============================================================
# STEPPING: accel -> vel -> pos (Euler-Cromer)
# ============================================================
def step_euler_cromer(x, z, vx, vz, dt, use_drag=True):
    # --- acceleration step ---
    if use_drag:
        v = math.hypot(vx, vz)
        ax = -k * v * vx
        az = -g - k * v * vz
    else:
        ax = 0.0
        az = -g

    # --- velocity step ---
    vx_new = vx + ax * dt
    vz_new = vz + az * dt

    # --- position step ---
    x_new = x + vx_new * dt
    z_new = z + vz_new * dt

    return x_new, z_new, vx_new, vz_new


# ============================================================
# Find x when z hits target ON THE WAY DOWN
# - must go ABOVE target first
# - then detect downward crossing: z_prev > zT and z <= zT
# ============================================================
def x_when_reach_target_descending(theta_deg, use_drag=True):
    th = math.radians(theta_deg)
    x, z = 0.0, z0
    vx, vz = v0 * math.cos(th), v0 * math.sin(th)

    t = 0.0
    went_above = (z > zT)

    while t < tmax:
        x_prev, z_prev, vz_prev = x, z, vz

        # accel -> vel -> pos
        x, z, vx, vz = step_euler_cromer(x, z, vx, vz, dt, use_drag=use_drag)
        t += dt

        # If we hit the ground, stop
        if z < 0.0:
            return np.nan

        # Track if we've ever gone above the target
        if z > zT:
            went_above = True

        # After we've been above, look for downward crossing
        if went_above and (z_prev > zT) and (z <= zT) and (vz < 0.0):
            # interpolate to get x at exactly zT
            frac = (zT - z_prev) / (z - z_prev) if (z - z_prev) != 0 else 0.0
            return x_prev + frac * (x - x_prev)

    return np.nan


# ============================================================
# No-drag analytic: choose the LARGER time root (descending)
# z(t)=z0 + v0*sin(th)*t - 0.5*g*t^2 = zT
# ============================================================
def x_when_reach_target_descending_nodrag_analytic(theta_deg):
    th = math.radians(theta_deg)
    vx = v0 * math.cos(th)
    vz0 = v0 * math.sin(th)
    dz = zT - z0

    # 0.5*g*t^2 - vz0*t + dz = 0
    A_q = 0.5 * g
    B_q = -vz0
    C_q = dz
    disc = B_q*B_q - 4*A_q*C_q
    if disc < 0:
        return np.nan

    t1 = (-B_q - math.sqrt(disc)) / (2*A_q)
    t2 = (-B_q + math.sqrt(disc)) / (2*A_q)

    # descending hit is the larger positive root
    ts = [t for t in (t1, t2) if t > 0]
    if len(ts) == 0:
        return np.nan
    t_desc = max(ts)
    return vx * t_desc


# ============================================================
# Sweep angles and plot "distance when coming DOWN through zT"
# ============================================================
x_drag = np.array([x_when_reach_target_descending(a, use_drag=True) for a in angles_deg], dtype=float)
x_nodrag_num = np.array([x_when_reach_target_descending(a, use_drag=False) for a in angles_deg], dtype=float)
x_nodrag_ana = np.array([x_when_reach_target_descending_nodrag_analytic(a) for a in angles_deg], dtype=float)

plt.figure()
plt.plot(angles_deg, x_drag, label="With quadratic drag (descending hit)")
plt.plot(angles_deg, x_nodrag_ana, linestyle="--", label="No drag (analytic, descending hit)")
plt.xlabel("Launch angle (deg)")
plt.ylabel("Horizontal distance when crossing z = 72 in on the way DOWN (m)")
plt.title("Descending crossing distance to z=72 in (from z0=18 in, v0=11 m/s)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

valid = np.isfinite(x_drag)
if np.any(valid):
    i_best = np.nanargmax(x_drag)
    print(f"Mass_mode='{mass_mode}', mass={m:.4f} kg, diameter={d:.4f} m")
    print(f"Max descending-crossing distance (drag): {x_drag[i_best]*M_TO_FT:.2f} ft at {angles_deg[i_best]:.1f} deg")
else:
    print("No angles in sweep went above 72 in and then crossed it descending (check v0/angles).")


# ============================================================
# Optional: plot an example trajectory and mark descending crossing
# ============================================================
def simulate_trajectory(theta_deg, use_drag=True):
    th = math.radians(theta_deg)
    x, z = 0.0, z0
    vx, vz = v0 * math.cos(th), v0 * math.sin(th)

    ts, xs, zs = [0.0], [x], [z]
    t = 0.0
    while t < tmax and z >= 0.0:
        x, z, vx, vz = step_euler_cromer(x, z, vx, vz, dt, use_drag=use_drag)
        t += dt
        ts.append(t); xs.append(x); zs.append(z)
    return np.array(ts), np.array(xs), np.array(zs)

if example_angle_deg is not None:
    tD, xD, zD = simulate_trajectory(example_angle_deg, use_drag=True)
    tN, xN, zN = simulate_trajectory(example_angle_deg, use_drag=False)
