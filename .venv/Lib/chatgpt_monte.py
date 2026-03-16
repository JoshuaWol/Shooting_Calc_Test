import math
import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# USER INPUTS
# ============================================================
N = 5000

v0_nom = 11.0  # m/s

# 3-sigma specs (interpreting "3 sigma: 10% v0, 3deg theta" as ± at 3σ)
sigma_v = (0.10 * v0_nom) / 3.0   # m/s
sigma_theta_deg = 3.0 / 3.0       # deg => 1 deg

# Heights (in)
z0_in = 18.0
zT_in = 72.0

# Goal + ball geometry (in)
goal_diam_in = 20.85   # circle opening diameter
ball_diam_in = 6.0     # ball diameter

# Ball weight / drag model
weight_lb = 0.475
mass_mode = "lbf"      # "lbf" treat 0.475 as weight (lbf); "lbm" treat as mass (lbm)
Cd = 0.47
rho = 1.225
g = 9.80665

# Integration
dt = 0.002   # (speed-up vs 0.001; reduce if you want more accuracy)
tmax = 8.0

# Distance sweep (m)
D_min = 1.25
D_max = 6.5
num_D = 120  # increase if you want smoother curve (costs time)

seed = 123
# ============================================================


# ============================================================
# CONVERSIONS + PARAMETERS
# ============================================================
IN_TO_M = 0.0254
LBF_TO_N = 4.4482216152605
LBM_TO_KG = 0.45359237

z0 = z0_in * IN_TO_M
zT = zT_in * IN_TO_M

ball_d = ball_diam_in * IN_TO_M
A = math.pi * ball_d**2 / 4.0

goal_R = (goal_diam_in * IN_TO_M) / 2.0
ball_R = (ball_diam_in * IN_TO_M) / 2.0

# Clearance for the BALL CENTER to pass through the circular opening
tol = max(0.0, goal_R - ball_R)  # meters

if mass_mode.lower() == "lbf":
    W_N = weight_lb * LBF_TO_N
    m = W_N / g
elif mass_mode.lower() == "lbm":
    m = weight_lb * LBM_TO_KG
else:
    raise ValueError("mass_mode must be 'lbf' or 'lbm'")

# Quadratic drag accel: a_drag = -k*|v|*v_vec
k = 0.5 * rho * Cd * A / m


# ============================================================
# STEPPING: accel -> vel -> pos (Euler-Cromer)
# ============================================================
def step_euler_cromer(x, z, vx, vz, dt):
    v = math.hypot(vx, vz)
    ax = -k * v * vx
    az = -g - k * v * vz

    vx_new = vx + ax * dt
    vz_new = vz + az * dt

    x_new = x + vx_new * dt
    z_new = z + vz_new * dt

    return x_new, z_new, vx_new, vz_new


def x_hit_descending(v0, theta_deg):
    """
    Return x (m) when z crosses zT on the way DOWN.
    Must go above zT first. Returns np.nan if never happens.
    """
    th = math.radians(theta_deg)
    x, z = 0.0, z0
    vx, vz = v0 * math.cos(th), v0 * math.sin(th)

    went_above = (z > zT)
    t = 0.0

    while t < tmax:
        x_prev, z_prev = x, z

        x, z, vx, vz = step_euler_cromer(x, z, vx, vz, dt)
        t += dt

        # hit ground
        if z < 0.0:
            return np.nan

        # track whether we got above target
        if z > zT:
            went_above = True

        # downward crossing after going above
        if went_above and (z_prev > zT) and (z <= zT) and (vz < 0.0):
            frac = (zT - z_prev) / (z - z_prev) if (z - z_prev) != 0 else 0.0
            return x_prev + frac * (x - x_prev)

    return np.nan


# ============================================================
# Find nominal aim angle theta*(D) such that x_hit_descending(v0_nom, theta*) ~= D
# We'll scan for a bracket then bisect.
# ============================================================
def find_theta_for_distance(D, theta_min=5.0, theta_max=85.0, scan_step=0.5, iters=40):
    thetas = np.arange(theta_min, theta_max + scan_step, scan_step)
    xs = np.array([x_hit_descending(v0_nom, th) for th in thetas], dtype=float)

    # Only consider finite results
    finite = np.isfinite(xs)
    if finite.sum() < 2:
        return np.nan

    thetas_f = thetas[finite]
    xs_f = xs[finite]

    # Need a sign change in f(theta)=x(theta)-D to bisect
    f = xs_f - D

    # Find an interval where f changes sign
    for i in range(len(f) - 1):
        if f[i] == 0:
            return float(thetas_f[i])
        if f[i] * f[i + 1] < 0:
            a, b = float(thetas_f[i]), float(thetas_f[i + 1])
            fa, fb = float(f[i]), float(f[i + 1])

            # Bisection
            for _ in range(iters):
                mid = 0.5 * (a + b)
                xmid = x_hit_descending(v0_nom, mid)
                if not np.isfinite(xmid):
                    # If mid invalid, nudge slightly (rare)
                    mid += 1e-3
                    xmid = x_hit_descending(v0_nom, mid)
                    if not np.isfinite(xmid):
                        return np.nan
                fmid = xmid - D
                if fa * fmid <= 0:
                    b, fb = mid, fmid
                else:
                    a, fa = mid, fmid
            return 0.5 * (a + b)

    return np.nan  # no bracket => no solution in that angle range


# ============================================================
# MONTE CARLO vs distance
# For each D:
#  1) compute theta*(D) so nominal hits center
#  2) simulate N trials around v0 and theta
#  3) make if |x_hit - D| <= tol
# ============================================================
rng = np.random.default_rng(488)

# Pre-sample random deltas (reuse for each distance)
dv = rng.normal(0.0, sigma_v, size=N)
dtheta = rng.normal(0.0, sigma_theta_deg, size=N)

D_vals = np.linspace(D_min, D_max, num_D)
p_make = np.zeros_like(D_vals)
theta_star = np.full_like(D_vals, np.nan, dtype=float)

# Diagnostics counters
no_solution = 0
for j, D in enumerate(D_vals):
    th_star = find_theta_for_distance(D)
    theta_star[j] = th_star

    if not np.isfinite(th_star):
        no_solution += 1
        p_make[j] = 0.0
        continue

    v_samp = np.clip(v0_nom + dv, 0.0, None)
    th_samp = th_star + dtheta

    x_hits = np.empty(N, dtype=float)
    for i in range(N):
        x_hits[i] = x_hit_descending(v_samp[i], th_samp[i])

    ok = np.isfinite(x_hits)
    makes = ok & (np.abs(x_hits - D) <= tol)
    p_make[j] = makes.mean()

print(f"Mass={m:.4f} kg, diameter={ball_d:.4f} m, Cd={Cd}, rho={rho}, dt={dt}")
print(f"Goal diam={goal_diam_in} in, Ball diam={ball_diam_in} in => tol(center)={tol:.4f} m")
print(f"D sweep: {D_min}..{D_max} m, points={num_D}, Monte Carlo N={N}")
print(f"Distances with no aim solution in [5°,85°]: {no_solution}/{num_D}")

# ============================================================
# PLOTS
# ============================================================
plt.figure()
plt.plot(D_vals, p_make)
plt.xlabel("Goal distance D (m)")
plt.ylabel("Probability of make")
plt.title("Make probability vs distance (aimed so nominal hits center)")
plt.grid(True)
plt.tight_layout()
plt.show()

plt.figure()
plt.plot(D_vals, theta_star)
plt.xlabel("Goal distance D (m)")
plt.ylabel("Nominal aim angle θ*(D) (deg)")
plt.title("Aim angle that centers the shot at z=72 in (descending)")
plt.grid(True)
plt.tight_layout()
plt.show()
