import os
import mujoco
import numpy as np
from scipy.optimize import minimize
from fuzzy_controller import FuzzyPD

MODEL_PATH = os.path.join(os.path.dirname(__file__), "full_pendulum_assem.xml")

def evaluate_controller(params, render=False):
    """
    params: [Ke_ang, Kd_ang, Ku_ang, Ke_pos, Kd_pos, Ku_pos]
    """
    Ke_ang, Kd_ang, Ku_ang, Ke_pos, Kd_pos, Ku_pos = params
    
    # Check positivity
    if any(p <= 0 for p in params):
        return 1e6

    inner_ang = FuzzyPD(Ke=Ke_ang, Kd=Kd_ang, Ku=Ku_ang, out_limit=3.0)
    outer_pos = FuzzyPD(Ke=Ke_pos, Kd=Kd_pos, Ku=Ku_pos, out_limit=0.15) # max tilt ~ 8.6 deg

    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)

    # Initial perturbation: theta = 0.08 rad (~4.6 deg), cart at x = 0
    d.qpos[0] = 0.0
    d.qvel[0] = 0.0
    d.qpos[1] = 0.08
    d.qvel[1] = 0.0

    dt = m.opt.timestep
    total_time = 4.0
    steps = int(total_time / dt)

    cost = 0.0
    fell = False

    for step in range(steps):
        t = step * dt
        x = d.qpos[0]
        x_dot = d.qvel[0]
        theta = d.qpos[1]
        theta_dot = d.qvel[1]

        if abs(theta) > np.deg2rad(45) or abs(x) > 0.38:
            fell = True
            # Heavy penalty proportional to remaining time
            remaining = steps - step
            cost += remaining * 100.0
            break

        # Cascade logic:
        # Lean-to-steer:
        # If cart is at x > 0, to return to 0, cart must steer left.
        # Steering left requires pole leaning left (theta_ref < 0).
        # outer_pos gives positive when x > 0. So theta_ref = - outer_pos.compute(x, x_dot)
        th_ref = -outer_pos.compute(x, x_dot)
        
        # Inner loop:
        # If theta > th_ref (pole is leaning more right than ref), cart must accelerate right (F > 0).
        # inner_ang gives positive when error > 0.
        e_th = theta - th_ref
        de_th = theta_dot
        F = inner_ang.compute(e_th, de_th)

        d.ctrl[0] = F
        mujoco.mj_step(m, d)

        # Cost: integrated quadratic error
        cost += (50.0 * theta**2 + 5.0 * theta_dot**2 + 10.0 * x**2 + 2.0 * x_dot**2 + 0.1 * F**2) * dt

    return cost

def tune():
    print("=== Auto-Tuning Fuzzy Controller for HUST Cart-Pendulum in MuJoCo ===")
    # Initial guess: [Ke_ang, Kd_ang, Ku_ang, Ke_pos, Kd_pos, Ku_pos]
    init_params = [8.0, 1.5, 1.2, 2.0, 1.5, 0.05]
    print(f"Initial cost: {evaluate_controller(init_params):.2f}")

    bounds = [
        (1.0, 30.0),   # Ke_ang
        (0.1, 10.0),   # Kd_ang
        (0.1, 5.0),    # Ku_ang (Force scale)
        (0.5, 10.0),   # Ke_pos
        (0.1, 10.0),   # Kd_pos
        (0.01, 0.20)   # Ku_pos (Angle ref scale)
    ]

    res = minimize(
        evaluate_controller,
        init_params,
        method='Nelder-Mead',
        options={'maxiter': 300, 'disp': True}
    )

    print("\nOptimization Finished!")
    print("Optimal Parameters:")
    names = ['Ke_ang', 'Kd_ang', 'Ku_ang (Force)', 'Ke_pos', 'Kd_pos', 'Ku_pos (Tilt ref)']
    for n, v in zip(names, res.x):
        print(f"  {n:20s}: {v:.4f}")
    print(f"Final Cost: {res.fun:.4f}")
    return res.x

if __name__ == "__main__":
    best_params = tune()
