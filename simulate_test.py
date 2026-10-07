import mujoco
import numpy as np
import os
from fuzzy_controller import CascadeFuzzyController

def run_simulation(duration=5.0, theta_init=0.1, x_ref=0.0, gains=None):
    model_path = os.path.join(os.path.dirname(__file__), "cart_pendulum_hust.xml")
    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)

    if gains is None:
        controller = CascadeFuzzyController()
    else:
        controller = CascadeFuzzyController(pos_gains=gains['pos'], ang_gains=gains['ang'])

    # Set initial state
    d.qpos[0] = 0.0          # x = 0
    d.qvel[0] = 0.0          # x_dot = 0
    d.qpos[1] = theta_init   # theta
    d.qvel[1] = 0.0          # theta_dot = 0

    dt = m.opt.timestep
    n_steps = int(duration / dt)

    t_hist = []
    x_hist = []
    xdot_hist = []
    theta_hist = []
    thetadot_hist = []
    f_hist = []
    thref_hist = []

    for step in range(n_steps):
        t = step * dt
        x = d.qpos[0]
        x_dot = d.qvel[0]
        theta = d.qpos[1]
        theta_dot = d.qvel[1]

        # Compute control
        F, theta_ref = controller.compute(x, x_dot, theta, theta_dot, x_ref=x_ref)
        d.ctrl[0] = F

        t_hist.append(t)
        x_hist.append(x)
        xdot_hist.append(x_dot)
        theta_hist.append(theta)
        thetadot_hist.append(theta_dot)
        f_hist.append(F)
        thref_hist.append(theta_ref)

        mujoco.mj_step(m, d)

        # Check if pendulum fell beyond 60 degrees (unstable)
        if abs(theta) > np.deg2rad(60):
            print(f"Instability detected at t = {t:.3f}s: theta = {np.rad2deg(theta):.1f} deg")
            return False, (t_hist, x_hist, theta_hist, f_hist)

    print(f"Simulation completed successfully! Final x = {x_hist[-1]:.4f} m, final theta = {np.rad2deg(theta_hist[-1]):.3f} deg")
    return True, (t_hist, x_hist, theta_hist, f_hist, thref_hist)

if __name__ == "__main__":
    success, res = run_simulation(duration=5.0, theta_init=0.1, x_ref=0.0)
    print("Success:", success)
