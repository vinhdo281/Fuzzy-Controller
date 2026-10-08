import os
import mujoco
import numpy as np
from fuzzy_single_controller import SingleUnifiedFuzzyController

def run_simulation(duration=5.0, theta_init_deg=5.0, x_ref=0.0):
    """
    Run MuJoCo simulation test with Cart-Pendulum HUST model.
    Parameters:
        duration: simulation duration in seconds
        theta_init_deg: initial tilt angle in degrees (e.g. 5.0, 20.0, 22.0)
        x_ref: target position of cart in meters
    """
    model_path = os.path.join(os.path.dirname(__file__), "full_pendulum_assem.xml")
    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)

    controller = SingleUnifiedFuzzyController()

    theta_init_rad = np.deg2rad(theta_init_deg)

    # Initial state
    d.qpos[0] = 0.0              # x = 0
    d.qvel[0] = 0.0              # x_dot = 0
    d.qpos[1] = theta_init_rad   # theta
    d.qvel[1] = 0.0              # theta_dot = 0

    dt = m.opt.timestep
    n_steps = int(duration / dt)

    t_hist = []
    x_hist = []
    theta_hist = []
    f_hist = []

    print("=== Testing: Single Unified Fuzzy Controller ===")
    print(f"Initial tilt: {theta_init_deg:.1f} deg ({theta_init_rad:.3f} rad), Target x_ref: {x_ref:.2f} m")

    for step in range(n_steps):
        t = step * dt
        x = d.qpos[0]
        x_dot = d.qvel[0]
        theta = d.qpos[1]
        theta_dot = d.qvel[1]

        F = controller.compute(theta=theta, theta_dot=theta_dot, x=x, x_dot=x_dot, x_ref=x_ref)

        d.ctrl[0] = F
        t_hist.append(t)
        x_hist.append(x)
        theta_hist.append(theta)
        f_hist.append(F)

        mujoco.mj_step(m, d)

        # Check if pendulum fell beyond 60 degrees (unstable)
        if abs(theta) > np.deg2rad(60):
            print(f"[FAIL] Instability detected at t = {t:.3f}s: theta = {np.rad2deg(theta):.1f} deg")
            return False, (t_hist, x_hist, theta_hist, f_hist)

    print(f"[SUCCESS] Final x = {x_hist[-1]:.4f} m, final theta = {np.rad2deg(theta_hist[-1]):.3f} deg")
    return True, (t_hist, x_hist, theta_hist, f_hist)

if __name__ == "__main__":
    # Test 1: Single Unified Controller with 5 degrees initial tilt
    print("\n--- Test 1: Single Unified Controller (theta_0 = 5 deg) ---")
    success1, res1 = run_simulation(duration=5.0, theta_init_deg=5.0, x_ref=0.0)
    print("Success:", success1)

    # Test 2: Single Unified Controller with 20 degrees initial tilt (User requirement)
    print("\n--- Test 2: Single Unified Controller (theta_0 = 20 deg) ---")
    success2, res2 = run_simulation(duration=5.0, theta_init_deg=20.0, x_ref=0.0)
    print("Success:", success2)

    # Test 3: Single Unified Controller with 22 degrees initial tilt (Extreme limit test)
    print("\n--- Test 3: Single Unified Controller (theta_0 = 22 deg) ---")
    success3, res3 = run_simulation(duration=5.0, theta_init_deg=22.0, x_ref=0.0)
    print("Success:", success3)
