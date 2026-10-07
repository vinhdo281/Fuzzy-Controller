"""
Interactive 3D Simulation of HUST Cart-Pendulum with Single Unified Fuzzy Controller in MuJoCo.
Usage:
    python run_mujoco_viewer.py
Controls:
    - Right-click & drag on cart or pole to push it (apply external disturbances)
    - Space: pause/resume simulation
    - Backspace: reset simulation
"""

import os
import time
import mujoco
import mujoco.viewer
import numpy as np
from fuzzy_single_controller import SingleUnifiedFuzzyController

MODEL_PATH = os.path.join(os.path.dirname(__file__), "cart_pendulum_hust.xml")

def main():
    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)

    # Initial perturbation: 20 degrees tilt (User specified large perturbation test)
    d.qpos[1] = np.deg2rad(20.0)

    # Single Unified Fuzzy Controller (4 Inputs -> 1 Output Force)
    ctrl = SingleUnifiedFuzzyController()

    x_target = 0.0

    print("=" * 68)
    print("  HUST Inverted Cart-Pendulum MuJoCo (Single Unified Fuzzy Controller)")
    print("=" * 68)
    print("  Architecture: 1 Single FIS (4 Inputs: [Theta, Theta_dot, x, x_dot] -> Force)")
    print("  Parameters: M = 0.059 kg, m = 0.020 kg, L_com = 0.111 m")
    print("  Interactive controls:")
    print("    - Right-click & drag in 3D view to push pole / cart")
    print("    - Target position: x_ref = 0.0 m")
    print("=" * 68)

    with mujoco.viewer.launch_passive(m, d) as viewer:
        step_count = 0

        while viewer.is_running():
            step_start = time.time()

            x = d.qpos[0]
            xd = d.qvel[0]
            th = d.qpos[1]
            thd = d.qvel[1]

            # Single Unified Fuzzy Controller
            F = ctrl.compute(theta=th, theta_dot=thd, x=x, x_dot=xd, x_ref=x_target)

            d.ctrl[0] = F

            mujoco.mj_step(m, d)
            viewer.sync()
            step_count += 1

            # Print status every 500 steps (1.0s)
            if step_count % 500 == 0:
                print(f"[t={d.time:5.2f}s] Cart x={x:+.3f}m | Angle theta={np.rad2deg(th):+.2f} deg | Force F={F:+.3f}N")

            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()
