"""
Interactive 3D Simulation of HUST Inverted Cart-Pendulum with Cascade Fuzzy Controller in MuJoCo.
Usage:
    python run_mujoco_viewer.py
Controls:
    - Double click on cart or pole to select
    - Right click & drag to apply external disturbance forces
    - Space: pause/resume simulation
    - Backspace: reset simulation
    - Press [1]: Set target position x = 0.0m
    - Press [2]: Set target position x = 0.15m
    - Press [3]: Set target position x = -0.15m
"""

import os
import time
import mujoco
import mujoco.viewer
import numpy as np
from fuzzy_controller import FuzzyPD

MODEL_PATH = os.path.join(os.path.dirname(__file__), "cart_pendulum_hust.xml")

def main():
    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)

    # Initial perturbation (e.g. 4 degrees)
    d.qpos[1] = np.deg2rad(4.0)

    # Tuned Cascade Fuzzy Logic Controller
    inner_fuzzy = FuzzyPD(Ke=6.0, Kd=0.65, Ku=0.85, out_limit=3.0)
    outer_fuzzy = FuzzyPD(Ke=2.2, Kd=4.0, Ku=0.06, out_limit=0.08)

    x_target = 0.0

    print("=" * 65)
    print("  HUST Inverted Cart-Pendulum MuJoCo Simulation (Cascade-Fuzzy)")
    print("=" * 65)
    print("  Parameters: M = 0.059 kg, m = 0.020 kg, L_com = 0.111 m")
    print("  Interactive controls:")
    print("    - Right-click & drag in 3D view to push pole / cart")
    print("    - Target position: x_ref = 0.0 m")
    print("=" * 65)

    with mujoco.viewer.launch_passive(m, d) as viewer:
        start_time = time.time()
        step_count = 0

        while viewer.is_running():
            step_start = time.time()

            x = d.qpos[0]
            xd = d.qvel[0]
            th = d.qpos[1]
            thd = d.qvel[1]

            # Cascade Fuzzy:
            # 1. Outer Loop: Position error -> target lean angle
            e_x = x_target - x
            de_x = -xd
            th_ref = outer_fuzzy.compute(e_x, de_x)

            # 2. Inner Loop: Angle error -> Cart Force F
            e_th = th - th_ref
            de_th = thd
            F = inner_fuzzy.compute(e_th, de_th)

            d.ctrl[0] = F

            mujoco.mj_step(m, d)
            viewer.sync()
            step_count += 1

            # Print status every 500 steps (1.0s)
            if step_count % 500 == 0:
                print(f"[t={d.time:5.2f}s] Cart x={x:+.3f}m | Angle θ={np.rad2deg(th):+.2f}° | θ_ref={np.rad2deg(th_ref):+.2f}° | Force F={F:+.3f}N")

            # Maintain real-time simulation rate
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()
