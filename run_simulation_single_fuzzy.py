import os
import mujoco
import numpy as np
import matplotlib.pyplot as plt
from fuzzy_single_controller import SingleUnifiedFuzzyController

MODEL_PATH = os.path.join(os.path.dirname(__file__), "cart_pendulum_hust.xml")

def run_simulation(scenario="perturbation", duration=6.0):
    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)

    dt = m.opt.timestep
    steps = int(duration / dt)

    ctrl = SingleUnifiedFuzzyController()

    if scenario == "perturbation":
        # 5 degrees initial tilt (0.087 rad)
        d.qpos[1] = np.deg2rad(5.0)
        x_target_func = lambda t: 0.0
    elif scenario == "step":
        # Start at 0, step to 0.15 m at t = 1.0s
        d.qpos[1] = 0.0
        x_target_func = lambda t: 0.15 if t >= 1.0 else 0.0
    else:
        d.qpos[1] = np.deg2rad(5.0)
        x_target_func = lambda t: 0.0

    t_vec = []
    x_vec = []
    th_deg_vec = []
    F_vec = []
    xref_vec = []

    for step in range(steps):
        t = step * dt
        x = d.qpos[0]
        xd = d.qvel[0]
        th = d.qpos[1]
        thd = d.qvel[1]

        x_ref = x_target_func(t)

        # Single FIS computation: 4 inputs -> 1 Force
        F = ctrl.compute(theta=th, theta_dot=thd, x=x, x_dot=xd, x_ref=x_ref)

        d.ctrl[0] = F
        mujoco.mj_step(m, d)

        t_vec.append(t)
        x_vec.append(x)
        th_deg_vec.append(np.rad2deg(th))
        F_vec.append(F)
        xref_vec.append(x_ref)

    return {
        't': np.array(t_vec),
        'x': np.array(x_vec),
        'x_ref': np.array(xref_vec),
        'theta': np.array(th_deg_vec),
        'F': np.array(F_vec)
    }

def generate_plots():
    print("Running MuJoCo simulations with Single Unified Fuzzy Controller...")
    res1 = run_simulation("perturbation", duration=6.0)
    res2 = run_simulation("step", duration=7.0)

    fig, axs = plt.subplots(3, 2, figsize=(14, 9), sharex='col')
    fig.suptitle("MuJoCo Simulation: Single Unified Fuzzy Controller (4 Inputs -> 1 Force)\n"
                 "(Gộp hoàn toàn thành 1 bộ FIS - Non-Cascade Architecture | HUST SolidWorks Model)",
                 fontsize=12, fontweight='bold')

    # Column 1: Initial Perturbation (Theta_0 = 5 deg)
    axs[0, 0].set_title("Scenario 1: Angle Perturbation (5 deg)", fontsize=11, fontweight='bold', color='navy')
    axs[0, 0].plot(res1['t'], res1['theta'], 'r-', lw=2, label=r'$\theta$ (Angle)')
    axs[0, 0].axhline(0, color='gray', ls=':')
    axs[0, 0].set_ylabel("Angle [deg]")
    axs[0, 0].grid(True, alpha=0.3)
    axs[0, 0].legend(loc='upper right')

    axs[1, 0].plot(res1['t'], res1['x'], 'b-', lw=2, label=r'$x$ (Cart Position)')
    axs[1, 0].plot(res1['t'], res1['x_ref'], 'g--', lw=1.2, label=r'$x_{\rm ref}$')
    axs[1, 0].axhline(0, color='gray', ls=':')
    axs[1, 0].set_ylabel("Position [m]")
    axs[1, 0].grid(True, alpha=0.3)
    axs[1, 0].legend(loc='upper right')

    axs[2, 0].plot(res1['t'], res1['F'], 'm-', lw=1.8, label=r'$F$ (Control Force)')
    axs[2, 0].set_ylabel("Force [N]")
    axs[2, 0].set_xlabel("Time [s]")
    axs[2, 0].grid(True, alpha=0.3)
    axs[2, 0].legend(loc='upper right')

    # Column 2: Step Response (x_ref = 0.15m at t=1.0s)
    axs[0, 1].set_title("Scenario 2: Setpoint Tracking (x_ref = 0.15m)", fontsize=11, fontweight='bold', color='navy')
    axs[0, 1].plot(res2['t'], res2['theta'], 'r-', lw=2, label=r'$\theta$ (Angle)')
    axs[0, 1].axhline(0, color='gray', ls=':')
    axs[0, 1].set_ylabel("Angle [deg]")
    axs[0, 1].grid(True, alpha=0.3)
    axs[0, 1].legend(loc='upper right')

    axs[1, 1].plot(res2['t'], res2['x'], 'b-', lw=2, label=r'$x$ (Cart Position)')
    axs[1, 1].plot(res2['t'], res2['x_ref'], 'g--', lw=1.2, label=r'$x_{\rm ref}$ (Target)')
    axs[1, 1].set_ylabel("Position [m]")
    axs[1, 1].grid(True, alpha=0.3)
    axs[1, 1].legend(loc='lower right')

    axs[2, 1].plot(res2['t'], res2['F'], 'm-', lw=1.8, label=r'$F$ (Control Force)')
    axs[2, 1].set_ylabel("Force [N]")
    axs[2, 1].set_xlabel("Time [s]")
    axs[2, 1].grid(True, alpha=0.3)
    axs[2, 1].legend(loc='upper right')

    plt.tight_layout()
    output_png = os.path.join(os.path.dirname(__file__), "fuzzy_single_mujoco_response.png")
    plt.savefig(output_png, dpi=300)
    print(f"Plot saved successfully to: {output_png}")

if __name__ == "__main__":
    generate_plots()
