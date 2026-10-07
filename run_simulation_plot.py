import os
import mujoco
import numpy as np
import matplotlib.pyplot as plt
from fuzzy_controller import FuzzyPD

# Use model with exact HUST SolidWorks parameters
MODEL_PATH = os.path.join(os.path.dirname(__file__), "full_pendulum_assem.xml")

def run_simulation(scenario="perturbation", duration=5.0):
    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)

    dt = m.opt.timestep
    steps = int(duration / dt)

    # Controller gains tuned for HUST hardware parameters
    inner_fuzzy = FuzzyPD(Ke=6.0, Kd=0.65, Ku=0.85, out_limit=3.0)
    outer_fuzzy = FuzzyPD(Ke=2.2, Kd=4.0, Ku=0.06, out_limit=0.08)

    # Initial condition setup
    if scenario == "perturbation":
        # Initial angle tilt of 5 degrees (0.087 rad), cart at 0
        d.qpos[1] = np.deg2rad(5.0)
        x_target_func = lambda t: 0.0
    elif scenario == "step":
        # Start vertical, step command to 0.15m at t = 1.0s
        d.qpos[1] = 0.0
        x_target_func = lambda t: 0.15 if t >= 1.0 else 0.0
    elif scenario == "disturbance":
        # Disturbance impulse at t = 2.0s
        d.qpos[1] = 0.0
        x_target_func = lambda t: 0.0
    else:
        d.qpos[1] = np.deg2rad(5.0)
        x_target_func = lambda t: 0.0

    t_vec = []
    x_vec = []
    th_deg_vec = []
    thref_deg_vec = []
    F_vec = []
    xref_vec = []

    for step in range(steps):
        t = step * dt
        x = d.qpos[0]
        xd = d.qvel[0]
        th = d.qpos[1]
        thd = d.qvel[1]

        x_ref = x_target_func(t)

        # Disturbance force at t = 2.0s lasting 0.05s
        dist_force = 0.0
        if scenario == "disturbance" and 2.0 <= t <= 2.05:
            dist_force = 0.35 # 0.35 N external push on cart

        # 1. Outer Loop: Position error -> target tilt angle (Lean-to-Steer)
        e_x = x_ref - x
        de_x = -xd
        th_ref = outer_fuzzy.compute(e_x, de_x)

        # 2. Inner Loop: Angle error -> Actuator Force F
        e_th = th - th_ref
        de_th = thd
        F = inner_fuzzy.compute(e_th, de_th)

        d.ctrl[0] = F + dist_force
        mujoco.mj_step(m, d)

        t_vec.append(t)
        x_vec.append(x)
        th_deg_vec.append(np.rad2deg(th))
        thref_deg_vec.append(np.rad2deg(th_ref))
        F_vec.append(F)
        xref_vec.append(x_ref)

    return {
        't': np.array(t_vec),
        'x': np.array(x_vec),
        'x_ref': np.array(xref_vec),
        'theta': np.array(th_deg_vec),
        'theta_ref': np.array(thref_deg_vec),
        'F': np.array(F_vec)
    }

def generate_plots():
    print("Running simulations for 2 scenarios...")
    res1 = run_simulation("perturbation", duration=6.0)
    res2 = run_simulation("step", duration=7.0)

    fig, axs = plt.subplots(3, 2, figsize=(14, 9), sharex='col')
    fig.suptitle("MuJoCo Simulation: Cascade Fuzzy Logic Controller for Cart-Pendulum\n"
                 "(Parameters from full_pendulum_assem_DataFile3.m: M=0.059kg, m=0.020kg, L_com=0.111m)",
                 fontsize=12, fontweight='bold')

    # Column 1: Initial Perturbation (Theta_0 = 5 deg)
    axs[0, 0].set_title("Scenario 1: Initial Angle Perturbation (5 deg)", fontsize=11, fontweight='bold', color='navy')
    axs[0, 0].plot(res1['t'], res1['theta'], 'r-', lw=2, label=r'$\theta$ (Angle)')
    axs[0, 0].plot(res1['t'], res1['theta_ref'], 'k--', lw=1.2, label=r'$\theta_{\rm ref}$ (Fuzzy Outer Loop)')
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
    axs[0, 1].plot(res2['t'], res2['theta_ref'], 'k--', lw=1.2, label=r'$\theta_{\rm ref}$ (Lean-to-Steer)')
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
    output_png = os.path.join(os.path.dirname(__file__), "fuzzy_mujoco_response.png")
    plt.savefig(output_png, dpi=300)
    print(f"Plot saved successfully to: {output_png}")

if __name__ == "__main__":
    generate_plots()
