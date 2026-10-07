"""
Plot Latest Simulation Run:
Single Unified Fuzzy Controller for HUST Cart-Pendulum Inverted Pendulum
Evaluating 20-Degree Large Tilt Recovery and Step Tracking.
"""

import os
import mujoco
import numpy as np
import matplotlib.pyplot as plt
from fuzzy_single_controller import SingleUnifiedFuzzyController

MODEL_PATH = os.path.join(os.path.dirname(__file__), "cart_pendulum_hust.xml")

def simulate_system(theta_init_deg=20.0, x_ref_func=None, duration=6.0):
    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)
    ctrl = SingleUnifiedFuzzyController()

    dt = m.opt.timestep
    steps = int(duration / dt)

    d.qpos[0] = 0.0
    d.qvel[0] = 0.0
    d.qpos[1] = np.deg2rad(theta_init_deg)
    d.qvel[1] = 0.0

    t_arr = np.zeros(steps)
    th_arr = np.zeros(steps)
    thd_arr = np.zeros(steps)
    x_arr = np.zeros(steps)
    xd_arr = np.zeros(steps)
    f_arr = np.zeros(steps)
    xref_arr = np.zeros(steps)

    for i in range(steps):
        t = i * dt
        xref = x_ref_func(t) if x_ref_func else 0.0

        x = d.qpos[0]
        xd = d.qvel[0]
        th = d.qpos[1]
        thd = d.qvel[1]

        f = ctrl.compute(theta=th, theta_dot=thd, x=x, x_dot=xd, x_ref=xref)
        d.ctrl[0] = f

        t_arr[i] = t
        th_arr[i] = np.rad2deg(th)
        thd_arr[i] = np.rad2deg(thd)
        x_arr[i] = x
        xd_arr[i] = xd
        f_arr[i] = f
        xref_arr[i] = xref

        mujoco.mj_step(m, d)

    return {
        't': t_arr,
        'theta': th_arr,
        'theta_dot': thd_arr,
        'x': x_arr,
        'x_dot': xd_arr,
        'f': f_arr,
        'xref': xref_arr
    }

def main():
    print("=" * 65)
    print("  Generating Comprehensive Response Plots for Latest Run")
    print("  Single Unified Fuzzy Controller (20 deg Tilt Recovery)")
    print("=" * 65)

    # 1. Main Run: 20 degrees recovery
    print("Running 20-degree perturbation simulation...")
    res_20 = simulate_system(theta_init_deg=20.0, duration=6.0)

    # 2. Step Tracking: 0.15m at t = 1.0s
    print("Running setpoint tracking simulation (x_ref = 0.15m)...")
    step_func = lambda t: 0.15 if t >= 1.0 else 0.0
    res_step = simulate_system(theta_init_deg=0.0, x_ref_func=step_func, duration=7.0)

    # 3. Angle comparisons: 5, 10, 15, 20 degrees
    print("Running multi-angle comparison simulations (5, 10, 15, 20 deg)...")
    angles = [5.0, 10.0, 15.0, 20.0]
    multi_res = {ang: simulate_system(theta_init_deg=ang, duration=5.0) for ang in angles}

    # -------------------------------------------------------------
    # Create Figure with 3x2 Grid
    # -------------------------------------------------------------
    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
    fig, axs = plt.subplots(3, 2, figsize=(15, 12))
    fig.patch.set_facecolor('#f8f9fa')

    fig.suptitle("HUST Cart-Pendulum Inverted Pendulum | Single Unified Fuzzy Controller\n"
                 "Latest Simulation Run: 20-Degree Tilt Recovery & Trajectory Tracking",
                 fontsize=14, fontweight='bold', y=0.98)

    # (1, 0) Scenario 1: Pendulum Angle (20 deg)
    ax = axs[0, 0]
    ax.set_title("Scenario 1: Pendulum Tilt Recovery (Theta_0 = 20 deg)", fontsize=11, fontweight='bold', color='#1a365d')
    ax.plot(res_20['t'], res_20['theta'], color='#e53e3e', lw=2.2, label=r'Angle $\theta(t)$')
    ax.axhline(0, color='black', ls='-', lw=0.8, alpha=0.5)
    ax.axhline(1.0, color='gray', ls='--', lw=0.8, alpha=0.6, label='Settling Band (+/-1 deg)')
    ax.axhline(-1.0, color='gray', ls='--', lw=0.8, alpha=0.6)
    # Highlight settling time
    idx_settled = np.where(np.abs(res_20['theta']) > 1.0)[0]
    t_settle = res_20['t'][idx_settled[-1]] if len(idx_settled) > 0 else 0.0
    ax.axvline(t_settle, color='#2b6cb0', ls=':', lw=1.5, label=f'Settling time ts = {t_settle:.2f}s')
    ax.set_ylabel("Angle [deg]", fontsize=10, fontweight='bold')
    ax.grid(True, alpha=0.4)
    ax.legend(loc='upper right', framealpha=0.9)
    ax.set_xlim([0, 5.0])

    # (1, 1) Scenario 1: Cart Position & Rail Limits
    ax = axs[0, 1]
    ax.set_title("Scenario 1: Cart Displacement & Physical Rail Limits", fontsize=11, fontweight='bold', color='#1a365d')
    ax.plot(res_20['t'], res_20['x'], color='#2b6cb0', lw=2.2, label=r'Cart Position $x(t)$')
    ax.axhline(0, color='black', ls='-', lw=0.8, alpha=0.5)
    ax.axhline(0.40, color='#c53030', ls='--', lw=1.5, label='Physical Rail Limit (+/-0.40m)')
    ax.axhline(-0.40, color='#c53030', ls='--', lw=1.5)
    max_x = np.max(np.abs(res_20['x']))
    ax.annotate(f'Peak x = {max_x:.3f} m\n(Safe inside rail)',
                xy=(res_20['t'][np.argmax(res_20['x'])], max_x),
                xytext=(res_20['t'][np.argmax(res_20['x'])] + 0.5, max_x + 0.05),
                arrowprops=dict(facecolor='#2b6cb0', shrink=0.08, width=1.5, headwidth=6),
                fontsize=9, fontweight='bold', bbox=dict(boxstyle="round,pad=0.3", fc="#ebf8ff", ec="#3182ce"))
    ax.set_ylabel("Position [m]", fontsize=10, fontweight='bold')
    ax.set_ylim([-0.45, 0.45])
    ax.grid(True, alpha=0.4)
    ax.legend(loc='lower right', framealpha=0.9)
    ax.set_xlim([0, 5.0])

    # (2, 0) Scenario 1: Control Force F(t)
    ax = axs[1, 0]
    ax.set_title("Scenario 1: Control Force Action & Actuator Saturation", fontsize=11, fontweight='bold', color='#1a365d')
    ax.plot(res_20['t'], res_20['f'], color='#805ad5', lw=2.0, label=r'Control Force $F(t)$')
    ax.axhline(10.0, color='#e53e3e', ls=':', lw=1.2, label='Motor Limits (+/-10 N)')
    ax.axhline(-10.0, color='#e53e3e', ls=':', lw=1.2)
    ax.axhline(0, color='black', ls='-', lw=0.8, alpha=0.5)
    ax.set_ylabel("Force [N]", fontsize=10, fontweight='bold')
    ax.set_ylim([-12.0, 12.0])
    ax.grid(True, alpha=0.4)
    ax.legend(loc='upper right', framealpha=0.9)
    ax.set_xlim([0, 5.0])

    # (2, 1) Scenario 1: Phase Portrait (Theta vs Theta_dot)
    ax = axs[1, 1]
    ax.set_title("Scenario 1: Phase Portrait Trajectory (Theta vs Theta_dot)", fontsize=11, fontweight='bold', color='#1a365d')
    ax.plot(res_20['theta'], res_20['theta_dot'], color='#319795', lw=2.0, label='Trajectory')
    ax.plot(res_20['theta'][0], res_20['theta_dot'][0], 'ro', markersize=8, label=f'Start (20 deg, 0)')
    ax.plot(res_20['theta'][-1], res_20['theta_dot'][-1], 'go', markersize=8, label=f'End ({res_20["theta"][-1]:.2f} deg, 0)')
    ax.axhline(0, color='black', ls='-', lw=0.6, alpha=0.4)
    ax.axvline(0, color='black', ls='-', lw=0.6, alpha=0.4)
    ax.set_xlabel(r'Angle $\theta$ [deg]', fontsize=10, fontweight='bold')
    ax.set_ylabel(r'Angular Velocity $\dot{\theta}$ [deg/s]', fontsize=10, fontweight='bold')
    ax.grid(True, alpha=0.4)
    ax.legend(loc='upper right', framealpha=0.9)

    # (3, 0) Multi-Angle Comparison: Recovery from 5, 10, 15, 20 deg
    ax = axs[2, 0]
    ax.set_title("Robustness: Tilt Recovery from Multiple Angles (5, 10, 15, 20 deg)", fontsize=11, fontweight='bold', color='#1a365d')
    colors = ['#38a169', '#3182ce', '#dd6b20', '#e53e3e']
    for ang, col in zip(angles, colors):
        r = multi_res[ang]
        ax.plot(r['t'], r['theta'], color=col, lw=1.8, label=f'θ0 = {ang:.0f} deg (max x = {np.max(np.abs(r["x"])):.2f}m)')
    ax.axhline(0, color='black', ls='-', lw=0.8, alpha=0.5)
    ax.set_ylabel("Angle [deg]", fontsize=10, fontweight='bold')
    ax.set_xlabel("Time [s]", fontsize=10, fontweight='bold')
    ax.grid(True, alpha=0.4)
    ax.legend(loc='upper right', framealpha=0.9)
    ax.set_xlim([0, 4.0])

    # (3, 1) Scenario 2: Setpoint Tracking (x_ref = 0.15m)
    ax = axs[2, 1]
    ax.set_title("Scenario 2: Cart Position Tracking Step Response (x_ref = 0.15m)", fontsize=11, fontweight='bold', color='#1a365d')
    ax.plot(res_step['t'], res_step['x'], color='#2b6cb0', lw=2.2, label=r'Position $x(t)$')
    ax.plot(res_step['t'], res_step['xref'], color='#38a169', ls='--', lw=2.0, label=r'Target $x_{\rm ref}$')
    ax.axhline(0, color='black', ls='-', lw=0.8, alpha=0.5)
    ax.set_ylabel("Position [m]", fontsize=10, fontweight='bold')
    ax.set_xlabel("Time [s]", fontsize=10, fontweight='bold')
    final_x = res_step['x'][-1]
    ax.annotate(f'Final x = {final_x:.4f} m\n(Target: 0.150 m)',
                xy=(res_step['t'][-1], final_x),
                xytext=(res_step['t'][-1] - 1.8, final_x - 0.05),
                arrowprops=dict(facecolor='#38a169', shrink=0.08, width=1.5, headwidth=6),
                fontsize=9, fontweight='bold', bbox=dict(boxstyle="round,pad=0.3", fc="#f0fff4", ec="#38a169"))
    ax.grid(True, alpha=0.4)
    ax.legend(loc='lower right', framealpha=0.9)
    ax.set_xlim([0, 6.0])

    plt.tight_layout(rect=[0, 0.02, 1, 0.95])

    out_file = os.path.join(os.path.dirname(__file__), "latest_run_response_20deg.png")
    plt.savefig(out_file, dpi=300, facecolor=fig.get_facecolor(), edgecolor='none')
    print(f"\n[SUCCESS] Dedicated plot generated and saved to:")
    print(f" -> {out_file}")

if __name__ == "__main__":
    main()
