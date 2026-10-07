"""
Single Unified Fuzzy Logic Controller (4 Inputs -> 1 Output)
Non-cascade architecture matching cartpole_single_fuzzy.m exactly.
Inputs: Theta, Theta_dot, x_err, x_dot
Output: Force F
"""

import numpy as np

def zmf(x, a, b):
    x = float(x)
    if x <= a: return 1.0
    if x >= b: return 0.0
    m = (a + b) / 2.0
    if x <= m: return 1.0 - 2.0 * ((x - a) / (b - a)) ** 2
    return 2.0 * ((x - b) / (b - a)) ** 2

def smf(x, a, b):
    x = float(x)
    if x <= a: return 0.0
    if x >= b: return 1.0
    m = (a + b) / 2.0
    if x <= m: return 2.0 * ((x - a) / (b - a)) ** 2
    return 1.0 - 2.0 * ((x - b) / (b - a)) ** 2

def gbellmf(x, a, b, c):
    return 1.0 / (1.0 + np.abs((x - c) / a) ** (2.0 * b))

class SingleUnifiedFuzzyController:
    def __init__(self, n_points=301):
        self.F_grid = np.linspace(-10.0, 10.0, n_points)
        # Precompute output membership functions
        self.mf_NL = gbellmf(self.F_grid, 3.0, 2.0, -8.0)
        self.mf_NM = gbellmf(self.F_grid, 2.0, 2.0, -3.5)
        self.mf_PM = gbellmf(self.F_grid, 2.0, 2.0,  3.5)
        self.mf_PL = gbellmf(self.F_grid, 3.0, 2.0,  8.0)

    def compute(self, theta, theta_dot, x, x_dot, x_ref=0.0):
        e_x = x - x_ref

        # 1. Fuzzification
        mu_th_neg = zmf(theta, -0.15, 0.15)
        mu_th_pos = smf(theta, -0.15, 0.15)

        mu_thd_neg = zmf(theta_dot, -2.5, 2.5)
        mu_thd_pos = smf(theta_dot, -2.5, 2.5)

        mu_x_neg = zmf(e_x, -0.25, 0.25)
        mu_x_pos = smf(e_x, -0.25, 0.25)

        mu_xd_neg = zmf(x_dot, -0.5, 0.5)
        mu_xd_pos = smf(x_dot, -0.5, 0.5)

        # 2. Rule evaluation (Sum-Aggregation)
        # Angle rules (Weight = 1.0)
        clip1 = 1.0 * np.minimum(mu_th_neg, self.mf_NM)
        clip2 = 1.0 * np.minimum(mu_th_pos, self.mf_PM)
        clip3 = 1.0 * np.minimum(mu_thd_neg, self.mf_NL)
        clip4 = 1.0 * np.minimum(mu_thd_pos, self.mf_PL)

        # Position rules (Weight = 0.25)
        clip5 = 0.25 * np.minimum(mu_x_neg, self.mf_NM)
        clip6 = 0.25 * np.minimum(mu_x_pos, self.mf_PM)
        clip7 = 0.25 * np.minimum(mu_xd_neg, self.mf_NL)
        clip8 = 0.25 * np.minimum(mu_xd_pos, self.mf_PL)

        # 3. Aggregation (Sum)
        agg = clip1 + clip2 + clip3 + clip4 + clip5 + clip6 + clip7 + clip8

        # 4. Defuzzification (Centroid)
        area = np.sum(agg)
        if area > 1e-9:
            force = np.sum(agg * self.F_grid) / area
        else:
            force = 0.0

        return force

if __name__ == "__main__":
    ctrl = SingleUnifiedFuzzyController()
    print("Zero state Force:", ctrl.compute(0, 0, 0, 0))
    print("Theta = +0.1 rad:", ctrl.compute(0.1, 0, 0, 0))
    print("Theta = -0.1 rad:", ctrl.compute(-0.1, 0, 0, 0))
    print("x = +0.2 m (should push positive to tilt pole left):", ctrl.compute(0, 0, 0.2, 0))
