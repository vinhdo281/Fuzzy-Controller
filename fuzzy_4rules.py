"""
Python Implementation of the 4-rule Mamdani Fuzzy Controller
Matching exact MATLAB FIS: cartpole_fuzzy_4rules.m
Inputs: Theta, Theta_dot -> Output: Force
"""

import numpy as np

def zmf(x, a, b):
    """Z-shaped membership function"""
    x = np.asarray(x, dtype=float)
    y = np.zeros_like(x)
    m = (a + b) / 2.0
    
    mask1 = x <= a
    y[mask1] = 1.0
    
    mask2 = (x > a) & (x <= m)
    y[mask2] = 1.0 - 2.0 * ((x[mask2] - a) / (b - a)) ** 2
    
    mask3 = (x > m) & (x < b)
    y[mask3] = 2.0 * ((x[mask3] - b) / (b - a)) ** 2
    
    mask4 = x >= b
    y[mask4] = 0.0
    
    return float(y) if x.ndim == 0 else y

def smf(x, a, b):
    """S-shaped membership function"""
    x = np.asarray(x, dtype=float)
    y = np.zeros_like(x)
    m = (a + b) / 2.0
    
    mask1 = x <= a
    y[mask1] = 0.0
    
    mask2 = (x > a) & (x <= m)
    y[mask2] = 2.0 * ((x[mask2] - a) / (b - a)) ** 2
    
    mask3 = (x > m) & (x < b)
    y[mask3] = 1.0 - 2.0 * ((x[mask3] - b) / (b - a)) ** 2
    
    mask4 = x >= b
    y[mask4] = 1.0
    
    return float(y) if x.ndim == 0 else y

def gbellmf(x, a, b, c):
    """Generalized bell-shaped membership function"""
    return 1.0 / (1.0 + np.abs((x - c) / a) ** (2.0 * b))

class Fuzzy4RulesMamdani:
    def __init__(self, n_points=500):
        # Force domain [-10, 10]
        self.force_range = np.linspace(-10.0, 10.0, n_points)
        
        # Precompute output membership functions
        self.mf_NM = gbellmf(self.force_range, 2.5, 2.0, -4.0)
        self.mf_PM = gbellmf(self.force_range, 2.5, 2.0, 4.0)
        self.mf_NL = gbellmf(self.force_range, 3.0, 2.0, -10.0)
        self.mf_PL = gbellmf(self.force_range, 3.0, 2.0, 10.0)

    def compute(self, theta, theta_dot):
        # 1. Fuzzification
        # Theta [-0.2, 0.2]
        mu_th_neg = zmf(theta, -0.2, 0.2)
        mu_th_pos = smf(theta, -0.2, 0.2)
        
        # Theta_dot [-3.0, 3.0]
        mu_thd_neg = zmf(theta_dot, -3.0, 3.0)
        mu_thd_pos = smf(theta_dot, -3.0, 3.0)

        # 2. Rule evaluation (Mamdani Min implication)
        # Rule 1: If Theta is Negative then Force is NM
        clip1 = np.minimum(mu_th_neg, self.mf_NM)
        # Rule 2: If Theta is Positive then Force is PM
        clip2 = np.minimum(mu_th_pos, self.mf_PM)
        # Rule 3: If Theta_dot is Negative then Force is NL
        clip3 = np.minimum(mu_thd_neg, self.mf_NL)
        # Rule 4: If Theta_dot is Positive then Force is PL
        clip4 = np.minimum(mu_thd_pos, self.mf_PL)

        # 3. Aggregation (Max)
        aggregated = np.maximum.reduce([clip1, clip2, clip3, clip4])

        # 4. Defuzzification (Centroid)
        total_area = np.sum(aggregated)
        if total_area > 1e-9:
            force = np.sum(aggregated * self.force_range) / total_area
        else:
            force = 0.0

        return force

if __name__ == "__main__":
    fis = Fuzzy4RulesMamdani()
    print("Testing 4-Rule Mamdani Controller:")
    print("  Theta = +0.1 rad, Theta_dot = 0.0     -> Force =", fis.compute(0.1, 0.0))
    print("  Theta = -0.1 rad, Theta_dot = 0.0     -> Force =", fis.compute(-0.1, 0.0))
    print("  Theta = 0.0 rad,  Theta_dot = +2.0    -> Force =", fis.compute(0.0, 2.0))
    print("  Theta = 0.0 rad,  Theta_dot = -2.0    -> Force =", fis.compute(0.0, -2.0))
