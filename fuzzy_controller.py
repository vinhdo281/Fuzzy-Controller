import numpy as np

class FuzzyPD:
    """
    Standard 5x5 Fuzzy PD Controller using Takagi-Sugeno or Mamdani
    Inputs: e (error), de (derivative of error)
    Output: u (control action)
    Normalized universe of discourse: [-1, 1] for inputs, [-1, 1] for output.
    """
    def __init__(self, Ke=1.0, Kd=1.0, Ku=1.0, out_limit=None):
        self.Ke = Ke
        self.Kd = Kd
        self.Ku = Ku
        self.out_limit = out_limit

        # Centers of 5 triangular membership functions on [-1, 1]
        # NB: -1.0, NS: -0.5, ZE: 0.0, PS: 0.5, PB: 1.0
        self.centers = np.array([-1.0, -0.5, 0.0, 0.5, 1.0])

        # Rule base 5x5: Output singleton values [-1, -0.5, 0, 0.5, 1]
        # Rows: e (NB, NS, ZE, PS, PB), Cols: de (NB, NS, ZE, PS, PB)
        self.rule_table = np.array([
            [-1.0, -1.0, -1.0, -0.5,  0.0],  # e = NB
            [-1.0, -1.0, -0.5,  0.0,  0.5],  # e = NS
            [-1.0, -0.5,  0.0,  0.5,  1.0],  # e = ZE
            [-0.5,  0.0,  0.5,  1.0,  1.0],  # e = PS
            [ 0.0,  0.5,  1.0,  1.0,  1.0]   # e = PB
        ])

    def _triangle_mf(self, x, c, w=0.5):
        """Triangular membership function centered at c with half-width w"""
        return np.maximum(0.0, 1.0 - np.abs(x - c) / w)

    def compute(self, e, de):
        # 1. Scale inputs to normalized domain [-1, 1]
        e_norm = np.clip(e * self.Ke, -1.0, 1.0)
        de_norm = np.clip(de * self.Kd, -1.0, 1.0)

        # 2. Fuzzification (degree of membership for each MF)
        # For boundaries NB and PB, use shoulder
        mu_e = np.zeros(5)
        mu_de = np.zeros(5)

        for i in range(5):
            mu_e[i] = self._triangle_mf(e_norm, self.centers[i])
            mu_de[i] = self._triangle_mf(de_norm, self.centers[i])

        if e_norm <= -1.0:
            mu_e[0] = 1.0
        elif e_norm >= 1.0:
            mu_e[4] = 1.0

        if de_norm <= -1.0:
            mu_de[0] = 1.0
        elif de_norm >= 1.0:
            mu_de[4] = 1.0

        # 3. Rule firing strength (product t-norm)
        # weight_matrix[i, j] = mu_e[i] * mu_de[j]
        weights = np.outer(mu_e, mu_de)
        total_weight = np.sum(weights)

        # 4. Defuzzification (Weighted average / Sugeno 0-th order)
        if total_weight > 1e-9:
            u_norm = np.sum(weights * self.rule_table) / total_weight
        else:
            u_norm = 0.0

        # 5. Output scaling
        u = u_norm * self.Ku

        if self.out_limit is not None:
            u = np.clip(u, -self.out_limit, self.out_limit)

        return u


class CascadeFuzzyController:
    """
    Cascade Fuzzy Controller for Cart-Pendulum
    Outer Loop (Position):
        e_x = x_ref - x, de_x = -x_dot
        Output: theta_ref (desired tilt angle)
    Inner Loop (Angle):
        e_theta = theta_ref - theta, de_theta = -theta_dot
        Output: Control Force F
    """
    def __init__(self,
                 pos_gains={'Ke': 2.5, 'Kd': 1.2, 'Ku': 0.15, 'limit': 0.20},
                 ang_gains={'Ke': 8.0, 'Kd': 2.0, 'Ku': 4.0, 'limit': 10.0}):
        self.outer_pos = FuzzyPD(
            Ke=pos_gains['Ke'],
            Kd=pos_gains['Kd'],
            Ku=pos_gains['Ku'],
            out_limit=pos_gains['limit']
        )
        self.inner_ang = FuzzyPD(
            Ke=ang_gains['Ke'],
            Kd=ang_gains['Kd'],
            Ku=ang_gains['Ku'],
            out_limit=ang_gains['limit']
        )

    def compute(self, x, x_dot, theta, theta_dot, x_ref=0.0):
        # Outer loop: Position error -> Target tilt angle theta_ref
        # Physics: to move cart to +X (when x < x_ref => e_x > 0),
        # cart must tilt pole forward (+theta), so theta_ref should have negative/positive sign
        # Let's test the sign:
        # If e_x = x_ref - x > 0: we want cart to accelerate in +X.
        # Accelerating in +X requires pole to tilt to +X (lean forward).
        # Inner loop: e_theta = theta_ref - theta.
        # If theta_ref > 0, inner loop sees e_theta > 0 => pushes cart forward (F > 0).
        e_x = x_ref - x
        de_x = -x_dot
        theta_ref = self.outer_pos.compute(e_x, de_x)

        # Inner loop: Angle error -> Actuator Force F
        e_theta = theta_ref - theta
        de_theta = -theta_dot
        F = self.inner_ang.compute(e_theta, de_theta)

        return F, theta_ref
