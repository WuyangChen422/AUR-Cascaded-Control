import numpy as np

class AURDynamics:
    """
    Standard 3-DOF Autonomous Underwater Robot (AUR) dynamics.
    Implements the Fossen marine craft equations of motion including 
    Rigid-Body Mass, Added Mass, Coriolis/Centripetal forces, and Non-linear Damping.
    """
    def __init__(self):
        """
        Initialize the physical parameters and the Inertia Matrix (M).
        """
        self.m = 18.0
        self.I_z = 1.0
        # Added mass coefficients
        self.X_udot, self.Y_vdot, self.N_rdot = -1.0, -10.0, -1.0
        # Linear skin friction
        self.X_u, self.Y_v, self.N_r = -2.0, -5.0, -2.0
        # Quadratic viscous drag
        self.X_uu, self.Y_vv, self.N_rr = -1.5, -10.0, -5.0

        # Inertia Matrix (M) including added mass terms
        self.M = np.array([
            [self.m - self.X_udot, 0, 0],
            [0, self.m - self.Y_vdot, 0],
            [0, 0, self.I_z - self.N_rdot]
        ])
        self.M_inv = np.linalg.inv(self.M)

    def update_state(self, eta, nu, tau, dt, d_real):
        """
        Perform numerical integration for kinetics (Body frame) and kinematics (Earth frame).
        
        Args:
            eta: Earth-fixed pose [x, y, psi].
            nu: Body-fixed velocities [u, v, r].
            tau: Control inputs [tau_surge, tau_sway, tau_yaw].
            dt: Time step.
            d_real: External environmental disturbance (e.g., ocean currents).
            
        Returns:
            eta_next, nu_next: Updated states.
        """
        u, v, r = nu[0], nu[1], nu[2]
        
        # Coriolis and Centripetal Matrix (C)
        C = np.array([
            [0, 0, -(self.m - self.Y_vdot) * v],
            [0, 0,  (self.m - self.X_udot) * u],
            [(self.m - self.Y_vdot) * v, -(self.m - self.X_udot) * u, 0]
        ])
        
        # Damping Matrix (D) combining linear and quadratic terms
        D = np.array([
            [-self.X_u - self.X_uu * abs(u), 0, 0],
            [0, -self.Y_v - self.Y_vv * abs(v), 0],
            [0, 0, -self.N_r - self.N_rr * abs(r)]
        ])

        # Kinetics: M * nu_dot + C * nu + D * nu = tau + tau_dist
        dist_vec = np.array([0, 0, d_real * self.M[2,2]])
        nu_dot = self.M_inv.dot(tau - C.dot(nu) - D.dot(nu) + dist_vec)
        nu_next = nu + nu_dot * dt

        # Kinematics: eta_dot = R(psi) * nu
        psi = eta[2]
        R = np.array([
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi),  np.cos(psi), 0],
            [0, 0, 1]
        ])
        eta_next = eta + R.dot(nu) * dt
        # Normalize heading to [-pi, pi]
        eta_next[2] = (eta_next[2] + np.pi) % (2 * np.pi) - np.pi
        
        return eta_next, nu_next