import numpy as np

class FirstOrderCommandFilter:
    """
    First-order low-pass filter to smooth out step commands from the planner 
    and satisfy initial acceleration constraints.
    """
    def __init__(self, max_rate, max_accel, dt, tau):
        self.max_rate = max_rate
        self.max_accel = max_accel
        self.dt = dt
        self.tau = tau
        self.r_f = 0.0

    def update(self, r_cmd_raw):
        r_dot_desired = (r_cmd_raw - self.r_f) / self.tau
        r_dot_limited = np.clip(r_dot_desired, -self.max_accel, self.max_accel)
        self.r_f += r_dot_limited * self.dt
        self.r_f = np.clip(self.r_f, -self.max_rate, self.max_rate)
        return self.r_f

def fal(e, alpha, delta):
    """
    Optimal non-linear function for the ADRC framework to ensure smooth 
    convergence without chattering near the origin.
    """
    if abs(e) <= delta:
        return e / (delta ** (1.0 - alpha))
    return (abs(e) ** alpha) * np.sign(e)

class NonlinearESO:
    """
    Non-linear Extended State Observer (NLESO).
    Lumps unmodeled dynamics (e.g., complex damping) and external environmental 
    disturbances into a single extended state (z2) for real-time feedforward compensation.
    """
    def __init__(self, b0, omega_o, dt, alpha, delta):
        self.b0 = b0
        self.dt = dt
        self.beta1 = 2.0 * omega_o
        self.beta2 = omega_o ** 2
        self.alpha = alpha
        self.delta = delta
        self.z = np.zeros(2)

    def update(self, y, u):
        e = self.z[0] - y
        z1_dot = self.z[1] - self.beta1 * e + self.b0 * u
        z2_dot = -self.beta2 * fal(e, self.alpha, self.delta)
        self.z += np.array([z1_dot, z2_dot]) * self.dt
        return self.z[0], self.z[1]

class JerkLimitedGovernor:
    """
    Reference Governor (RG) with constraints on Torque, Acceleration, and Jerk.
    Calculates the maximum feasible control commands based on the remaining 
    actuator capacity after disturbance rejection, preventing torque saturation.
    """
    def __init__(self, tau_max, max_r_dot, max_r_ddot, dt, safety_margin=0.9):
        self.tau_max = tau_max
        self.safe_tau_max = tau_max * safety_margin
        self.max_r_dot = max_r_dot
        self.max_r_ddot = max_r_ddot # Jerk Limit
        self.dt = dt
        self.r_o_prev = 0.0
        self.r_dot_prev = 0.0

    def govern(self, r_cmd, r_measure, d_hat, b0, kp_r):
        # 1. Calc current feasible acceleration range based on remaining torque
        r_dot_min_tau = (d_hat - self.safe_tau_max * b0) / kp_r
        r_dot_max_tau = (d_hat + self.safe_tau_max * b0) / kp_r
        
        # 2. Apply Jerk limit to smooth out sudden acceleration changes
        r_dot_lower_jerk = self.r_dot_prev - self.max_r_ddot * self.dt
        r_dot_upper_jerk = self.r_dot_prev + self.max_r_ddot * self.dt
        
        # 3. Intersection of constraints
        r_dot_min = max(r_dot_min_tau, r_dot_lower_jerk, -self.max_r_dot)
        r_dot_max = min(r_dot_max_tau, r_dot_upper_jerk, self.max_r_dot)
        
        # 4. Command shaping
        r_o = np.clip(r_cmd, self.r_o_prev + r_dot_min * self.dt, self.r_o_prev + r_dot_max * self.dt)
        
        self.r_dot_prev = (r_o - self.r_o_prev) / self.dt
        self.r_o_prev = r_o
        return r_o