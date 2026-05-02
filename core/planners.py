import numpy as np

def soft_switch(dist, threshold=15.0, width=5.0, val_safe=25.0, val_danger=5.0):
    """
    Sigmoid-based soft switch to dynamically adjust steering damping 
    as the AUR approaches obstacles, suppressing control spikes.
    """
    z = (dist - threshold) / (width / 4)
    alpha = 1.0 / (1.0 + np.exp(-z))
    return val_danger + (val_safe - val_danger) * alpha

class AdvancedRHP:
    """
    Receding Horizon Planner (RHP) with Flexible Potential Fields and CTE Constraints.
    Predicts trajectories over a finite horizon and selects the optimal yaw rate 
    command that minimizes distance, cross-track error, and obstacle penalties.
    """
    def __init__(self, horizon_time, dt, max_yaw_rate, num_samples):
        self.horizon_time = horizon_time
        self.dt = dt
        self.horizon_steps = int(horizon_time / dt)
        self.r_candidates = np.linspace(-max_yaw_rate, max_yaw_rate, num_samples)
        self.time_array = np.arange(self.horizon_steps) * self.dt
        self.prev_optimal_r = 0.0

    def predict_trajectory(self, eta_init, nu_surge, r_cmd):
        """Predict the kinematic trajectory under a constant yaw rate."""
        eta_pred = np.zeros((self.horizon_steps, 3))
        eta_curr = np.copy(eta_init)
        for i in range(self.horizon_steps):
            psi = eta_curr[2]
            eta_curr[0] += nu_surge * np.cos(psi) * self.dt
            eta_curr[1] += nu_surge * np.sin(psi) * self.dt
            eta_curr[2] += r_cmd * self.dt
            eta_curr[2] = (eta_curr[2] + np.pi) % (2 * np.pi) - np.pi
            eta_pred[i] = eta_curr
        return eta_pred

    def calculate_barrier_penalty(self, min_dist, obs_radius, AUR_RADIUS=2.0):
        """
        Mathematical barrier field: Retains non-zero gradients using polynomial functions 
        instead of returning 'inf', enabling the optimizer to escape deep penetrations.
        """
        if min_dist <= obs_radius + AUR_RADIUS:
            # Depth of penetration
            penetration = (obs_radius + AUR_RADIUS) - min_dist
            # Quadratic resistance + Cubic repulsion for steep gradients
            return 500.0 + 3000.0 * (penetration ** 2) + 10000.0 * (penetration ** 3)
        else:
            # Exponential decay outside the obstacle bounds
            return 500.0 * np.exp(-2.0 * (min_dist - (obs_radius + AUR_RADIUS)))

    def evaluate_cost(self, traj, wp_prev, wp_next, static_obs, line_obs, dynamic_obs, r_cmd, current_time, s_max):
        """
        Evaluate the total cost of a candidate trajectory based on multi-objective constraints.
        """
        final_pos = traj[-1, 0:2]
        traj_pos = traj[:, 0:2]

        min_dist_all = 100.0
        for obs in static_obs:
            min_dist_all = min(min_dist_all, np.min(np.linalg.norm(traj_pos - np.array([obs[0], obs[1]]), axis=1)) - obs[2])
        for dobs in dynamic_obs:
            d = np.linalg.norm(traj_pos[0] - np.array([dobs[0]+dobs[2]*current_time, dobs[1]+dobs[3]*current_time])) - dobs[4]
            min_dist_all = min(min_dist_all, d)

        penalty_factor = soft_switch(min_dist_all, threshold=15.0, width=5.0)
        chatter_penalty = penalty_factor * abs(r_cmd - self.prev_optimal_r)

        # 1. Target Attraction Cost
        dist_cost = 2.0 * np.linalg.norm(wp_next - final_pos)

        # 2. Path Adherence Cost (Cross-Track Error & Heading)
        seg_vec = wp_next - wp_prev
        seg_len = np.linalg.norm(seg_vec)
        if seg_len > 0:
            seg_dir = seg_vec / seg_len
            path_yaw = np.arctan2(seg_dir[1], seg_dir[0])

            # Calculate Cross-Track Error (CTE) via vector projection
            AP = final_pos - wp_prev
            proj = np.dot(AP, seg_dir)
            perp_vec = AP - proj * seg_dir
            cte = np.linalg.norm(perp_vec)
            cte_cost = 1.5 * cte      

            # Calculate Heading Error relative to the path
            final_yaw = traj[-1, 2]
            yaw_err = abs((final_yaw - path_yaw + np.pi) % (2 * np.pi) - np.pi)
            yaw_cost = 1.0 * yaw_err  
        else:
            cte_cost = 0.0
            yaw_cost = 0.0

        obs_penalty = 0.0

        # Fail-fast mechanism: immediate discard if 'inf' is returned by collisions
        for obs in static_obs:
            dists = np.linalg.norm(traj_pos - np.array([obs[0], obs[1]]), axis=1)
            penalty = self.calculate_barrier_penalty(np.min(dists), obs[2])
            if penalty == float('inf'): return float('inf')
            obs_penalty += penalty

        for dobs in dynamic_obs:
            obs_x = dobs[0] + dobs[2] * (current_time + self.time_array)
            obs_y = dobs[1] + dobs[3] * (current_time + self.time_array)
            dists = np.linalg.norm(traj_pos - np.column_stack((obs_x, obs_y)), axis=1)
            penalty = self.calculate_barrier_penalty(np.min(dists), dobs[4])
            if penalty == float('inf'): return float('inf')
            obs_penalty += penalty * 1.5

        for l_obs in line_obs:
            A, B = np.array([l_obs[0], l_obs[1]]), np.array([l_obs[2], l_obs[3]])
            AB = B - A
            len_sq = np.dot(AB, AB)
            if len_sq == 0:
                dists = np.linalg.norm(traj_pos - A, axis=1)
            else:
                t = np.clip(np.dot(traj_pos - A, AB) / len_sq, 0.0, 1.0)
                proj = A + np.outer(t, AB)
                dists = np.linalg.norm(traj_pos - proj, axis=1)
            penalty = self.calculate_barrier_penalty(np.min(dists), l_obs[4])
            if penalty == float('inf'): return float('inf')
            obs_penalty += penalty

        effort_penalty = 1.0 * abs(r_cmd)

        return dist_cost + cte_cost + yaw_cost + obs_penalty + chatter_penalty + effort_penalty

    def get_optimal_command(self, eta, nu_surge, wp_prev, wp_next, static_obs, line_obs, dynamic_obs, current_time, s_max):
        best_cost = float('inf')
        optimal_r = 0.0
        for r_cmd in self.r_candidates:
            traj = self.predict_trajectory(eta, nu_surge, r_cmd)
            cost = self.evaluate_cost(traj, wp_prev, wp_next, static_obs, line_obs, dynamic_obs, r_cmd, current_time, s_max)
            if cost < best_cost:
                best_cost = cost
                optimal_r = r_cmd
        
        if best_cost >= 1e9:
            optimal_r = -self.r_candidates[-1]
        else:
            self.prev_optimal_r = optimal_r
            
        return optimal_r