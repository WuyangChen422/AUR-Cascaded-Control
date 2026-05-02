import numpy as np

# Import decoupled modules
from config.env_config import AUR_CONFIG
from core.dynamics import AURDynamics
from core.controllers import FirstOrderCommandFilter, NonlinearESO, JerkLimitedGovernor
from core.planners import AdvancedRHP
from utils.visualization import plot_comprehensive_analysis

def run_comprehensive_simulation(config=AUR_CONFIG):
    """
    Main simulation loop orchestrating the cascaded framework:
    Global Waypoints -> RHP Planner -> Governor -> ESO -> Fossen Dynamics.
    """
    dt = config['dt']
    steps = int(config['sim_time'] / dt)

    # Initial states
    eta = np.array([config['waypoints'][0][0], config['waypoints'][0][1], 0.0])
    nu = np.array([1.0, 0.0, 0.0]) 

    wps = config['waypoints']
    current_wp_idx = 1
    b0, kp_r = config['b0'], config['kp_r']

    # Initialize sub-modules
    planner = AdvancedRHP(config['rhp_horizon_time'], dt, config['rhp_max_yaw_rate'], config['rhp_num_samples'])
    eso = NonlinearESO(b0, config['omega_o'], dt, config['alpha'], config['eso_delta'])
    rg = JerkLimitedGovernor(config['tau_max'], config['max_r_dot'], config.get('max_r_ddot', 1.0), dt)
    cmd_filter = FirstOrderCommandFilter(config['max_yaw_rate'], config['max_r_dot'], dt, tau=config['filter_tau'])
    dynamics = AURDynamics()

    # Logging structures
    log = {
        'eta': np.zeros((steps, 3)), 'tau': np.zeros(steps),
        'd_real': np.zeros(steps), 'd_hat': np.zeros(steps),
        'r_cmd_raw': np.zeros(steps), 'r_o': np.zeros(steps),
        'r_actual': np.zeros(steps), 
        'dist_to_obs': [] 
    }

    critical_events = {idx: (float('inf'), 0, None, None) for idx in range(len(config['dynamic_obs']))}
    s_max = 0.0
    r_raw_history = [] 

    # ---------------- MAIN LOOP ----------------
    for i in range(steps):
        time_sec = i * dt
        log['eta'][i] = eta
        wp_k, wp_k_prev = wps[current_wp_idx], wps[current_wp_idx - 1]

        # 1. Arrival Detection
        if current_wp_idx == len(wps) - 1:
            dist_to_final = np.linalg.norm(eta[0:2] - wp_k[0:2])
            if dist_to_final <= config['acceptance_radius']:
                print(f"Goal reached at {time_sec:.2f}s. Ending simulation.")
                for key in log:
                    if isinstance(log[key], np.ndarray): log[key] = log[key][:i]
                break

        # 2. Near-Miss Tracking for critical events
        for idx, dobs in enumerate(config['dynamic_obs']):
            obs_pos = np.array([dobs[0] + dobs[2]*time_sec, dobs[1] + dobs[3]*time_sec])
            d = np.linalg.norm(eta[0:2] - obs_pos)
            if d < critical_events[idx][0]:
                critical_events[idx] = (d, i, np.copy(eta), obs_pos)

        # 3. Waypoint Switching Logic (Topological Tracker)
        seg_vec = wp_k - wp_k_prev
        pi_p = np.arctan2(seg_vec[1], seg_vec[0])
        s = (eta[0]-wp_k_prev[0])*np.cos(pi_p) + (eta[1]-wp_k_prev[1])*np.sin(pi_p)
        dist_to_wp = np.linalg.norm(eta[0:2] - wp_k[0:2])
        
        if (s >= np.linalg.norm(seg_vec) or dist_to_wp < config['acceptance_radius']) and current_wp_idx < len(wps) - 1:
            current_wp_idx += 1
            wp_k, wp_k_prev = wps[current_wp_idx], wps[current_wp_idx-1]
            s_max = 0.0
        else:
            s_max = max(s_max, s)

        # 4. RHP Planning & B-Spline Smoothing
        r_cmd_raw = planner.get_optimal_command(eta, nu[0], wp_k_prev, wp_k,
                                               config['static_obs'], config['line_obs'], config['dynamic_obs'],
                                               time_sec, s_max)
        
        r_raw_history.append(r_cmd_raw)
        if len(r_raw_history) > 10: r_raw_history.pop(0)
        
        r_cmd_smoothed = np.mean(r_raw_history)
        r_cmd_f = cmd_filter.update(r_cmd_smoothed)
        log['r_cmd_raw'][i] = r_cmd_raw

        # 5. Active Disturbance Rejection & Reference Governance
        d_hat = eso.z[1]
        r_o = rg.govern(r_cmd_f, nu[2], d_hat, b0, kp_r)
        log['r_o'][i] = r_o

        # Feedforward Control Law Computation
        tau_yaw_req = kp_r * (r_o - nu[2])
        tau_yaw = (tau_yaw_req - d_hat) / b0
        tau_yaw = np.clip(tau_yaw, -config['tau_max'], config['tau_max'])

        # Simple Surge Control (Constant Speed Assumption)
        tau_surge = 10.0 * (1.0 - nu[0]) + 3.5 
        tau_vec = np.array([tau_surge, 0.0, tau_yaw])
        
        log['tau'][i] = tau_yaw
        log['r_actual'][i] = nu[2] 

        # 6. Environmental Disturbance Generation & State Update
        d_real = 0.2 * np.sin(0.15 * time_sec) + 0.1 * np.cos(0.05 * time_sec)
        log['d_real'][i] = d_real
        log['d_hat'][i] = d_hat

        eso.update(y=nu[2], u=tau_yaw)
        eta, nu = dynamics.update_state(eta, nu, tau_vec, dt, d_real)

    return log, config, critical_events

if __name__ == "__main__":
    simulation_log, sim_config, critical_events = run_comprehensive_simulation()
    plot_comprehensive_analysis(simulation_log, sim_config, critical_events)