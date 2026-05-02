import numpy as np

# =====================================================================
# UNIFIED ENVIRONMENT CONFIGURATION
# Defines simulation limits, controller parameters, and obstacle maps.
# =====================================================================

AUR_CONFIG = {
    'dt': 0.1,
    'sim_time': 600.0,
    'm_r': 10.0, 'd_r': 2.0,
    'delta': 5.0, 'kp_psi': 1.0, 'max_yaw_rate': 0.5, 'acceptance_radius': 4.0,
    
    # Kinetic Controller & Reference Governor Parameters
    'kp_r': 6.0, 'tau_max': 12.0, 'max_r_dot': 0.6, 
    'max_r_ddot': 1.0, 
    
    'filter_tau': 0.4,
    'omega_o': 2.0, 'alpha': 0.75, 'eso_delta': 0.05,
    'rhp_horizon_time': 8.0, 'rhp_max_yaw_rate': 0.35, 'rhp_num_samples': 41,

    # ================= COMPLEX MARINE ENVIRONMENT =================
    # Scenario: The Slalom Intercept (连续发卡弯与斜向截击)

    # Waypoints representing the topological global guidance path
    'waypoints': np.array([
        [0, 0], [100, 100], [200, 0], [300, 100]
    ]),

    # Static mines forcing tight cornering around the apices
    'static_obs': [
        [50, 65, 8.0],
        [100, 80, 12.0],  
        [150, 35, 8.0],
        [200, 20, 12.0]   
    ],

    # Absolute line boundaries (walls) defining the navigable corridor
    'line_obs': [
        [0, 120, 300, 120, 4.0],
        [0, -20, 300, -20, 4.0]
    ],

    # Dynamic moving vessels [x_init, y_init, vx, vy, radius]
    'dynamic_obs': [
        [0, 100, 0.7, -0.7, 6.0],     # Diagonal interception at the first leg
        [150, -55, 0.0, 0.5, 6.0],    # Vertical cut-off during the downhill phase
        [250, 150, 0.0, -0.3, 6.0]    # Lingering ghost ship near the final goal
    ]
}

# Nominal control gain for ESO disturbance lumping
AUR_CONFIG['b0'] = 0.5