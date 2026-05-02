import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

def draw_global_map(ax, trajectory, config, current_time=None):
    """
    Render the top-down 2D global environment map including boundaries, 
    static obstacles, dynamic targets, and the AUR trajectory.
    """
    wps = config['waypoints']
    ax.plot(wps[:, 0], wps[:, 1], 'k--', alpha=0.3, label='Global Route')
    ax.plot(trajectory[:, 0], trajectory[:, 1], 'b-', linewidth=2, label='AUR Path')
    
    for obs in config['static_obs']:
        ax.add_patch(plt.Circle((obs[0], obs[1]), obs[2], color='red', alpha=0.15))
    for l_obs in config['line_obs']:
        ax.plot([l_obs[0], l_obs[2]], [l_obs[1], l_obs[3]], 'r-', linewidth=2, alpha=0.3)
        
    for dobs in config['dynamic_obs']:
        if current_time is None:
            t_max = config['sim_time']
            alpha_line = 0.3
            ax.add_patch(plt.Circle((dobs[0], dobs[1]), dobs[4], color='orange', alpha=0.5, zorder=3))
            ax.text(dobs[0], dobs[1]+6, 'Start', color='darkorange', fontsize=8, ha='center', fontweight='bold')
        else:
            t_max = current_time
            alpha_line = 0.8
            cx, cy = dobs[0] + dobs[2]*current_time, dobs[1] + dobs[3]*current_time
            ax.add_patch(plt.Circle((cx, cy), dobs[4], color='orange', alpha=0.9, zorder=3))
            
        t_arr = np.linspace(0, t_max, max(10, int(t_max)))
        traj_x = dobs[0] + dobs[2]*t_arr
        traj_y = dobs[1] + dobs[3]*t_arr
        ax.plot(traj_x, traj_y, color='orange', linestyle='--', alpha=alpha_line, linewidth=1.5)
        
        if len(traj_x) > 1 and t_max > 0:
            dx = traj_x[-1] - traj_x[-2]
            dy = traj_y[-1] - traj_y[-2]
            norm = np.hypot(dx, dy)
            if norm > 0:
                ax.arrow(traj_x[-1], traj_y[-1], (dx/norm)*0.01, (dy/norm)*0.01, 
                         head_width=4.0, head_length=5.0, fc='darkorange', ec='darkorange', zorder=4)
            
    ax.set_aspect('equal')
    ax.set_xlim([-20, 320])
    ax.set_ylim([-60, 160]) 
    ax.grid(True, linestyle=':', alpha=0.5)

def plot_comprehensive_analysis(log, config, critical_events):
    """
    Generate a 6-panel comprehensive analysis plot detailing trajectory, 
    ESO disturbance tracking, torque bounds, and governor behavior.
    """
    dt = config['dt']
    steps = len(log['eta'])
    time_vector = np.linspace(0, steps * dt, steps)

    fig = plt.figure(figsize=(24, 16)) 
    gs = GridSpec(2, 6, figure=fig)

    # 1. Global View
    ax1 = fig.add_subplot(gs[0, 0:2])
    draw_global_map(ax1, log['eta'], config)
    ax1.set_title('Final State (Global View)')

    # 2. Critical Snapshots
    events = [e for e in critical_events.values() if e[2] is not None]
    sorted_ev = sorted(events, key=lambda x: x[0])[:2]
    for i, ev in enumerate(sorted_ev):
        ax_snap = fig.add_subplot(gs[0, 2+i*2 : 4+i*2])
        dist, idx, _, _ = ev
        draw_global_map(ax_snap, log['eta'][:idx], config, current_time=idx*dt)
        ax_snap.set_title(f'Snapshot at t={idx*dt:.1f}s\nMin Dist: {dist:.2f}m')

    # 3. ESO & Tracking Synergy (Dual Axis decoupled legends)
    ax2 = fig.add_subplot(gs[1, 0:2])
    ax2.plot(time_vector, log['d_hat'], 'm', label='Total Disturbance (d_hat)', linewidth=1.5)
    ax2.set_ylabel('Disturbance Estimate', color='m', fontweight='bold')
    
    ax2_v = ax2.twinx()
    ax2_v.plot(time_vector, log['r_o'], 'c--', label='Cmd r_o', alpha=0.8)
    ax2_v.plot(time_vector, log['r_actual'], 'g-', label='Actual r', alpha=0.5)
    ax2_v.set_ylabel('Angular Velocity [rad/s]', color='g', fontweight='bold')
    ax2.set_title('ESO & Velocity Tracking Synergy')
    ax2.grid(True)
    
    ax2.legend(loc='lower left')
    ax2_v.legend(loc='upper right')

    # 4. Smooth Torque Output
    ax3 = fig.add_subplot(gs[1, 2:4])
    ax3.plot(time_vector, log['tau'], 'b', linewidth=1.5)
    ax3.set_ylim([-config['tau_max']*1.2, config['tau_max']*1.2])
    ax3.set_ylabel('Torque [Nm]') 
    ax3.set_title('Shaped Yaw Torque (Jerk-Limited)')
    ax3.grid(True)

    # 5. Reference Governor Constraints
    ax4 = fig.add_subplot(gs[1, 4:6])
    ax4.plot(time_vector, log['r_o'], 'm', label='RG Output')
    ax4.set_ylabel('Commanded Yaw Rate [rad/s]') 
    ax4.set_title('Reference Governor Shaping')
    ax4.grid(True)

    plt.tight_layout()
    plt.show()