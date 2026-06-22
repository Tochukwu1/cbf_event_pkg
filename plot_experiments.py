import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

# --- Publication Aesthetics (ICML / IEEE Style) ---
plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'Times', 'DejaVu Serif'],
    'mathtext.fontset': 'stix', 
    'axes.labelsize': 18,
    'font.size': 16,
    'legend.fontsize': 14,
    'xtick.labelsize': 14,
    'ytick.labelsize': 14,
    'axes.linewidth': 1.2,
    'grid.alpha': 0.2,
    'grid.linestyle': '-'
})

# --- Configuration ---
obs_x, obs_y = 2.5, -0.2
obs_radius = 0.35
robot_radius = 0.18
safety_buffer = 0.05
safe_radius = obs_radius + robot_radius + safety_buffer
goal_x, goal_y = 5.0, 0.0

def generate_trajectory_plot(df, figures_dir):
    print("Generating Trajectory Plot...")
    fig, ax = plt.subplots(figsize=(7, 6))

    # Obstacle Boundary
    ax.add_patch(patches.Circle((obs_x, obs_y), safe_radius, facecolor='#CC0000', alpha=0.15, zorder=1))
    ax.add_patch(patches.Circle((obs_x, obs_y), safe_radius, edgecolor='#CC0000', facecolor='none', linewidth=2.0, zorder=2))
    ax.add_patch(patches.Circle((obs_x, obs_y), obs_radius, facecolor='black', alpha=0.2, zorder=1))

    # Trajectory
    ax.plot(df['x'].to_numpy(), df['y'].to_numpy(), color='#0072BD', linewidth=2.0, zorder=3)

    cbf_df = df[df['mode'] == 'CBF_ACTIVE']
    if not cbf_df.empty:
        ax.scatter(cbf_df['x'].to_numpy(), cbf_df['y'].to_numpy(), color='#D95319', s=15, zorder=4)

    start_x, start_y = df.iloc[0]['x'], df.iloc[0]['y']
    ax.plot(start_x, start_y, marker='o', markersize=8, markeredgecolor='black', markerfacecolor='#77AC30', linestyle='None', zorder=5)
    ax.plot(goal_x, goal_y, marker='x', markersize=10, markeredgecolor='black', markeredgewidth=2.0, linestyle='None', zorder=5)

    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel(r'$x$ (m)')
    ax.set_ylabel(r'$y$ (m)')
    ax.grid(True)

    padding = 0.8
    ax.set_xlim(min(start_x, goal_x, obs_x - safe_radius) - padding, max(start_x, goal_x, obs_x + safe_radius) + padding)
    ax.set_ylim(min(start_y, goal_y, obs_y - safe_radius) - padding, max(start_y, goal_y, obs_y + safe_radius) + padding)

    legend_elements = [
        patches.Patch(facecolor='#CC0000', alpha=0.15, edgecolor='#CC0000', linewidth=2, label=r'Obstacle ($h \leq 0$)'),
        Line2D([0], [0], color='#0072BD', linewidth=2.0, label='Experimental Path'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#D95319', markersize=7, label='CBF Intervention'),
        Line2D([0], [0], marker='o', color='w', markeredgecolor='black', markerfacecolor='#77AC30', markersize=8, label='Initial State'),
        Line2D([0], [0], marker='x', color='black', markersize=10, markeredgewidth=2, linestyle='None', label='Target')
    ]
    # Moved legend strictly outside the plot area
    ax.legend(handles=legend_elements, loc='center left', bbox_to_anchor=(1.05, 0.5), frameon=False)
    plt.tight_layout()
    
    plt.savefig(os.path.join(figures_dir, 'experiment_trajectory.pdf'), format='pdf', bbox_inches='tight')
    plt.close()

def generate_mode_plot(df, figures_dir):
    print("Generating Mode Switching Plot...")
    times = df['time'].to_numpy()
    distances = np.sqrt((df['x'].to_numpy() - obs_x)**2 + (df['y'].to_numpy() - obs_y)**2)
    h_vals = distances - safe_radius
    modes = df['mode'].to_numpy()

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(times, h_vals, color='black', linewidth=2.0)
    ax.axhline(0, color='red', linestyle='-', linewidth=1.5)

    for i in range(len(times) - 1):
        if modes[i] == 'CBF_ACTIVE':
            ax.axvspan(times[i], times[i+1], color='#D95319', alpha=0.3, lw=0)
        elif modes[i] == 'NOMINAL':
            ax.axvspan(times[i], times[i+1], color='#0072BD', alpha=0.1, lw=0)

    legend_elements = [
        Line2D([0], [0], color='black', lw=2, label=r'$h(x(t))$'),
        Line2D([0], [0], color='red', lw=1.5, label='Boundary ($h=0$)'),
        Patch(facecolor='#0072BD', alpha=0.2, label='Nominal Mode'),
        Patch(facecolor='#D95319', alpha=0.4, label='CBF Intervention')
    ]
    # Moved legend strictly outside the plot area
    ax.legend(handles=legend_elements, loc='center left', bbox_to_anchor=(1.05, 0.5), frameon=False)
    ax.set_xlabel(r'Time $t$ (s)')
    ax.set_ylabel(r'$h(x(t))$')
    ax.grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(figures_dir, 'experiment_modes.pdf'), format='pdf', bbox_inches='tight')
    plt.close()

def generate_jitter_plot(df, figures_dir):
    print("Generating Compute Jitter Plot...")
    u_vals = df[['u_x', 'u_y']].to_numpy()
    u_diffs = np.linalg.norm(np.diff(u_vals, axis=0), axis=1)
    
    event_indices = np.where(u_diffs > 1e-4)[0] + 1 
    event_indices = np.insert(event_indices, 0, 0)
    event_times = df['time'].iloc[event_indices].to_numpy()
    
    tau = np.diff(event_times)
    
    # Filter out massive start/stop delays to keep the histogram readable
    tau = tau[tau < 1.0]
    
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.hist(tau, bins=30, color='#0072BD', edgecolor='black', alpha=0.7)
    
    t_min_theoretical = 0.05 
    ax.axvline(t_min_theoretical, color='#D95319', linestyle='--', linewidth=2.5, label=r'Theoretical $T_{\min}$ Bound')

    ax.set_xlabel(r'Measured Inter-Event Time $\tau$ (s)')
    ax.set_ylabel('Frequency')
    ax.grid(True)
    
    # Moved legend strictly outside the plot area
    ax.legend(loc='center left', bbox_to_anchor=(1.05, 0.5), frameon=False)
    
    plt.tight_layout()
    plt.savefig(os.path.join(figures_dir, 'experiment_jitter.pdf'), format='pdf', bbox_inches='tight')
    plt.close()

def main():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(current_dir, 'data', 'sim_data.csv')
    
    figures_dir = os.path.join(current_dir, 'figures')
    os.makedirs(figures_dir, exist_ok=True)
    
    try:
        df = pd.read_csv(csv_path)
        print(f"Successfully loaded data from: {csv_path}")
    except FileNotFoundError:
        print(f"Error: Could not find {csv_path}.")
        return

    # Normalizing time so the experiment starts exactly at t = 0.0
    df['time'] = df['time'] - df['time'].iloc[0]

    generate_trajectory_plot(df, figures_dir)
    generate_mode_plot(df, figures_dir)
    generate_jitter_plot(df, figures_dir)
    print(f"All experimental figures successfully saved to: {figures_dir}")

if __name__ == '__main__':
    main()