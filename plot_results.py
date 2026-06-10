import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

def main():
    try:
        df = pd.read_csv('simulation_results.csv')
    except FileNotFoundError:
        print("Error: 'simulation_results.csv' not found. Run the C++ node first!")
        return

    z, r = np.array([1.0, 2.0]), 0.5
    eps_values = df['epsilon'].unique()
    colors = ['purple', 'green', 'deepskyblue', 'firebrick']

    plt.figure(figsize=(8, 5))
    for i, eps in enumerate(eps_values):
        data = df[df['epsilon'] == eps]
        plt.plot(data['t'], data['T'], label=f'$\epsilon = {eps}$', color=colors[i % len(colors)], linewidth=1.5)
    plt.axhline(0, color='black', linewidth=2)
    plt.xlabel('$t$ (seconds)', fontsize=14)
    plt.ylabel('Event Trigger $\Gamma_\epsilon(x(t))$', fontsize=14)
    plt.legend(loc='upper right')
    plt.grid(True, linestyle='--', alpha=0.6)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.add_patch(patches.Circle((z[0], z[1]), r, color='red', alpha=0.5, label='Obstacle'))
    plt.plot(0, 0, 'go', markersize=12, label='Goal')
    plt.plot(0, 3.0, 'ko', markersize=8, label='Start')
    for i, eps in enumerate(eps_values):
        data = df[df['epsilon'] == eps]
        plt.plot(data['x1'], data['x2'], label=f'$\epsilon = {eps}$', color=colors[i % len(colors)], linewidth=1.5)
    plt.xlabel('$x_1(t)$', fontsize=14)
    plt.ylabel('$x_2(t)$', fontsize=14)
    plt.legend(loc='upper right')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.axis('equal')

    plt.figure(figsize=(8, 5))
    for i, eps in enumerate(eps_values):
        data = df[df['epsilon'] == eps]
        plt.plot(data['t'], data['h'], label=f'$\epsilon = {eps}$', color=colors[i % len(colors)], linewidth=1.5)
        viol = data[data['h'] < 0]
        if not viol.empty:
            plt.plot(viol['t'], viol['h'], 'r.', markersize=8)
    plt.axhline(0, color='black', linewidth=2, label='Safety Boundary (h=0)')
    plt.xlabel('$t$ (seconds)', fontsize=14)
    plt.ylabel('Barrier Function $h(x(t))$', fontsize=14)
    plt.legend(loc='upper right')
    plt.grid(True, linestyle='--', alpha=0.6)

    plt.show()

if __name__ == '__main__':
    main()
