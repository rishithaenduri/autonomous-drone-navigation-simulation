# visualizer.py
# 2D Real-Time Path Visualization
# ══════════════════════════════════════════════════════════

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from config import GRID_RESOLUTION, FLIGHT_ALTITUDE


def show_path(grid, path, start, goal, save=True):
    """
    Renders a 2D top-down view of the drone navigation path.
    Shows obstacles, danger zones, path, start and goal.
    """
    z_level = start[2]
    grid_2d = grid[:, :, z_level]

    fig, ax = plt.subplots(figsize=(11, 11))
    fig.patch.set_facecolor('#1e1e2e')
    ax.set_facecolor('#1e1e2e')

    # Draw grid (red = obstacle/danger, green = safe)
    img = ax.imshow(
        grid_2d.T,
        origin='lower',
        cmap='RdYlGn_r',
        vmin=0, vmax=1,
        alpha=0.85
    )

    # Draw A* path
    if path and len(path) > 1:
        px = [p[0] for p in path]
        py = [p[1] for p in path]
        ax.plot(px, py, color='#4fc3f7',
                linewidth=2.5, label='A* Path', zorder=3)
        # Draw direction arrows
        for k in range(0, len(path) - 1, max(1, len(path) // 8)):
            ax.annotate('',
                xy=(px[k+1], py[k+1]),
                xytext=(px[k], py[k]),
                arrowprops=dict(
                    arrowstyle='->',
                    color='#4fc3f7',
                    lw=1.5
                )
            )

    # Draw all waypoints
    for i, wp in enumerate(path):
        ax.plot(wp[0], wp[1], 'o',
                color='#4fc3f7', markersize=5, zorder=4)
        ax.text(wp[0]+0.3, wp[1]+0.3, str(i+1),
                color='#4fc3f7', fontsize=7)

    # Start and goal markers
    ax.plot(start[0], start[1], 'o',
            color='#69ff47', markersize=16,
            label='Start', zorder=5)
    ax.plot(goal[0], goal[1], '*',
            color='#ff4d4d', markersize=20,
            label='Goal', zorder=5)
    ax.text(start[0]+0.5, start[1]+0.5, 'START',
            color='#69ff47', fontsize=10, fontweight='bold')
    ax.text(goal[0]+0.5, goal[1]+0.5, 'GOAL',
            color='#ff4d4d', fontsize=10, fontweight='bold')

    # Labels and formatting
    title = (f'Autonomous Drone Navigation — A* Path\n'
             f'Altitude: {abs(FLIGHT_ALTITUDE):.0f}m  |  '
             f'Waypoints: {len(path)}  |  '
             f'Grid Z-slice: {z_level}')
    ax.set_title(title, color='white', fontsize=12, pad=14)
    ax.set_xlabel('X (grid cells)', color='white', fontsize=11)
    ax.set_ylabel('Y (grid cells)', color='white', fontsize=11)
    ax.tick_params(colors='white')

    legend = ax.legend(
        facecolor='#2d2d2d',
        labelcolor='white',
        fontsize=10,
        loc='upper right'
    )

    cbar = plt.colorbar(img, ax=ax)
    cbar.set_label('Danger level (1=obstacle)', color='white')
    cbar.ax.yaxis.set_tick_params(color='white')
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color='white')

    plt.tight_layout()

    if save:
        plt.savefig('path_result.png', dpi=150, bbox_inches='tight')
        print("  Visualization saved: path_result.png")

    plt.show()