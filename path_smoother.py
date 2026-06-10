# path_smoother.py
# Removes unnecessary zigzag waypoints from A* path.

import numpy as np


def has_clear_line(grid, p1, p2):
    """Check if straight line from p1 to p2 is clear of obstacles."""
    steps = max(abs(p2[i] - p1[i]) for i in range(3))
    if steps == 0:
        return True
    NX, NY, NZ = grid.shape
    for i in range(steps + 1):
        t = i / steps
        cx = round(p1[0] + t * (p2[0] - p1[0]))
        cy = round(p1[1] + t * (p2[1] - p1[1]))
        cz = round(p1[2] + t * (p2[2] - p1[2]))
        if not (0 <= cx < NX and 0 <= cy < NY and 0 <= cz < NZ):
            return False
        if grid[cx, cy, cz] >= 1.0:
            return False
    return True


def smooth_path(path, grid):
    """Remove unnecessary middle waypoints using line-of-sight check."""
    if len(path) <= 2:
        return path

    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        farthest = i + 1
        for j in range(i + 2, len(path)):
            if has_clear_line(grid, path[i], path[j]):
                farthest = j
            else:
                break
        smoothed.append(path[farthest])
        i = farthest

    print(f"\n  Path smoothed:")
    print(f"  Before : {len(path)} waypoints")
    print(f"  After  : {len(smoothed)} waypoints")
    return smoothed
