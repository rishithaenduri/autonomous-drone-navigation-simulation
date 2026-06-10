# astar.py — COMPLETE FIX VERSION
#
# KEY FIX: Only searches in 2D (X and Y directions).
# Z is kept fixed at flight altitude.
# This means drone NEVER changes height during flight —
# it stays at 15m above ground and only moves left/right/forward/back.
# Much simpler and much more reliable.

import heapq
import numpy as np
from config import SAFETY_WEIGHT


def heuristic(a, b):
    # Only X and Y distance — ignore Z
    return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


def get_neighbors_2d(cell, grid):
    """
    Only moves in X and Y — 8 directions (no height changes).
    Drone stays at same Z (altitude) the whole flight.
    """
    x, y, z = cell
    NX, NY, NZ = grid.shape
    neighbors = []

    # 8 directions — horizontal only (no Z change)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nx2 = x + dx
            ny2 = y + dy
            nz2 = z   # Z never changes

            if (0 <= nx2 < NX and
                0 <= ny2 < NY and
                0 <= nz2 < NZ and
                    grid[nx2, ny2, nz2] < 1.0):
                neighbors.append((nx2, ny2, nz2))

    return neighbors


def astar(grid, start, goal):
    print(f"\n  A* running (2D horizontal path)...")
    print(f"  Start : {start}")
    print(f"  Goal  : {goal}")

    # Force both to same Z level
    start = (start[0], start[1], start[2])
    goal  = (goal[0],  goal[1],  start[2])  # same Z as start

    open_set = []
    heapq.heappush(open_set, (0.0, start))

    came_from = {}
    g_score = {start: 0.0}
    explored = 0

    while open_set:
        _, current = heapq.heappop(open_set)
        explored += 1

        if explored % 1000 == 0:
            print(f"  ...explored {explored} cells")

        # Reached goal (check only X and Y)
        if current[0] == goal[0] and current[1] == goal[1]:
            # Reconstruct path
            path = []
            node = current
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()
            print(f"\n  Path found!")
            print(f"  Waypoints     : {len(path)}")
            print(f"  Cells explored: {explored}")
            return path

        for nb in get_neighbors_2d(current, grid):
            dx = abs(nb[0] - current[0])
            dy = abs(nb[1] - current[1])
            move_cost = np.sqrt(dx*dx + dy*dy)
            safety    = grid[nb] * SAFETY_WEIGHT
            new_g     = g_score[current] + move_cost + safety

            if nb not in g_score or new_g < g_score[nb]:
                came_from[nb] = current
                g_score[nb]   = new_g
                f = new_g + heuristic(nb, goal)
                heapq.heappush(open_set, (f, nb))

    print("\n  No path found!")
    print("  Try: Start=(5,5,5) Target=(110,110,5)")
    return None


if __name__ == "__main__":
    import numpy as np
    print("Testing astar.py...")
    g = np.zeros((50, 50, 10), dtype=float)
    g[15:35, 15:35, :] = 1.0   # Big wall
    path = astar(g, (2, 2, 5), (45, 45, 5))
    if path:
        print(f"Test PASSED — {len(path)} waypoints")
    else:
        print("Test FAILED")
