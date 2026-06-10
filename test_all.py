# ============================================================
# test_all.py
# TEST ALL FILES WITHOUT NEEDING AIRSIM OPEN
# ============================================================
#
# Run this first to check everything works:
#   python test_all.py
#
# It tests: config, grid_builder, astar, path_smoother
# It does NOT test drone_control (needs AirSim running)
# ============================================================

import sys
import numpy as np

print("=" * 55)
print("  DRONE NAVIGATION — FULL TEST")
print("  Testing all modules (no AirSim needed)")
print("=" * 55)

passed = 0
failed = 0


def test(name, condition, detail=""):
    global passed, failed
    if condition:
        print(f"  PASS  {name}")
        passed += 1
    else:
        print(f"  FAIL  {name}  {detail}")
        failed += 1


# ── TEST 1: config.py ─────────────────────────────────────────
print("\n[1] Testing config.py...")
try:
    from config import (GRID_RESOLUTION, GRID_SIZE_X, GRID_SIZE_Y,
                        GRID_SIZE_Z, SAFETY_RADIUS, SAFETY_WEIGHT,
                        FLIGHT_ALTITUDE, FLIGHT_SPEED,
                        TAKEOFF_HEIGHT, AIRSIM_IP, DRONE_NAME)

    test("GRID_RESOLUTION is positive",    GRID_RESOLUTION > 0)
    test("GRID_SIZE_X is at least 20",     GRID_SIZE_X >= 20)
    test("GRID_SIZE_Y is at least 20",     GRID_SIZE_Y >= 20)
    test("GRID_SIZE_Z is at least 5",      GRID_SIZE_Z >= 5)
    test("SAFETY_RADIUS is positive",      SAFETY_RADIUS > 0)
    test("SAFETY_WEIGHT is positive",      SAFETY_WEIGHT > 0)
    test("FLIGHT_ALTITUDE is negative",    FLIGHT_ALTITUDE < 0,
         "(AirSim up = negative Z)")
    test("FLIGHT_SPEED is positive",       FLIGHT_SPEED > 0)
    test("DRONE_NAME is set",              len(DRONE_NAME) > 0)
    test("AIRSIM_IP is set",               len(AIRSIM_IP) > 0)

except ImportError as e:
    print(f"  FAIL  Cannot import config.py: {e}")
    failed += 5


# ── TEST 2: grid_builder.py ───────────────────────────────────
print("\n[2] Testing grid_builder.py...")
try:
    from grid_builder import build_grid

    grid = build_grid()

    test("Grid is a numpy array",
         isinstance(grid, np.ndarray))
    test("Grid is 3D",
         grid.ndim == 3)
    test("Grid has correct X size",
         grid.shape[0] == int(GRID_SIZE_X / GRID_RESOLUTION))
    test("Grid has correct Y size",
         grid.shape[1] == int(GRID_SIZE_Y / GRID_RESOLUTION))
    test("Grid has correct Z size",
         grid.shape[2] == int(GRID_SIZE_Z / GRID_RESOLUTION))
    test("Grid min value is 0.0",
         grid.min() == 0.0)
    test("Grid max value is 1.0",
         grid.max() == 1.0)
    test("Grid has obstacle cells",
         np.sum(grid == 1.0) > 0)
    test("Grid has free cells",
         np.sum(grid == 0.0) > 0)
    test("Grid has danger zone cells",
         np.sum((grid > 0) & (grid < 1)) > 0)

    print(f"  Grid shape: {grid.shape}")
    print(f"  Obstacle cells: {int(np.sum(grid==1.0))}")

except Exception as e:
    print(f"  FAIL  grid_builder error: {e}")
    failed += 5


# ── TEST 3: astar.py ──────────────────────────────────────────
print("\n[3] Testing astar.py...")
try:
    from astar import astar, heuristic, get_neighbors

    # Test heuristic
    h = heuristic((0, 0, 0), (3, 4, 0))
    test("Heuristic distance (0,0,0)→(3,4,0) = 5.0",
         abs(h - 5.0) < 0.001,
         f"got {h:.3f}")

    # Test get_neighbors on a simple grid
    simple_grid = np.zeros((10, 10, 10), dtype=float)
    neighbors = get_neighbors((5, 5, 5), simple_grid)
    test("Cell (5,5,5) has 26 neighbors in empty grid",
         len(neighbors) == 26,
         f"got {len(neighbors)}")

    # Test A* on a simple empty grid
    empty_grid = np.zeros((20, 20, 10), dtype=float)
    path = astar(empty_grid, (0, 0, 5), (19, 19, 5))
    test("A* finds path on empty grid",
         path is not None)
    test("Path starts at (0,0,5)",
         path is not None and path[0] == (0, 0, 5))
    test("Path ends at (19,19,5)",
         path is not None and path[-1] == (19, 19, 5))

    # Test A* on grid with obstacle
    obstacle_grid = np.zeros((20, 20, 10), dtype=float)
    obstacle_grid[5:15, 0:18, 0:9] = 1.0  # Big wall
    path2 = astar(obstacle_grid, (0, 0, 5), (19, 5, 5))
    test("A* finds path around obstacle",
         path2 is not None)

    # Test A* on full grid from grid_builder
    full_path = astar(grid, (5, 5, 5), (55, 55, 5))
    test("A* finds path on Blocks grid (5,5,5)→(55,55,5)",
         full_path is not None)

    if full_path:
        test("Full path starts at (5,5,5)",
             full_path[0] == (5, 5, 5))
        test("Full path ends at (55,55,5)",
             full_path[-1] == (55, 55, 5))
        print(f"  Path length: {len(full_path)} waypoints")

except Exception as e:
    print(f"  FAIL  astar error: {e}")
    import traceback
    traceback.print_exc()
    failed += 5


# ── TEST 4: path_smoother.py ──────────────────────────────────
print("\n[4] Testing path_smoother.py...")
try:
    from path_smoother import smooth_path, has_clear_line

    # Test has_clear_line on empty grid
    empty = np.zeros((20, 20, 10), dtype=float)
    test("Clear line on empty grid",
         has_clear_line(empty, (0, 0, 5), (19, 19, 5)))

    # Test blocked line
    blocked = np.zeros((20, 20, 10), dtype=float)
    blocked[10, :, :] = 1.0   # Wall at x=10
    test("Blocked line detected",
         not has_clear_line(blocked, (0, 5, 5), (19, 5, 5)))

    # Test smooth on short path
    short = [(0,0,5), (1,1,5)]
    result = smooth_path(short, empty)
    test("Short path (2 points) returned unchanged",
         len(result) == 2)

    # Test smooth reduces a zigzag path
    zigzag = [(i, i, 5) for i in range(10)]
    smoothed = smooth_path(zigzag, empty)
    test("Zigzag path reduced by smoothing",
         len(smoothed) < len(zigzag))

    # Test smooth on real A* path
    if full_path:
        smooth = smooth_path(full_path, grid)
        test("Smooth path is shorter than raw path",
             len(smooth) <= len(full_path))
        test("Smooth path starts at same point",
             smooth[0] == full_path[0])
        test("Smooth path ends at same point",
             smooth[-1] == full_path[-1])
        print(f"  Raw: {len(full_path)} → Smooth: {len(smooth)} waypoints")

except Exception as e:
    print(f"  FAIL  path_smoother error: {e}")
    import traceback
    traceback.print_exc()
    failed += 5


# ── FINAL SUMMARY ─────────────────────────────────────────────
print("\n" + "=" * 55)
print(f"  RESULTS: {passed} passed  |  {failed} failed")

if failed == 0:
    print("\n  ALL TESTS PASSED!")
    print("  Your project is ready.")
    print("  Next step: Open AirSim Blocks.exe, then run main.py")
else:
    print(f"\n  {failed} test(s) failed.")
    print("  Fix the errors above before running main.py")
    print("  Read the error message carefully — it tells you what is wrong.")

print("=" * 55)
