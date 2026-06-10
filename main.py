# main.py — FINAL COMPLETE VERSION
# Run: python main.py

from config          import GRID_RESOLUTION, FLIGHT_ALTITUDE
from grid_builder    import build_grid
from astar           import astar
from path_smoother   import smooth_path
from drone_control   import connect, takeoff, fly_path, land
from visualizer      import show_path


def line(c="=", n=52): print(c * n)

def ask_int(prompt, lo, hi):
    while True:
        try:
            v = int(input(prompt))
            if lo <= v <= hi: return v
            print(f"    Enter {lo} to {hi}")
        except ValueError:
            print("    Numbers only")

def w2g(v): return int(v / GRID_RESOLUTION)
def g2w(v): return int(v * GRID_RESOLUTION)

def nearest_free(gx, gy, grid):
    NX,NY = grid.shape[0], grid.shape[1]
    if grid[gx,gy,5] == 0.0: return (gx,gy)
    for r in range(1,15):
        for dx in range(-r,r+1):
            for dy in range(-r,r+1):
                x2,y2 = gx+dx, gy+dy
                if (0<=x2<NX and 0<=y2<NY
                        and grid[x2,y2,5]==0.0):
                    return (x2,y2)
    return None


def main():
    line()
    print("  AUTONOMOUS DRONE NAVIGATION SYSTEM")
    print("  AirSim Blocks + A* + Obstacle Detection")
    line()

    # STEP 1 — Build grid
    print("\nStep 1: Building obstacle grid...")
    grid = build_grid()
    nx,ny,_ = grid.shape
    max_w = int((nx-1) * GRID_RESOLUTION)

    # STEP 2 — Find safe coordinates automatically
    free = [(x,y) for x in range(nx)
                  for y in range(ny)
                  if grid[x,y,5] == 0.0]
    if len(free) < 2:
        print("ERROR: No free cells. Check grid_builder.py")
        return

    sc = free[0]
    gc = next((c for c in reversed(free)
               if abs(c[0]-sc[0])+abs(c[1]-sc[1]) > 15),
              free[-1])

    print(f"\nStep 2: Enter coordinates (0 to {max_w} meters)")
    print(f"  Suggested START  : X={g2w(sc[0])}, Y={g2w(sc[1])}")
    print(f"  Suggested TARGET : X={g2w(gc[0])}, Y={g2w(gc[1])}")
    print(f"  Drone flies at {abs(FLIGHT_ALTITUDE):.0f}m altitude (automatic)")

    print("\n  -- START --")
    sx = ask_int(f"    X (0-{max_w}): ", 0, max_w)
    sy = ask_int(f"    Y (0-{max_w}): ", 0, max_w)

    print("\n  -- TARGET --")
    gx = ask_int(f"    X (0-{max_w}): ", 0, max_w)
    gy = ask_int(f"    Y (0-{max_w}): ", 0, max_w)

    # Auto-correct to nearest free cell
    sf = nearest_free(w2g(sx), w2g(sy), grid)
    gf = nearest_free(w2g(gx), w2g(gy), grid)

    if sf is None or gf is None:
        print("ERROR: Cannot find free start/target. Try X=0,Y=0")
        return

    if sf != (w2g(sx), w2g(sy)):
        print(f"  Start auto-corrected to ({g2w(sf[0])},{g2w(sf[1])})")
    if gf != (w2g(gx), w2g(gy)):
        print(f"  Target auto-corrected to ({g2w(gf[0])},{g2w(gf[1])})")

    start = (sf[0], sf[1], 5)
    goal  = (gf[0], gf[1], 5)

    # STEP 3 — Run A*
    print(f"\nStep 3: Running A*...")
    raw = astar(grid, start, goal)
    if raw is None:
        print("No path found. Try different coordinates.")
        return

    # STEP 4 — Smooth
    path = smooth_path(raw, grid)

    # STEP 5 — Visualize
    print("\nStep 4: Showing path visualization...")
    show_path(grid, path, start, goal)

    # STEP 6 — Summary
    print("\nPATH SUMMARY")
    line("-")
    print(f"  Start          : ({sx}m, {sy}m)")
    print(f"  Target         : ({gx}m, {gy}m)")
    print(f"  Raw waypoints  : {len(raw)}")
    print(f"  Smooth WP      : {len(path)}")
    print(f"  Flight altitude: {abs(FLIGHT_ALTITUDE):.0f}m")
    print(f"  Obstacle detect: ON (scans before every waypoint)")
    print("\n  Flight plan:")
    for i, wp in enumerate(path):
        print(f"    {i+1}. ({g2w(wp[0])}m, {g2w(wp[1])}m) "
              f"at {abs(FLIGHT_ALTITUDE):.0f}m altitude")
    line("-")

    # STEP 7 — Confirm
    print("\nAirSim Blocks must be open and fully loaded.")
    go = input("Press ENTER to fly  (or 'no' to cancel): ").strip().lower()
    if go == 'no':
        return

    # STEP 8 — Fly with full obstacle detection
    print("\nFlying now...")
    line()
    goal_world = (g2w(goal[0]), g2w(goal[1]), FLIGHT_ALTITUDE)

    try:
        client = connect()
        takeoff(client)
        fly_path(
            client, path,
            grid=grid,
            goal_cell=goal,
            goal_world=goal_world
        )
        land(client)
    except KeyboardInterrupt:
        print("\nStopped by user.")
        try:
            client.landAsync().join()
            client.armDisarm(False)
            client.enableApiControl(False)
        except: pass
    except Exception as e:
        print(f"\nERROR: {e}")

    line()
    print("  MISSION COMPLETE!")
    print(f"  Flew from ({sx},{sy}) to ({gx},{gy})")
    line()


if __name__ == "__main__":
    main()
