# obstacle_detector.py — FIXED VERSION
# Detects obstacles using AirSim distance sensors.
# Called before EVERY waypoint move during flight.

import airsim
import time
from config import GRID_RESOLUTION, FLIGHT_ALTITUDE, REPLAN_DISTANCE

DANGER_DISTANCE = 8.0   # Stop if obstacle within 8 meters
BACKUP_DISTANCE = 3.0   # Back up this many meters


def check_distance_sensors(client):
    """
    Reads all 6 AirSim distance sensors.
    Returns dict of direction: distance (meters).
    Returns {} if sensors not configured in settings.json.
    """
    results = {}
    names = {
        'Front' : 'DistanceFront',
        'Back'  : 'DistanceBack',
        'Left'  : 'DistanceLeft',
        'Right' : 'DistanceRight',
        'Up'    : 'DistanceUp',
        'Down'  : 'DistanceDown',
    }
    for direction, sensor_name in names.items():
        try:
            data = client.getDistanceSensorData(
                distance_sensor_name=sensor_name
            )
            results[direction] = round(data.distance, 2)
        except Exception:
            results[direction] = 999.0
    return results


def get_drone_position(client):
    """Returns current drone (x, y, z) world position."""
    state = client.getMultirotorState()
    p = state.kinematics_estimated.position
    return p.x_val, p.y_val, p.z_val


def scan_grid_around_drone(client, grid):
    """
    Scans the occupancy grid around the drone's current position.
    Returns True if any obstacle cell is within REPLAN_DISTANCE meters.
    This works even without distance sensors — uses grid directly.
    """
    cx, cy, cz = get_drone_position(client)
    gx = int(cx / GRID_RESOLUTION)
    gy = int(cy / GRID_RESOLUTION)
    gz = 5   # Fixed Z level

    scan_cells = int(REPLAN_DISTANCE / GRID_RESOLUTION)
    NX, NY, NZ = grid.shape

    for dx in range(-scan_cells, scan_cells+1):
        for dy in range(-scan_cells, scan_cells+1):
            nx2, ny2 = gx+dx, gy+dy
            if (0 <= nx2 < NX and 0 <= ny2 < NY
                    and grid[nx2, ny2, gz] >= 1.0):
                return True, (cx, cy, cz)

    return False, (cx, cy, cz)


def is_path_clear(grid, start_cell, end_cell):
    """
    Checks if the straight line from start to end
    passes through any obstacle in the grid.
    Returns True if clear, False if blocked.
    """
    x1,y1,z1 = start_cell
    x2,y2,z2 = end_cell
    steps = max(abs(x2-x1), abs(y2-y1))
    if steps == 0:
        return True
    NX,NY,NZ = grid.shape
    for i in range(steps+1):
        t = i/steps
        cx = round(x1 + t*(x2-x1))
        cy = round(y1 + t*(y2-y1))
        cz = round(z1 + t*(z2-z1))
        if not (0<=cx<NX and 0<=cy<NY and 0<=cz<NZ):
            return False
        if grid[cx,cy,cz] >= 1.0:
            return False
    return True


def backup_drone(client, direction='Front'):
    """
    Moves drone away from obstacle direction.
    """
    cx, cy, cz = get_drone_position(client)

    moves = {
        'Front': (cx - BACKUP_DISTANCE, cy,                  FLIGHT_ALTITUDE),
        'Back' : (cx + BACKUP_DISTANCE, cy,                  FLIGHT_ALTITUDE),
        'Left' : (cx,                  cy + BACKUP_DISTANCE, FLIGHT_ALTITUDE),
        'Right': (cx,                  cy - BACKUP_DISTANCE, FLIGHT_ALTITUDE),
        'Up'   : (cx,                  cy,                   FLIGHT_ALTITUDE),
        'Down' : (cx,                  cy,                   FLIGHT_ALTITUDE),
    }
    bx, by, bz = moves.get(direction, (cx, cy, FLIGHT_ALTITUDE))

    print(f"  Backing up from {direction}...")
    client.moveToPositionAsync(
        bx, by, bz, 2.0, timeout_sec=8,
        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
        yaw_mode=airsim.YawMode(False, 0)
    ).join()
    time.sleep(0.5)


def replan(client, grid, goal_cell):
    """
    Re-runs A* from drone's current position to goal.
    Called when obstacle is detected mid-flight.
    Returns new smooth path or None.
    """
    from astar import astar
    from path_smoother import smooth_path

    cx, cy, cz = get_drone_position(client)
    gx = max(0, min(int(cx/GRID_RESOLUTION), grid.shape[0]-1))
    gy = max(0, min(int(cy/GRID_RESOLUTION), grid.shape[1]-1))

    # Find nearest free cell
    NX,NY = grid.shape[0], grid.shape[1]
    if grid[gx,gy,5] >= 1.0:
        for r in range(1,10):
            found = False
            for dx in range(-r,r+1):
                for dy in range(-r,r+1):
                    x2,y2 = gx+dx, gy+dy
                    if (0<=x2<NX and 0<=y2<NY
                            and grid[x2,y2,5]==0.0):
                        gx,gy = x2,y2
                        found = True
                        break
                if found: break
            if found: break

    new_start = (gx, gy, 5)
    print(f"  Replanning from {new_start} to {goal_cell}...")

    raw = astar(grid, new_start, goal_cell)
    if raw is None:
        print("  Replan failed — no path found.")
        return None

    path = smooth_path(raw, grid)
    print(f"  New path: {len(path)} waypoints.")
    return path
