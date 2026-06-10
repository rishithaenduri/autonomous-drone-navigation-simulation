# drone_control.py — CRASH FIX + OBSTACLE DETECTION VERSION
# ════════════════════════════════════════════════════════════
#
# KEY FIXES:
#  1. Scans grid around drone before EVERY waypoint
#  2. If obstacle detected → stop → backup → replan
#  3. Altitude locked at 15m — never changes during flight
#  4. Slow speed (1.5 m/s) for accuracy
#  5. Handles both sensor-based and grid-based detection

import airsim
import time
from config import (AIRSIM_IP, DRONE_NAME, FLIGHT_SPEED,
                    TAKEOFF_HEIGHT, FLIGHT_ALTITUDE, GRID_RESOLUTION)
from obstacle_detector import (scan_grid_around_drone, check_distance_sensors,
                                backup_drone, replan, is_path_clear,
                                DANGER_DISTANCE, get_drone_position)


def connect():
    print("\nConnecting to AirSim...")
    client = airsim.MultirotorClient(ip=AIRSIM_IP)
    client.confirmConnection()
    print("  Connected!")
    client.enableApiControl(True)
    client.armDisarm(True)
    print("  Armed!\n")
    return client


def takeoff(client):
    print("Taking off...")
    client.takeoffAsync().join()
    time.sleep(1.0)
    print(f"  Climbing to {abs(FLIGHT_ALTITUDE):.0f}m...")
    client.moveToZAsync(FLIGHT_ALTITUDE, 3.0).join()
    time.sleep(2.0)
    cx,cy,cz = get_drone_position(client)
    print(f"  Altitude confirmed: {abs(cz):.1f}m\n")


def fly_waypoint(client, wx, wy):
    """Fly to one waypoint at fixed altitude."""
    wz = FLIGHT_ALTITUDE
    # Lock altitude first
    client.moveToZAsync(FLIGHT_ALTITUDE, 3.0).join()
    # Fly horizontally
    client.moveToPositionAsync(
        wx, wy, wz, FLIGHT_SPEED,
        timeout_sec=60,
        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
        yaw_mode=airsim.YawMode(False, 0)
    ).join()


def fly_path(client, path, grid=None, goal_cell=None,
             writer=None, goal_world=None):
    """
    Flies the complete path.
    Before each waypoint:
      - Scans grid for nearby obstacles
      - Checks distance sensors
      - If obstacle found: backup + replan
    """
    print(f"Flying {len(path)} waypoints...")
    print(f"Obstacle scanning: ON (grid + sensors)")
    print("-" * 50)

    replans = 0
    MAX_REPLANS = 5

    i = 0
    while i < len(path):
        cell = path[i]
        wx = cell[0] * GRID_RESOLUTION
        wy = cell[1] * GRID_RESOLUTION

        print(f"\n  Waypoint [{i+1}/{len(path)}]: "
              f"({wx:.0f}m, {wy:.0f}m)")

        # ── STEP 1: Scan grid around drone ────────────────
        if grid is not None:
            danger_found, pos = scan_grid_around_drone(client, grid)
            if danger_found and replans < MAX_REPLANS:
                print(f"  OBSTACLE in sensor range! Stopping...")
                client.hoverAsync().join()
                time.sleep(0.5)

                # Back up
                sensors = check_distance_sensors(client)
                closest = min(sensors, key=sensors.get,
                              default='Front')
                if sensors.get(closest, 999) < DANGER_DISTANCE:
                    backup_drone(client, closest)
                else:
                    backup_drone(client, 'Front')

                # Replan
                if goal_cell:
                    new_path = replan(client, grid, goal_cell)
                    if new_path:
                        path = new_path
                        i = 0
                        replans += 1
                        print(f"  Replanned ({replans}/{MAX_REPLANS})")
                        continue

        # ── STEP 2: Check path is clear to waypoint ───────
        if grid is not None:
            cx,cy,cz = get_drone_position(client)
            curr_cell = (
                max(0, min(int(cx/GRID_RESOLUTION), grid.shape[0]-1)),
                max(0, min(int(cy/GRID_RESOLUTION), grid.shape[1]-1)),
                5
            )
            target_cell = (cell[0], cell[1], 5)
            if not is_path_clear(grid, curr_cell, target_cell):
                print(f"  Path to waypoint is blocked! Replanning...")
                if goal_cell and replans < MAX_REPLANS:
                    new_path = replan(client, grid, goal_cell)
                    if new_path:
                        path = new_path
                        i = 0
                        replans += 1
                        continue

        # ── STEP 3: Fly to waypoint ───────────────────────
        fly_waypoint(client, wx, wy)

        
        # ── STEP 4: Check altitude after waypoint ─────────
        cx,cy,cz = get_drone_position(client)
        if abs(cz) < abs(FLIGHT_ALTITUDE) - 3:
            print(f"  Altitude dropped to {abs(cz):.1f}m — re-climbing...")
            client.moveToZAsync(FLIGHT_ALTITUDE, 3.0).join()

        print(f"  Waypoint {i+1} reached!")
        i += 1

    print("-" * 50)
    print("TARGET REACHED!\n")


def land(client):
    print("Landing...")
    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)
    print("Landed safely.\n")
