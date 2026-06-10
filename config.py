# config.py — FINAL FIXED VERSION
GRID_RESOLUTION = 2.0
GRID_SIZE_X     = 80
GRID_SIZE_Y     = 80
GRID_SIZE_Z     = 20

SAFETY_RADIUS   = 8.0    # Stay 8m away from all obstacles
SAFETY_WEIGHT   = 15.0   # Strongly avoid danger zones

FLIGHT_ALTITUDE = -15.0  # 15m high — above ALL blocks and balls
FLIGHT_SPEED    =  1.5   # Slow and accurate
TAKEOFF_HEIGHT  = -12.0

SENSOR_RANGE    = 10.0   # Detect obstacles within 10 meters
REPLAN_DISTANCE =  8.0   # Replan if obstacle within 8 meters

AIRSIM_IP  = "127.0.0.1"
DRONE_NAME = "Drone1"
