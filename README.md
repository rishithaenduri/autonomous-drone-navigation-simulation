# Autonomous Drone Navigation Simulation
## AirSim Blocks 3D + A* Algorithm

---

## What This Project Does

You type a **start position** and a **target position**.  
The A* algorithm finds the **shortest and safest path** around buildings and trees.  
The drone **flies that path automatically** inside AirSim Blocks 3D simulation.

---

## Requirements

- Python 3.10+
- AirSim Blocks .exe (download from GitHub releases)
- pip packages: `airsim`, `numpy`

Install packages:
```
pip install airsim numpy
```

---

## Project Files

| File | Purpose |
|------|---------|
| `config.py` | All settings (speed, grid size, safety radius) |
| `grid_builder.py` | Builds 3D grid from Blocks world obstacles |
| `astar.py` | A* pathfinding algorithm |
| `path_smoother.py` | Removes zigzag steps from path |
| `drone_control.py` | Controls drone in AirSim |
| `main.py` | Entry point — run this file |
| `test_all.py` | Tests everything without needing AirSim open |

---

## How to Run

### Step 1 — Test first (no AirSim needed)
```
python test_all.py
```
All tests should show PASS.

### Step 2 — Open AirSim Blocks
Double-click `Blocks.exe` and wait for the 3D world to load.  
You should see buildings, trees, and a drone on the ground.

### Step 3 — Run the project
```
python main.py
```

### Step 4 — Enter coordinates
```
START POSITION
  Enter X: 5
  Enter Y: 5
  Enter Z: 5

TARGET POSITION
  Enter X: 55
  Enter Y: 55
  Enter Z: 5
```

### Step 5 — Watch the drone fly!

---

## AirSim settings.json

Create this file at: `C:\Users\YourName\Documents\AirSim\settings.json`

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ClockSpeed": 1,
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "AutoCreate": true
    }
  }
}
```

---

## Safe Test Coordinates

| Test | Start | Target |
|------|-------|--------|
| Basic flight | (5, 5, 5) | (55, 55, 5) |
| High altitude | (5, 5, 15) | (55, 55, 15) |
| Short hop | (5, 5, 5) | (15, 5, 5) |

---

## Algorithm

**f(n) = g(n) + h(n) + safety(n)**

- `g(n)` — distance traveled from start  
- `h(n)` — estimated distance to goal (Euclidean)  
- `safety(n)` — penalty for flying near buildings/trees

---

*Project by: Rishitha*
