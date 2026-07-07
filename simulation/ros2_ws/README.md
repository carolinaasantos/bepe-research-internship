# README: Simulation — Multi-Robot MAPF Controller

## Division

This README is divided into:
1. Simulation: ROS2 Controller
2. Simulation: ROS2 Launch Files

---

# 1. Controller

## Overview

`mapf_path_planning.py` is the ROS2 node that executes MAPF trajectories on simulated DOTS robots inside Gazebo. Each robot instance loads its assigned path, waits for localization to stabilise, and then navigates through its sequence of pickup and delivery waypoints. The controller supports two execution modes depending on which planner produced the paths:

- **ICBS mode** — robots synchronize at every pickup and delivery goal using a shared ROS2 topic before proceeding to the next segment.
- **RHCR mode** — robots operate independently, advancing to the next goal as soon as the current one is reached.

The controller is also responsible for actuating the box lifter mechanism and publishing telemetry for offline experiment analysis.

## Input Data

The `instances` folder is copied from the VRPPD solver directory into the ROS2 workspace at:

```
dots_templates/src/dots_example_controllers/dots_example_controllers/instances/
```

The controller loads a converted trajectory file depending on the selected planning algorithm:

- ICBS: `*_icbs_converted_paths.txt`
- RHCR: `*_rhcr_converted_paths.txt`

These files are produced by the `file_conversion/` scripts after the MAPF planner has run.

File format:

```text
agent_id
x1 y1 x2 y2 ...    # pickup segment
x1 y1 x2 y2 ...    # delivery segment

next_agent_id
x1 y1 x2 y2 ...
x1 y1 x2 y2 ...
```

Each agent section contains alternating pickup and delivery path segments. The last waypoint of each segment is the goal location.

Example:

```text
0
0 0 1 0 2 0
2 0 2 1 2 2

1
5 5 4 5 3 5
3 5 3 4 3 3
```

## Configuration

The following parameters are set directly in the controller code and must be edited before running:

```python
self.mode = "ICBS"       # "ICBS" or "RHCR"
self.box_quantity = 8    # number of boxes in the scenario
self.example = 14        # example number
```

Global motion parameters are defined in the `G` class:

| Parameter | Value | Description |
|---|---|---|
| `mv` | 0.2 | Maximum linear velocity (m/s) |
| `w` | 3.0 | Maximum angular velocity (rad/s) |
| `ekf_wait` | 15.0 | Localization stabilisation delay (s) |

## Robot Identification

In simulation, each robot is identified by its ROS2 namespace. The robot ID is extracted automatically from the namespace suffix:

```
robot_00000001  →  ID 0
robot_00000002  →  ID 1
robot_00000003  →  ID 2
```

This determines which section of the trajectory file each controller instance loads.

## Coordinate Conversion

Grid coordinates from the MAPF planner are converted into Gazebo world coordinates during path loading. The arena is 3.7 m × 3.7 m with its origin at the centre. The conversion assumes a 6-column grid:

```python
x = col * (3.7 / 6) - 1.85
y = 1.85 - row * (3.7 / 6)
```

## ICBS Synchronization

In **ICBS mode**, all robots synchronize at every pickup and delivery goal before advancing to the next segment. This enforces the time-coordinated schedule produced by the ICBS planner.

Synchronization procedure:

```text
Reach goal
    ↓
Publish completion on /robots_sync
    ↓
Wait until all robots have published
    ↓
Actuate lifter
    ↓
Proceed to next segment
```

Robots that finish early continue re-broadcasting their status so that robots that join the topic later do not miss messages.

In **RHCR mode**, synchronization is not needed. Each robot proceeds to the next goal immediately after completing the current one, mirroring the independent replanning behaviour of the RHCR algorithm.

## Lifter Control

The controller actuates the robot lifter at each goal location to simulate box pickup and delivery:

| Goal type | Action | Delay |
|---|---|---|
| Pickup | Lifter UP | 1 s after goal reached |
| Delivery | Lifter DOWN | 1 s after goal reached |

An additional hold period follows each lifter actuation before the robot advances to the next segment.

## Telemetry

The controller publishes runtime data as JSON strings on dedicated ROS2 topics. These are intended to be captured by a rosbag for offline analysis.

### Published Topics

| Topic | Content |
|---|---|
| `/experiment_metadata` | Algorithm, robot count, box count, example number, timestamp |
| `/tasks` | All pickup and delivery goal locations assigned to this robot |
| `/task_events` | High-level events: pickup/delivery started, finished, waiting sync |
| `/<robot_name>/next_waypoint` | Current goal location |
| `/<robot_name>/robot_state` | Current operational state |
| `/<robot_name>/planned_path` | Full waypoint sequence currently being executed |
| `/robots_sync` | ICBS synchronization barrier messages |
| `lifter` | Lifter command (Bool) |

### Subscribed Topics

| Topic | Content |
|---|---|
| `/robots_sync` | ICBS completion signals from other robots |

## Execution Workflow

```text
Load MAPF trajectory
    ↓
Assign segment to this robot by ID
    ↓
Wait for localization to stabilise (EKF delay)
    ↓
Execute pickup navigation
    ↓
(ICBS) Wait for all robots to reach pickup
    ↓
Actuate lifter UP
    ↓
Execute delivery navigation
    ↓
(ICBS) Wait for all robots to reach delivery
    ↓
Actuate lifter DOWN
    ↓
Advance to next task
    ↓
Repeat until all tasks complete
```

## Output Example

```text
robot_00000001: loaded waypoint (0.62, -1.23)
robot_00000001: loaded waypoint (0.62,  0.62)

Waiting for EKF...  (×N)

Lift initialized DOWN

Sending path (3 waypoints)

pickup:robot_00000001 → published on /robots_sync
All robots synchronized — Lift UP at pickup waypoint

Sending path (3 waypoints)

delivery:robot_00000001 → published on /robots_sync
All robots synchronized — Lift DOWN at delivery waypoint

All tasks finished, signaling done to peers!
```

---

# 2. Launch Files

## Overview

The simulation workspace includes three launch files and one world generation script. They together initialise the Gazebo environment, spawn the robots at the correct positions, and start one controller instance per robot.

---

## `sim_3_mapf_path_planning.launch.py`

The main simulation launch file. It starts the complete environment for a three-robot MAPF simulation.

### What it launches

For each robot:

- `basic_cam_ekf.launch.py` — localization pipeline (wheel odometry, camera, ArUco detection, and EKF).
- `mapf_path_planning.launch.py` — MAPF trajectory controller for that robot.

Additionally:

- `gazebo_rviz.launch.py` — Gazebo simulator with the selected world file. RViz is disabled by default.

### Robot start positions

Initial poses are defined as Gazebo world coordinates directly in the launch file:

```python
robots = [
    ('robot_00000001', (0.000000,  0.616667, 0.0)),
    ('robot_00000002', (-1.233333, 0.616667, 0.0)),
    ('robot_00000003', (1.233333,  0.616667, 0.0)),
]
```

These poses must match the starting positions in the selected MAPF scenario. Use `world_converting.py` to update them automatically.

### Launch parameters

| Parameter | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use Gazebo simulation clock |
| `use_rviz` | `false` | Open RViz alongside Gazebo |
| `world_file` | `arena_8boxes_example14.world` | Gazebo world to load |

### Usage

```bash
cd dots_templates
colcon build --symlink-install
source install/setup.bash
ros2 launch dots_example_controllers sim_3_mapf_path_planning.launch.py
```

---

## `mapf_path_planning.launch.py`

Per-robot launch file. It starts a single controller node inside the specified robot namespace.

The robot name defaults to the host machine name if not provided. In multi-robot simulation this file is called once per robot by `sim_3_mapf_path_planning.launch.py`, each time with a different `robot_name`.

### Launch parameters

| Parameter | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Use simulation clock |
| `robot_name` | hostname | ROS2 namespace for this robot |
| `robot_id` | `0` | MAPF agent identifier |

### Usage (standalone, single robot)

```bash
ros2 launch dots_example_controllers mapf_path_planning.launch.py robot_name:=robot_00000001
```

---

## `world_converting.py`

A utility script that generates the Gazebo `.world` file for a given MAPF scenario and updates the robot start poses in `sim_3_mapf_path_planning.launch.py` automatically.

It reads the MAPF instance file to locate box positions, converts them to Gazebo coordinates, and writes a `.world` file based on the `model_arena.world` template. It then patches the `robots` list in the simulation launch file with the correct starting positions for the selected example.

### Usage

```bash
python3 worlds/world_converting.py <example_num> <num_boxes>
```

### Output

- `worlds/arena_<num_boxes>boxes_example<example_num>.world` — Gazebo world file with box models placed at their initial positions.
- `launch/sim_3_mapf_path_planning.launch.py` — updated with the robot starting poses for the selected scenario.

Run this script before launching the simulation whenever the example or box count changes.
