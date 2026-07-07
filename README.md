# BEPE Project — Swarm-Based Vehicle Routing for Capacitated Intralogistics

**Author:** Carolina da Silva Santos - Universidade Federal de São Carlos, Brazil  
**Supervisor:** Sabine Hauert - University of Bristol, United Kingdom  
**Funding:** FAPESP — BEPE Scientific Initiation

---

## Overview

This repository contains all code, data, and experiments developed during a BEPE scientific initiation project focused on coordinating multiple robots for pickup-and-delivery tasks in warehouse-style environments.

The project addresses the problem of moving boxes between pickup and delivery locations using a team of capacity-constrained robots. The approach combines:

1. A **VRPPD solver** that assigns pickup-and-delivery tasks to each robot while respecting capacity and routing constraints.
2. Two **MAPF planners** (ICBS and RHCR) that compute collision-free paths for all robots simultaneously.
3. A **Gazebo simulation** where the generated plans are executed on virtual DOTS robots.
4. A **physical robot deployment** using the same ROS2 controller on real DOTS robots.
5. **Metrics and analysis** tools that compare planning time, makespan, and real trajectory accuracy.

Although the project was originally framed around swarm robotics, centralized planning algorithms were adopted to ensure solution quality and reliability in constrained warehouse scenarios.

---

## Problem

Three robots operate on an 8 × 8 grid. Each scenario places pickup and delivery cells at random positions, assigns a random weight to each box, and gives each robot a carrying capacity. The robots must collectively collect and deliver all boxes without colliding, respecting capacity limits, and carrying at most one box at a time. To conduct the experiments with real robots, a 6x6 grid was necessary to better suit the arena used.

Boxes are represented as obstacle cells in the map. A pickup event opens the cell (the box is removed from the shelf) and a delivery event closes it again (the box is placed at the destination), so the set of traversable cells changes dynamically during execution.

---

## Pipeline

```
generate_scenarios.py          →  random VRPPD instance files
plot_scenarios.py              →  visual layout for inspection
vrppd_solver.py                →  task allocation + route ordering
convert_map.py (ICBS / RHCR)  →  map format conversion
run_experiments.py / run_rhcr.py  →  MAPF path planning
icbs_conversion.py / rhcr_conversion.py  →  output format for ROS2
ROS2 controller                →  Gazebo simulation or real robots
plot_trajectory.py             →  trajectory analysis
```

---

## Repository Structure

```
BEPE-Carolina/
├── simulation/
│   ├── VRPPD_solver/          # Scenario generator and VRPPD solver
│   ├── MAPF_ICBS/             # ICBS path planner (Python)
│   ├── MAPF_RHCR/             # RHCR lifelong MAPF planner (C++)
│   ├── file_conversion/       # Output converters for ROS2
│   ├── scripts/               # Shell scripts for full pipeline automation
│   └── ros2_ws/               # Gazebo simulation workspace (DOTS)
└── experiments/
    ├── ros2_ws/               # Physical robot ROS2 workspace (DOTS)
    ├── logs/                  # CSV trajectory logs from real experiments
    └── analysis/              # Trajectory visualization and analysis
```

---

## Modules

### Scenario Generator (`simulation/VRPPD_solver/`)

`generate_scenarios.py` creates random VRPPD instances on an 8 × 8 grid with 3 robots and a configurable number of boxes. For each scenario it writes:

- `*_map_input.txt` — grid layout with obstacle and free cells.
- `*_vrppd_data.txt` — robot start positions, box positions, and box weights.

`plot_scenarios.py` generates a PNG layout for visual inspection of each scenario.

```bash
python3 simulation/VRPPD_solver/generate_scenarios.py <num_scenarios> <num_boxes>
python3 simulation/VRPPD_solver/plot_scenarios.py <example_num> <num_boxes>
```

---

### VRPPD Solver (`simulation/VRPPD_solver/vrppd_solver.py`)

Solves the Vehicle Routing Problem with Pickup and Delivery using **Google OR-Tools**. For each instance it assigns tasks to robots while respecting:

- Capacity constraints (total box weight ≤ robot capacity).
- One-shot constraint (at most one box carried at a time).
- Pickup-before-delivery ordering.

The objective function minimises total Manhattan travel distance while balancing route lengths across robots. The solver uses `PARALLEL_CHEAPEST_INSERTION` for the initial solution and `GUIDED_LOCAL_SEARCH` as the local search strategy, with a 2-second time limit.

**Outputs per example:**
- `*_vrppd_paths.txt` — ordered waypoint sequences (start + pickups + deliveries) for each robot.

See [`simulation/VRPPD_solver/README.md`](simulation/VRPPD_solver/README.md) for full details.

```bash
python3 simulation/VRPPD_solver/vrppd_solver.py <example_num> <num_boxes>
```

---

### MAPF: ICBS (`simulation/MAPF_ICBS/`)

**Improved Conflict-Based Search (ICBS)** is an optimal two-level MAPF algorithm. The high-level search builds a constraint tree; the low-level search finds individual paths using A\*. ICBS adds cardinal conflict detection, conflict bypassing, and MA-CBS with merge-and-restart on top of the original CBS.

The implementation was adapted from [github.com/gloriyo/MAPF-ICBS](https://github.com/gloriyo/MAPF-ICBS) with the following extensions for VRPPD support:

- **Segment-by-segment planning**: the solver is called once per pickup/delivery waypoint, updating the map dynamically between segments.
- **Agent-specific obstacle access** (`robot_can_traverse`): each agent may enter its own box cell as a goal, but not those of other agents.
- **Timeout**: each solver call runs in a separate process and is terminated if it exceeds the time limit.
- **Planning time tracking**: total time is recorded to `ICBS_planning_time.txt`.

See [`simulation/MAPF_ICBS/README.md`](simulation/MAPF_ICBS/README.md) for a complete description of all modifications.

```bash
# Convert map to ICBS format
python3 simulation/MAPF_ICBS/convert_map.py <example_num> <num_boxes>

# Run the planner
python3 simulation/MAPF_ICBS/run_experiments.py <example_num> <num_boxes>

# Visualize result
python3 simulation/MAPF_ICBS/visualize.py <example_num> <num_boxes>
```

**Outputs:** `*_icbs_paths.txt`, `ICBS_planning_time.txt`, `ICBS_makespan.txt`.

---

### MAPF: RHCR (`simulation/MAPF_RHCR/`)

**Rolling-Horizon Collision Resolution (RHCR)** is a lifelong MAPF framework that replans every *h* timesteps using a windowed MAPF solver (PBS, ECBS, or WHCA\*). It is better suited to scenarios where robots receive new goals continuously.

The implementation was adapted from [github.com/Jiaoyang-Li/RHCR](https://github.com/Jiaoyang-Li/RHCR) (USC Research License, preserved in `original_license/`) with the following extensions:

- **Fixed-location mode**: pre-assigned VRPPD routes are loaded from files, replacing the standard induct/eject queue system.
- **Dynamic obstacles**: pickup and delivery events update the traversable cell set during the simulation.
- **Per-agent obstacle overrides**: each robot may enter its own assigned box cells.
- **Mandatory goal wait**: robots hold at each pickup/delivery goal for a configurable number of timesteps.
- **Cascade fallback**: when the primary solver (PBS) fails, the system retries with WHCA\* then LRA\*.
- **Termination guards**: stops after too many consecutive planning failures or too many idle timesteps.

The binary must be compiled before use. See [`simulation/MAPF_RHCR/README.md`](simulation/MAPF_RHCR/README.md) for a full description of all modifications and build instructions.

```bash
# Build (requires Boost)
cd simulation/MAPF_RHCR && cmake . && make

# Convert map and paths to RHCR format
python3 simulation/MAPF_RHCR/convert_map.py <example_num> <num_boxes>
python3 simulation/MAPF_RHCR/convert_vrppd_path_to_node.py <example_num> <num_boxes>

# Run the planner
python3 simulation/MAPF_RHCR/run_rhcr.py <example_num> <num_boxes>

# Visualize result
python3 simulation/MAPF_RHCR/visualize_rhcr.py <example_num> <num_boxes>
```

**Outputs:** `*_rhcr_paths.txt`, `RHCR_planning_time.txt`, `RHCR_makespan.txt`.

---

### File Conversion (`simulation/file_conversion/`)

After planning, the raw ICBS and RHCR outputs are converted into the segment-by-segment format expected by the ROS2 controllers. Each output file lists one agent per section, with one line per goal-to-goal subpath.

| Script | Input | Output |
|---|---|---|
| `icbs_conversion_visualize.py` | ICBS paths + VRPPD goals | Visualization-ready file |
| `icbs_conversion.py` | ICBS paths + VRPPD goals | ROS2-compatible segment file |
| `rhcr_conversion.py` | RHCR assigned-goal paths | ROS2-compatible segment file |

---

### Automation Scripts (`simulation/scripts/`)

Two shell scripts automate the full workflow. Configure `AMOUNT_INST` (number of scenarios) and `QNT_BOX` (boxes per scenario) at the top of each script before running.

**`prepare_and_run_scenarios.sh`** — full pipeline from scenario generation to planner output and file conversion:

```bash
chmod +x simulation/scripts/prepare_and_run_scenarios.sh
./simulation/scripts/prepare_and_run_scenarios.sh
```

**`run_metrics.sh`** — collects planning time (3 runs per scenario per planner) and makespan from the visualizers. Run only on scenarios that already produced successful solutions:

```bash
chmod +x simulation/scripts/run_metrics.sh
./simulation/scripts/run_metrics.sh
```

See [`simulation/scripts/README.md`](simulation/scripts/README.md) for configuration details.

---

### Gazebo Simulation (`simulation/ros2_ws/`)

A ROS2 workspace for executing the computed plans in the Gazebo simulator using DOTS robots. The main components are:

- **`mapf_path_planning.py`** — ROS2 node that loads the ICBS or RHCR segment file for each robot, converts grid coordinates to Gazebo world coordinates, and executes the trajectory via the navigation stack. Coordinates lifter actions (pickup and delivery) and inter-robot synchronization.
- **`world_converting.py`** — generates the Gazebo `.world` file for a given scenario.
- **`sim_3_mapf_path_planning.launch.py`** — launch file that starts three controller instances simultaneously.

**To run the simulation** (inside the Sirius container — run `sirius -w <workspace>` first):

```bash
# 1. Generate the Gazebo world
python3 simulation/ros2_ws/dots_templates/src/dots_example_controllers/worlds/world_converting.py <example_num> <num_boxes>

# 2. Build and launch
cd dots_templates
colcon build --symlink-install
source install/setup.bash
ros2 launch dots_example_controllers sim_3_mapf_path_planning.launch.py
```

The Gazebo GUI opens automatically (requires a local display or X forwarding).

---

### Physical Robot Experiments (`experiments/`)

The `experiments/` directory contains everything needed to run the planned trajectories on real DOTS robots and to analyse the results.

#### ROS2 Controller (`experiments/ros2_ws/`)

`mapf_trajectory.py` is the ROS2 node used on physical robots. It mirrors the simulation controller but reads odometry from the real robot, optionally applies ArUco-based localization corrections, and writes a CSV log for each robot during execution.

Before running, assign each physical robot hostname to the correct agent ID in `HOSTNAME_TO_AGENT_ID`:

```python
HOSTNAME_TO_AGENT_ID = {
    'robot_31c4efef': 0,
    'robot_b6bd40d3': 1,
    'robot_68d8f877': 2,
}
```

**To run on physical robots:**

**1. Start the server** (run on the arena server machine):

```bash
sirius server up
```

This starts the arena GUI and the Zenoh middleware bridge. Turn on the robots and wait for them to appear in the GUI with all camera feeds healthy.

**2. Connect to a robot** (by robot number):

```bash
sirius -r <robot_num> -w <your_workspace_name>
```

This SSH-es into the robot and starts a `simonj23/dots_skeleton_minimal:iron` container there, with your workspace mounted.

**3. Build and launch the controller on the robot:**

```bash
cd dots_templates_arm
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch dots_example_controllers run_mapf_trajectory.launch.py
```

**4.** Trigger the controller using the **Start button** on the game controller (the robot must first be taken out of E-stop via Reset → Power in the server GUI).

**5. Stop the server** when done:

```bash
sirius server down
```

#### Trajectory Logs (`experiments/logs/`)

Each run produces one CSV file per robot, containing timestamped odometry readings, planned waypoints, and ArUco correction events. Logs are organised by timestamp and example number.

#### Trajectory Analysis (`experiments/analysis/`)

`plot_trajectory.py` loads the CSV logs and generates comparison plots for each robot:

- **Baseline path** — the planned waypoint sequence.
- **Odometry trajectory** — the path estimated by the robot's own filtered pose.
- **ArUco trajectory** — positions corrected by the external camera-based localization system.
- **Time-coloured path** — the trajectory coloured by elapsed time to show speed variation.

```bash
python3 experiments/analysis/plot_trajectory.py
```

Output plots are saved to `experiments/analysis/plots/`.

---

## Metrics

The following metrics are collected and compared between ICBS and RHCR:

| Metric | Description | Source |
|---|---|---|
| **Planning time** | Wall-clock time to compute all paths | `ICBS_planning_time.txt`, `RHCR_planning_time.txt` |
| **Makespan** | Total timesteps until the last robot finishes | `ICBS_makespan.txt`, `RHCR_makespan.txt` |
| **Trajectory accuracy** | Deviation of real paths from planned paths | `plot_trajectory.py` output |

Planning time and makespan are measured across multiple scenarios and box counts. Each scenario is replanned three times to reduce variability.

---

## Installation

### Requirements

- Python 3.10+
- C++ compiler with CMake (for RHCR)
- [Boost](https://www.boost.org/) (for RHCR)
- [Google OR-Tools](https://developers.google.com/optimization) (for the VRPPD solver)
- ROS2 Humble (for simulation and physical deployment)
- DOTS Sirius platform (for physical robot experiments)
- Docker (for the Sirius containerised environment)
- `git` and `make`
- **Windows only**: WSL2 with Ubuntu 20.04 and Docker Desktop (enable the Ubuntu integration under Docker Desktop → Settings → Resources → WSL Integration)

### Python dependencies (ICBS)

```bash
cd simulation/MAPF_ICBS
pipenv install --dev
```

### RHCR build

```bash
sudo apt install libboost-all-dev
cd simulation/MAPF_RHCR
cmake .
make
```

### ROS2 workspace

```bash
cd simulation/ros2_ws/dots_templates
colcon build --symlink-install
source install/setup.bash
```

### DOTS Sirius Environment

The Sirius system provides a fully containerised ROS2 Iron environment for developing and running code on DOTS robots. All development happens inside Docker, so the setup is the same on Linux, macOS, and Windows (via WSL2).

**SSH Key Setup** — the sirius environment clones private Bitbucket repos, so you need an SSH key added to your Bitbucket account before proceeding:

```bash
ssh-keygen        # press Enter to all prompts
cat ~/.ssh/id_rsa.pub
```

Copy the output and add it under Personal Settings → SSH Keys on Bitbucket.

**Installation** — from the root of the NFS share (or wherever you keep your projects):

```bash
mkdir dots_sirius
cd dots_sirius
git clone git@bitbucket.org:hauertlab/dots_containers.git
./dots_containers/scripts/setup.sh
```

This clones the master container repo, creates a `workspaces/` directory, and installs the `sirius` command-line tool to `/usr/local/bin`.

**Entering the environment** — the concept of a workspace keeps each user's work isolated:

```bash
sirius -w <your_workspace_name>
```

If the workspace does not already exist, it is created and populated with the `dots_templates` starter repo. The command pulls the `simonj23/dots_skeleton:iron` Docker image and drops you into a full ROS2 Iron shell. Multiple terminal windows can connect to the same running container by specifying the same workspace name. Inside the container, your workspace directory is mounted at `/workspace`, so all changes persist on the host filesystem.

---

## License

The RHCR code is derived from the original RHCR repository by Jiaoyang Li et al. and is released under the USC Research License. The original license is preserved at [`simulation/MAPF_RHCR/original_license/license.md`](simulation/MAPF_RHCR/original_license/license.md).

The ICBS code is derived from [github.com/gloriyo/MAPF-ICBS](https://github.com/gloriyo/MAPF-ICBS).

All modifications are the work of the project author.
