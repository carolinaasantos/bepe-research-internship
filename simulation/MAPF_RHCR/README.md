# README: MAPF-RHCR

## Overview

This module adapts the **Rolling-Horizon Collision Resolution (RHCR)** lifelong MAPF solver to support **VRPPD (Vehicle Routing Problem with Pickup and Delivery)** scenarios. Robots follow pre-assigned routes in which pickup events open obstacle cells (boxes are picked up) and delivery events close them again (boxes are placed). The solver is extended to load these fixed routes from files, enforce mandatory wait times at each goal, and dynamically update the set of traversable cells throughout the simulation.

The compiled binary (`lifelong`) is invoked by the Python wrapper `run_rhcr.py`, which handles file path resolution and planning time recording.

**Original source:** https://github.com/Jiaoyang-Li/RHCR

---

## Modified Files

### `src/driver.cpp`

The CLI entry point that parses command-line options and launches the simulation.

- **New CLI options**: `--goal_wait` (timesteps each agent must wait at every pickup/delivery goal), `--close_delivery_obstacles` (whether delivery events re-close the cell), `--max_failed_plans` (consecutive replanning failures before termination), `--max_no_progress_timesteps` (idle timesteps before termination), `--debug_positions` (print per-agent position at every timestep), `--locations` (path to the pre-assigned start/goal file), `--traverse` (path to the per-agent traversable obstacle file).
- **`goal_wait` floor**: `std::max(5, goal_wait)` ensures the wait is never below 5 timesteps.

### `src/BasicSystem.cpp`

The abstract base class for all simulation scenarios. Most of the VRPPD-specific logic is implemented here.

- **`load_locations_from_txt()`**: reads per-agent start and goal sequences from a `.txt` file supplied via `--locations`. When this file is present, the simulation runs in *fixed locations mode* and stops as soon as all agents have reached their final goals.
- **`load_traverse_from_txt()`**: reads per-agent lists of traversable obstacle cell IDs from a `.txt` file supplied via `--traverse`, populating the per-agent override sets used by the path planners.
- **`all_assigned_goals_finished()`**: checks whether every agent has completed all pre-assigned goals, enabling early termination.
- **`reset_dynamic_obstacles_from_assigned_goals()`**: reconstructs the dynamic obstacle state from the remaining unfinished goals on startup, so the simulation starts with the correct box configuration.
- **`refresh_effective_traversable()`**: rebuilds the per-agent traversable sets after each obstacle state change by merging the base assigned traversable with the current set of opened dynamic obstacle cells.
- **`mandatory_goal_wait`**: agents now hold at each pickup/delivery goal for a configurable number of timesteps before being assigned the next goal.
- **Cascade fallback solver**: when the primary solver (PBS or ECBS) fails, the system automatically retries with WHCA\* and then LRA\*, instead of immediately terminating.
- **Termination counters**: `consecutive_failed_plans` and `terminated_no_solution` stop the simulation after `max_consecutive_failed_plans` consecutive failures; `timesteps_without_progress` stops it after `max_no_progress_timesteps` timesteps with no completed task.
- **Output renamed**: the result file is written as `rhcr_paths.txt` instead of `assigned_goal_paths.txt`.
- **`debug_positions` flag and `print_positions_at_timestep()`**: prints each agent's grid position and orientation at every timestep when enabled.

### `src/SortingSystem.cpp`

The concrete simulation scenario class used for warehouse sorting layouts.

- **`fixed_locations_mode`**: when enabled (start/goal file provided), standard induct/eject queue assignment is skipped and the simulation uses the pre-assigned routes exclusively.
- **`assign_random_travel_location()`**: fallback goal for agents whose induct/eject queues are empty; picks any non-obstacle cell at random.
- **Guarded induct/eject assignment**: `!G.inducts.empty()` and `!G.ejects.empty()` checks prevent crashes on maps with no induct or eject stations (e.g. open grid maps used for VRPPD).
- **`Travel`-type goal handling** in `update_goal_locations()`: agents assigned to generic traversable cells are handled correctly alongside induct/eject goals.
- **Simulation loop changed from `for` to `while`**: the horizon advances correctly when `move()` returns early after a goal is reached mid-window.
- **Startup validation**: the system exits with a diagnostic message if any start→goal segment is disconnected or invalid.
- **Congestion check skipped** in `fixed_locations_mode` to avoid false positives during mandatory wait holds.

### `src/BasicGraph.cpp`

The graph data structure used by all planners.

- **`is_traversable()`**: a new method that centralises obstacle reachability checking, respecting both statically blocked cells and per-agent dynamic obstacle overrides.
- **`valid_move()`**: replaces direct `weights[loc][dir] < WEIGHT_MAX` checks throughout the codebase. It respects `dynamic_blocked` cells and per-agent traversable overrides, and allows an agent to leave a cell that was dynamically closed under it.
- **`get_neighbors()`, `get_reverse_neighbors()`, `get_weight()`**: updated to accept an optional `traversable` pointer and delegate to `valid_move()`. `get_weight()` returns a cost of 1 (instead of `WEIGHT_MAX`) for agent-specific override edges.
- **Removed post-hoc obstacle re-labelling**: the pass that set `types[j]="Obstacle"` for disconnected nodes was removed because it was incompatible with dynamically opened cells.

### `src/SIPP.cpp`

The Safe Interval Path Planning single-agent solver.

- **Fixed dummy start states**: instead of emitting off-map `State(-1,-1)` for timesteps before the first safe interval, agents now wait at their real start location and orientation.
- **Timeout check**: the main search loop checks `max_planning_time` and stops if the time budget is exceeded.
- **`wait_on_current_goal` guard**: when an agent is at its current goal within the mandatory wait window, only in-place wait moves are generated.
- **`can_enter_override_obstacle()` check**: prevents routing through dynamic obstacle cells that are not the agent's current goal.
- **`current_traversable()` passed to `get_weight()` and `valid_move()`**: edge validity and costs respect per-agent dynamic obstacle overrides.

### `src/StateTimeAStar.cpp`

The Space-Time A\* single-agent solver.

- Same set of additions as `SIPP.cpp`: timeout check, `wait_on_current_goal` guard, `can_enter_override_obstacle()` check, and `current_traversable()` propagated to `get_neighbors()` and `get_weight()`.
- **Explicit break on empty open list during restart scan**: prevents an infinite loop when no legal restart state exists under current constraints.

### `src/PBS.cpp`

The Priority-Based Search multi-agent solver.

- **`path_planner.current_agent` and `path_planner.max_planning_time`** set before each single-agent path search in `find_path()` and `generate_root_node()`, so each agent respects its traversable set and the remaining time budget.
- **Fixed use-after-free**: `best_node` is no longer cleared when the search ends; only non-best nodes are cleared. Previously, clearing `best_node` destroyed `Path` objects while `PBS::paths` still held raw pointers to them.
- **Safe heuristic initialisation in `get_lower_bound()`**: replaced unchecked `G.heuristics.at(goal)[start]` with a bounds-checked lookup that falls back to Manhattan distance for obstacle-overridden goal cells whose heuristic table was never computed.

### `src/LRAStar.cpp`

The Local Repair A\* multi-agent solver (fallback).

- **`path_planner.current_agent`** set before the per-agent path search so traversable overrides are applied to the correct agent.
- **`wait_on_current_goal` guard**, **`can_enter_override_obstacle()` check**, and **`current_traversable()`** propagated to `get_weight()` and `get_neighbors()`.

### `src/SingleAgentSolver.cpp`

The base class for single-agent solvers; provides shared utilities used by SIPP, StateTimeAStar, and others.

- **Safe heuristic lookup in `compute_h_value()`**: replaced unchecked `G.heuristics.at(goal)[curr]` with a bounds-checked version that uses Manhattan distance as a fallback for obstacle-override nodes whose heuristic table was never computed. The same fallback is applied for intermediate goal-to-goal segments.
- **`can_enter_override_obstacle()`**: determines whether an agent may step onto a dynamically blocked cell. A box cell (dynamic obstacle) is only enterable when it is exactly the agent's current goal (pickup). Cells that have been opened (box collected) or are not dynamic obstacles are freely traversable.

---

## New Files

### `run_rhcr.py`

Python wrapper that invokes the compiled `lifelong` binary for a given example and box count.

- Auto-resolves all file paths (map, node locations, output) from `num_example` and `num_boxes`.
- Passes a configurable set of solver parameters (PBS solver, SIPP single-agent solver, planning window, suboptimal bound, goal wait, etc.).
- Enforces a process-level timeout via `subprocess.run(..., timeout=...)` and reports the exit code.
- Records total wall-clock planning time in seconds, milliseconds, microseconds, and nanoseconds and appends the result to `RHCR_planning_time.txt`.

```bash
python3 run_rhcr.py <num_example> <num_boxes> [--boxes-dir <dir>] [--timeout <s>] [--cutoff-time <s>]
```

### `convert_map.py`

Converts a grid map file (`*_map_input.txt`) into the RHCR `.grid` format required by the `lifelong` binary. Each cell is annotated with its type (`Travel` or `Obstacle`) and edge weights (1 for passable edges, `inf` for blocked ones). Column headers in the output file are named explicitly so the direction mapping is unambiguous.

```bash
python3 convert_map.py <num_example> [num_boxes] [--boxes-dir <dir>]
```

### `convert_vrppd_path_to_node.py`

Converts VRPPD path coordinates into the node-index format required by the `lifelong` binary (`*_node_locations.txt`). Each (row, col) coordinate is translated to a flat node ID using row-major order. The output file lists one agent per line with comma-separated node IDs representing the start position and all subsequent waypoints.

```bash
python3 convert_vrppd_path_to_node.py <num_example> [num_boxes] [--boxes-dir <dir>]
```

### `visualize_rhcr.py`

Standalone visualizer for RHCR output paths. Reads the `.grid` map and the `rhcr_paths.txt` output file and animates agent movements on the grid. Supports both the detailed `assigned_goal_paths` format (with goal and timing information) and the basic `paths.txt` format.

- **Dynamic obstacle toggling**: pickup events open box cells and delivery events close them again during the animation, in sync with the recorded goal-reached timestamps.
- **Makespan output**: total animation duration is printed and saved to `RHCR_makespan.txt`.

```bash
python3 visualize_rhcr.py <num_example> <num_boxes> [--boxes-dir <dir>] [--save <file>] [--not-show]
```

---

## Build

The C++ binary must be compiled before use. Requires [Boost](https://www.boost.org/).

```bash
sudo apt install libboost-all-dev
cd simulation/MAPF_RHCR
cmake .
make
```

This produces the `lifelong` binary in the `MAPF_RHCR` directory.

---

## Usage

### 1. Convert map to RHCR format

```bash
python3 convert_map.py 1 9
```

### 2. Convert VRPPD paths to node locations

```bash
python3 convert_vrppd_path_to_node.py 1 9
```

### 3. Run the solver

```bash
python3 run_rhcr.py 1 9
```

### 4. Visualize results

```bash
python3 visualize_rhcr.py 1 9
```

### Output files

| File | Description |
|---|---|
| `*_rhcr_paths.txt` | Per-agent paths with goal and timing records |
| `RHCR_planning_time.txt` | Wall-clock planning time in s / ms / µs / ns |
| `RHCR_makespan.txt` | Total makespan in timesteps |

---

## References

[1] J. Li, A. Tinka, S. Kiesel, J. W. Durham, T. K. S. Kumar, and S. Koenig, "Lifelong Multi-Agent Path Finding in Large-Scale Warehouses," AAAI, 2021.

[2] J. Li, A. Tinka, S. Kiesel, J. W. Durham, T. K. S. Kumar, and S. Koenig, "Lifelong Multi-Agent Path Finding in Large-Scale Warehouses (extended abstract)," AAMAS, 2020.
