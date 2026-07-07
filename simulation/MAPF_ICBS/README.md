# README: MAPF-ICBS

## Overview

This module adapts the **Improved Conflict-Based Search (ICBS)** algorithm to support **VRPPD (Vehicle Routing Problem with Pickup and Delivery)** scenarios. Robots navigate a grid map where boxes are represented as obstacle cells. Pickup events open those cells and delivery events close them again, so the set of traversable cells changes dynamically during planning.

The solver receives a sequence of waypoints per robot (produced by the VRPPD solver), resolves conflicts segment by segment using ICBS, and outputs the concatenated collision-free paths used by the rest of the pipeline.

**Original source:** https://github.com/gloriyo/MAPF-ICBS

---

## Modified Files

### `run_experiments.py`

The main entry point. Significant additions were made to support VRPPD workflows on top of the original single-instance batch runner.

- **Timeout mechanism** (`solve_once`): the solver is now launched in a separate `multiprocessing.Process` so that instances that would run indefinitely are safely terminated after a configurable time limit.
- **VRPPD segment-by-segment planning** (`solve_segment_fast`): instead of solving a single start→goal problem, the solver iterates through the waypoint sequence. At each step, only active agents (those that still have a pending waypoint) are included in the ICBS subproblem; idle agents are treated as static obstacles. After each segment the map is updated to reflect pickup (box removed) or delivery (box placed) events.
- **Path management utilities**: `parse_vrppd_paths` reads the per-agent waypoint file; `concatenate_paths` joins segment results into a single path per agent; `pad_paths_to_same_length` aligns path lengths within a segment; `save_concatenated_paths` writes the final output.
- **Dynamic traverse restriction** (`set_runtime_traverse_for_segment`): before each segment, the module writes the current per-agent set of allowed obstacle cells into `a_star_class.robot_can_traverse`, so the A\* low-level planner knows which obstacle cells each agent is allowed to enter as a goal.
- **Shortcut mode** (`apply_shortcut_mode`, `resolve_example_dir`): positional CLI arguments (`num_example`, `num_boxes`) are enough to auto-resolve all file paths, the instance file, and the solver type, removing the need to pass every flag explicitly.
- **Planning time tracking**: when the ICBS solver is used, total wall-clock planning time is recorded in seconds, milliseconds, microseconds, and nanoseconds and appended to `ICBS_planning_time.txt` in the example directory.
- **CLI updates**: new positional arguments (`num_example`, `num_boxes`) and optional flags (`--boxes-dir`, `--vrppd_paths`, `--solver_verbose`, `--output`). Path resolution uses `pathlib.Path` and a `PROJECT_ROOT` constant. Solver instantiation was extracted into a `build_solver` helper.

### `code/a_star_class.py`

The low-level A\* planner used by all CBS variants. Several additions were made to support agent-specific dynamic obstacle access.

- **`robot_can_traverse` dict**: a module-level dictionary mapping each agent ID to the set of obstacle cells that agent is allowed to enter. It is written at runtime by `run_experiments.py` before each planning segment.
- **`is_blocked()` function**: centralises all obstacle checking. A cell is passable only when (1) it is not an obstacle in the static map, or (2) it belongs to `robot_can_traverse` for the querying agent *and* it is exactly that agent's current search goal. This prevents agents from routing through other agents' reserved pickup/delivery cells as shortcuts.
- **`compute_heuristics()` updated**: now accepts an `agent_id` parameter and calls `is_blocked()` instead of accessing `my_map[x][y]` directly, so heuristic distances respect the same traversability rules applied during path expansion.
- **`A_Star.agent_id_map`**: a new parameter in `A_Star.__init__()` that maps local meta-agent indices back to the original global agent IDs. This is needed after MA-CBS merges agents, since the local position inside `self.agents` no longer corresponds to the original ID used by `robot_can_traverse`.
- **Safe heuristic lookup** (`.get(loc, 0)`): dynamic obstacle permissions can leave some cells outside the precomputed heuristic table. Defaulting to 0 avoids `KeyError` and keeps the search running.

### `code/icbs_complete.py`

The full ICBS solver, which includes cardinal conflict detection, bypass, and MA-CBS with merge-and-restart.

- **`agent_ids` parameter**: added to `ICBS_Solver.__init__()` and stored as `self.agent_ids`. The list is forwarded through all `A_Star` constructor calls via `agent_id_map=self.agent_ids`, ensuring that per-agent traversable overrides survive meta-agent merges and remain associated with the correct robots.
- **`compute_heuristics()` calls**: updated to pass the real agent ID so that heuristics are computed under the same dynamic obstacle permissions used during actual path search.

---

## Files Without Modifications

The following files were taken from the original repository without changes:

| File | Description |
|---|---|
| `code/cbs_basic.py` | Original CBS solver with standard and disjoint splitting |
| `code/icbs_cardinal_bypass.py` | CBS with cardinal conflict detection and bypass only |
| `code/single_agent_planner.py` | Original single-agent A\* planner (used by CBS variants) |

---

## New Files

### `convert_map.py`

Converts a grid map file (`*_map_input.txt`) and a VRPPD paths file (`*_vrppd_paths.txt`) into the ICBS instance format (`*_icbs_map.txt`). The map grid is preserved as-is; agent paths are reduced to (start, goal) pairs for the first segment.

```
python3 convert_map.py <num_example> [num_boxes] [--boxes-dir <dir>]
```

### `visualize.py`

Standalone visualizer for VRPPD paths produced by ICBS. Extends the original animation class with:

- **Multiple goals per agent**: all pickup and delivery waypoints are rendered as colored squares on the map.
- **Dynamic obstacles**: box cells appear and disappear in sync with pickup and delivery events along each agent's path.
- **Makespan output**: total animation duration is printed and saved to `ICBS_makespan.txt`.

```
python3 visualize.py <num_example> [num_boxes] [--boxes-dir <dir>] [--save <file>] [--not-show]
```

---

## Usage

### Dependencies

Install with `pipenv`:

```bash
pipenv install --dev
```

### Convert map to ICBS format

```bash
python3 convert_map.py 1 9
```

### Run ICBS with VRPPD paths

```bash
python3 run_experiments.py 1 9
```

This auto-resolves the instance file, VRPPD paths file, and output path from the example directory. Equivalent explicit form:

```bash
python3 run_experiments.py \
  --instance ../solver/instances/9_boxes/example1/example1_icbs_map.txt \
  --vrppd_paths ../solver/instances/9_boxes/example1/example1_vrppd_paths.txt \
  --hlsolver ICBS \
  --output ../solver/instances/9_boxes/example1/example1_icbs_paths.txt \
  --batch
```

### Visualize results

```bash
python3 visualize.py 1 9
```

### Output files

| File | Description |
|---|---|
| `*_icbs_paths.txt` | Concatenated collision-free paths, one agent per line |
| `ICBS_planning_time.txt` | Wall-clock planning time in s / ms / µs / ns |
| `ICBS_makespan.txt` | Total makespan in timesteps |