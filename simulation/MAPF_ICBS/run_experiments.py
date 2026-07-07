# ------------------------------------------------------------------
# Adapted from MAPF-ICBS.
#
# Original source:
# https://github.com/gloriyo/MAPF-ICBS
#
# Modifications in this file:
# - Added a timeout mechanism using Python's multiprocessing (`mp.Process`)
#   inside `solve_once` to safely terminate solvers that exceed the time limit.
# - Added comprehensive support for VRPPD (Vehicle Routing Problem with 
#   Pickup and Delivery), including segment-by-segment fast planning 
#   (`solve_segment_fast`) and dynamic map updates for box manipulation.
# - Added custom path management utilities: `parse_vrppd_paths`, 
#   `concatenate_paths`, `pad_paths_to_same_length`, and `save_concatenated_paths`.
# - Added a dynamic traverse restriction system tied to an external 
#   module (`a_star_runtime.robot_can_traverse`).
# - Added a shortcut mode feature (`apply_shortcut_mode` and `resolve_example_dir`)
#   to auto-configure file paths, instances, and solvers based on positional arguments.
# - Added automated execution time tracking for the ICBS solver, saving metrics
#   in seconds, milliseconds, microseconds, and nanoseconds to `ICBS_planning_time.txt`.
# - Updated the CLI argument parsing with `argparse` to accept positional arguments
#   (`num_example`, `num_boxes`) and new optional flags (`--boxes-dir`, 
#   `--vrppd_paths`, `--solver_verbose`, `--output`).
# - Updated path resolution using `pathlib.Path` with absolute and relative 
#   project root handling (`PROJECT_ROOT`).
# - Refactored solver instantiation into a dedicated helper function (`build_solver`).
# ------------------------------------------------------------------

import argparse
import glob
import os
import contextlib
import time
from pathlib import Path
import sys

# --------------------------------------------------------------
# Project root dynamic resolution
#
# Establishes the absolute path to the project root directory.
#
# This allows the script to safely append internal modules (like 'code')
# to the system path and resolve relative file locations independently 
# of where the execution command was triggered.
# --------------------------------------------------------------

PROJECT_ROOT = Path(__file__).resolve().parent

sys.path.append(str(PROJECT_ROOT / "code"))

from cbs_basic import CBSSolver # original cbs with standard/disjoint splitting

# cbs with different improvements
from icbs_cardinal_bypass import ICBS_CB_Solver # only cardinal dectection and bypass
from icbs_complete import ICBS_Solver # all improvements including MA-CBS

#from independent import IndependentSolver
#from prioritized import PrioritizedPlanningSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost
import a_star_class as a_star_runtime

import multiprocessing as mp

HLSOLVER = "CBS"
LLSOLVER = "a_star"

def _run_solver(queue, solver, disjoint):
    try:
        solution = solver.find_solution(disjoint)
        queue.put(solution)
    except Exception as e:
        queue.put(e)

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")

    f = open(filename, 'r')

    line = f.readline()
    rows, columns = [int(x) for x in line.split()]
    rows = int(rows)
    columns = int(columns)

    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)

    line = f.readline()
    num_agents = int(line)

    starts = []
    goals = []

    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))

    f.close()

    return my_map, starts, goals


def build_solver(my_map, starts, goals, hl_solver, agent_ids=None):
    if hl_solver == "CBS":
        print("***Run CBS***")
        return CBSSolver(my_map, starts, goals)
    if hl_solver == "ICBS_CB":
        print("***Run ICBS with CB***")
        return ICBS_CB_Solver(my_map, starts, goals)
    if hl_solver == "ICBS":
        print("***Run ICBS***")
        return ICBS_Solver(my_map, starts, goals, agent_ids=agent_ids)
    raise RuntimeError("Unknown solver!")


# --------------------------------------------------------------
# Subprocess execution with strict timeout
#
# Spawns a dedicated multiprocessing pool to execute the MAPF solver.
#
# If the execution time exceeds the specified timeout threshold, 
# the process is forcefully terminated to prevent infinite loops 
# or severe slowdowns in heavy instances.
#
# Returns:
# - Planned paths, generated nodes, and expanded nodes if successful
# - None and zeroed metrics if a timeout or internal error occurs
# --------------------------------------------------------------

def solve_once(
    my_map,
    starts,
    goals,
    hl_solver,
    disjoint,
    solver_verbose=False,
    agent_ids=None,
    timeout=5
):
    solver = build_solver(my_map, starts, goals, hl_solver, agent_ids=agent_ids)

    # Queue to receive the result of the process
    q = mp.Queue()

    # Separate process running the solver
    p = mp.Process(target=_run_solver, args=(q, solver, disjoint))
    p.start()
    p.join(timeout)

    # If it's still running → timeout (infinite loop or very slow)
    if p.is_alive():
        print("Solution not found: Timeout")
        p.terminate()
        p.join()
        return None, 0, 0

    # If it's finished, get the result
    if q.empty():
        return None, 0, 0

    result = q.get()

    # If an error occurred within the solver
    if isinstance(result, Exception):
        print("Exception", result)
        return None, 0, 0

    if result is None:
        return None, 0, 0

    paths, nodes_gen, nodes_exp = [result[i] for i in range(3)]

    if paths is None:
        return None, 0, 0

    return paths, nodes_gen, nodes_exp

# --------------------------------------------------------------
# VRPPD path parsing and validation
#
# Reads and decodes sequential agent waypoints from a VRPPD file.
#
# Supports both legacy sequential coordinates and modern formats 
# containing explicit agent identifiers. Validates formatting symmetry 
# and ensures the agent count aligns with the map instance.
#
# Returns:
# - A list of start coordinates per agent
# - A list of lists containing sequential waypoints per agent
# --------------------------------------------------------------

def parse_vrppd_paths(vrppd_file, num_agents):
    f = Path(vrppd_file)
    if not f.is_file():
        raise BaseException(vrppd_file + " does not exist.")
    agent_starts = {}
    agent_waypoints = {}
    with open(vrppd_file, "r") as fh:
        lines = [raw.split("#", 1)[0].strip() for raw in fh]
        lines = [l for l in lines if l]

    # Detect format: new format has single-integer lines as agent IDs
    has_agent_ids = any(len(l.split()) == 1 and l.lstrip('-').isdigit() for l in lines)

    if has_agent_ids:
        i = 0
        while i < len(lines):
            vals = lines[i].split()
            if len(vals) == 1:
                agent_id = int(vals[0])
                i += 1
                if i >= len(lines):
                    raise BaseException(f"Agent {agent_id} has no path line.")
                path_vals = [int(x) for x in lines[i].split()]
                if len(path_vals) < 4 or len(path_vals) % 2 != 0:
                    if len(path_vals) == 2:
                        raise BaseException(
                            f"Agent {agent_id} has only start position and no waypoints. "
                            "If this is intentional, please provide a path line with repeated start: sx sy sx sy"
                        )
                    raise BaseException("Each path line must have even count >= 4: " + lines[i])
                coords = [(path_vals[j], path_vals[j + 1]) for j in range(0, len(path_vals), 2)]
                agent_starts[agent_id] = coords[0]
                agent_waypoints[agent_id] = coords[1:]
            else:
                raise BaseException("Expected agent ID line, got: " + lines[i])
            i += 1
        starts = [agent_starts[a] for a in sorted(agent_starts)]
        waypoints = [agent_waypoints[a] for a in sorted(agent_waypoints)]
    else:
        starts = []
        waypoints = []
        for line in lines:
            vals = [int(x) for x in line.split()]
            if len(vals) < 4 or len(vals) % 2 != 0:
                raise BaseException("Each line in vrppd_paths must have even count >= 4: " + line)
            coords = [(vals[i], vals[i + 1]) for i in range(0, len(vals), 2)]
            starts.append(coords[0])
            waypoints.append(coords[1:])

    if len(starts) != num_agents:
        raise BaseException(
            f"vrppd_paths has {len(starts)} agents, but instance has {num_agents} agents."
        )
    return starts, waypoints

def concatenate_paths(total_paths, segment_paths):
    if total_paths is None:
        return [list(p) for p in segment_paths]
    for a in range(len(total_paths)):
        if len(segment_paths[a]) > 1:
            total_paths[a].extend(segment_paths[a][1:])
    return total_paths

def pad_paths_to_same_length(paths):
    max_len = max(len(p) for p in paths)
    out = []
    for p in paths:
        q = list(p)
        while len(q) < max_len:
            q.append(q[-1])
        out.append(q)
    return out

# --------------------------------------------------------------
# Segment-based fast path planning
#
# Computes localized paths for active agents towards their next waypoint.
#
# To optimize performance, idle agents (already at their destination) 
# are temporarily injected into the map matrix as static obstacles, 
# narrowing down the search space for the active moving subset.
#
# Returns:
# - Padded synchronous paths for all agents in the current segment
# - Subproblem generation and expansion search metrics
# --------------------------------------------------------------

def solve_segment_fast(my_map, current, step_goals, hl_solver, disjoint, solver_verbose=False):
    n = len(current)
    active = [i for i in range(n) if current[i] != step_goals[i]]
    if not active:
        idle_paths = [[current[i]] for i in range(n)]
        return idle_paths, 0, 0

    # Build subproblem only with active agents.
    # Inactive agents are treated as static obstacles for this segment.
    blocked = {current[i] for i in range(n) if i not in active}
    sub_map = [row[:] for row in my_map]
    for r, c in blocked:
        sub_map[r][c] = True

    sub_starts = [current[i] for i in active]
    sub_goals = [step_goals[i] for i in active]
    sub_paths, nodes_gen, nodes_exp = solve_once(
        sub_map, sub_starts, sub_goals, hl_solver, disjoint, solver_verbose, agent_ids=active
    )

    if sub_paths is None:
        print("Timeout ou sem solução no segmento!")
        return None, 0, 0

    sub_paths = pad_paths_to_same_length(sub_paths)
    seg_len = max(len(p) for p in sub_paths)

    full_paths = []
    active_idx = {agent: k for k, agent in enumerate(active)}
    for i in range(n):
        if i in active_idx:
            full_paths.append(sub_paths[active_idx[i]])
        else:
            full_paths.append([current[i]] * seg_len)
    return full_paths, nodes_gen, nodes_exp

def clone_traverse_map(traverse_map):
    return {agent: set(cells) for agent, cells in traverse_map.items()}

def build_traverse_map_from_vrppd(starts, waypoints):
    traverse_map = {}
    for agent_id, (start, agent_waypoints) in enumerate(zip(starts, waypoints)):
        allowed = {start}
        allowed.update(agent_waypoints)
        traverse_map[agent_id] = allowed
    return traverse_map

# --------------------------------------------------------------
# Runtime traversal matrix restriction
#
# Enforces kinematic and logistical boundaries on the A* runtime state.
#
# Dynamically configures which grid cells are traversable by specific 
# robots during the current segment execution, preventing agents from 
# crossing unauthorized zones or colliding with blocked structural cells.
# --------------------------------------------------------------

def set_runtime_traverse_for_segment(base_traverse, current, segment_map):
    # Static obstacles are blocked for everyone except declared traversable cells.
    # If an agent starts on a currently blocked cell (e.g., just delivered),
    # allow only that agent to leave this cell in the next segment.
    runtime_traverse = clone_traverse_map(base_traverse)
    for agent, start in enumerate(current):
        if segment_map[start[0]][start[1]]:
            runtime_traverse.setdefault(agent, set()).add(start)
    a_star_runtime.robot_can_traverse = runtime_traverse

# --------------------------------------------------------------
# Dynamic pickup and delivery grid updates
#
# Alters the grid topology dynamically based on task completion.
#
# Simulates physical object interaction by alternating cell blockages:
# - Reaching a pickup spot removes a box (clears grid obstacle)
# - Reaching a delivery spot places a box (creates grid obstacle)
# --------------------------------------------------------------

def apply_pickup_delivery_updates(segment_map, step_goals, has_goal_for_agent, segment_idx):
    # Goal index is 0-based: 0,2,4... are pickups (open); 1,3,5... are deliveries (close).
    for a, goal in enumerate(step_goals):
        if not has_goal_for_agent[a]:
            continue
        gx, gy = goal
        if segment_idx % 2 == 0:  # pickup reached -> remove box
            segment_map[gx][gy] = False
        else:  # delivery reached -> place box
            segment_map[gx][gy] = True

def save_robot_moves(moves_file, paths):
    for path in paths:
        line_moves = []
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            line_moves.extend([str(x1), str(y1), str(x2), str(y2)])
        moves_file.write(" ".join(line_moves) + "\n")

def save_concatenated_paths(output_file, paths):
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", buffering=1) as f:
        for path in paths:
            vals = []
            for x, y in path:
                vals.extend([str(x), str(y)])
            f.write(" ".join(vals) + "\n")

def resolve_output_path(path_str):
    p = Path(path_str)

    if p.is_absolute():
        return p

    return PROJECT_ROOT / p

def resolve_example_dir(project_root: Path, number: str, boxes_dir: str) -> Path:
    base = project_root.parent / "solver" / "instances"

    with_boxes = base / boxes_dir / f"example{number}"
    legacy = base / f"example{number}"

    if with_boxes.exists():
        return with_boxes

    if legacy.exists():
        return legacy

    return with_boxes

# --------------------------------------------------------------
# CLI execution shortcut injector
#
# Automatically infers configuration paths based on minimal parameters.
#
# If a specific example number and box directory structure are provided, 
# it auto-resolves the target map instance, expected VRPPD routes, 
# and output paths, enabling rapid batch benchmarks.
# --------------------------------------------------------------

def apply_shortcut_mode(args):
    if args.num_example is None:
        return

    boxes_dir = args.boxes_dir or f"{args.num_boxes}_boxes"
    project_root = PROJECT_ROOT
    example_dir = resolve_example_dir(project_root, args.num_example, boxes_dir)

    if args.instance is None:
        args.instance = str(example_dir / f"example{args.num_example}_icbs_map.txt")
    if args.vrppd_paths is None:
        args.vrppd_paths = str(example_dir / f"example{args.num_example}_vrppd_paths.txt")
    if args.output is None:
        args.output = str(example_dir / f"example{args.num_example}_icbs_paths.txt")
    if args.hlsolver is None:
        args.hlsolver = "ICBS"
    if not args.batch:
        args.batch = True

# ----------------------------------------------------------
# ICBS configuration
#
# The command defines:
# - Scenario type
# - Specific instance file(s)
# - Batch execution mode
# - High-level solver (ICBS)
# - VRPPD input paths file
# - Solver debug logging
# - Output file for generated ICBS paths
# ----------------------------------------------------------

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')

    parser.add_argument("num_example", nargs="?",
                        help="Example number (ex: 1 for example1)")
    parser.add_argument("num_boxes", nargs="?", type=int, default=9,
                        help="Number of boxes (defines the folder <num boxes>boxes by default)")
    parser.add_argument('--boxes-dir', type=str, default=None,
                        help='Folder inside solver/instances (ex.: 8_boxes, 10_boxes)')

    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')

    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')

    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')

    parser.add_argument('--hlsolver', type=str, default=None,
                        help='The solver to use (CBS,ICBS_CB,ICBS)')
    parser.add_argument('--vrppd_paths', type=str, default=None,
                        help='txt with one line per agent: sx sy gx1 gy1 gx2 gy2 ...')
    parser.add_argument('--solver_verbose', action='store_true', default=False,
                        help='print internal solver debug logs')
    parser.add_argument('--output', type=str, default=None,
                        help='output file for concatenated ICBS paths when using --vrppd_paths')

    args = parser.parse_args()
    apply_shortcut_mode(args)
    if args.hlsolver is None:
        args.hlsolver = HLSOLVER
    if args.output is None:
        args.output = "solver/instances/icbs_paths.txt"

    result_file = open(PROJECT_ROOT / "results.csv", "w", buffering=1)
    nodes_gen_file = open(PROJECT_ROOT / "nodes-gen-cleaned.csv", "w", buffering=1)
    nodes_exp_file = open(PROJECT_ROOT / "nodes-exp-cleaned.csv", "w", buffering=1)

    if args.batch:
        if args.instance:
            input_instance = sorted(glob.glob(args.instance))
        else:
            input_instance = sorted(glob.glob("instances/test*"))
    else:
        input_instance = sorted(glob.glob(args.instance))

    for file in input_instance:

        print("***Import an instance***")

        print(file)

        my_map, starts, goals = import_mapf_instance(file)

        print_mapf_instance(my_map, starts, goals)
        paths = None
        final_starts = starts
        final_goals = goals
        total_nodes_gen = 0
        total_nodes_exp = 0
        planning_time_ns = 0

        if args.vrppd_paths:
            print("***Run iterative VRPPD goals***")
            starts, waypoints = parse_vrppd_paths(args.vrppd_paths, len(starts))
            current = list(starts)
            all_paths = None
            segment_idx = 0
            dynamic_map = [row[:] for row in my_map]
            base_traverse = build_traverse_map_from_vrppd(starts, waypoints)
            planning_start_ns = time.perf_counter_ns()
            while True:
                step_goals = []
                has_pending = False
                has_goal_for_agent = [False] * len(current)
                for a in range(len(current)):
                    if segment_idx < len(waypoints[a]):
                        step_goals.append(waypoints[a][segment_idx])
                        has_pending = True
                        has_goal_for_agent[a] = True
                    else:
                        step_goals.append(current[a])
                if not has_pending:
                    break

                print(f"*** VRPPD segment {segment_idx}: solve current -> next waypoint ***")
                set_runtime_traverse_for_segment(base_traverse, current, dynamic_map)
                seg_paths, nodes_gen, nodes_exp = solve_segment_fast(
                    dynamic_map, current, step_goals, args.hlsolver, args.disjoint, args.solver_verbose
                )

                if seg_paths is None:
                    print("Segment failed (timeout). Aborting instance.")
                    break

                total_nodes_gen += nodes_gen
                total_nodes_exp += nodes_exp
                all_paths = concatenate_paths(all_paths, seg_paths)
                current = [p[-1] for p in seg_paths]
                apply_pickup_delivery_updates(dynamic_map, step_goals, has_goal_for_agent, segment_idx)
                segment_idx += 1
            # Keep module state clean for future runs in the same Python process
            a_star_runtime.robot_can_traverse = clone_traverse_map(base_traverse)

            if all_paths is None:
                raise BaseException("VRPPD failed: no segment was successfully planned (timeout or failure).")
            paths = all_paths
            final_starts = starts
            final_goals = current
            planning_time_ns = time.perf_counter_ns() - planning_start_ns
            output_path = resolve_output_path(args.output)
            print(f"Saving ICBS paths to: {output_path}")
            save_concatenated_paths(output_path, paths)
        else:
            planning_start_ns = time.perf_counter_ns()
            paths, nodes_gen, nodes_exp = solve_once(
                my_map, starts, goals, args.hlsolver, args.disjoint, args.solver_verbose
            )
            planning_time_ns = time.perf_counter_ns() - planning_start_ns
            total_nodes_gen = nodes_gen
            total_nodes_exp = nodes_exp

        # ----------------------------------------------------------
        # Convert runtime to multiple time units for reporting
        # ----------------------------------------------------------

        if args.hlsolver == "ICBS":
            planning_time_s = planning_time_ns / 1_000_000_000
            planning_time_ms = planning_time_ns / 1_000_000
            planning_time_us = planning_time_ns / 1_000

            msg = (
                "Tempo total de planejamento (ICBS): "
                f"{planning_time_s:.9f} s "
                f"({planning_time_ms:.6f} ms | {planning_time_us:.3f} us | {planning_time_ns} ns)"
            )

            print(msg)

            output_dir = Path(args.output).parent
            planning_file = output_dir / "ICBS_planning_time.txt"

            with open(planning_file, "a", encoding="utf-8") as f:
                f.write(msg + "\n")

        cost = get_sum_of_cost(paths)
        result_file.write("{},{}\n".format(file, cost))
        nodes_gen_file.write("{},{}\n".format(file, total_nodes_gen))
        nodes_exp_file.write("{},{}\n".format(file, total_nodes_exp))

        if not args.batch:

            print("***Test paths on a simulation***")
            if args.vrppd_paths:
                _, goal_sequences = parse_vrppd_paths(args.vrppd_paths, len(final_starts))
                animation = Animation(my_map, final_starts, paths, goal_sequences=goal_sequences)
            else:
                animation = Animation(my_map, final_starts, paths)

            animation.show()

    result_file.close()
    nodes_gen_file.close()
    nodes_exp_file.close()