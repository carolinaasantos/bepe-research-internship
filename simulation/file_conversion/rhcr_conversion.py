# ------------------------------------------------------------------
# RHCR PATH CONVERTER
#
# This script converts complete ICBS trajectories into individual
# agent and task-oriented subpaths for MAPF integration.
#
# The conversion process:
# - Loads VRPPD goal sequences for each agent
# - Parses RHCR assigned_goal_paths output
# - Extracts agent trajectories
# - Identifies goal locations reached during execution
# - Splits trajectories into goal-to-goal subpaths
# ------------------------------------------------------------------

import argparse
import re
from typing import Dict, List, Optional, Tuple
from pathlib import Path

Point = Tuple[int, int]

# Path entry format: (location_id, (row, column), timestep)
PathEntry = Tuple[int, Point, int]

# --------------------------------------------------------------
# Regular expressions used to parse RHCR output files.
#
# AGENT_LINE_RE extracts:
# - Agent identifier
# - Assigned goals
# - Goal completion information
# - Complete path description
#
# PATH_POINT_RE extracts individual path entries encoded as:
# location[row,column]@timestep
# --------------------------------------------------------------

AGENT_LINE_RE = re.compile(
    r"^agent\s+(?P<agent>\d+),(?P<start>[^,]*),(?P<goals>[^,]*),(?P<reached_all>[^,]*),(?P<reached_times>[^,]*),path=(?P<path>.*)$"
)

PATH_POINT_RE = re.compile(r"(\d+)\[(\d+),(\d+)\]@(\d+);")

# --------------------------------------------------------------
# Safely converts a whitespace-separated string into integers.
#
# Returns:
# - List of integers if conversion succeeds
# - None if invalid values are encountered
# --------------------------------------------------------------

def _parse_ints(line: str) -> Optional[List[int]]:
    try:
        return list(map(int, line.split()))
    except ValueError:
        return None

# --------------------------------------------------------------
# Converts a flat coordinate sequence into point tuples.
#
# Example:
# [1, 2, 3, 4]
# ->
# [(1, 2), (3, 4)]
# --------------------------------------------------------------

def _pairwise_to_points(nums: List[int]) -> List[Point]:
    return [(nums[i], nums[i + 1]) for i in range(0, len(nums), 2)]

# --------------------------------------------------------------
# Loads VRPPD goal sequences.
#
# Supported formats:
#
# Format 1:
# x1 y1 x2 y2 ...
#
# Format 2:
# <agent_id>
# x1 y1 x2 y2 ...
#
# The output stores the ordered goal locations assigned to
# each agent.
# --------------------------------------------------------------

def read_goals(file: str) -> List[List[Point]]:
    agents: List[List[Point]] = []
    pending_agent: Optional[int] = None

    with open(file, "r") as f:
        for raw in f:
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue

            nums = _parse_ints(line)
            if nums is None:
                continue

            if len(nums) == 1:
                pending_agent = nums[0]
                continue

            if len(nums) % 2 == 1:
                # Prevent malformed files from generating invalid 
                # coordinate pairs
                nums = nums[:-1]
            if not nums:
                continue

            coords = _pairwise_to_points(nums)

            if pending_agent is None:
                agents.append(coords)
            else:
                # Supports both agent-indexed files and
                # files that start with the number of agents
                if not agents and pending_agent > 1:
                    agents.append(coords)
                else:
                    while len(agents) <= pending_agent:
                        agents.append([])
                    agents[pending_agent] = coords
                pending_agent = None

    return agents

# --------------------------------------------------------------
# Parses RHCR assigned_goal_paths output.
#
# For each agent, extracts:
# - Ordered goal identifiers
# - Complete path entries
#
# Each path entry contains:
# - Location identifier
# - Grid coordinates
# - Timestep
# --------------------------------------------------------------

def read_assigned_goal_paths(file: str) -> Dict[int, Dict[str, object]]:
    agents: Dict[int, Dict[str, object]] = {}

    with open(file, "r") as f:
        for raw in f:
            line = raw.strip()
            if not line.startswith("agent "):
                continue

            m = AGENT_LINE_RE.match(line)
            if not m:
                continue

            agent_id = int(m.group("agent"))
            goals_ids = [int(x) for x in m.group("goals").split("|") if x.strip()]

            entries: List[PathEntry] = []
            for loc, row, col, ts in PATH_POINT_RE.findall(m.group("path")):
                entries.append((int(loc), (int(row), int(col)), int(ts)))

            agents[agent_id] = {
                "goals_ids": goals_ids,
                "entries": entries,
            }

    return agents

# --------------------------------------------------------------
# Removes consecutive duplicate positions from a trajectory.
#
# Waiting actions are represented by repeated coordinates.
# This function preserves only actual movement transitions.
# --------------------------------------------------------------

def remove_repeated_consecutives(path: List[Point]) -> List[Point]:
    if not path:
        return path

    new = [path[0]]
    for p in path[1:]:
        if p != new[-1]:
            new.append(p)
    return new

# --------------------------------------------------------------
# Ensures that the goal sequence starts at the trajectory origin.
#
# If the first goal differs from the first trajectory position,
# the start location is prepended to the goal list.
# --------------------------------------------------------------

def normalize_goals(path: List[Point], goals: List[Point]) -> List[Point]:
    if not path:
        return []
    if not goals:
        return [path[0]]
    if goals[0] != path[0]:
        return [path[0]] + goals
    return goals

# --------------------------------------------------------------
# Verifies whether a goal sequence appears in the same order
# inside a trajectory.
#
# Returns True if all goals can be matched sequentially.
# --------------------------------------------------------------

def goals_in_order(path: List[Point], goals: List[Point]) -> bool:
    if not path or not goals:
        return False

    idx = 0
    for goal in goals[1:]:
        while idx < len(path) and path[idx] != goal:
            idx += 1
        if idx >= len(path):
            return False
        idx += 1
    return True

# --------------------------------------------------------------
# Reconstructs goal locations directly from RHCR assigned paths.
#
# This serves as a fallback mechanism when external VRPPD goals
# cannot be reliably matched to the trajectory.
# --------------------------------------------------------------

def goals_from_assigned(record: Dict[str, object]) -> List[Point]:
    entries: List[PathEntry] = record["entries"]
    goals_ids: List[int] = record["goals_ids"]

    if not entries:
        return []

    goals: List[Point] = [entries[0][1]]  # start
    i = 0
    for goal_id in goals_ids:
        while i < len(entries) and entries[i][0] != goal_id:
            i += 1
        if i >= len(entries):
            break
        goals.append(entries[i][1])
        i += 1
    return goals

# --------------------------------------------------------------
# Selects the most reliable goal sequence for an agent.
#
# Priority:
# 1. VRPPD goals
# 2. Coordinate-swapped VRPPD goals
# 3. Goals reconstructed from RHCR output
#
# This improves robustness against coordinate convention
# mismatches and malformed datasets.
# --------------------------------------------------------------

def choose_reference_goals(
    agent_id: int,
    path_clean: List[Point],
    goals_agents: List[List[Point]],
    record: Dict[str, object],
) -> List[Point]:
    candidates: List[List[Point]] = []

    if 0 <= agent_id < len(goals_agents) and goals_agents[agent_id]:
        ext = goals_agents[agent_id]
        candidates.append(ext)

        ext_swapped = [(y, x) for (x, y) in ext]
        if ext_swapped != ext:
            candidates.append(ext_swapped)

    for c in candidates:
        c_norm = normalize_goals(path_clean, c)
        if goals_in_order(path_clean, c_norm):
            return c_norm

    # Fallback to goals reconstructed directly from RHCR data
    return normalize_goals(path_clean, goals_from_assigned(record))

# --------------------------------------------------------------
# Splits a trajectory into goal-to-goal subpaths.
#
# The first point is considered the start location and is not
# treated as a splitting goal.
#
# A new subpath is generated whenever the next goal is reached.
# --------------------------------------------------------------

def divide_by_goals(path: List[Point], goals: List[Point]) -> List[List[Point]]:
    subpaths: List[List[Point]] = []
    current: List[Point] = []

    goal_idx = 1  # ignores the first point (start)

    for p in path:
        current.append(p)

        if goal_idx < len(goals) and p == goals[goal_idx]:
            subpaths.append(current)
            current = [p]
            goal_idx += 1

    return subpaths

# --------------------------------------------------------------
# Saves generated subpaths to an output file.
#
# For each agent:
# - Agent identifier is written first
# - Each subpath occupies one line
# - Blank lines separate agents
#
# The resulting format remains compatible with visualization
# and post-processing utilities.
# --------------------------------------------------------------

def save_output(file: str, agents_subpaths: Dict[int, List[List[Point]]]) -> None:
    with open(file, "w") as f:
        for agent_id in sorted(agents_subpaths.keys()):
            f.write(f"{agent_id}\n")
            for sub in agents_subpaths[agent_id]:
                line = " ".join(f"{x} {y}" for x, y in sub)
                f.write(line + "\n")
            f.write("\n")

# --------------------------------------------------------------
# Main execution routine.
#
# This function:
# - Parses command-line arguments
# - Locates the selected dataset instance
# - Loads VRPPD goal assignments
# - Loads RHCR assigned goal paths
# - Generates goal-to-goal subpaths
# - Saves the converted output file
# --------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Converte assigned_goal_paths RHCR em subpaths por goal."
    )
    parser.add_argument("num_example", help="Example number (ex: 1 for example1)")
    parser.add_argument("num_boxes", nargs="?", type=int, default=9, help="Number of boxes (default: 9)",)
    parser.add_argument("--boxes-dir", default=None, help="Folder inside solver/instances (ex.: 8_boxes, 9_boxes, 10_boxes)",)

    args = parser.parse_args()

    boxes_dir = args.boxes_dir or f"{args.num_boxes}_boxes"

    current_dir = Path(__file__).resolve().parent

    example_dir = (
        current_dir.parent
        / "solver"
        / "instances"
        / boxes_dir
        / f"example{args.num_example}"
    )

    file_goals = example_dir / f"example{args.num_example}_vrppd_paths.txt"
    file_paths = example_dir / f"example{args.num_example}_rhcr_paths.txt/rhcr_paths.txt"
    output_file = example_dir / f"example{args.num_example}_rhcr_converted_paths.txt"

    if not file_goals.exists():
        raise FileNotFoundError(f"File not found: {file_goals}")

    if not file_paths.exists():
        raise FileNotFoundError(f"File not found: {file_paths}")

    goals_agents = read_goals(file_goals)
    assigned = read_assigned_goal_paths(file_paths)

    agents_subpaths: Dict[int, List[List[Point]]] = {}

    for agent_id in sorted(assigned.keys()):
        record = assigned[agent_id]
        entries: List[PathEntry] = record["entries"]
        path = [coord for (_loc, coord, _t) in entries]
        path_clean = remove_repeated_consecutives(path)

        goals_ref = choose_reference_goals(agent_id, path_clean, goals_agents, record)
        subpaths = divide_by_goals(path_clean, goals_ref)

        agents_subpaths[agent_id] = subpaths

    save_output(output_file, agents_subpaths)

    print(f"File generated successfully in: {output_file}")


if __name__ == "__main__":
    main()
