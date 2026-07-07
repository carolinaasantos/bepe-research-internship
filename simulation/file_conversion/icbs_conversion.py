# ------------------------------------------------------------------
# ICBS PATH CONVERTER
#
# This script converts complete ICBS trajectories into individual
# agent and task-oriented subpaths for MAPF integration.
#
# The conversion process:
# - Loads the ordered goal sequence assigned to each agent
# - Loads the corresponding ICBS trajectory
# - Splits the trajectory whenever a goal is reached
# - Removes consecutive duplicate positions inside each segment
# ------------------------------------------------------------------

import argparse
from pathlib import Path

# --------------------------------------------------------------
# Loads the ordered goal sequence for each agent.
#
# The input file stores:
# - Agent identifiers on individual lines
# - Goal coordinates on the following line
#
# The output is a list where each element contains the ordered
# pickup and delivery locations assigned to one agent.
# --------------------------------------------------------------

def parse_goals(file_path):
    agent_goals = []
    current_goals = None

    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()

            if not line:
                continue

            parts = line.split()

            if len(parts) == 1:
                if current_goals is not None:
                    agent_goals.append(current_goals)
                current_goals = []
            else:
                nums = list(map(int, parts))
                goals = [(nums[i], nums[i+1]) for i in range(0, len(nums), 2)]
                current_goals.extend(goals)

        if current_goals is not None:
            agent_goals.append(current_goals)

    return agent_goals

# --------------------------------------------------------------
# Loads ICBS trajectories for all agents.
#
# Each line contains a complete path represented as a sequence
# of coordinate pairs:
# x1 y1 x2 y2 x3 y3 ...
#
# The output is a list containing one trajectory per agent.
# --------------------------------------------------------------

def parse_paths(file_path):
    agent_paths = []

    with open(file_path, 'r') as f:
        for line in f:
            nums = list(map(int, line.strip().split()))
            path = [(nums[i], nums[i+1]) for i in range(0, len(nums), 2)]
            agent_paths.append(path)

    return agent_paths

# --------------------------------------------------------------
# Removes consecutive duplicate positions from a path.
#
# This is useful when paths contain waiting actions represented
# by repeated coordinates. Only changes of position are kept.
# --------------------------------------------------------------

def remove_repeated_consecutives(path):
    if not path:
        return path

    new_path = [path[0]]

    for waypoint in path[1:]:
        if waypoint != new_path[-1]:
            new_path.append(waypoint)

    return new_path

# --------------------------------------------------------------
# Splits a complete ICBS trajectory into multiple subpaths.
#
# The trajectory is divided whenever the agent reaches one of
# its assigned VRPPD goals. Each generated segment corresponds
# to a task execution stage between consecutive goals.
#
# Consecutive duplicate positions are removed before storing
# each subpath.
# --------------------------------------------------------------

def split_path(path, goals):
    subpaths = []
    current = []

    goal_idx = 1

    for waypoint in path:
        current.append(waypoint)

        if goal_idx < len(goals) and waypoint == goals[goal_idx]:
            current_clean = remove_repeated_consecutives(current)
            subpaths.append(current_clean)

            current = [waypoint]
            goal_idx += 1

    return subpaths

# --------------------------------------------------------------
# Saves the generated subpaths to an output file.
#
# For each agent:
# - The agent identifier is written first
# - Each subpath is stored on a separate line
# - Blank lines separate different agents
#
# The coordinate format remains compatible with the remaining
# ICBS and visualization utilities.
# --------------------------------------------------------------

def save_output(output_path, agents_subpaths):
    with open(output_path, 'w') as f:
        for i, subpaths in enumerate(agents_subpaths):
            f.write(f"{i}\n")

            for sp in subpaths:
                line = " ".join(f"{x} {y}" for x, y in sp)
                f.write(f"{line}\n")

            f.write("\n")

# --------------------------------------------------------------
# Main execution routine.
#
# This function:
# - Parses command-line arguments
# - Locates the selected dataset instance
# - Loads VRPPD goals and ICBS trajectories
# - Generates task-level subpaths
# - Saves the converted output file
# --------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Converts vrppd_paths + concatenated_paths to ICBS subpaths."
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

    goals_file = example_dir / f"example{args.num_example}_vrppd_paths.txt"
    paths_file = example_dir / f"example{args.num_example}_icbs_paths.txt"
    output_file = example_dir / f"example{args.num_example}_icbs_converted_paths.txt"

    if not goals_file.exists():
        raise FileNotFoundError(f"File not found: {goals_file}")

    if not paths_file.exists():
        raise FileNotFoundError(f"File not found: {paths_file}")

    agents_goals = parse_goals(goals_file)
    agents_paths = parse_paths(paths_file)

    agents_subpaths = []

    for goals, path in zip(agents_goals, agents_paths):
        subpaths = split_path(path, goals)
        agents_subpaths.append(subpaths)

    save_output(output_file, agents_subpaths)

    print(f"File generated successfully: {output_file}")


if __name__ == "__main__":
    main()