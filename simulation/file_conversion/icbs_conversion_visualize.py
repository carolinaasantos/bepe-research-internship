# ------------------------------------------------------------------
# ICBS PATH VISUALIZATION CONVERTER
#
# This script converts ICBS-generated paths into a visualization-
# friendly format by introducing pauses at task completion points.
#
# The conversion process combines:
# - ICBS agent trajectories
# - VRPPD goal locations
#
# Whenever an agent reaches a goal location (except the first one),
# the position is repeated multiple times to create a visible pause
# during animation and trajectory playback.
# ------------------------------------------------------------------

import argparse
from pathlib import Path

# --------------------------------------------------------------
# Loads ICBS agent paths from a text file.
#
# Each line represents a single agent path encoded as a sequence
# of coordinate pairs:
# x1 y1 x2 y2 x3 y3 ...
#
# The output is a list containing the ordered path of each agent.
# --------------------------------------------------------------

def read_icbs_file(path):
    agents_paths = []
    with open(path, 'r') as f:
        for line in f:
            nums = list(map(int, line.strip().split()))
            points = [(nums[i], nums[i+1]) for i in range(0, len(nums), 2)]
            agents_paths.append(points)
    return agents_paths

# --------------------------------------------------------------
# Loads VRPPD goal sequences for each agent.
#
# The file stores:
# - Agent identifier
# - Ordered list of goal coordinates
#
# The output maps each agent ID to the sequence of pickup and
# delivery locations assigned to that agent.
# --------------------------------------------------------------

def read_vrppd_file(path):
    agents_goals = {}
    with open(path, 'r') as f:
        lines = [line.strip() for line in f if line.strip()]

    i = 0
    while i < len(lines):
        agent_id = int(lines[i])
        goals_line = list(map(int, lines[i+1].split()))
        goals = [(goals_line[j], goals_line[j+1]) for j in range(0, len(goals_line), 2)]
        agents_goals[agent_id] = goals
        i += 2

    return agents_goals

# --------------------------------------------------------------
# Inserts waiting periods into ICBS paths at VRPPD goal locations.
#
# For each agent:
# - The path is traversed sequentially
# - Goal locations are matched in order
# - Every goal after the first one is repeated multiple times
#
# This produces smoother visualizations by highlighting moments
# where pickups and deliveries occur.
# --------------------------------------------------------------

def process_paths(icbs_paths, vrppd_goals, repeat=5):
    new_paths = []

    for agent_id, path in enumerate(icbs_paths):
        goals = vrppd_goals.get(agent_id, [])
        goal_index = 0
        new_path = []

        for point in path:
            new_path.append(point)

            # Repeat the current position when a goal is reached,
            # except for the first goal in the sequence
            if goal_index < len(goals) and point == goals[goal_index]:
                if goal_index > 0:
                    for _ in range(repeat - 1):
                        new_path.append(point)
                goal_index += 1

        new_paths.append(new_path)

    return new_paths

# --------------------------------------------------------------
# Writes the processed paths to an output file.
#
# Each path is flattened into a sequence of coordinates using
# the same format expected by visualization tools:
# x1 y1 x2 y2 x3 y3 ...
# --------------------------------------------------------------

def write_output(paths, output_file):
    with open(output_file, 'w') as f:
        for path in paths:
            flat = []
            for x, y in path:
                flat.extend([x, y])
            f.write(' '.join(map(str, flat)) + '\n')

# --------------------------------------------------------------
# Main execution routine.
#
# This function:
# - Parses command-line arguments
# - Locates the corresponding example directory
# - Loads ICBS and VRPPD data files
# - Generates visualization-friendly paths
# - Saves the converted output file
# --------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Converts ICBS paths for visualization")
    parser.add_argument("num_example", help="Example number (ex: 1 for example1)")
    parser.add_argument("num_boxes", nargs="?", type=int, default=9, help="Number of boxes (default: 9)")
    parser.add_argument("--boxes-dir", default=None, help="Folder inside solver/instances (optional)")
    parser.add_argument("--repeat", type=int, default=5, help="Number of repetitions of each goal (default=5)")
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

    icbs_file = example_dir / f"example{args.num_example}_icbs_paths.txt"
    vrppd_file = example_dir / f"example{args.num_example}_vrppd_paths.txt"
    output_file = example_dir / f"example{args.num_example}_icbs_visualize.txt"

    if not icbs_file.exists() or not vrppd_file.exists():
        raise FileNotFoundError("ICBS or VRPPD files not found.")

    icbs_paths = read_icbs_file(icbs_file)
    vrppd_goals = read_vrppd_file(vrppd_file)

    new_paths = process_paths(icbs_paths, vrppd_goals, repeat=args.repeat)
    write_output(new_paths, output_file)

    print(f"File generated successfully: {output_file}")

if __name__ == "__main__":
    main()