# ------------------------------------------------------------------
# ICBS MAP INPUT CONVERTER
#
# This script converts a grid map and VRPPD-based agent paths into
# a format compatible with ICBS.
#
# The conversion process combines:
# - Grid map representation (obstacles and free cells)
# - VRPPD path definitions containing agent start and goal pairs
#
# The map is preserved in its original structure, while agent paths
# are reduced to (start, goal) pairs. Each agent is extracted from
# the VRPPD file and mapped into the ICBS input format, which
# requires explicit start and goal coordinates per agent.
# ------------------------------------------------------------------

import argparse
from pathlib import Path

# --------------------------------------------------------------
# Map loader
#
# This module reads a grid map file used in ICBS/VRPPD instances.
#
# Expected format:
# - First line: <rows> <cols>
# - Remaining lines: grid representation (one row per line)
#
# The grid is preserved as a list of strings for compatibility
# with downstream solvers and visualizers.
# --------------------------------------------------------------

def read_map(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()

    if not lines:
        raise ValueError("Empty map file.")

    rows, cols = map(int, lines[0].split())
    grid = [line.strip() for line in lines[1:]]

    # Ensures structural consistency between declared and actual grid size
    if len(grid) != rows:
        raise ValueError("The number of rows in the grid does not match the specified number.")

    return rows, cols, grid

# --------------------------------------------------------------
# Path parser
#
# Reads VRPPD-style agent path descriptions and extracts:
# - Start position
# - Goal position (if available)
#
# Expected format per agent:
# Line 1: agent_id (not used downstream, kept for consistency)
# Line 2: x1 y1 x2 y2 ...
#
# If only a single coordinate pair exists, the agent is assumed
# to remain stationary (start == goal).
# --------------------------------------------------------------

def read_paths(filename):
    agents = []

    with open(filename, 'r') as f:
        lines = [line.strip() for line in f.readlines() if line.strip()]

    i = 0
    while i < len(lines):
        agent_id = lines[i]  # not in use, but preserved for traceability
        i += 1

        if i >= len(lines):
            raise ValueError(f"Agent {agent_id} without coordinate lines.")

        coords = list(map(int, lines[i].split()))
        i += 1

        # Minimal validation to ensure at least a start position exists
        if len(coords) < 2:
            raise ValueError(f"Agent {agent_id} without a valid starting position.")

        start = (coords[0], coords[1])

        # If a second pair exists, treat it as goal; otherwise stationary agent
        if len(coords) >= 4:
            goal = (coords[2], coords[3])
        else:
            goal = start

        agents.append((start, goal))

    return agents

# --------------------------------------------------------------
# Output writer (ICBS format)
#
# Generates a file compatible with ICBS solvers.
#
# Output structure:
# 1. Map dimensions
# 2. Grid layout
# 3. Number of agents
# 4. Start/goal pairs per agent
# --------------------------------------------------------------

def write_output(filename, rows, cols, grid, agents):
    with open(filename, 'w') as f:
        # Write map header
        f.write(f"{rows} {cols}\n")
        for line in grid:
            f.write(line + "\n")

        f.write(f"{len(agents)}\n")

        # Each line defines: start_x start_y goal_x goal_y
        for start, goal in agents:
            f.write(f"{start[0]} {start[1]} {goal[0]} {goal[1]}\n")

# --------------------------------------------------------------
# Main pipeline
#
# Workflow:
# 1. Parse CLI arguments
# 2. Resolve dataset directory
# 3. Load map structure
# 4. Load VRPPD agent paths
# 5. Convert to ICBS input format
# 6. Write output file
# --------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Converts map_input + vrppd_paths to ICBS input format."
    )
    parser.add_argument("num_example", help="Example number (ex: 1 for example1)")
    parser.add_argument(
        "num_boxes",
        nargs="?",
        type=int,
        default=9,
        help="Number of boxes (defines folder <num_boxes>_boxes by default)",
    )
    parser.add_argument(
        "--boxes-dir",
        default=None,
        help="Folder inside solver/instances (ex.: 8_boxes, 9_boxes, 10_boxes).",
    )
    args = parser.parse_args()

    boxes_dir = args.boxes_dir or f"{args.num_boxes}_boxes"

    project_root = Path(__file__).resolve().parents[1]
    example_dir = project_root / "solver" / "instances" / boxes_dir / f"example{args.num_example}"

    map_file = example_dir / f"example{args.num_example}_map_input.txt"
    paths_file = example_dir / f"example{args.num_example}_vrppd_paths.txt"
    output_file = example_dir / f"example{args.num_example}_icbs_map.txt"

    if not map_file.exists():
        raise FileNotFoundError(f"Map file not found: {map_file}")

    if not paths_file.exists():
        raise FileNotFoundError(f"Paths file not found: {paths_file}")

    rows, cols, grid = read_map(map_file)
    agents = read_paths(paths_file)

    write_output(output_file, rows, cols, grid, agents)

    print(f"File successfully generated in: {output_file}")

if __name__ == "__main__":
    main()