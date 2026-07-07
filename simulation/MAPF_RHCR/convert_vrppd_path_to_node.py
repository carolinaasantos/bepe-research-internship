# ------------------------------------------------------------------
# RHCR NODE LOCATION CONVERTER
#
# This script converts VRPPD path descriptions into the
# node_locations format required by RHCR-based solvers.
#
# The conversion process transforms:
# - Coordinate-based agent paths (x, y)
#
# Into:
# - Node-based path representations
#
# Each coordinate pair is mapped to a unique node identifier
# using a row-major indexing scheme. The resulting output
# contains one line per agent, where each line stores the
# sequence of node IDs visited by that agent.
#
# The output file begins with the total number of agents,
# followed by the node sequences associated with each agent.
# ------------------------------------------------------------------

import argparse
from pathlib import Path

# --------------------------------------------------------------
# Coordinate-to-node converter
#
# Maps a grid coordinate into a unique node identifier.
#
# Indexing scheme:
# node_id = x * grid_size + y
#
# This representation is used by RHCR to reference locations
# using a single integer value instead of coordinate pairs.
# --------------------------------------------------------------

def coords_to_node(x, y, grid_size=9):
    return x * grid_size + y

# --------------------------------------------------------------
# VRPPD path processor
#
# Reads agent paths from a VRPPD file and converts every
# coordinate pair into a node identifier.
#
# Expected agent format:
# Line 1: agent identifier
# Line 2: x1 y1 x2 y2 x3 y3 ...
#
# Output format:
# First line: total number of agents
# Remaining lines:
# node1,node2,node3,...
#
# Each output line represents the sequence of locations
# traversed by a single agent.
# --------------------------------------------------------------

def process_file(input_file, output_file):
    with open(input_file, 'r') as f:
        lines = [line.strip() for line in f if line.strip()]

    result = []
    i = 0

    total_agents = 0

    while i < len(lines):

        # Second line contains the coordinate sequence
        coords_line = lines[i + 1]
        values = list(map(int, coords_line.split()))

        nodes = []

        # Convert every coordinate pair into a node ID
        for j in range(0, len(values), 2):
            x = values[j]
            y = values[j + 1]

            node = coords_to_node(x, y)
            nodes.append(str(node))

        result.append(",".join(nodes))

        i += 2
        total_agents += 1

    with open(output_file, 'w') as f:
        f.write(f"{total_agents}\n")

        for line in result:
            f.write(line + '\n')

# --------------------------------------------------------------
# Example directory resolver
#
# Locates the directory containing the selected instance.
#
# Search order:
# 1. solver/instances/<boxes_dir>/example<number>
# 2. solver/instances/example<number> (legacy layout)
#
# Returns the first valid path found. If none exists,
# returns the expected path under <boxes_dir>.
# --------------------------------------------------------------

def resolve_example_dir(project_root: Path, number: str, boxes_dir: str) -> Path:
    base = project_root / "solver" / "instances"

    with_boxes = base / boxes_dir / f"example{number}"
    legacy = base / f"example{number}"

    if with_boxes.exists():
        return with_boxes

    if legacy.exists():
        return legacy

    return with_boxes


# --------------------------------------------------------------
# Main pipeline
#
# Workflow:
# 1. Parse CLI arguments
# 2. Resolve instance directory
# 3. Locate VRPPD path file
# 4. Convert coordinates into node IDs
# 5. Generate RHCR node_locations file
# --------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Converts vrppd_paths to node_locations in RHCR format."
    )

    parser.add_argument(
        "num_example",
        help="Example number (ex: 1 for example1)"
    )

    parser.add_argument(
        "num_boxes",
        nargs="?",
        type=int,
        default=9,
        help="Number of boxes (defines the folder <num_boxes>_boxes by default)",
    )

    parser.add_argument(
        "--boxes-dir",
        default=None,
        help="Folder inside solver/instances (ex: 8_boxes, 9_boxes, 10_boxes).",
    )

    args = parser.parse_args()

    boxes_dir = args.boxes_dir or f"{args.num_boxes}_boxes"

    project_root = Path(__file__).resolve().parents[1]
    example_dir = resolve_example_dir(
        project_root,
        args.num_example,
        boxes_dir
    )

    input = example_dir / f"example{args.num_example}_vrppd_paths.txt"
    output = example_dir / f"example{args.num_example}_node_locations.txt"

    process_file(input, output)

    print("File generated successfully!")


if __name__ == "__main__":
    main()