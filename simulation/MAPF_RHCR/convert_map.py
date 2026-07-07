# ------------------------------------------------------------------
# RHCR GRID MAP CONVERTER
#
# This script converts a grid map instance into the RHCR
# (Rolling Horizon Collision Resolution) map format.
#
# The conversion process transforms:
# - Grid map representation (obstacles and free cells)
#
# Into a graph-oriented node representation where each cell
# becomes a node containing:
# - Unique identifier
# - Cell type (Travel or Obstacle)
# - Cartesian coordinates
# - Movement costs to neighboring cells
# - Wait action cost
#
# Traversable cells receive movement costs based on their
# connectivity, while obstacles are marked with infinite
# transition costs.
# ------------------------------------------------------------------

import argparse
from pathlib import Path

# --------------------------------------------------------------
# Grid map loader
#
# Reads a map file containing a grid representation.
#
# Expected format:
# - First line: <rows> <cols>
# - Remaining lines: grid cells separated by spaces
#
# Returns:
# - Number of rows
# - Number of columns
# - Grid matrix representation
# --------------------------------------------------------------

def read_grid(file_name):
    with open(file_name, 'r') as f:
        lines = f.readlines()

    n, m = map(int, lines[0].split())

    grid = []
    for i in range(1, n + 1):
        line = lines[i].strip().split()
        grid.append(line)

    return n, m, grid

# --------------------------------------------------------------
# Cell traversability checker
#
# Determines whether a coordinate corresponds to a valid
# traversable cell inside the grid boundaries.
#
# A move is considered valid if:
# - Coordinates are inside map limits
# - Target cell is marked as free ('.')
#
# Returns:
# - True if movement is allowed
# - False otherwise
# --------------------------------------------------------------

def can_move(grid, n, m, x, y):
    if 0 <= y < n and 0 <= x < m:
        return grid[y][x] == '.'
    return False

# --------------------------------------------------------------
# RHCR map generator
#
# Converts the input grid map into RHCR node format.
#
# Output structure:
# 1. Grid dimensions
# 2. Node table header
# 3. One entry per grid cell
#
# Each node contains:
# - Unique row-major identifier
# - Cell classification
# - Coordinates
# - Movement costs to neighboring cells
# - Wait action cost
#
# Obstacle cells receive infinite movement costs.
# Traversable cells receive unit-cost edges to accessible
# neighboring cells.
# --------------------------------------------------------------

def generate_output(input_name, output_name):
    n, m, grid = read_grid(input_name)

    with open(output_name, 'w') as f:
        f.write("Grid size (x, y)\n")
        f.write(f"{m},{n}\n")
        f.write("id,type,station,x,y,weight_to_NORTH,weight_to_WEST,weight_to_SOUTH,weight_to_EAST,weight_for_WAIT\n")

        for y in range(n):
            for x in range(m):

                # Row-major node indexing:
                # node_id = row * num_columns + column
                node_id = y * m + x

                if grid[y][x] == '@':
                    cell_type = "Obstacle"
                    weights = ["inf", "inf", "inf", "inf", "1"]
                else:
                    cell_type = "Travel"

                    # Compute connectivity costs to neighboring cells
                    north = "1" if can_move(grid, n, m, x, y - 1) else "inf"
                    west = "1" if can_move(grid, n, m, x - 1, y) else "inf"
                    south = "1" if can_move(grid, n, m, x, y + 1) else "inf"
                    east = "1" if can_move(grid, n, m, x + 1, y) else "inf"

                    weights = [north, west, south, east, "1"]

                line = (
                    f"{node_id},{cell_type},None,{x},{y},"
                    f"{weights[0]},{weights[1]},{weights[2]},"
                    f"{weights[3]},{weights[4]}\n"
                )
                f.write(line)

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
# Input map resolver
#
# Locates the map file associated with an example.
#
# Supported formats:
# - exampleX_map_input.txt (current)
# - exampleX_map.txt (legacy)
#
# Returns the preferred available file.
# --------------------------------------------------------------

def resolve_map_input(example_dir: Path, number: str) -> Path:
    map_input = example_dir / f"example{number}_map_input.txt"
    legacy_map = example_dir / f"example{number}_map.txt"

    if map_input.exists():
        return map_input

    return legacy_map

# --------------------------------------------------------------
# Main pipeline
#
# Workflow:
# 1. Parse CLI arguments
# 2. Resolve instance directory
# 3. Locate map input file
# 4. Convert map into RHCR format
# 5. Write output file
# --------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Converts map_input to sorting_map.grid in RHCR format."
    )

    parser.add_argument("num_example",help="Example number (ex: 1 for example1)")

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

    input = resolve_map_input(example_dir, args.num_example)
    output = example_dir / f"example{args.num_example}_map.grid"

    generate_output(input, output)

    print("File generated successfully!")

if __name__ == "__main__":
    main()