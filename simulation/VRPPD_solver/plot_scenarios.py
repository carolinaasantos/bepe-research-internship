# ------------------------------------------------------------------
# PLOTTING SCENARIOS
# 
# This script generates a visual representation of a VRPPD scenario. It plots robots, pickup 
# locations, and delivery locations on a grid to help analyze the spatial structure of each 
# instance.
#
# In the visualization:
# - Robots are represented as circles
# - Boxes (pickup and delivery points) are represented as square markers
# ------------------------------------------------------------------

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt

# Grid size used for plotting the VRPPD scenario layout
grid = 7

# Parses command-line arguments
def parse_args():
    parser = argparse.ArgumentParser(
        description="Plots the VRPPD scenario layout for a given example."
    )

    parser.add_argument(
        "example_number",
        help="Example number (e.g., 1 for example1)"
    )

    parser.add_argument(
        "num_boxes",
        type=int,
        help="Number of boxes (defines <num_boxes>_boxes directory by default)"
    )

    parser.add_argument(
        "--boxes-dir",
        default=None,
        help="Folder inside VRPPD_solver/instances (e.g., 8_boxes, 9_boxes, 10_boxes)"
    )

    return parser.parse_args()


# --------------------------------------------------------------
# Resolves the dataset directory for a given VRPPD instance.
#
# This function supports both:
# - New structure: VRPPD_solver/instances/<boxes_dir>/exampleX
# - Legacy structure: VRPPD_solver/instances/exampleX
# --------------------------------------------------------------

def resolve_example_dir(project_root: Path, number: str, boxes_dir: str) -> Path:
    base = project_root / "VRPPD_solver" / "instances"

    with_boxes = base / boxes_dir / f"example{number}"
    legacy = base / f"example{number}"

    if with_boxes.exists():
        return with_boxes
    if legacy.exists():
        return legacy

    return with_boxes


# --------------------------------------------------------------
# Loads a VRPPD instance file and extracts the "data" object.
#
# The input file is a Python script containing a dictionary
# definition.
# --------------------------------------------------------------

def load_data_file(file_path):
    with open(file_path, "r", encoding="utf-8") as f:
        code = f.read()

    context = {}
    exec(code, context)

    return context["data"]


# --------------------------------------------------------------
# Generates a visual grid representation of a VRPPD scenario.
#
# This includes:
# - Robot starting positions
# - Pickup locations (filled colored squares)
# - Delivery locations (outlined squares)
#
# Each request is color-coded for clarity. The output is saved
# as a PNG image for analysis and debugging of instances.
# --------------------------------------------------------------

def save_grid_image(data, example_number, grid_size=grid, filename="layout.png"):
    filename = Path(filename)
    filename.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(8, 8))

    ax.set_xlim(0, grid_size - 1)
    ax.set_ylim(0, grid_size - 1)
    ax.set_xticks(range(grid_size))
    ax.set_yticks(range(grid_size))
    ax.set_aspect('equal')

    ax.invert_yaxis()

    ax.set_facecolor('#f0f0f0')
    ax.grid(color='gray', linestyle='-', linewidth=1, zorder=0)

    colors = [
        'orange', 'green', 'red', 'purple', 'cyan',
        'magenta', 'yellow', 'pink', 'brown', 'gray'
    ]

    # Plot all pickup and delivery pairs (VRPPD requests)
    for i, req in enumerate(data['requests']):
        pick_idx = req["pickup"]
        y_pick, x_pick = data['locations'][pick_idx]
        weight = req["weight"]
        color = colors[i % len(colors)]

        rect_pick = plt.Rectangle(
            (x_pick - 0.3, y_pick - 0.3),
            0.6,
            0.6,
            color=color,
            zorder=2
        )
        ax.add_patch(rect_pick)

        ax.text(
            x_pick,
            y_pick,
            f"{weight}",
            ha='center',
            va='center',
            fontsize=9,
            fontweight='bold'
        )

        deliv_idx = req["delivery"]
        y_deliv, x_deliv = data['locations'][deliv_idx]

        rect_deliv = plt.Rectangle(
            (x_deliv - 0.3, y_deliv - 0.3),
            0.6,
            0.6,
            fill=False,
            edgecolor=color,
            linestyle='--',
            linewidth=2,
            zorder=2
        )
        ax.add_patch(rect_deliv)

    # Plot robot starting positions
    light_blue = '#66b3ff'

    for i in range(3):
        y, x = data['locations'][i]
        capacity = data['real_robot_capacities'][i]

        circ = plt.Circle((x, y), 0.35, color=light_blue, zorder=3)
        ax.add_patch(circ)

        ax.text(
            x,
            y,
            f"{i}",
            color='black',
            ha='center',
            va='center',
            fontsize=10,
            fontweight='bold',
            zorder=4
        )

        ax.text(
            x,
            y - 0.6,
            f"{capacity}",
            color=light_blue,
            ha='center',
            va='center',
            fontsize=9,
            fontweight='bold',
            zorder=4
        )

    plt.title(f"Scenario {example_number} - Robots and Boxes", fontsize=14)

    plt.savefig(filename, dpi=300, bbox_inches='tight')
    plt.close()

    print(f"Image saved at: {filename}")

def main():
    args = parse_args()

    number = args.example_number
    boxes_dir = args.boxes_dir or f"{args.num_boxes}_boxes"

    name = f"example{number}"
    project_root = Path(__file__).resolve().parent.parent

    example_dir = resolve_example_dir(project_root, number, boxes_dir)

    file = example_dir / f"{name}.txt"

    if not file.is_file():
        print(f"Input file not found: {file}")
        sys.exit(1)

    data = load_data_file(file)

    save_grid_image(
        data,
        number,
        filename=example_dir / f"{name}_layout.png"
    )


if __name__ == "__main__":
    main()