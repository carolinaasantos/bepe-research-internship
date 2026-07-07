# ------------------------------------------------------------------
# RHCR EXECUTION WRAPPER
#
# This script executes the RHCR (Rolling Horizon Collision
# Resolution) solver for a selected benchmark instance.
#
# The execution pipeline:
# - Locates the instance directory
# - Loads RHCR-compatible map and agent files
# - Invokes the RHCR lifelong executable
# - Monitors execution time and timeout limits
# - Stores generated agent paths
# - Records planning runtime statistics
#
# The script acts as an interface between the generated
# benchmark instances and the RHCR solver binary, providing
# automatic file resolution and execution management.
# ------------------------------------------------------------------

import argparse
import subprocess
import sys
import time
from pathlib import Path

# --------------------------------------------------------------
# Command-line argument parser
#
# Defines all parameters required to execute RHCR.
#
# Supported options:
# - Example identifier
# - Number of boxes
# - Instance directory override
# - Global process timeout
# - Internal RHCR cutoff time
#
# Returns:
# - Parsed argument namespace
# --------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description="Executes RHCR (lifelong) for a given sample and quantity of boxes."
    )

    parser.add_argument(
        "num_example",
        help="Example number (ex: 1 for example1)"
    )

    parser.add_argument(
        "num_boxes",
        type=int,
        help="Number of boxes (defines the folder <num_boxes>_boxes by default)",
    )

    parser.add_argument(
        "--boxes-dir",
        default=None,
        help="Folder inside solver/instances (ex: 8_boxes, 9_boxes, 10_boxes).",
    )

    parser.add_argument(
        "--timeout",
        type=int,
        default=5,
        help="Total timeout in seconds for the RHCR process.",
    )

    parser.add_argument(
        "--cutoff-time",
        type=int,
        default=20,
        help="Cutoff time in seconds per solver call within RHCR.",
    )

    return parser.parse_args()

# --------------------------------------------------------------
# Example directory resolver
#
# Locates the directory containing the selected benchmark.
#
# Search order:
# 1. solver/instances/<boxes_dir>/example<number>
# 2. solver/instances/example<number> (legacy layout)
#
# Returns the first valid path found. If no directory exists,
# returns the expected location under <boxes_dir>.
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
# Main execution pipeline
#
# Workflow:
# 1. Parse CLI arguments
# 2. Resolve benchmark directory
# 3. Locate RHCR input files
# 4. Validate required resources
# 5. Build RHCR command
# 6. Execute solver
# 7. Measure planning time
# 8. Save runtime statistics
# --------------------------------------------------------------

def main():
    args = parse_args()

    number = args.num_example
    boxes_dir = args.boxes_dir or f"{args.num_boxes}_boxes"

    project_root = Path(__file__).resolve().parents[1]

    example_dir = resolve_example_dir(
        project_root,
        number,
        boxes_dir
    )

    name = f"example{number}"

    # ----------------------------------------------------------
    # Input/output file definitions
    # ----------------------------------------------------------

    lifelong_bin = project_root / "MAPF_RHCR" / "lifelong"

    map_file = example_dir / f"{name}_map.grid"
    node_locations = example_dir / f"{name}_node_locations.txt"

    output_file = example_dir / f"{name}_rhcr_paths.txt"

    # ----------------------------------------------------------
    # Validate required resources before execution
    # ----------------------------------------------------------

    if not lifelong_bin.is_file():
        print(f"Binary not found: {lifelong_bin}")
        return 1

    if not map_file.is_file():
        print(f"Map file not found: {map_file}")
        return 1

    if not node_locations.is_file():
        print(f"The node_locations file could not be found: {node_locations}")
        return 1

    # ----------------------------------------------------------
    # RHCR configuration
    #
    # The command defines:
    # - Scenario type
    # - Map file
    # - Agent locations
    # - Solver configuration
    # - Planning horizon parameters
    # - Logging and debugging options
    # ----------------------------------------------------------

    cmd = [
        str(lifelong_bin),

        "--scenario",
        "SORTING",

        "--map",
        str(map_file),

        "--agentNum",
        "3",

        "--cutoffTime",
        str(args.cutoff_time),

        "--locations",
        str(node_locations),

        "--traverse",
        str(node_locations),

        "--solver",
        #"ECBS",
        "PBS",

        "--single_agent_solver",
        "SIPP",

        "--simulation_time",
        "5000",

        "--simulation_window",
        "5",

        "--output",
        str(output_file),

        "--suboptimal_bound",
        "1.8",
        #"3.0",

        "--planning_window",
        "20",
        #"50",

        "--goal_wait",
        "5",

        "--close_delivery_obstacles",
        "true",

        "--max_failed_plans",
        "25",

        "--max_no_progress_timesteps",
        "200",

        "--log",
        "true",

        "--debug_positions",
        "false",
    ]

    # ----------------------------------------------------------
    # Execute RHCR and measure total planning time
    # ----------------------------------------------------------

    planning_start_ns = time.perf_counter_ns()

    try:
        result = subprocess.run(
            cmd,
            cwd=project_root,
            timeout=args.timeout
        )

    except subprocess.TimeoutExpired:
        print(
            f"Timeout: RHCR exceeded {args.timeout}s and was interrupted."
        )
        return 124

    planning_time_ns = time.perf_counter_ns() - planning_start_ns

    # ----------------------------------------------------------
    # Convert runtime to multiple time units for reporting
    # ----------------------------------------------------------

    planning_time_s = planning_time_ns / 1_000_000_000
    planning_time_ms = planning_time_ns / 1_000_000
    planning_time_us = planning_time_ns / 1_000

    msg = (
        "Total planning time (RHCR): "
        f"{planning_time_s:.9f} s "
        f"({planning_time_ms:.6f} ms | "
        f"{planning_time_us:.3f} us | "
        f"{planning_time_ns} ns)"
    )

    print(msg)

    # ----------------------------------------------------------
    # Persist planning statistics for future analysis
    #
    # Results are appended to:
    # RHCR_planning_time.txt
    # ----------------------------------------------------------

    planning_file = example_dir / "RHCR_planning_time.txt"

    with open(planning_file, "a", encoding="utf-8") as f:
        f.write(msg + "\n")

    return result.returncode


if __name__ == "__main__":
    sys.exit(main())