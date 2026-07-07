# ------------------------------------------------------------------
# VRPPD SOLVER
#
# This module solves a Vehicle Routing Problem with Pickup and Delivery (VRPPD) 
# using Google OR-Tools. Given a set of pickup–delivery requests, the solver 
# assigns tasks to robots while respecting capacity and routing constraints. 
# The generated solution is later used by the path-planning stage.
# ------------------------------------------------------------------

import argparse
import sys
from pathlib import Path
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# Parse CLI arguments
def parse_args():
    parser = argparse.ArgumentParser(
        description="Resolves VRPPD and generates map/route files for an example."
    )
    parser.add_argument("num_example", help="Example number (e.g., 1 for example1)")
    parser.add_argument(
        "num_boxes",
        type=int,
        help="Number of boxes (defines the folder <num_boxes>_boxes by default)",
    )
    parser.add_argument(
        "--boxes-dir",
        default=None,
        help="Folder inside solver/instances (e.g.: 8_boxes, 9_boxes, 10_boxes).",
    )
    return parser.parse_args()

# Resolve instance directory
def resolve_example_dir(project_root: Path, number: str, boxes_dir: str) -> Path:
    base = project_root / "solver" / "instances"
    with_boxes = base / boxes_dir / f"example{number}"
    legacy = base / f"example{number}"
    if with_boxes.exists():
        return with_boxes
    if legacy.exists():
        return legacy
    return with_boxes

# Build full path to input file
args = parse_args()
number = args.num_example
num_boxes = args.num_boxes
boxes_dir = args.boxes_dir or f"{num_boxes}_boxes"

name = f"example{number}"
project_root = Path(__file__).resolve().parent.parent
example_dir = resolve_example_dir(project_root, number, boxes_dir)
file = example_dir / f"{name}.txt"

# Validate input file existence
if not file.is_file():
    print(f"Input file not found: {file}")
    sys.exit(1)

def load_file_data(file_path):
    with open(file_path, "r", encoding="utf-8") as f:
        code = f.read()

    context = {}
    exec(code, context)

    return context["data"]

# ------------------------------------------------------------------
# Creates and enriches the VRPPD data model used by OR-Tools.
#
# Responsibilities:
# - Load instance data from file
# - Convert floating-point weights into integers
#   (OR-Tools dimensions only operate on integers)
# - Configure vehicle start nodes
# - Create a common dummy end node (allows flexible route termination)
# - Define map dimensions used by MAPF planners
# ------------------------------------------------------------------

def create_data_model():
    
    data = {}
    
    data = load_file_data(file)

    # Weights converted to integers
    data['box_weights'] = [int(2 * r["weight"]) for r in data['requests']]
    data['real_box_weights'] = [r["weight"] for r in data['requests']]

    # Robot payload capacities converted to integers
    data['robot_capacities'] = [int(2 * c) for c in data['real_robot_capacities']]

    # Define number of vehicles
    data['num_vehicles'] = 3

    # Define robot start nodes
    data['starts'] = [0, 1, 2]

    # Dummy end node to allow flexible route termination
    data['locations'].append((0, 0))
    data['end_node'] = len(data['locations']) - 1

    data['ends'] = [data['end_node']] * data['num_vehicles']

    # Define grid size for map generation (used by MAPF planner)
    data['grid_size'] = (7, 7)  # (lines, columns)

    return data

# ------------------------------------------------------------------
# Exports the computed VRPPD routes into a text file.
#
# Format:
#
# vehicle_id
# x1 y1 x2 y2 x3 y3 ...
#
# In which (x1, y1) is the start position, followed by the sequence of 
# coordinates (x,y) representing the routes of each robot.
#
# This file is later consumed by the MAPF stage.
# ------------------------------------------------------------------

def save_vrppd_paths(filename, manager, routing, solution, data):
    filename.parent.mkdir(parents=True, exist_ok=True)
    with open(filename, "w") as f:
        for v in range(data['num_vehicles']):
            idx = routing.Start(v)
            f.write(f"{v}\n")

            path_coords = []

            while not routing.IsEnd(idx):
                node = manager.IndexToNode(idx)
                coord = data['locations'][node]
                path_coords.append(coord)
                idx = solution.Value(routing.NextVar(idx))

            # If there is only one point (no goals), repeat 3 times to ensure 
            # MAPF has a valid path
            if len(path_coords) == 1:
                coord = path_coords[0]
                f.write(f"{coord[0]} {coord[1]} {coord[0]} {coord[1]} {coord[0]} {coord[1]}")
            else:
                for coord in path_coords:
                    f.write(f"{coord[0]} {coord[1]} ")

            f.write("\n")

# ------------------------------------------------------------------
# Generates a grid map representation for path planning.
#
# Symbols:
#   . = free cell
#   @ = obstacle
#
# Obstacles include:
# - Map boundaries (edges are impassable)
# - Pickup locations
#
# The resulting file is consumed by the MAPF planner.
# ------------------------------------------------------------------

def save_map_input(filename, data):
    filename.parent.mkdir(parents=True, exist_ok=True)
    rows, cols = data['grid_size']
    grid = [["." for _ in range(cols)] for _ in range(rows)]

    # Defines edges as obstacles
    for i in range(rows):
        grid[i][0] = "@"
        grid[i][cols - 1] = "@"

    for j in range(cols):
        grid[0][j] = "@"
        grid[rows - 1][j] = "@"

    # Marks pickups as obstacles
    for req in data['requests']:
        x, y = data['locations'][req["pickup"]]
        if 0 <= x < rows and 0 <= y < cols:
            grid[x][y] = "@"

    # Save file
    with open(filename, "w") as f:
        f.write(f"{rows} {cols}\n")
        for r in range(rows):
            f.write(" ".join(grid[r]) + "\n")

def main():
    data = create_data_model()

    manager = pywrapcp.RoutingIndexManager(
        len(data['locations']),
        data['num_vehicles'],
        data['starts'],
        data['ends']
    )

    routing = pywrapcp.RoutingModel(manager)

    # ------------------------------------------------------------------
    # Transit callback used as routing cost.
    #
    # Uses Manhattan Distance, which is appropriate for grid-based maps:
    #
    # |x1 - x2| + |y1 - y2|
    #
    # Distances are scaled by 50 to increase optimization resolution.
    #
    # Moving to the dummy end node has zero cost so vehicles can finish
    # their routes anywhere (allowing flexible route termination).
    # ------------------------------------------------------------------

    def manhattan_distance(from_idx, to_idx):
        from_node = manager.IndexToNode(from_idx)
        to_node = manager.IndexToNode(to_idx)

        # Zero cost to reach the final node
        if to_node == data['end_node']:
            return 0

        x1, y1 = data['locations'][from_node]
        x2, y2 = data['locations'][to_node]

        dist = abs(x1 - x2) + abs(y1 - y2)
        return int(dist * 50)

    dist_idx = routing.RegisterTransitCallback(manhattan_distance)
    routing.SetArcCostEvaluatorOfAllVehicles(dist_idx)

    routing.AddDimension(dist_idx, 0, 3000, True, 'Distance')
    routing.GetDimensionOrDie('Distance').SetGlobalSpanCostCoefficient(100)

    # ------------------------------------------------------------------
    # Load capacity constraint.
    #
    # Pickup  -> positive load increase
    # Delivery -> negative load decrease
    #
    # Used by the Weight dimension to enforce robot capacity limits.
    # ------------------------------------------------------------------

    def weight_callback(from_idx):
        node = manager.IndexToNode(from_idx)
        for i, req in enumerate(data['requests']):
            if node == req["pickup"]:
                return data['box_weights'][i]
            if node == req["delivery"]:
                return -data['box_weights'][i]
        return 0

    weight_idx = routing.RegisterUnaryTransitCallback(weight_callback)
    routing.AddDimensionWithVehicleCapacity(
        weight_idx, 0, data['robot_capacities'], True, 'Weight'
    )

    # ------------------------------------------------------------------
    # Single-box carrying constraint.
    #
    # Pickup  -> +1
    # Delivery -> -1
    #
    # Vehicle capacity in this dimension is fixed at 1,
    # ensuring a robot can carry only one box at a time.
    # ------------------------------------------------------------------

    def one_shot_callback(from_idx):
        node = manager.IndexToNode(from_idx)
        for req in data['requests']:
            if node == req["pickup"]:
                return 1
            if node == req["delivery"]:
                return -1
        return 0

    one_shot_idx = routing.RegisterUnaryTransitCallback(one_shot_callback)
    routing.AddDimensionWithVehicleCapacity(
        one_shot_idx, 0, [1] * data['num_vehicles'], True, 'OneShot'
    )

    # ------------------------------------------------------------------
    # Pickup-and-delivery constraints.
    #
    # For each transportation request:
    #
    # 1. Pickup and delivery must belong to the same robot.
    # 2. Pickup must occur before delivery.
    #
    # The ordering is enforced using the cumulative distance variable.
    # ------------------------------------------------------------------
    for req in data['requests']:
        p_idx = manager.NodeToIndex(req["pickup"])
        d_idx = manager.NodeToIndex(req["delivery"])

        routing.AddPickupAndDelivery(p_idx, d_idx)
        routing.solver().Add(routing.VehicleVar(p_idx) == routing.VehicleVar(d_idx))

        routing.GetDimensionOrDie('Distance').CumulVar(p_idx) <= \
            routing.GetDimensionOrDie('Distance').CumulVar(d_idx)

    # ------------------------------------------------------------------
    # Solver configuration.
    #
    # Initial solution:
    #     PARALLEL_CHEAPEST_INSERTION
    #
    # Local improvement:
    #     GUIDED_LOCAL_SEARCH
    #
    # Time limit:
    #     2 seconds
    # ------------------------------------------------------------------
    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.seconds = 2

    solution = routing.SolveWithParameters(params)

    # ------------------------------------------------------------------
    # Display solution in the terminal.
    #
    # Each robot route is printed showing:
    # - Start position
    # - Pickup operations
    # - Delivery operations
    # - Route termination
    # ------------------------------------------------------------------
    if solution:
        print("\n=== SOLUTION FOUND ===\n")

        for v in range(data['num_vehicles']):
            idx = routing.Start(v)
            print(f"ROBOT {v+1} (Capacity: {data['real_robot_capacities'][v]} kg)")

            while not routing.IsEnd(idx):
                node = manager.IndexToNode(idx)

                label = f"-> Node {node}"
                for i, req in enumerate(data['requests']):
                    if node == req["pickup"]:
                        label = f"-> [PICKUP Box {i+1} ({req['weight']} kg)]"
                    elif node == req["delivery"]:
                        label = f"-> [DELIVERY Box {i+1}]"

                if node in data['starts']:
                    label = f"(Starts in node {data['locations'][node]})"

                print(" ", label)
                idx = solution.Value(routing.NextVar(idx))

            print("  -> [END OF ROUTE]\n" + "-" * 40)

        # Save VRPPD paths and map input for the MAPF stage
        vrppd_output = example_dir / f"{name}_vrppd_paths.txt"
        map_output = example_dir / f"{name}_map_input.txt"
        save_vrppd_paths(vrppd_output, manager, routing, solution, data)
        save_map_input(map_output, data)
        print(f"Files generated successfully:\n- {vrppd_output}\n- {map_output}")

    else:
        print("No solution found! Check if capacities allow carrying boxes individually.")


if __name__ == "__main__":
    main()