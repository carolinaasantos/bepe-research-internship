# ------------------------------------------------------------------
# SCENARIO GENERATOR
# 
# This script generates synthetic VRPPD instances by randomly placing robots, 
# pickup points, and delivery points on a grid, and assigning random weights
# to each request. The generated scenarios are saved as structured input 
# files compatible with the VRPPD solver.
# ------------------------------------------------------------------

import random
import os
import sys

# Grid size for positioning robots, pickups, and deliveries
grid = 8

# ----------------------------------------------------------------------
# Generates synthetic VRPPD instance files for testing the solver.
#
# Parameters:
# - files_qnt: number of instances to generate
# - num_boxes: number of pickup-delivery pairs per instance
# - output_folder: base output directory
# ----------------------------------------------------------------------

def generate_instances(files_qnt, num_boxes, output_folder="VRPPD_solver/instances"):
    # Build dynamic output folder based on num_boxes
    output_folder = os.path.join(output_folder, f"{num_boxes}_boxes")

    # Create output directory if it does not exist
    os.makedirs(output_folder, exist_ok=True)

    # Fixed robot capacities for all generated instances
    robot_capacities = [1.5, 2, 1.5]

    # Possible weights assigned randomly to boxes
    possible_weights = [0.5, 1.0, 1.5, 2.0]

    for file_id in range(1, files_qnt + 1):

        # Generate all possible free grid positions (excluding borders)
        all_positions = [(x, y) for x in range(1, grid) for y in range(1, grid)]

        # Total required positions:
        # 3 robots + num_boxes pickups + num_boxes deliveries
        total_required = 3 + (2 * num_boxes)

        # Validate if grid is large enough
        if len(all_positions) < total_required:
            raise ValueError("Not enough positions for the number of boxes!")

        # Shuffle positions to randomize scenario layout
        random.shuffle(all_positions)

        # Select only required number of positions
        selected = all_positions[:total_required]

        # Assign first 3 positions to robots
        robots = selected[:3]

        # Next positions are pickups
        pickups = selected[3:3 + num_boxes]

        # Remaining positions are deliveries
        deliveries = selected[3 + num_boxes:3 + 2 * num_boxes]

        # Randomly assign weights to each box
        weights = [random.choice(possible_weights) for _ in range(num_boxes)]

        # Define output file path for this scenario
        file_name = os.path.join(
            output_folder,
            f"example{file_id + 13}/example{file_id + 13}.txt"
        )

        # Ensure directory exists for this specific instance
        os.makedirs(os.path.dirname(file_name), exist_ok=True)

        with open(file_name, "w") as f:
            f.write("data = {}\n\n")

            # LOCATION LIST
            f.write("data['locations'] = [\n")

            idx = 0

            # Robots
            for i, coord in enumerate(robots):
                f.write(f"    {coord},  # {idx} - Robot {i+1}\n")
                idx += 1

            # Pickups
            for i, coord in enumerate(pickups):
                f.write(f"    {coord},  # {idx} - Pickup {i+1}\n")
                idx += 1

            # Deliveries
            for i, coord in enumerate(deliveries):
                f.write(f"    {coord},  # {idx} - Delivery {i+1}\n")
                idx += 1

            f.write("]\n\n")

            # REQUEST LIST
            f.write("data['requests'] = [\n")

            for i in range(num_boxes):
                pickup_idx = 3 + i
                delivery_idx = 3 + num_boxes + i
                weight = weights[i]

                f.write(
                    f"    {{\"pickup\": {pickup_idx}, \"delivery\": {delivery_idx}, \"weight\": {weight}}}, "
                    f"# Box {i+1}\n"
                )

            f.write("]\n\n")

            f.write(f"data['real_robot_capacities'] = {robot_capacities}\n")

    print(f"{files_qnt} files generated in '{output_folder}' with {num_boxes} boxes each.")


# CLI entry point
if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python gerador_scenarios.py <quantity> <num_boxes>")
        sys.exit(1)

    quantidade = int(sys.argv[1])
    num_boxes = int(sys.argv[2])

    generate_instances(quantidade, num_boxes)