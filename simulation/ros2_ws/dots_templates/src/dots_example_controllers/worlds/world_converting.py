# ------------------------------------------------------------------
# GAZEBO WORLD CONVERTING
#
# Responsibilities:
# - Locate the Gazebo world template
# - Locate instance definitions
# - Convert MAPF instance definitions into Gazebo world files
# - Update the sim_3 launch file with robot start poses.
#
# Usage:
# python3 world_converting.py <example_number> <num_boxes>
# ------------------------------------------------------------------

import sys
import os
import re

# File directories
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
INSTANCES_DIR = os.path.normpath(
    os.path.join(SCRIPT_DIR, '..', 'dots_example_controllers','instances')
)
TEMPLATE_WORLD = os.path.join(SCRIPT_DIR, 'model_arena.world')
SIM3_LAUNCH = os.path.normpath(
    os.path.join(SCRIPT_DIR, '..','launch', 'sim_3_mapf_path_planning.launch.py')
)

# ------------------------------------------------------------------
# Converts grid coordinates into Gazebo world coordinates.
#
# The warehouse instances use discrete (col, row) positions,
# while Gazebo requires metric coordinates in the arena frame.
# The transformation below scales the grid to the physical arena
# dimensions and shifts the origin to the arena center.
# ------------------------------------------------------------------

def grid_to_gazebo(col, row):
    # GRID 8: 3.7m x 3.7m
    # x = row * (3.7/8) - 1.85
    # y = 1.85 - col * (3.7/8)

    # GRID 6: 3.7m x 3.7m
    x = row * (3.7/6) - 1.85
    y = 1.85 - col * (3.7/6)
    return x, y

# ------------------------------------------------------------------
# Loads a warehouse instance and extracts relevant simulation data.
#
# The instance file contains Python data structures describing:
#   - warehouse locations
#   - pickup requests
#   - robot capacities
#
# This function converts the relevant positions into Gazebo
# coordinates so they can be used directly by the simulator.
# ------------------------------------------------------------------

def load_instance(example_num, num_boxes):
    instance_dir = os.path.join(INSTANCES_DIR, f'{num_boxes}_boxes', f'example{example_num}')
    instance_file = os.path.join(instance_dir, f'example{example_num}.txt')

    if not os.path.isfile(instance_file):
        print(f"ERROR: File not found: {instance_file}")
        sys.exit(1)

    namespace = {}
    with open(instance_file, 'r') as f:
        exec(f.read(), namespace)

    data = namespace['data']
    locations = data['locations']
    requests = data['requests']

    pickup_positions = []
    for req in requests:
        idx = req['pickup']
        col, row = locations[idx]
        x, y = grid_to_gazebo(col, row)
        pickup_positions.append((x, y))

    num_robots = len(data.get('real_robot_capacities', []))
    if num_robots <= 0:
        print("ERROR: Could not infer robot count from 'real_robot_capacities'.")
        sys.exit(1)

    robot_positions = []
    for i in range(num_robots):
        col, row = locations[i]
        x, y = grid_to_gazebo(col, row)
        robot_positions.append((x, y))

    return pickup_positions, robot_positions

# ------------------------------------------------------------------
# Generates Gazebo model include blocks for all carriers.
#
# Each pickup location becomes a carrier model positioned at the
# corresponding warehouse cell. Carrier IDs start at 100 to match
# the naming convention used by the simulator assets.
# ------------------------------------------------------------------

def build_carrier_lines(pickup_positions):
    lines = []
    for i, (x, y) in enumerate(pickup_positions):
        carrier_id = 100 + i
        lines.append(
            f'    <include>'
            f'<uri>model://carrier{carrier_id}</uri>'
            f'<name>carrier{carrier_id}</name>'
            f'<pose>{x:.4f} {y:.4f} 0 0 0 0</pose>'
            f'</include>'
        )
    return '\n'.join(lines)

# ------------------------------------------------------------------
# Creates the robot start pose block used by the launch file.
#
# Responsibilities:
# - Generate robot initial positions
# - Format poses according to launch file conventions
# - Annotate the generated scenario information
# ------------------------------------------------------------------

def build_robot_scenario_block(example_num, num_boxes, robot_positions):
    lines = [
        f'                    # Example {example_num} - {num_boxes} boxes (auto-generated start poses)',
    ]
    for i, (x, y) in enumerate(robot_positions, start=1):
        lines.append(
            f"                    ('robot_{i:08d}',     (    {x:.6f},    {y:.6f},      0.0   )),"
        )
    lines.append('')
    return '\n'.join(lines)

# ------------------------------------------------------------------
# Updates the ROS 2 launch file with generated robot start poses.
#
# The launch file contains an auto-generated section delimited by
# marker comments. Whenever possible, only that section is replaced,
# allowing manual edits elsewhere in the launch file to remain intact.
# ------------------------------------------------------------------

def update_sim3_launch(example_num, num_boxes, robot_positions):
    if not os.path.isfile(SIM3_LAUNCH):
        print(f"WARNING: sim_3 launch file not found: {SIM3_LAUNCH}")
        return

    with open(SIM3_LAUNCH, 'r') as f:
        launch_text = f.read()

    generated_start = '# GENERATED START POSES BEGIN'
    generated_end = '# GENERATED START POSES END'
    new_generated_block = (
        f'                    {generated_start}\n'
        f'{build_robot_scenario_block(example_num, num_boxes, robot_positions)}\n'
        f'                    {generated_end}\n'
    )

    existing_generated_pattern = re.compile(
        r'^[ \t]*# GENERATED START POSES BEGIN.*?^[ \t]*# GENERATED START POSES END[ \t]*\n?',
        re.DOTALL | re.MULTILINE
    )

    if existing_generated_pattern.search(launch_text):
        launch_text = existing_generated_pattern.sub(new_generated_block, launch_text, count=1)
    else:
        insert_anchor = '                    # scenario1 - 3 robots'
        if insert_anchor not in launch_text:
            print("WARNING: Could not find insertion point for robot start poses in sim_3 launch.")
            return
        launch_text = launch_text.replace(insert_anchor, new_generated_block + insert_anchor, 1)

    with open(SIM3_LAUNCH, 'w') as f:
        f.write(launch_text)

    print(f"Updated robot start poses in: {SIM3_LAUNCH}")

# ------------------------------------------------------------------
# Updates the Gazebo world file used by the sim_3 launch file.
#
# Rather than requiring manual edits after each conversion, the
# launch file is updated automatically to reference the scenario
# currently being generated.
# ------------------------------------------------------------------

def update_sim3_world(example_num, num_boxes):

    if not os.path.isfile(SIM3_LAUNCH):
        print(f"WARNING: sim_3 launch file not found: {SIM3_LAUNCH}")
        return

    with open(SIM3_LAUNCH, 'r') as f:
        launch_text = f.read()

    new_world = f'arena_{num_boxes}boxes_example{example_num}.world'

    pattern = re.compile(
        r"(world_file'\s*:\s*')([^']+)(')"
    )

    launch_text, count = pattern.subn(
        rf"\1{new_world}\3",
        launch_text,
        count=1
    )

    if count == 0:
        print("WARNING: Could not find world_file entry in sim_3 launch.")
        return

    with open(SIM3_LAUNCH, 'w') as f:
        f.write(launch_text)

    print(f"Updated world file in: {SIM3_LAUNCH}")
    print(f"  world_file = {new_world}")

# ------------------------------------------------------------------
# Generates a complete Gazebo world file from an instance.
#
# Workflow:
#   1. Load the selected MAPF instance
#   2. Convert pickup locations into carrier models
#   3. Inject the generated carriers into the world template
#   4. Save a scenario-specific Gazebo world
#   5. Update launch files to use the new scenario
# ------------------------------------------------------------------

def generate_world(example_num, num_boxes):
    pickup_positions, robot_positions = load_instance(example_num, num_boxes)

    with open(TEMPLATE_WORLD, 'r') as f:
        template = f.read()

    carrier_block_pattern = re.compile(
        r'(\s*<include>\s*<uri>model://carrier\d+</uri>.*?</include>\s*)+',
        re.DOTALL
    )

    new_carrier_block = '\n' + build_carrier_lines(pickup_positions) + '\n\n'
    result, count = carrier_block_pattern.subn(new_carrier_block, template, count=1)

    if count == 0:
        print("ERROR: Could not find carrier block in carolina_arena.world to replace.")
        sys.exit(1)

    output_filename = f'arena_{num_boxes}boxes_example{example_num}.world'
    output_path = os.path.join(SCRIPT_DIR, output_filename)

    with open(output_path, 'w') as f:
        f.write(result)

    print(f"Generated: {output_path}")
    print(f"  {len(pickup_positions)} carriers placed at pickup positions:")
    for i, (x, y) in enumerate(pickup_positions):
        print(f"    carrier{100+i}: ({x:.4f}, {y:.4f})")
    print(f"  {len(robot_positions)} robot start poses converted.")

    update_sim3_launch(example_num, num_boxes, robot_positions)
    update_sim3_world(example_num, num_boxes)


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python3 world_converting.py <example_number> <num_boxes>")
        sys.exit(1)

    example_num = int(sys.argv[1])
    num_boxes = int(sys.argv[2])
    generate_world(example_num, num_boxes)
