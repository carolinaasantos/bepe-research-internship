
# ------------------------------------------------------------------
# SIMULATION: Multi-Robot MAPF Simulation Launch File
#
# This launch file initializes a complete multi-robot MAPF simulation
# environment in Gazebo. It spawns multiple robots at predefined start
# positions, launches the perception and localization pipeline for each
# robot, starts the MAPF path planning controller instances, and optionally
# enables RViz visualization. The simulation world is configured for the
# selected MAPF scenario and synchronized using simulation time.
# ------------------------------------------------------------------

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from scripts import GazeboRosPaths
import glob

def generate_launch_description():

    # Remove any temporary URDF files from previous simulation runs
    for f in glob.glob('/tmp/*.urdf'):
        try:
            os.remove(f)
        except OSError:
            print('Could not remove %s' % f)

    # Define the initial poses for all robots in the MAPF scenario
    robots = [
        # Example 13 - 8 boxes
        # ('robot_00000001',     (    0.616666,    -1.2333,      0.0   )),
        # ('robot_00000002',     (   0.0000,    0.616666,      0.0   )),
        # ('robot_00000003',     (    0.616666,    0.616666,      0.0   )),

        # GENERATED START POSES BEGIN
        # Example 14 - 8 boxes (generated start poses)
        ('robot_00000001',     (    0.000000,    0.616667,      0.0   )),
        ('robot_00000002',     (    -1.233333,    0.616667,      0.0   )),
        ('robot_00000003',     (    1.233333,    0.616667,      0.0   )),

        # GENERATED START POSES END
    ]

    # Retrieve package directories used by the simulation
    pkg_sim         = get_package_share_directory('dots_sim')
    pkg_launch      = get_package_share_directory('dots_launch')
    pkg_controller  = get_package_share_directory('dots_example_controllers')

    # Launch-time configurable parameters
    use_sim_time    = LaunchConfiguration('use_sim_time', default='True')
    use_rviz        = LaunchConfiguration('use_rviz', default='False')

    # Declare launch arguments with default values
    declare_use_sim_time    = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_use_rviz        = DeclareLaunchArgument('use_rviz', default_value='false')

    # Build the launch description
    ld = LaunchDescription()

    # Launch the localization and planning stack for each robot
    for r in robots:
        print('robot:%s pose:%s' % (r[0],r[1]))
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_launch, 'launch', 'basic_cam_ekf.launch.py')),
            launch_arguments    = { 'robot_name'    : PythonExpression(['"', r[0], '"']),
                                    'robot_pose'    : '%f,%f,%f' % (r[1][0], r[1][1], r[1][2]),
                                    'use_sim_time'  : use_sim_time,
            }.items()
        ))

        # Start the MAPF path planning controller instance
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'mapf_path_planning.launch.py')),
            launch_arguments    = { 'robot_name'    : PythonExpression(['"', r[0], '"']),
                                    'robot_pose'    : '%f,%f,%f' % (r[1][0], r[1][1], r[1][2]),
                                    'use_sim_time'  : use_sim_time,
            }.items()
        ))

    # Launch the Gazebo simulation environment and optional RViz visualization
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource( os.path.join(pkg_sim, 'launch', 'gazebo_rviz.launch.py')),
        launch_arguments = { 'use_rviz': use_rviz, 
                             'world_file': 'arena_8boxes_example14.world'
        }.items()
    ))

    # Register launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)

    return ld