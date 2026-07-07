# ------------------------------------------------------------------
# EXPERIMENT: ROS2 MAPF Path Planning Launch File
#
# This launch file sets up the necessary nodes and parameters to run the
# MAPF trajectory follower example. It includes the perception and localization
# pipeline and configures the trajectory follower with options for simulation time,
# robot namespace, speed, dwell time at goals, and robot ID for synchronization.
# ------------------------------------------------------------------

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import socket

def generate_launch_description():

    # Use the host machine name as the default robot namespace
    robot_name_str = socket.gethostname().replace('-', '_')

    # Retrieve the shared launch package path
    pkg_launch = get_package_share_directory('dots_launch')

    # Launch-time configurable parameters
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    speed = LaunchConfiguration('speed')
    dwell = LaunchConfiguration('dwell')
    robot_id = LaunchConfiguration('robot_id')

    # Declare launch arguments with default values
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_robot_name = DeclareLaunchArgument('robot_name', default_value=robot_name_str)
    declare_speed = DeclareLaunchArgument('speed', default_value='0.1')
    declare_dwell = DeclareLaunchArgument('dwell', default_value='3')
    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='-1')

    # Launch the perception and localization pipeline
    setup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_launch, 'launch', 'basic_cam_ekf.launch.py')),
    )

    # Start the trajectory follower node for MAPF execution
    mapf_trajectory_cmd = Node(
        package='dots_example_controllers',
        executable='mapf_trajectory',
        namespace=robot_name,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'speed': speed},
            {'arrival_count': dwell},
            {'robot_id': robot_id},
        ],
        # Forward joystick messages from the global topic
        remappings=[('joy', '/joy')],
    )

    # Assemble the launch description
    ld = LaunchDescription()

    # Register launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_speed)
    ld.add_action(declare_dwell)
    ld.add_action(declare_robot_id)

    # Add launch components to be executed
    ld.add_action(setup_cmd)
    ld.add_action(mapf_trajectory_cmd)

    return ld
