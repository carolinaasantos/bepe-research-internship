# ------------------------------------------------------------------
# ROS2 MAPF Path Planning Launch File
#
# This launch file starts the MAPF path planning controller node and
# configures the required runtime parameters. It provides launch-time
# options for simulation time, robot namespace, and robot ID, enabling
# multi-robot execution within a shared planning framework.
# ------------------------------------------------------------------

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
import socket

def generate_launch_description():
    # Use the host machine name as the default robot namespace
    robot_name_str      = socket.gethostname()
    robot_name_str      = robot_name_str.replace('-', '_')

    # Launch-time configurable parameters
    use_sim_time    = LaunchConfiguration('use_sim_time')
    robot_name      = LaunchConfiguration('robot_name')    
    robot_id = LaunchConfiguration('robot_id')

    # Declare launch arguments with default values
    declare_use_sim_time    = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_robot_name      = DeclareLaunchArgument('robot_name', default_value=robot_name_str)
    
    # Start the MAPF path planning controller node
    controller_cmd = Node(
        package     = 'dots_example_controllers',
        executable  = 'mapf_path_planning',
        namespace   = robot_name,
        output      = 'screen',
        parameters  = [{'use_sim_time' : use_sim_time, 'robot_id': robot_id}]
    )

    # Build the launch description
    ld = LaunchDescription()

    # Register launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_robot_name)
    ld.add_action(DeclareLaunchArgument('robot_id', default_value='0'))

    # Add launch components to be executed
    ld.add_action(controller_cmd)

    return ld
    