#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch.actions import SetEnvironmentVariable
import os
import xacro


def generate_launch_description():
    # Get the share directory of your package
    share_dir = get_package_share_directory('j8_xacro_model')

    # Instead of using os.path.join to build a string, use PathJoinSubstitution.
    # This creates a substitution that is compatible with the launch system.
    world_arg = PathJoinSubstitution([
        FindPackageShare('car'),
        'worlds',
        'world_1_b1.world'
    ])

    # Get the car URDF by processing the xacro file
    xacro_file = os.path.join(share_dir, 'urdf', 'argo_j8.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    goal_yaml = PathJoinSubstitution([
        FindPackageShare('car'),        # paquete donde está goal.yaml
        'config',                       # subcarpeta
        'circuit_1.yaml'                     # nombre del archivo
    ])


    check_goal = Node(
        package='car_cpp',
        executable='check_goal',
        name='check_goal',
        parameters=[{
            'goal_threshold_xy': 3.5,   # robot converge a ~3.3m con local-waypoint; 3.5m da margen
            'robot_frame': 'base_link',
            'goal_frame':  'map',
        }],
    )

    

    points_goal = Node(
        package     = 'car',
        executable  = 'circuit1',
        name        = 'circuit1',
        parameters  = [
            goal_yaml,
            {
                'goal_points_relative_to_robot': False,   # coords absolutas en map
                'goal_frame_id': 'map',
                'compute_yaw_from_path': True,
                'odom_topic': '/fixposition/odometry',
            },
        ]
    )




    return LaunchDescription([

        check_goal,
        points_goal,

    ])
