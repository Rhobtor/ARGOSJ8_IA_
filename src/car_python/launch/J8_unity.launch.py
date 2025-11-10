#!/usr/bin/env python3
# robot_with_frames.launch.py

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
import xacro

def generate_launch_description():
    # ========== AJUSTA ESTO A TU PAQUETE/RUTA ==========
    # Paquete que contiene el xacro/urdf y su ruta
    desc_pkg  = 'j8_xacro_model'         # <- cámbialo si tu URDF está en otro paquete
    xacro_path = os.path.join(
        get_package_share_directory(desc_pkg),
        'urdf',
        'argo_j8.xacro'                 # <- cámbialo si el nombre difiere
    )

    # Procesar el xacro → robot_description (XML)
    robot_description_config = xacro.process_file(xacro_path)
    robot_urdf = robot_description_config.toxml()

    # ======= Parámetro opcional para el alias Velodyne_link → lidar_link =======
    use_lidar_alias_arg = DeclareLaunchArgument(
        'use_lidar_alias',
        default_value='true',
        description='Crear alias TF Velodyne_link -> lidar_link (identidad)'
    )
    use_lidar_alias = LaunchConfiguration('use_lidar_alias')

    # ========== NODOS ==========
    # Robot State Publisher (publica TF del URDF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_urdf}]
    )

    # Joint State Publisher (si no tienes otro publicador de joints, fija a 0)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_gui': False}]
    )

    # map -> odom (identidad provisional)
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # # odom -> base_link (identidad provisional hasta tener odometría)
    # odom_base_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_odom_to_base',
    #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    #     output='screen'
    # )

    # base_link -> camera_link (posición tomada del URDF que me pasaste; RPY=0 para mapeo "frame base")
    base_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_camera',
        arguments=['0.9206628', '0.0075201', '0.75542', '0', '0', '0', 'base_link', 'camera_link'],
        output='screen'
    )

    # (Opcional) alias Velodyne_link -> lidar_link (identidad)
    velodyne_to_lidar_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne_to_lidar_alias',
        arguments=['0', '0', '0', '0', '0', '0', 'Velodyne_link', 'lidar_link'],
        output='screen',
        condition=IfCondition(use_lidar_alias)
    )

    return LaunchDescription([
        use_lidar_alias_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        map_odom_tf,
        base_camera_tf,
        velodyne_to_lidar_alias,
    ])
