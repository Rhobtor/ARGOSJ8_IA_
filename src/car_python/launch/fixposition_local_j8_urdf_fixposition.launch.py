from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Frames (our TF tree)
    map_frame_arg = DeclareLaunchArgument("map_frame", default_value="map")
    odom_frame_arg = DeclareLaunchArgument("odom_frame", default_value="odom")
    base_link_frame_arg = DeclareLaunchArgument("base_link_frame", default_value="base_link")

    # Fixposition input
    odom_topic_arg = DeclareLaunchArgument("odom_topic", default_value="/fixposition/odometry_enu")
    fp_frame_arg = DeclareLaunchArgument("fp_frame", default_value="FP_ENU0")
    fixposition_robot_frame_arg = DeclareLaunchArgument("fixposition_robot_frame", default_value="FP_POI")
    publish_map_to_fp_tf_arg = DeclareLaunchArgument(
        "publish_map_to_fp_tf",
        default_value="False",
        description="If True, also publish map->fp_frame. Keep False when Fixposition already publishes its own FP TF tree.",
    )

    anchor_on_arg = DeclareLaunchArgument(
        "anchor_on",
        default_value="base_link_frame",
        description="Anchor map origin on 'fixposition_robot_frame' (FP_POI) or 'base_link_frame'",
    )
    use_msg_stamp_arg = DeclareLaunchArgument(
        "use_msg_stamp",
        default_value="False",
        description="If True, stamp TF with odometry stamp; else stamp with now() to avoid TF_OLD_DATA.",
    )

    # Optional rigid offset Fixposition robot frame -> base_link
    x_arg = DeclareLaunchArgument("x", default_value="-0.8191864")
    y_arg = DeclareLaunchArgument("y", default_value="-0.9168644")
    z_arg = DeclareLaunchArgument("z", default_value="-0.0165064")
    roll_arg = DeclareLaunchArgument("roll", default_value="1.5771804")
    pitch_arg = DeclareLaunchArgument("pitch", default_value="-0.0023478")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="1.5707248")
    apply_orientation_offset_arg = DeclareLaunchArgument(
        "apply_orientation_offset",
        default_value="False",
        description="If True, apply the rigid FP_POI->base_link rotation to the dynamic odom->base_link TF. Keep False when Fixposition orientation already matches base_link motion axes.",
    )
    align_yaw_arg = DeclareLaunchArgument(
        "align_yaw",
        default_value="False",
        description="If True, anchor map so the initial base_link yaw is 0 (keeps only yaw alignment).",
    )

    # Mounting TFs
    publish_base_camera_tf_arg = DeclareLaunchArgument(
        "publish_base_camera_tf",
        default_value="True",
        description="Publish a static TF base_link->camera_link",
    )
    camera_parent_frame_arg = DeclareLaunchArgument("camera_parent_frame", default_value="base_link")
    camera_frame_arg = DeclareLaunchArgument("camera_frame", default_value="camera_link")
    camera_x_arg = DeclareLaunchArgument("camera_x", default_value="0.9206628")
    camera_y_arg = DeclareLaunchArgument("camera_y", default_value="0.0075201")
    camera_z_arg = DeclareLaunchArgument("camera_z", default_value="0.75542")
    camera_roll_arg = DeclareLaunchArgument("camera_roll", default_value="0.0")
    camera_pitch_arg = DeclareLaunchArgument("camera_pitch", default_value="0.0")
    camera_yaw_arg = DeclareLaunchArgument("camera_yaw", default_value="0.0")

    publish_camera_to_zed_link_tf_arg = DeclareLaunchArgument(
        "publish_camera_to_zed_link_tf",
        default_value="True",
        description="Publish TF camera_link->zed_camera_link so the real ZED mounting matches the current robot setup.",
    )
    zed_link_parent_frame_arg = DeclareLaunchArgument("zed_link_parent_frame", default_value="camera_link")
    zed_link_frame_arg = DeclareLaunchArgument("zed_link_frame", default_value="zed_camera_link")
    zed_link_x_arg = DeclareLaunchArgument("zed_link_x", default_value="0.0")
    zed_link_y_arg = DeclareLaunchArgument("zed_link_y", default_value="0.0")
    zed_link_z_arg = DeclareLaunchArgument("zed_link_z", default_value="0.0585")
    zed_link_roll_arg = DeclareLaunchArgument("zed_link_roll", default_value="0.0")
    zed_link_pitch_arg = DeclareLaunchArgument("zed_link_pitch", default_value="0.0")
    zed_link_yaw_arg = DeclareLaunchArgument("zed_link_yaw", default_value="0.0")

    publish_zed_internal_tf_arg = DeclareLaunchArgument(
        "publish_zed_internal_tf",
        default_value="True",
        description="Publish a minimal ZED internal TF tree rooted at zed_camera_link.",
    )
    zed_baseline_arg = DeclareLaunchArgument(
        "zed_baseline_m",
        default_value="0.12",
        description="ZED stereo baseline in meters.",
    )

    publish_velodyne_alias_tf_arg = DeclareLaunchArgument(
        "publish_velodyne_alias_tf",
        default_value="True",
        description="Publish TF Velodyne_link->velodyne for remote point clouds that arrive in frame velodyne.",
    )
    velodyne_alias_yaw_arg = DeclareLaunchArgument(
        "velodyne_alias_yaw",
        default_value="0.0",
        description="Yaw offset in radians for Velodyne_link->velodyne. Default keeps the point cloud aligned with Velodyne_link.",
    )

    # URDF
    desc_pkg = "j8_xacro_model"
    xacro_file = os.path.join(get_package_share_directory(desc_pkg), "urdf", "argo_j8.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_urdf}],
        output="screen",
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_gui": False}],
    )

    base_to_camera_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_tf",
        output="screen",
        condition=IfCondition(LaunchConfiguration("publish_base_camera_tf")),
        arguments=[
            "--frame-id", LaunchConfiguration("camera_parent_frame"),
            "--child-frame-id", LaunchConfiguration("camera_frame"),
            "--x", LaunchConfiguration("camera_x"),
            "--y", LaunchConfiguration("camera_y"),
            "--z", LaunchConfiguration("camera_z"),
            "--roll", LaunchConfiguration("camera_roll"),
            "--pitch", LaunchConfiguration("camera_pitch"),
            "--yaw", LaunchConfiguration("camera_yaw"),
        ],
    )

    camera_to_zed_link_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_to_zed_link_tf",
        output="screen",
        condition=IfCondition(LaunchConfiguration("publish_camera_to_zed_link_tf")),
        arguments=[
            "--frame-id", LaunchConfiguration("zed_link_parent_frame"),
            "--child-frame-id", LaunchConfiguration("zed_link_frame"),
            "--x", LaunchConfiguration("zed_link_x"),
            "--y", LaunchConfiguration("zed_link_y"),
            "--z", LaunchConfiguration("zed_link_z"),
            "--roll", LaunchConfiguration("zed_link_roll"),
            "--pitch", LaunchConfiguration("zed_link_pitch"),
            "--yaw", LaunchConfiguration("zed_link_yaw"),
        ],
    )

    velodyne_link_to_velodyne_alias = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="velodyne_link_to_velodyne_alias",
        arguments=[
            "--frame-id", "Velodyne_link",
            "--child-frame-id", "velodyne",
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", LaunchConfiguration("velodyne_alias_yaw"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("publish_velodyne_alias_tf")),
    )

    # Fixposition -> map/odom/base_link + mounting TFs
    fixposition_tf_tree_node = Node(
        package="car",
        executable="fixposition_tf_tree_for_zed",
        name="fixposition_tf_tree_for_zed",
        output="screen",
        parameters=[
            {
                "odom_topic": LaunchConfiguration("odom_topic"),
                "map_frame": LaunchConfiguration("map_frame"),
                "odom_frame": LaunchConfiguration("odom_frame"),
                "base_link_frame": LaunchConfiguration("base_link_frame"),
                "fp_frame": LaunchConfiguration("fp_frame"),
                "fixposition_robot_frame": LaunchConfiguration("fixposition_robot_frame"),
                "publish_map_to_fp_tf": LaunchConfiguration("publish_map_to_fp_tf"),
                "anchor_on": LaunchConfiguration("anchor_on"),
                "use_msg_stamp": LaunchConfiguration("use_msg_stamp"),
                "x": LaunchConfiguration("x"),
                "y": LaunchConfiguration("y"),
                "z": LaunchConfiguration("z"),
                "roll": LaunchConfiguration("roll"),
                "pitch": LaunchConfiguration("pitch"),
                "yaw": LaunchConfiguration("yaw"),
                "apply_orientation_offset": LaunchConfiguration("apply_orientation_offset"),
                "align_yaw": LaunchConfiguration("align_yaw"),
                "publish_base_camera_tf": False,
                "camera_parent_frame": LaunchConfiguration("camera_parent_frame"),
                "camera_frame": LaunchConfiguration("camera_frame"),
                "camera_x": LaunchConfiguration("camera_x"),
                "camera_y": LaunchConfiguration("camera_y"),
                "camera_z": LaunchConfiguration("camera_z"),
                "camera_roll": LaunchConfiguration("camera_roll"),
                "camera_pitch": LaunchConfiguration("camera_pitch"),
                "camera_yaw": LaunchConfiguration("camera_yaw"),
                "publish_camera_to_zed_link_tf": False,
                "zed_link_parent_frame": LaunchConfiguration("zed_link_parent_frame"),
                "zed_link_frame": LaunchConfiguration("zed_link_frame"),
                "zed_link_x": LaunchConfiguration("zed_link_x"),
                "zed_link_y": LaunchConfiguration("zed_link_y"),
                "zed_link_z": LaunchConfiguration("zed_link_z"),
                "zed_link_roll": LaunchConfiguration("zed_link_roll"),
                "zed_link_pitch": LaunchConfiguration("zed_link_pitch"),
                "zed_link_yaw": LaunchConfiguration("zed_link_yaw"),
            }
        ],
    )

    zed_internal_tf_tree_node = Node(
        package="car",
        executable="zed_static_tf_tree",
        name="zed_static_tf_tree",
        output="screen",
        condition=IfCondition(LaunchConfiguration("publish_zed_internal_tf")),
        parameters=[
            {
                "root_frame": LaunchConfiguration("zed_link_frame"),
                "baseline_m": LaunchConfiguration("zed_baseline_m"),
            }
        ],
    )

    return LaunchDescription(
        [
            map_frame_arg,
            odom_frame_arg,
            base_link_frame_arg,
            odom_topic_arg,
            fp_frame_arg,
            fixposition_robot_frame_arg,
            publish_map_to_fp_tf_arg,
            anchor_on_arg,
            use_msg_stamp_arg,
            x_arg,
            y_arg,
            z_arg,
            roll_arg,
            pitch_arg,
            yaw_arg,
            apply_orientation_offset_arg,
            align_yaw_arg,
            publish_base_camera_tf_arg,
            camera_parent_frame_arg,
            camera_frame_arg,
            camera_x_arg,
            camera_y_arg,
            camera_z_arg,
            camera_roll_arg,
            camera_pitch_arg,
            camera_yaw_arg,
            publish_camera_to_zed_link_tf_arg,
            zed_link_parent_frame_arg,
            zed_link_frame_arg,
            zed_link_x_arg,
            zed_link_y_arg,
            zed_link_z_arg,
            zed_link_roll_arg,
            zed_link_pitch_arg,
            zed_link_yaw_arg,
            publish_zed_internal_tf_arg,
            zed_baseline_arg,
            publish_velodyne_alias_tf_arg,
            velodyne_alias_yaw_arg,
            robot_state_publisher_node,
            joint_state_publisher_node,
            base_to_camera_tf_node,
            camera_to_zed_link_tf_node,
            velodyne_link_to_velodyne_alias,
            fixposition_tf_tree_node,
            zed_internal_tf_tree_node,
        ]
    )
