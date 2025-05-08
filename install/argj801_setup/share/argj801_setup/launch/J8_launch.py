import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, TimerAction, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
import yaml

def load_yaml(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument('robot', default_value='false', description='Launch robot nodes if true'))
    ld.add_action(DeclareLaunchArgument('simulator', default_value='false', description='Launch simulator nodes if true'))
    ld.add_action(DeclareLaunchArgument('use_gui', default_value='false', description='Launch GUI node if true'))

    # Load the configuration
    config_path = os.path.join(get_package_share_directory('argj801_setup'), 'config', 'J8_params.yaml')
    yaml_config = load_yaml(config_path)
    global_params = yaml_config['ARGJ801']['global_parameters']

    # Function to select appropriate parameters
    def select_params(node_name):
        return yaml_config['ARGJ801'].get(node_name, {})

    # Function to set operation_mode parameter
    def get_ctrl_plataform_node(context):
        robot = LaunchConfiguration('robot').perform(context)
        operation_mode = 1 if robot == 'true' else 2
        ctrlPlataformNode = LifecycleNode(
            package='argj801_ctl_platform', executable='ARGJ801_ctl_platform', name='argj801_ctrl_platform_node', namespace='ARGJ801', output='screen',
            parameters=[global_params, select_params('argj801_ctrl_platform_node'), {'operation_mode': operation_mode}]
        )

        # Add ctrlPlataformNode to config and activate events
        config_events.append(
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(ctrlPlataformNode),
                    transition_id=Transition.TRANSITION_CONFIGURE
                )
            )
        )
        activate_events.append(
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(ctrlPlataformNode),
                    transition_id=Transition.TRANSITION_ACTIVATE
                )
            )
        )
        return [ctrlPlataformNode]

    # Define nodes with conditions to select appropriate parameters
    controlmissionNode = LifecycleNode(
        package='ctl_mission', executable='ctl_mission', name='ctl_mission_node', namespace='ARGJ801', output='screen', parameters=[global_params])
    controllerNode = LifecycleNode(
        package='ctl_mission', executable='controller_node', name='controller_node', namespace='ARGJ801', output='screen',
        parameters=[global_params, select_params('controller_node')])
    pathfollowingNode = LifecycleNode(
        package='ctl_mission', executable='path_following', name='path_following_node', namespace='ARGJ801', output='screen', parameters=[global_params, select_params('path_following_node')])
    teleoperationNode = LifecycleNode(
        package='ctl_mission', executable='teleoperation_node', name='teleoperation_node', namespace='ARGJ801', output='screen', parameters=[global_params, select_params('teleoperation_node')])
    pathRecordNode = LifecycleNode(
        package='ctl_mission', executable='path_record_node', name='path_record_node', namespace='ARGJ801', output='screen', parameters=[global_params, select_params('path_record_node')])
    readyNode = LifecycleNode(
        package='ctl_mission', executable='ready_node', name='ready_node', namespace='ARGJ801', output='screen', parameters=[global_params])
    estopNode = LifecycleNode(
        package='ctl_mission', executable='estop_node', name='estop_node', namespace='ARGJ801', output='screen', parameters=[global_params])
    backhomeNode = LifecycleNode(
        package='ctl_mission', executable='back_home_node', name='back_home_node', namespace='ARGJ801', output='screen', parameters=[global_params])
    pathManagerNode = LifecycleNode(
        package='path_manager', executable='path_manager_node', name='path_manager_node', namespace='ARGJ801', output='screen',
        parameters=[global_params, select_params('path_manager_node')])
    securityCheckNode = LifecycleNode(
        package='security_check', executable='security_check_node', name='security_check_node', namespace='ARGJ801', output='screen',
        parameters=[global_params, select_params('security_check_node')])
    MPCPlannerNode = LifecycleNode(
        package='ctl_mission', executable='mpc_node.py', name='mpc_node', namespace='ARGJ801', output='screen',
        parameters=[global_params, select_params('mpc_node')])
    joystickNode = LifecycleNode(
        package='joy', executable='joy_node', name='joy_node', namespace='ARGJ801', output='screen',
        parameters=[global_params, select_params('argj801_ctrl_platform_node')])
    fixpositionDriverNode = Node(
        package='fixposition_driver_ros2', executable='fixposition_driver_ros2_exec', name='fixposition_driver_ros2', output='screen',
        parameters=[global_params, select_params('fixposition_driver_ros2')])
    argj801_sensors = LifecycleNode(
        package='argj801_sensors', executable='ARGJ801_sensors_node', name='ARGJ801_sensors_node', namespace='ARGJ801', output='screen',
        parameters=[global_params, select_params('argj801_sensors')])
    android_server_node = LifecycleNode(
        package='android_ros2_server', executable='tcp_server_node', name='android_server', namespace='ARGJ801', output='log',
        parameters=[global_params, select_params('android_server_node')])
    tf_node_velodyne = Node(
        package='tf2_ros', executable='static_transform_publisher', name='static_transform_publisher', output='screen',
        arguments=['1.99348', '0', '0.27133', '1', '0', '0', '0', yaml_config['ARGJ801']['global_parameters']['robot_frame'], yaml_config['ARGJ801']['global_parameters']['velodyne_frame']])
    laser_segmentation_node = LifecycleNode(
        package='laser_segmentation', namespace='ARGJ801', executable='laser_segmentation', name='laser_segmentation',
        parameters=[global_params, select_params('laser_segmentation')])
    tf_node_sick = Node(
        package='tf2_ros', executable='static_transform_publisher', name='static_transform_publisher', output='screen',
        arguments=['1.86558', '0', '0.37865', '1', '0', '0', '0', yaml_config['ARGJ801']['global_parameters']['robot_frame'], yaml_config['ARGJ801']['global_parameters']['sick_frame']])
    gui_node = Node(
        package='GUI_pkg', executable='gui_node', name='gui_node', namespace='ARGJ801', output='screen',
        parameters=[global_params])

    # Configuration and activation events
    global config_events, activate_events
    config_events = []
    activate_events = []

    nodes_to_configure = [
        controlmissionNode, controllerNode, pathfollowingNode, backhomeNode, MPCPlannerNode,
        android_server_node, laser_segmentation_node, pathRecordNode, readyNode, estopNode,
        teleoperationNode, pathManagerNode, securityCheckNode
    ]

    for node in nodes_to_configure:
        config_events.append(
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=Transition.TRANSITION_CONFIGURE
                )
            )
        )

    nodes_to_activate = [
        controlmissionNode, readyNode, pathManagerNode,
        securityCheckNode, android_server_node, laser_segmentation_node
    ]

    for node in nodes_to_activate:
        activate_events.append(
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                )
            )
        )

    # Adding nodes to launch description based on conditions
    ld.add_action(LogInfo(condition=IfCondition(LaunchConfiguration('simulator')), msg="Launching simulator nodes"))
    ld.add_action(LogInfo(condition=IfCondition(LaunchConfiguration('robot')), msg="Launching robot nodes"))
    ld.add_action(LogInfo(condition=IfCondition(LaunchConfiguration('use_gui')), msg="Launching GUI node"))

    # Common nodes
    for node in [controlmissionNode, controllerNode, pathfollowingNode, teleoperationNode, pathRecordNode, readyNode, estopNode, backhomeNode, pathManagerNode, securityCheckNode, MPCPlannerNode, android_server_node, laser_segmentation_node]:
        ld.add_action(node)

    # Robot-specific nodes
    robot_specific_nodes = [fixpositionDriverNode, tf_node_velodyne, tf_node_sick, argj801_sensors]
    for node in robot_specific_nodes:
        ld.add_action(GroupAction(actions=[node], condition=IfCondition(LaunchConfiguration('robot'))))

    # Simulator-specific nodes
    simulator_specific_nodes = [joystickNode]
    for node in simulator_specific_nodes:
        ld.add_action(GroupAction(actions=[node], condition=IfCondition(LaunchConfiguration('simulator'))))

    # GUI node
    ld.add_action(GroupAction(actions=[gui_node], condition=IfCondition(LaunchConfiguration('use_gui'))))

    # Delayed configuration and activation
    ld.add_action(TimerAction(period=5.0, actions=config_events))
    ld.add_action(TimerAction(period=10.0, actions=activate_events))

    # Add the ctrlPlataformNode with the correct operation_mode
    ld.add_action(OpaqueFunction(function=get_ctrl_plataform_node))

    return ld

