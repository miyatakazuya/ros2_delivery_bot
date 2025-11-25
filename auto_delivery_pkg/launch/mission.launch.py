import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Load config
    pkg_share = get_package_share_directory('auto_delivery_pkg')
    config_path = os.path.join(pkg_share, 'config', 'node_config.yaml')
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            nodes_config = config['delivery_nodes']
    except FileNotFoundError:
        print(f"ERROR: Config file not found at {config_path}")
        return LaunchDescription()

    launch_actions = []

    if nodes_config.get('mission_controller', 1):
        launch_actions.append(Node(
            package='auto_delivery_pkg',
            executable='mission_controller',
            name='mission_controller',
            output='screen'
        ))

    if nodes_config.get('parking_controller', 1):
        launch_actions.append(Node(
            package='auto_delivery_pkg',
            executable='parking_controller',
            name='parking_controller',
            output='screen'
        ))

    if nodes_config.get('servo_controller', 1):
        launch_actions.append(Node(
            package='auto_delivery_pkg',
            executable='servo_controller',
            name='servo_controller',
            output='screen'
        ))

    if nodes_config.get('box_detection', False):
        launch_actions.append(Node(
            package='auto_delivery_pkg',
            executable='box_detection',
            name='box_detection',
            output='screen'
        ))
    
    if nodes_config.get('apriltag_node', False):
        launch_actions.append(Node(
            package='auto_delivery_pkg',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen'
        ))

    return LaunchDescription(launch_actions)