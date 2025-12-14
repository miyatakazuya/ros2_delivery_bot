import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_share = get_package_share_directory('auto_delivery_pkg')
    
    node_config_path = os.path.join(pkg_share, 'config', 'node_config.yaml')
    try:
        with open(node_config_path, 'r') as f:
            config = yaml.safe_load(f)
            nodes_to_run = config['delivery_nodes']
    except Exception as e:
        print(f"ERROR loading node_config: {e}")
        return LaunchDescription()

    vis_config_path = os.path.join(pkg_share, 'config', 'visualization.yaml')

    launch_actions = []

    if nodes_to_run.get('mission_controller', 1):
        launch_actions.append(Node(
            package='auto_delivery_pkg',
            executable='mission_controller',
            name='mission_controller',
            output='screen'
        ))

        launch_actions.append(Node(
            package='auto_delivery_pkg',
            executable='motion_controller',
            name='motion_controller',
            output='screen'
        ))

    if nodes_to_run.get('box_detection', False):
        launch_actions.append(Node(
            package='auto_delivery_pkg',
            executable='oak_perception_node',
            name='oak_perception_node',
            output='screen',
            parameters=[vis_config_path] 
        ))
    
    if nodes_to_run.get('apriltag_node', False):
        launch_actions.append(Node(
            package='auto_delivery_pkg',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            parameters=[vis_config_path] 
        ))

    if nodes_to_run.get('apriltag_rear_node', False):
        launch_actions.append(Node(
            package='auto_delivery_pkg',
            executable='apriltag_rear_node',
            name='apriltag_rear_node',
            output='screen',
            parameters=[vis_config_path],
        ))

    if nodes_to_run.get('servo_controller', 1):
        launch_actions.append(Node(
            package='auto_delivery_pkg',
            executable='servo_controller',
            name='servo_controller',
            output='screen'
        ))

    return LaunchDescription(launch_actions)