from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rcj2025_interface',
            executable='manual_controller_node',
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            output='screen'
        ),
    ])