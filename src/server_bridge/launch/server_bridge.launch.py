"""Launch server_bridge node."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="server_bridge",
            executable="server_bridge_node",
            name="server_bridge_node",
            output="screen",
        ),
    ])
