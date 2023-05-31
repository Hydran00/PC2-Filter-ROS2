from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
import socket

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pc2_filter",
                executable="pc2_filter"
            )
        ]
    )
