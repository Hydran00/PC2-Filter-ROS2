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
                executable="pc2_filter",
                parameters=[{
                    "input_topic_name": "/camera/depth/color/points",
                    "output_topic_name": "/filtered_pc2",
                    "voxel_grid_size": 0.01,
                    "segment_distance_min": 0.1,
                    "segment_distance_max": 1
                    }]
            )
        ]
    )
