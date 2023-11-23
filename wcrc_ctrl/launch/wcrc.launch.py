from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wcrc_ctrl',
            executable='main',
            name='wcrc_main'
        ),
        Node(
            package='vision',
            executable='ShapeDetector',
            name='shape_detector'
        ),
    ])
