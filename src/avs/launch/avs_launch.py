from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='avs',
            executable='image_subscriber',
            name='image_sub_node',
            output='screen',
            parameters=[{'output_dir': '/path/to/output'}]
        )
    ])
