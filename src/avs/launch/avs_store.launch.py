# avs/launch/avs_three_subscribers.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Optional ROS namespace for all three subscribers",
    )

    ns = LaunchConfiguration("namespace")

    return LaunchDescription([
        ns_arg,

        Node(
            package="avs",
            executable="image_subscriber",
            name="image_subscriber",
            namespace=ns,
            output="screen",
        ),
        Node(
            package="avs",
            executable="lidar_subscriber",
            name="lidar_subscriber",
            namespace=ns,
            output="screen",
        ),
        Node(
            package="avs",
            executable="gps_subscriber",
            name="gps_subscriber",
            namespace=ns,
            output="screen",
        ),
    ])
