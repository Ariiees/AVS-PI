from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    cfg = LaunchConfiguration('config_path')
    q_img = LaunchConfiguration('max_queue_image')
    q_lid = LaunchConfiguration('max_queue_lidar')
    q_gps = LaunchConfiguration('max_queue_gps')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='ROS namespace'),
        DeclareLaunchArgument('config_path',
                              default_value='/home/avs/AVS-PI/src/avs/config/avs_config.yaml',
                              description='Path to AVS YAML config'),
        DeclareLaunchArgument('max_queue_image', default_value='10', description='Image queue bound'),
        DeclareLaunchArgument('max_queue_lidar', default_value='10', description='LiDAR queue bound'),
        DeclareLaunchArgument('max_queue_gps',   default_value='10', description='GPS queue bound'),

        
        TimerAction(
            period=0.0,
            actions=[Node(
                package='avs',
                executable='lidar_sub_bench',
                namespace=ns,
                name='lidar_sub_bench',
                output='screen',
                parameters=[{'config_path': cfg}, {'max_queue': q_lid}],
            )]
        ),
        TimerAction(
            period=10.0,
            actions=[Node(
                package='avs',
                executable='image_sub_bench',
                namespace=ns,
                name='image_sub_bench',
                output='screen',
                parameters=[{'config_path': cfg}, {'max_queue': q_img}],
            )]
        ),
        TimerAction(
            period=20.0,
            actions=[Node(
                package='avs',
                executable='gps_sub_bench',
                namespace=ns,
                name='gps_sub_bench',
                output='screen',
                parameters=[{'config_path': cfg}, {'max_queue': q_gps}],
            )]
        ),
    ])
