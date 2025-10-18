import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('startup')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time (server runs off robot clock if False).'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        # arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            os.path.join(pkg, 'config', 'slam_toolbox_parameters.yaml'),
            {
                'use_sim_time': use_sim_time,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'transform_publish_period': 0.05,
                'transform_timeout': 2.0,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='debug',  # Options: debug, info, warn, error, fatal
            description='Logging level for SLAM Toolbox'
        ),
        use_sim_time_arg,
        slam_toolbox_node,
    ])
