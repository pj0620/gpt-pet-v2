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

    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        remappings=[
            ('depth', '/kinect/depth/image_raw'),
            ('depth_camera_info', '/kinect/depth/camera_info'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'scan_time': 0.033,
            'output_frame': 'base_laser_link',
            'range_min': 0.1,
            'range_max': 5.0,
        }],
    )

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
                'transform_timeout': 0.5,
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
        depth_to_scan_node,
        slam_toolbox_node,
    ])
