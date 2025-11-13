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
    
    nodes = []

    ## RTAB-Map SLAM ##
    nodes.append(
      Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        # output='screen',
        parameters=[{
          'use_sim_time': use_sim_time,
          'approx_sync': True,
          'queue_size': 30,
        }],
        remappings=[
          ('/rgb/image', '/kinect/image_raw'),
          ('/rgb/camera_info', '/kinect/camera_info'),
          ('/depth/image', '/kinect/depth/image_raw'),
          ('/depth/camera_info', '/kinect/depth/camera_info'),
        ],
      )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='debug',  # Options: debug, info, warn, error, fatal
            description='Logging level for SLAM Toolbox'
        ),
        use_sim_time_arg,
        *nodes
    ])
