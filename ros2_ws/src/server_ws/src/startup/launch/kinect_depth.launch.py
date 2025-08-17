from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    depth_to_scan_node = Node(
      package='depthimage_to_laserscan',
      executable='depthimage_to_laserscan_node',
      name='depthimage_to_laserscan',
      output='screen',
      remappings=[
        ('depth', '/kinect/depth/image_raw'),
        ('depth_camera_info', '/kinect/depth/camera_info'),
        ('scan', '/scan')
      ],
      parameters=[{
        'scan_time': 0.033
      }]
    )

    # tf_static_publisher = Node(
    #   package='tf2_ros',
    #   executable='static_transform_publisher',
    #   name='static_tf_pub',
    #   arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_depth_frame']
    # )
    
    # Run SLAM to create /odom -> base_link dynamically
    # slam_toolbox = Node(
    #   package='slam_toolbox',
    #   executable='async_slam_toolbox_node',
    #   name='slam_toolbox',
    #   output='screen',
    #   parameters=[{
    #       'use_sim_time': False,
    #       'odom_frame': 'odom',  # SLAM needs this frame
    #       'base_frame': 'base_link',  # Robot's main frame
    #       'map_frame': 'map',
    #       'scan_topic': '/scan',
    #       'mode': 'mapping',
    #       'transform_publish_period': 0.05,  # Ensure transforms are published regularly
    #       'transform_timeout': 0.5  # Reduce timeout issues
    #   }]
    # )
    
    # navigation_launch = IncludeLaunchDescription(
    #   launch_description_source=PythonLaunchDescriptionSource(
    #     PathJoinSubstitution(
    #       [
    #         FindPackageShare("nav2_bringup"),
    #         "launch",
    #         "navigation_launch.py",
    #       ]
    #     )
    #   ),
    # )

    return LaunchDescription([
        depth_to_scan_node,
        # tf_static_publisher,
        # slam_toolbox,
        # navigation_launch
    ])
