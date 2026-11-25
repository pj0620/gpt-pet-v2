import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('startup')
    xacro_path = os.path.join(pkg, 'urdf', 'gptpet.xacro')

    # allow overriding via CLI: `ros2 launch bringup.launch.py urdf_file:=...`
    urdf_launch_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=xacro_path,
        description='Full path to robot xacro file'
    )

    robot_description = Command(['xacro ', LaunchConfiguration('urdf_file')])
    
    # Only use the robot_state_publisher node with the robot_description parameter
    
    # depth_to_scan_node = Node(
    #   package='depthimage_to_laserscan',
    #   executable='depthimage_to_laserscan_node',
    #   name='depthimage_to_laserscan',
    #   output='screen',
    #   remappings=[
    #     ('depth', '/kinect/depth/image_raw'),
    #     ('depth_camera_info', '/kinect/depth/camera_info'),
    #     ('scan', '/scan')
    #   ],
    #   parameters=[{
    #     'scan_time': 0.033
    #   }]
    # )
    
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

    return LaunchDescription([
        urdf_launch_arg,
        # depth_to_scan_node,
        # slam_toolbox,

        # 1) robot_state_publisher: publishes base_link→base_laser + other static TFs
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # 2) ros2_control_node: loads hardware + controllers.yaml
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            namespace='',  # Empty namespace to ensure controllers are in global namespace
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                os.path.join(pkg, 'config', 'controllers.yaml')
            ]
        ),
        
        # 3) spawn joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # 4) spawn your mecanum drive controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['mecanum_drive_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
    ])
