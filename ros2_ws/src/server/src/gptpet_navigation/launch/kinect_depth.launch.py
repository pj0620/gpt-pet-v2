from launch import LaunchDescription
from launch_ros.actions import Node

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

    tf_static_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_depth_frame']
    )
    
    # Bring up Nav2
    nav2_bringup = Node(
        package='nav2_bringup',
        executable='bringup_launch.py',
        output='screen',
        parameters=[{
            'use_sim_time': False,  # Set to True if using simulation
        }]
    )

    return LaunchDescription([
        depth_to_scan_node,
        tf_static_publisher,
        nav2_bringup
    ])