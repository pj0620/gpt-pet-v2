# my_nav2_package/launch/kinect_depth.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch the Kinect (or Kinect-like) depth camera driver and optionally a
    node to convert depth images to LaserScan.
    """

    # # 1. Kinect/OpenNI2 depth camera driver node
    # #    (Replace 'ros2_openni2_camera' and 'openni2_camera_node' with the
    # #     correct package/executable for your specific device/driver.)
    # camera_driver_node = Node(
    #     package='ros2_openni2_camera',
    #     executable='openni2_camera_node',
    #     name='openni2_camera',
    #     output='screen',
    #     parameters=[{
    #         'depth_registration': True  # Example parameter
    #     }],
    #     # remappings=[...]  # Example: remap topics if needed
    # )

    # 2. (Optional) Convert depth images to a LaserScan
    #    This is useful if Nav2 or other parts of your system consume LaserScans.
    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        remappings=[
            # Remap to match whatever your camera publishes
            ('depth', '/kinect/depth/image_raw'),
            ('depth_camera_info', '/kinect/depth/camera_info'),
            ('scan', '/scan')
        ],
        parameters=[{
            # For instance, min/max ranges or other parameters:
            # 'range_min': 0.2,
            # 'range_max': 5.0,
            'scan_time': 0.033
        }]
    )

    return LaunchDescription([
        # camera_driver_node,
        depth_to_scan_node
    ])
