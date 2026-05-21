import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('startup')

    topic_queue_size_arg = DeclareLaunchArgument(
        'topic_queue_size',
        default_value='30',
        description='Queue size for individual sensor subscriptions feeding RGB-D sync.'
    )
    sync_queue_size_arg = DeclareLaunchArgument(
        'sync_queue_size',
        default_value='5',
        description='Queue size used by message_filters synchronizers (lower reduces latency).'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time (server runs off robot clock if False).'
    )
    topic_queue_size = LaunchConfiguration('topic_queue_size')
    sync_queue_size = LaunchConfiguration('sync_queue_size')
    use_sim_time = LaunchConfiguration('use_sim_time')

    nodes = []

    # Provide TF tree for downstream consumers
    # xacro_file = os.path.join(pkg, 'urdf', 'gptpet.xacro')
    # robot_description = {'robot_description': Command(['xacro ', xacro_file])}
    # nodes.append(
    #     Node(
    #         package='robot_state_publisher',
    #         executable='robot_state_publisher',
    #         parameters=[robot_description, {
    #             'use_sim_time': use_sim_time,
    #         }]
    #     )
    # )

    # Use rtabmap_sync to align RGB-D topics before feeding SLAM
    nodes.append(
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            namespace='rtab_sync',
            parameters=[{
                'approx_sync': True,
                'topic_queue_size': topic_queue_size,
                'sync_queue_size': sync_queue_size,
                # Kinect bridge publishes RELIABLE; match it to establish DDS links.
                'qos': 1,  # 0=system default, 1=reliable, 2=best-effort
                'qos_camera_info': 1,
            }],
            remappings=[
                ('rgb/image', '/kinect/image_raw'),
                ('rgb/camera_info', '/kinect/camera_info'),
                ('depth/image', '/kinect/depth/image_raw'),
                ('depth/camera_info', '/kinect/depth/camera_info'),
                ('rgbd_image', '/rtab_sync/rgbd_image'),
            ],
        )
    )

    # nodes.append(
    #     Node(
    #         package='rtabmap_odom',
    #         executable='rgbd_odometry',
    #         name='rtabmap_odom',
    #         namespace='rtabmap_odom',
    #         parameters=[{
    #             'use_sim_time': use_sim_time,
    #             'subscribe_rgbd': True,
    #             'approx_sync': True,
    #             'topic_queue_size': 50,
    #             'frame_id': 'base_link',
    #             'odom_frame_id': 'rtabmap_odom',
    #             'publish_tf': True,
    #             'wait_for_transform': 0.2,
    #             'qos': 1,
    #         }],
    #         remappings=[
    #             ('imu', '/imu/combined'),
    #             # ('imu', '/imu/mag_raw'),
    #             ('rgbd_image', '/rtab_sync/rgbd_image'),
    #             ('odom', '/rtabmap_odom/odom'),
    #         ],
    #     )
    # )

    # cartographer_node = Node(
    #     package='cartographer_ros',
    #     executable='cartographer_node',
    #     name='cartographer_node',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #     }],
    #     arguments=[
    #         '-configuration_directory', os.path.join(pkg, 'config'),
    #         '-configuration_basename', 'cartographer_2d.lua',
    #         '--ros-args', '--log-level', LaunchConfiguration('log_level'),
    #     ],
    #     remappings=[
    #         ('scan', '/scan'),
    #         # Cartographer takes a single sensor_msgs/Imu input.
    #         # We publish a fused IMU on /imu/combined from:
    #         # - /imu/data_raw (gyro + accel)
    #         # - /imu/mag_orientation (orientation)
    #         ('imu', '/imu/combined'),
    #     ],
    # )
    # nodes.append(cartographer_node)

    # occupancy_grid_node = Node(
    #     package='cartographer_ros',
    #     executable='cartographer_occupancy_grid_node',
    #     name='occupancy_grid_node',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'resolution': 0.05,
    #         'publish_period_sec': 1.0,
    #     }],
    # )
    # nodes.append(occupancy_grid_node)

    ## MAGNETOMETER CONVERTER ##
    nodes.append(
        Node(
            package="startup",
            executable="magnetometer_converter",
            name="magnetometer_converter",
            parameters=[
                {"use_sim_time": use_sim_time},
            ],
        )
    )

    # IMU fusion: combine orientation from /imu/mag_orientation with
    # angular velocity and linear acceleration from /imu/data_raw
    nodes.append(
        Node(
            package="startup",
            executable="imu_fusion",
            name="imu_fusion",
            parameters=[
                {"use_sim_time": use_sim_time},
            ],
        )
    )

    ## STATIC TRANSFORMS ##
    # Add static transform from base_link to base_footprint if needed
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_publisher',
            arguments=['0', '0', '0', '0', '0',
                       '0', 'base_link', 'base_footprint'],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    )

    # Add static transform from base_link to laser (lidar frame 0.2m above base,
    # rotated 180 degrees about the x-axis)
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_laser_transform_publisher',
            # arguments=['0', '0', '0.2', '3.14', '0',
            arguments=['0', '0', '0.2', '0', '0',
                       '0', 'base_link', 'laser'],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    )

    # Add static transform from base_link to IMU frame
    # Adjust the position values based on where your IMU is mounted on the robot
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0',
                       '0', 'base_link', 'imu_icm20948'],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='info',  # Options: debug, info, warn, error, fatal
            description='Logging level for Cartographer'
        ),
        topic_queue_size_arg,
        sync_queue_size_arg,
        use_sim_time_arg,
        *nodes
    ])
