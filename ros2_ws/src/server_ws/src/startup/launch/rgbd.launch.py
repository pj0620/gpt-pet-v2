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
    # Use rtabmap_sync to align RGB-D topics before feeding SLAM
    nodes.append(
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            parameters=[{
                'approx_sync': True,
                'topic_queue_size': 100,
                'sync_queue_size': 100,
                # Kinect driver publishes with RELIABLE QoS; match it so endpoints connect.
                'qos': 1,  # 0=system default, 1=reliable, 2=best-effort
                'qos_camera_info': 1,
            }],
            remappings=[
                ('rgb/image', '/kinect/image_raw'),
                ('rgb/camera_info', '/kinect/camera_info'),
                ('depth/image', '/kinect/depth/image_raw'),
                ('depth/camera_info', '/kinect/depth/camera_info'),
                ('rgbd_image', '/rtabmap/rgbd_image'),
            ],
        )
    )

    nodes.append(
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            # output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'subscribe_rgbd': True,
                'subscribe_depth': False,
                'approx_sync': True,
                'sync_queue_size': 100,
                'topic_queue_size': 100,
                'subscribe_odom': False,
                'qos': 1,
            }],
            remappings=[
                ('rgbd_image', '/rtabmap/rgbd_image'),
            ],
        )
    )

    ## MAGNETOMETER CONVERTER ##
    nodes.append(
        Node(
            package="startup",
            executable="magnetometer_converter",
            name="magnetometer_converter",
            parameters=[
                {"use_sim_time": False},
            ],
        )
    )

    ## SENSOR FUSION - ROBOT LOCALIZATION ##
    # Fuse wheel odometry (x,y position) with IMU (orientation)
    nodes.append(
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'frequency': 0.5,
                'sensor_timeout': 0.5,  # Increased timeout to be more tolerant
                'two_d_mode': True,
                'transform_time_offset': 0.0,
                'transform_timeout': 0.0,
                'print_diagnostics': True,
                'debug': False,

                # Additional stability parameters
                'smooth_lagged_data': True,
                'history_length': 0.5,

                # Output frame configuration
                'map_frame': 'odom',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',

                # Odometry source configuration (wheel odometry) - more conservative
                'odom0': '/mecanum_drive_controller/odometry',
                'odom0_config': [
                    True,  # x position
                    True,  # y position
                    False,  # z position (2D mode)
                    False,  # roll
                    False,  # pitch
                    False,  # yaw (will use IMU for this)
                    # x velocity (disable to reduce drift when stationary)
                    False,
                    # y velocity (disable to reduce drift when stationary)
                    False,
                    False,  # z velocity
                    False,  # roll velocity
                    False,  # pitch velocity
                    False,  # yaw velocity (will use IMU for this)
                    False,  # x acceleration
                    False,  # y acceleration
                    False  # z acceleration
                ],
                'odom0_differential': True,  # Use differential mode for better stability
                'odom0_relative': False,
                'odom0_queue_size': 10,
                'odom0_nodelay': False,

                # IMU source configuration for angular velocity
                'imu0': '/imu/data_raw',
                'imu0_config': [
                    False,  # x position
                    False,  # y position
                    False,  # z position
                    False,  # roll
                    False,  # pitch
                    False,  # yaw (orientation not reliable from this IMU)
                    False,  # x velocity
                    False,  # y velocity
                    False,  # z velocity
                    False,  # roll velocity
                    False,  # pitch velocity
                    True,  # yaw velocity (angular velocity from gyroscope)
                    False,  # x acceleration
                    False,  # y acceleration
                    False  # z acceleration
                ],
                'imu0_differential': False,
                'imu0_relative': True,
                'imu0_queue_size': 10,
                'imu0_nodelay': False,
                'imu0_remove_gravitational_acceleration': True,

                # Magnetometer source configuration for absolute heading
                'imu1': '/imu/mag_orientation',
                'imu1_config': [
                    False,  # x position
                    False,  # y position
                    False,  # z position
                    False,  # roll
                    False,  # pitch
                    True,  # yaw (absolute orientation from magnetometer)
                    False,  # x velocity
                    False,  # y velocity
                    False,  # z velocity
                    False,  # roll velocity
                    False,  # pitch velocity
                    False,  # yaw velocity
                    False,  # x acceleration
                    False,  # y acceleration
                    False  # z acceleration
                ],
                'imu1_differential': False,
                'imu1_relative': False,  # Absolute measurements from magnetometer
                'imu1_queue_size': 10,
                'imu1_nodelay': False,

                # Additional IMU parameters for better acceleration handling
                'gravitational_acceleration': 9.80665,
                'use_control': False,
                'stamped_control': False,
                'control_timeout': 0.2,

                # Process noise covariance (conservative values for stability)
                'process_noise_covariance': [
                    0.02, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  0.02, 0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.03, 0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01,  0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.01,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.02, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.005, 0.0, 0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.005, 0.0, 0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.005, 0.0, 0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.005, 0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01
                ],

                # Initial estimate covariance
                'initial_estimate_covariance': [
                    1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9
                ]
            }],
            remappings=[
                ('/odometry/filtered', '/odom')
            ]
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
            parameters=[{'use_sim_time': False}]
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
            parameters=[{'use_sim_time': False}]
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
