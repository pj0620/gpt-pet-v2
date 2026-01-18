import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
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

    nodes.append(
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rtabmap_odom',
            namespace='rtabmap_odom',
            parameters=[{
                'use_sim_time': use_sim_time,
                'subscribe_rgbd': True,
                'approx_sync': True,
                'topic_queue_size': 50,
                'frame_id': 'base_link',
                'odom_frame_id': 'rtabmap_odom',
                'publish_tf': True,
                'wait_for_transform': 0.2,
                'qos': 1,
            }],
            remappings=[
                ('imu', '/imu/combined'),
                # ('imu', '/imu/mag_raw'),
                ('rgbd_image', '/rtab_sync/rgbd_image'),
                ('odom', '/rtabmap_odom/odom'),
            ],
        )
    )

    # nodes.append(
    #     Node(
    #         package='rtabmap_slam',
    #         executable='rtabmap',
    #         name='rtabmap',
    #         namespace='rtab_map_slam',
    #         # output='screen',
    #         parameters=[{
    #             'use_sim_time': use_sim_time,
    #             'subscribe_rgbd': True,
    #             'subscribe_depth': False,
    #             'approx_sync': True,
    #             'sync_queue_size': 100,
    #             'topic_queue_size': 100,
    #             'subscribe_odom': True,
    #             'qos': 1,
    #             'frame_id_prefix': 'rtabmap',
    #         }],
    #         remappings=[
    #             ('rgbd_image', '/rtabmap/rgbd_image'),
    #             ('odom', '/rtabmap/odom'),
    #         ],
    #     )
    # )

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

    # IMU fusion: combine orientation from /imu/mag_orientation with
    # angular velocity and linear acceleration from /imu/data_raw
    nodes.append(
        Node(
            package="startup",
            executable="imu_fusion",
            name="imu_fusion",
            parameters=[
                {"use_sim_time": False},
            ],
        )
    )

    ## SENSOR FUSION - ROBOT LOCALIZATION ##
    # Fuse wheel odometry (x,y position) with IMU (orientation)
    # nodes.append(
    #     Node(
    #         package='robot_localization',
    #         executable='ekf_node',
    #         name='ekf_filter_node',
    #         output='screen',
    #         parameters=[{
    #             'use_sim_time': False,
    #             'frequency': 0.5,
    #             'sensor_timeout': 0.5,  # Increased timeout to be more tolerant
    #             'two_d_mode': True,
    #             'transform_time_offset': 0.0,
    #             'transform_timeout': 0.0,
    #             'print_diagnostics': True,
    #             'debug': False,

    #             # Additional stability parameters
    #             'smooth_lagged_data': True,
    #             'history_length': 0.5,

    #             # Output frame configuration (map->odom->base_link chain must be unique)
    #             'map_frame': 'map',
    #             'odom_frame': 'odom',
    #             'base_link_frame': 'base_link',
    #             'world_frame': 'odom',

    #             # Odometry source configuration (wheel odometry) - more conservative
    #             'odom0': '/mecanum_drive_controller/odometry',
    #             'odom0_config': [
    #                 True,  # x position
    #                 True,  # y position
    #                 False,  # z position (2D mode)
    #                 False,  # roll
    #                 False,  # pitch
    #                 False,  # yaw (will use IMU for this)
    #                 # x velocity (disable to reduce drift when stationary)
    #                 False,
    #                 # y velocity (disable to reduce drift when stationary)
    #                 False,
    #                 False,  # z velocity
    #                 False,  # roll velocity
    #                 False,  # pitch velocity
    #                 False,  # yaw velocity (will use IMU for this)
    #                 False,  # x acceleration
    #                 False,  # y acceleration
    #                 False  # z acceleration
    #             ],
    #             'odom0_differential': True,  # Use differential mode for better stability
    #             'odom0_relative': False,
    #             'odom0_queue_size': 10,
    #             'odom0_nodelay': False,

    #             # IMU source configuration for angular velocity
    #             'imu0': '/imu/data_raw',
    #             'imu0_config': [
    #                 False,  # x position
    #                 False,  # y position
    #                 False,  # z position
    #                 False,  # roll
    #                 False,  # pitch
    #                 False,  # yaw (orientation not reliable from this IMU)
    #                 False,  # x velocity
    #                 False,  # y velocity
    #                 False,  # z velocity
    #                 False,  # roll velocity
    #                 False,  # pitch velocity
    #                 True,  # yaw velocity (angular velocity from gyroscope)
    #                 False,  # x acceleration
    #                 False,  # y acceleration
    #                 False  # z acceleration
    #             ],
    #             'imu0_differential': False,
    #             'imu0_relative': True,
    #             'imu0_queue_size': 10,
    #             'imu0_nodelay': False,
    #             'imu0_remove_gravitational_acceleration': True,

    #             # Magnetometer source configuration for absolute heading
    #             'imu1': '/imu/mag_orientation',
    #             'imu1_config': [
    #                 False,  # x position
    #                 False,  # y position
    #                 False,  # z position
    #                 False,  # roll
    #                 False,  # pitch
    #                 True,  # yaw (absolute orientation from magnetometer)
    #                 False,  # x velocity
    #                 False,  # y velocity
    #                 False,  # z velocity
    #                 False,  # roll velocity
    #                 False,  # pitch velocity
    #                 False,  # yaw velocity
    #                 False,  # x acceleration
    #                 False,  # y acceleration
    #                 False  # z acceleration
    #             ],
    #             'imu1_differential': False,
    #             'imu1_relative': False,  # Absolute measurements from magnetometer
    #             'imu1_queue_size': 10,
    #             'imu1_nodelay': False,

    #             # Additional IMU parameters for better acceleration handling
    #             'gravitational_acceleration': 9.80665,
    #             'use_control': False,
    #             'stamped_control': False,
    #             'control_timeout': 0.2,

    #             # Process noise covariance (conservative values for stability)
    #             'process_noise_covariance': [
    #                 0.02, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.02, 0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  0.03, 0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01,  0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.01,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.02, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.005, 0.0, 0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.005, 0.0, 0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.005, 0.0, 0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.005, 0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01
    #             ],

    #             # Initial estimate covariance
    #             'initial_estimate_covariance': [
    #                 1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
    #                 0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,
    #                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9
    #             ]
    #         }],
    #         remappings=[
    #             ('/odometry/filtered', '/odom')
    #         ]
    #     )
    # )

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
        topic_queue_size_arg,
        sync_queue_size_arg,
        use_sim_time_arg,
        *nodes
    ])
