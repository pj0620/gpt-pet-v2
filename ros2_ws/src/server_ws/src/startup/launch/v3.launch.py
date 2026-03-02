import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("startup")

    # Launch args
    topic_queue_size_arg = DeclareLaunchArgument(
        "topic_queue_size",
        default_value="30",
        description="Queue size for individual sensor subscriptions feeding RGB-D sync.",
    )
    sync_queue_size_arg = DeclareLaunchArgument(
        "sync_queue_size",
        default_value="5",
        description="Queue size used by message_filters synchronizers (lower reduces latency).",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation time.",
    )

    topic_queue_size = LaunchConfiguration("topic_queue_size")
    sync_queue_size = LaunchConfiguration("sync_queue_size")
    use_sim_time = LaunchConfiguration("use_sim_time")

    nodes = []

    # ------------------------------------------------------------
    # IMU pipeline (keep your existing converters/fusion)
    # NOTE: use_sim_time wired through so sim vs real is consistent
    # ------------------------------------------------------------
    nodes.append(
        Node(
            package="startup",
            executable="magnetometer_converter",
            name="magnetometer_converter",
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    nodes.append(
        Node(
            package="startup",
            executable="imu_fusion",
            name="imu_fusion",
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )
    
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

    # ------------------------------------------------------------
    # RTAB-Map RGB-D ODOMETRY (Tech Stack B)
    # This node publishes:
    #   TF:  odom -> base_link
    #   Topic: /odom (nav_msgs/Odometry)
    # ------------------------------------------------------------
    nodes.append(
        Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            name="rgbd_odometry",
            namespace="rtabmap_odom",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,

                    # Inputs
                    "subscribe_rgbd": True,
                    "approx_sync": True,
                    "topic_queue_size": topic_queue_size,
                    "sync_queue_size": sync_queue_size,

                    # Frames (IMPORTANT)
                    "frame_id": "base_link",
                    "odom_frame_id": "odom",   # <-- publish odom -> base_link in the standard frame

                    # TF
                    "publish_tf": True,
                    "wait_for_transform": 0.2,

                    # QoS / misc
                    "qos": 1,
                }
            ],
            remappings=[
                # Your fused IMU output
                ("imu", "/imu/combined"),

                # RGB-D sync output (whatever is producing this)
                ("rgbd_image", "/rtab_sync/rgbd_image"),

                # Optional: force odom topic name if you want it explicitly /odom
                # ("odom", "/odom"),
            ],
        )
    )

    # ------------------------------------------------------------
    # SLAM TOOLBOX (mapping)
    # Uses:
    #   scan + odom -> builds map and publishes TF map -> odom
    # ------------------------------------------------------------
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            os.path.join(pkg, "config", "slam_toolbox_parameters.yaml"),
            {
                "use_sim_time": use_sim_time,
                "odom_frame": "odom",        # <-- must match RTAB-Map odom_frame_id
                "base_frame": "base_link",
                "map_frame": "map",
                "scan_topic": "/scan",
                "mode": "mapping",

                # Fresh map every launch
                "load_state_filename": "",
                "save_state_filename": "",
                "use_map_saver": False,

                "transform_publish_period": 0.05,
                "transform_timeout": 2.0,
                "minimum_travel_distance": 0.1,
            },
        ],
    )
    nodes.append(slam_toolbox_node)

    # Optional: reset slam_toolbox on launch (keep if you like it)
    nodes.append(
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2", "service", "call",
                        "/slam_toolbox/reset",
                        "slam_toolbox/srv/Reset",
                        "{}",
                    ],
                    output="screen",
                )
            ],
        )
    )

    # ------------------------------------------------------------
    # STATIC TFs
    # IMPORTANT CHANGES:
    #  - DO NOT publish odom -> laser (wrong)
    #  - DO publish base_link -> laser (correct rigid mount)
    # ------------------------------------------------------------

    # base_link -> base_footprint (optional; keep if Nav2 config expects base_footprint)
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_footprint_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    # base_link -> laser (EDIT these numbers to match your real mount)
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser_tf",
            # x  y   z    roll pitch yaw  parent    child
            arguments=["0.2", "0.0", "0.15", "0", "0", "0", "base_link", "laser"],
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    # base_link -> IMU frame (keep, adjust position if needed)
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_transform_publisher",
            arguments=["0", "0", "0.1", "0", "0", "0", "base_link", "imu_icm20948"],
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )
    
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='kinect_transform_publisher',
            #           x     y     z  roll  pitch            yaw  parent    child
            arguments=["0", "0", "0.1", "0", str(-math.pi/2), str(1.5*math.pi), "base_link", "kinect_rgb"],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    )

    # ------------------------------------------------------------
    # NOTE: robot_localization EKF REMOVED for Tech Stack B
    # You do NOT want EKF publishing /odom if RTAB-Map is your odom source.
    # ------------------------------------------------------------

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logging level for SLAM Toolbox",
            ),
            topic_queue_size_arg,
            sync_queue_size_arg,
            use_sim_time_arg,
            *nodes,
        ]
    )
