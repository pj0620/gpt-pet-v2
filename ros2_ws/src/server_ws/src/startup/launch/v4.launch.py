"""
v4 launch – wheel-odometry + EKF variant
=========================================
Odometry: mecanum_drive_controller wheel encoders + IMU fused by robot_localization EKF.
SLAM:     slam_toolbox (async mapping) using fused odometry.
Starts a fresh map on every launch — no state is loaded or saved.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("startup")

    # Launch args
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation time.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    nodes = []

    # ------------------------------------------------------------
    # Clear any stale slam_toolbox map state from previous runs
    # ------------------------------------------------------------
    nodes.append(
        ExecuteProcess(
            cmd=["bash", "-c", "rm -f ~/.ros/*.posegraph ~/.ros/*.data /tmp/*.posegraph /tmp/*.data"],
            output="screen",
        )
    )

    # ------------------------------------------------------------
    # IMU pipeline
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

    # ------------------------------------------------------------
    # EKF: fuse wheel odometry (velocities) + IMU (yaw + accel)
    # Publishes fused /odometry/filtered and odom→base_link TF.
    # Replaces the raw tf_relay — EKF gives SLAM much better odometry.
    # ------------------------------------------------------------
    nodes.append(
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[
                os.path.join(pkg, "config", "ekf.yaml"),
                {"use_sim_time": use_sim_time},
            ],
        )
    )

    # ------------------------------------------------------------
    # SLAM TOOLBOX (mapping)
    # odom→base_link TF now comes from mecanum_drive_controller
    # on the client via ros2_control.
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
                "odom_frame": "odom",
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
                "minimum_travel_distance": 0.05,
            },
        ],
    )
    nodes.append(slam_toolbox_node)

    # ------------------------------------------------------------
    # STATIC TFs
    # ------------------------------------------------------------

    # base_link -> base_footprint
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_footprint_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    # base_link -> laser
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser_tf",
            arguments=["0.2", "0.0", "0.15", "0", "0", "0", "base_link", "laser"],
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    # base_link -> IMU frame
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_transform_publisher",
            arguments=["0", "0", "0.1", "0", "0", "0", "base_link", "imu_icm20948"],
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    # base_link -> kinect_rgb
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="kinect_transform_publisher",
            arguments=["0", "0", "0.1", "0", str(-math.pi / 2), str(1.5 * math.pi), "base_link", "kinect_rgb"],
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logging level for SLAM Toolbox",
            ),
            use_sim_time_arg,
            *nodes,
        ]
    )
