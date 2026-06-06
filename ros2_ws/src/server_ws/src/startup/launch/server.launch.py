"""
server launch — EKF + SLAM + Nav2
===================================
Odometry:  mecanum_drive_controller wheel encoders + IMU fused by robot_localization EKF.
SLAM:      slam_toolbox (async mapping) provides /map and map→odom TF.
Nav2:      MPPI controller (holonomic/Omni), NavFn planner, lifecycle-managed.
           Send goals via RViz "Nav2 Goal" button or NavigateToPose action.
Starts a fresh map on every launch — no state is loaded or saved.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
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
    # Clear stale slam_toolbox map state and reset controller odometry
    # so odom frame always starts at (0,0) on each server launch.
    # The mecanum_drive_controller accumulates odometry across sessions;
    # without this reset the EKF and controller TF sources conflict.
    # ------------------------------------------------------------
    nodes.append(
        ExecuteProcess(
            cmd=["bash", "-c",
                 "rm -f ~/.ros/*.posegraph ~/.ros/*.data /tmp/*.posegraph /tmp/*.data && "
                 "sleep 3 && "
                 "ros2 service call /mecanum_drive_controller/reset_odometry "
                 "std_srvs/srv/Empty '{}' 2>/dev/null || true"],
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
                "minimum_travel_distance": 0.02,
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

    # ------------------------------------------------------------
    # Nav2 — autonomous point-to-point navigation
    # Goals can be sent via RViz "Nav2 Goal" or NavigateToPose action.
    # SLAM toolbox provides /map and map→odom TF; EKF provides odom→base_link.
    # ------------------------------------------------------------
    use_nav2_arg = DeclareLaunchArgument(
        "use_nav2",
        default_value="True",
        description="Launch Nav2 navigation stack.",
    )
    use_nav2 = LaunchConfiguration("use_nav2")

    nav2_params = os.path.join(pkg, "config", "nav2_params.yaml")

    nav2_nodes = []
    for name, package, executable in [
        ("controller_server", "nav2_controller", "controller_server"),
        ("planner_server",    "nav2_planner",    "planner_server"),
        ("behavior_server",   "nav2_behaviors",  "behavior_server"),
        ("bt_navigator",      "nav2_bt_navigator", "bt_navigator"),
    ]:
        nav2_nodes.append(
            Node(
                package=package,
                executable=executable,
                name=name,
                output="screen",
                parameters=[nav2_params, {"use_sim_time": use_sim_time}],
                condition=IfCondition(use_nav2),
            )
        )

    nav2_nodes.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": True},
                {"bond_timeout": 4.0},
                {"node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                ]},
            ],
            condition=IfCondition(use_nav2),
        )
    )

    # Delay Nav2 startup so the EKF fills the TF buffer with fresh odom→base_link
    # transforms before Nav2 starts looking up sensor origins. Without this delay,
    # stale TF entries from the previous server session cause the costmap to report
    # the sensor (LIDAR) as being dozens of meters outside costmap bounds.
    nodes.append(TimerAction(period=5.0, actions=nav2_nodes))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logging level for SLAM Toolbox",
            ),
            use_sim_time_arg,
            use_nav2_arg,
            *nodes,
        ]
    )
