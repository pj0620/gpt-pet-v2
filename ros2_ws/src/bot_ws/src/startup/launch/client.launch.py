from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory

def generate_launch_description():
  nodes = []

  pkg_path = get_package_share_directory('startup')

  # Increase CycloneDDS MaxBlockedTime so service responses from the
  # controller_manager aren't dropped when the 50 Hz control loop is busy.
  cyclone_cfg = os.path.join(pkg_path, 'config', 'cyclonedds.xml')
  nodes.append(SetEnvironmentVariable('CYCLONEDDS_URI', cyclone_cfg))
  # qos_override_file = os.path.join(pkg_path, 'config', 'qos_overrides.yaml')
  # qos_override = SetEnvironmentVariable(
  #   'RMW_QOS_OVERRIDES_FILE',
  #   qos_override_file
  # )

  # Get URDF via xacro
  xacro_file = os.path.join(pkg_path, 'urdf', 'gptpet.xacro')
  robot_description_content = Command(['xacro ', xacro_file])
  
  ## ROS CONTROL ##
  robot_description = {'robot_description': robot_description_content}
  robot_controllers = os.path.join(pkg_path, 'config', 'controllers.yaml')
  control_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[robot_description, robot_controllers, {
      'use_sim_time': False,
    }],
    output='both',
    remappings=[
      ('/mecanum_drive_controller/reference_unstamped', '/cmd_vel'),
    ],
  )
  spawner_script = os.path.join(
    get_package_prefix('startup'), 'lib', 'startup', 'spawn_controllers.py',
  )

  # Custom spawner with 120s per-call timeout — the standard ros2_control
  # spawner hardcodes 10s which is too short for DDS on the non-RT Jetson Nano.
  # joint_state_broadcaster must be active before mecanum_drive_controller,
  # so spawn them sequentially: first at 15s, second at 90s after launch.
  spawn_joint_state = TimerAction(
    period=15.0,
    actions=[ExecuteProcess(
      cmd=['python3', spawner_script, 'joint_state_broadcaster'],
      output='screen',
    )],
  )
  spawn_mecanum = TimerAction(
    period=90.0,
    actions=[ExecuteProcess(
      cmd=['python3', spawner_script, 'mecanum_drive_controller'],
      output='screen',
    )],
  )
  nodes.extend([
    control_node,
    spawn_joint_state,
    spawn_mecanum,
  ])
  
  ## KINECT ##
  nodes.append(
    Node(
      package="kinect_ros2",
      executable="kinect_ros2_node",
      name="kinect_ros2",
      namespace="kinect",
      parameters=[
        {"use_sim_time": False}
      ]
    )
  )
  
  # IMU ##
  nodes.append(
    Node(
      package="ros2_icm20948",
      executable="icm20948_node",
      name="icm20948_node",
      parameters=[
        {"i2c_address": 0x69},
        {"frame_id": "imu_icm20948"},
        {"pub_rate": 50},
        {"use_sim_time": False},
      ],
    )
  )
  
  # LIDAR (RPLIDAR A1) ##
  nodes.append(
    Node(
      package="rplidar_ros",
      executable="rplidar_node",
      name="rplidar_node",
      parameters=[
        {"channel_type": "serial"},
        {"serial_port": "/dev/ttyUSB0"},
        {"serial_baudrate": 115200},
        {"frame_id": "laser"},
        {"inverted": False},
        {"angle_compensate": True},
        {"scan_mode": "Sensitivity"},
      ],
      output="screen",
    )
  )
  
  # NOTE: mecanum_drive_controller publishes wheel odometry on
  # /mecanum_drive_controller/odometry only; odom TF is provided by EKF on server.
  
  # ## TOPIC REMAPPING ##
  # # Remap /cmd_vel to /mecanum_drive_controller/reference
  # nodes.append(
  #   Node(
  #     package="topic_tools",
  #     executable="relay",
  #     name="cmd_vel_relay",
  #     arguments=["/cmd_vel", "/mecanum_drive_controller/reference"],
  #     parameters=[{"lazy": False}]
  #   )
  # )
  
  # # Remap /mecanum_drive_controller/odometry to /odom  
  # nodes.append(
  #   Node(
  #     package="topic_tools",
  #     executable="relay",
  #     name="odom_relay", 
  #     arguments=["/mecanum_drive_controller/odometry", "/odom"],
  #     parameters=[{"lazy": False}]
  #   )
  # )

  return LaunchDescription(nodes)
