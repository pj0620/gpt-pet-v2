from launch import LaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  nodes = []
  
  # Set QoS override file for sensor data compatibility
  pkg_path = get_package_share_directory('startup_cpp')
  qos_override_file = os.path.join(pkg_path, 'config', 'qos_overrides.yaml')
  qos_override = SetEnvironmentVariable(
    'RMW_QOS_OVERRIDES_FILE',
    qos_override_file
  )

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
      ('/mecanum_drive_controller/reference', '/cmd_vel'),
      ('/mecanum_drive_controller/odometry', '/odom'),
    ],
  )
  robot_state_pub_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[robot_description, {
      'use_sim_time': False,
    }]
  )
  joint_state_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
  )
  robot_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['mecanum_drive_controller', '--controller-manager', '/controller_manager'],
  )
  # Delay start of robot_controller after joint_state_broadcaster
  delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    event_handler=OnProcessExit(
      target_action=joint_state_broadcaster_spawner,
      on_exit=[robot_controller_spawner],
    )
  )
  nodes.extend([
    control_node,
    robot_state_pub_node,
    joint_state_broadcaster_spawner,
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner
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
  
  ## IMU ##
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
  
  ## STATIC TRANSFORMS ##
  # Add static transform from base_link to base_footprint if needed
  nodes.append(
    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='base_footprint_publisher',
      arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
      parameters=[{'use_sim_time': False}]
    )
  )
  
  # Temporary: Add odom to base_link transform as fallback
  # This should be removed once the mecanum controller is working properly
  nodes.append(
    Node(
      package='tf2_ros',
      executable='static_transform_publisher', 
      name='odom_base_link_publisher',
      arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
      parameters=[{'use_sim_time': False}]
    )
  )
  
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

  return LaunchDescription([qos_override] + nodes)
