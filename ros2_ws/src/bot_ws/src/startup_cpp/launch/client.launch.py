from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  nodes = []

  # Get URDF via xacro
  pkg_path = get_package_share_directory('startup_cpp')
  xacro_file = os.path.join(pkg_path, 'urdf', 'gptpet.xacro')
  robot_description_content = Command(['xacro ', xacro_file])
  
  ## ROS CONTROL ##
  robot_description = {'robot_description': robot_description_content}
  robot_controllers = os.path.join(pkg_path, 'config', 'controllers.yaml')
  control_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[robot_description, robot_controllers],
    output='both',
  )
  robot_state_pub_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[robot_description]
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
      namespace="kinect"
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
      ],
    )
  )

  return LaunchDescription(nodes)
