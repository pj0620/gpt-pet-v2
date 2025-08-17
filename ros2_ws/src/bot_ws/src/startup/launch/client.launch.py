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
        'frequency': 30.0,
        'sensor_timeout': 0.2,  # Increased timeout to be more tolerant
        'two_d_mode': True,
        'transform_time_offset': 0.0,
        'transform_timeout': 0.0,
        'print_diagnostics': True,
        'debug': False,
        
        # Additional stability parameters
        'smooth_lagged_data': True,
        'history_length': 0.5,
        
        # Output frame configuration
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_link_frame': 'base_link',
        'world_frame': 'odom',
        
        # Odometry source configuration (wheel odometry) - more conservative
        'odom0': '/mecanum_drive_controller/odometry',
        'odom0_config': [
          True,  # x position
          True,  # y position  
          False, # z position (2D mode)
          False, # roll
          False, # pitch
          False, # yaw (will use IMU for this)
          False, # x velocity (disable to reduce drift when stationary)
          False, # y velocity (disable to reduce drift when stationary)
          False, # z velocity
          False, # roll velocity
          False, # pitch velocity
          False, # yaw velocity (will use IMU for this)
          False, # x acceleration
          False, # y acceleration
          False  # z acceleration
        ],
        'odom0_differential': True,  # Use differential mode for better stability
        'odom0_relative': False,
        'odom0_queue_size': 10,
        'odom0_nodelay': False,
        
        # Magnetometer source configuration for absolute heading
        'imu0': '/imu/mag_raw',
        'imu0_config': [
          False, # x position
          False, # y position
          False, # z position
          False, # roll
          False, # pitch
          True,  # yaw (absolute orientation from magnetometer)
          False, # x velocity
          False, # y velocity
          False, # z velocity
          False, # roll velocity
          False, # pitch velocity
          False, # yaw velocity
          False, # x acceleration
          False, # y acceleration
          False  # z acceleration
        ],
        'imu0_differential': False,
        'imu0_relative': False,  # Absolute measurements from magnetometer
        'imu0_queue_size': 10,
        'imu0_nodelay': False,
        
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
      arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
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
      arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_icm20948'],
      parameters=[{'use_sim_time': False}]
    )
  )
  
  # NOTE: odom->base_link transform is now published by mecanum_drive_controller
  # via the /mecanum_drive_controller/tf_odometry -> /tf remapping
  
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
