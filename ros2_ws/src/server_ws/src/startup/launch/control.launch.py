from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('startup')
    urdf_path = os.path.join(pkg, 'urdf', 'gptpet.urdf')

    # allow overriding via CLI: `ros2 launch bringup.launch.py urdf_file:=...`
    urdf_launch_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=urdf_path,
        description='Full path to robot urdf file'
    )

    robot_description = Command(['xacro ', LaunchConfiguration('urdf_file')])
  
    # Paths
    # description_pkg = get_package_share_directory('mecanum_bot_description')
    # urdf_path = os.path.join(description_pkg, 'urdf', 'mecanum_bot.urdf.xacro')
    # controller_config_path = os.path.join(
    #     get_package_share_directory('mecanum_bot_bringup'),
    #     'config',
    #     'mecanum_drive_controller.yaml'
    # )

    # Load and parse the robot description from xacro
    robot_description = Command(['xacro ', urdf_path])

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description},
                        controller_config_path],
            output='screen'
        ),

        # Spawner for mecanum_drive_controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['mecanum_drive_controller'],
            output='screen'
        ),

        # Spawner for joint_state_broadcaster (needed for robot_state_publisher)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        )
    ])
