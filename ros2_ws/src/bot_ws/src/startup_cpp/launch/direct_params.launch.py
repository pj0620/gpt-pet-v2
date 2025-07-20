import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('startup_cpp')
    xacro_path = os.path.join(pkg, 'urdf', 'gptpet.xacro')

    # allow overriding via CLI: `ros2 launch bringup.launch.py urdf_file:=...`
    urdf_launch_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=xacro_path,
        description='Full path to robot xacro file'
    )

    robot_description = Command(['xacro ', LaunchConfiguration('urdf_file')])
    
    # Controller parameters - directly in the launch file
    controller_params = {
        'controller_manager': {
            'ros__parameters': {
                'update_rate': 100,
                'joint_state_broadcaster': {
                    'type': 'joint_state_broadcaster/JointStateBroadcaster'
                },
                'mecanum_drive_controller': {
                    'type': 'mecanum_drive_controller/MecanumDriveController'
                }
            }
        },
        'mecanum_drive_controller': {
            'ros__parameters': {
                'front_left_wheel_command_joint_name': 'velocity_left_1_joint',
                'front_right_wheel_command_joint_name': 'velocity_right_1_joint',
                'rear_left_wheel_command_joint_name': 'velocity_left_2_joint',
                'rear_right_wheel_command_joint_name': 'velocity_right_2_joint',
                'wheel_radius': 0.0485,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'enable_odom_tf': True
            }
        }
    }

    return LaunchDescription([
        urdf_launch_arg,

        # 1) robot_state_publisher: publishes base_link→base_laser + other static TFs
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # 2) ros2_control_node: loads hardware + controllers
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                controller_params
            ]
        ),
        
        # 3) spawn joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # 4) spawn your mecanum drive controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['mecanum_drive_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
    ])
