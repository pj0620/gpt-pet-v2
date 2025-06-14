import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('startup')

    # allow overriding via CLI: `ros2 launch bringup.launch.py urdf_file:=...`
    urdf_launch_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(pkg, 'urdf', 'gptpet.urdf'),
        description='Full path to robot urdf file'
    )

    robot_description = Command(['xacro ', LaunchConfiguration('urdf_file')])

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

        # 2) ros2_control_node: loads hardware + controllers.yaml
        # Node(
        #     package='controller_manager',
        #     executable='ros2_control_node',
        #     name='controller_manager',
        #     output='screen',
        #     parameters=[
        #         {'robot_description': robot_description},
        #         os.path.join(pkg, 'config', 'controllers.yaml')
        #     ]
        # ),

        # 3) spawn joint_state_broadcaster
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        #     output='screen'
        # ),

        # # 4) spawn your mecanum drive controller
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['mecanum_drive_controller', '--controller-manager', '/controller_manager'],
        #     output='screen'
        # ),
    ])
