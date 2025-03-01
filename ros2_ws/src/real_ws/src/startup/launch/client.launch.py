import launch_ros
import launch
import launch_ros.actions

def generate_launch_description():
    actions = []

    # Launch kinect_ros2_node with a unique internal node name override.
    actions.append(
        launch_ros.actions.Node(
            package="kinect_ros2",
            executable="kinect_ros2_node",
            name="kinect_ros2",
            namespace="kinect",
            arguments=["__node:=unique_kinect_node"]
        )
    )

    # Launch motor control service node.
    actions.append(
        launch_ros.actions.Node(
            package="motors",
            executable="service",
            name="motor_control_service",
            parameters=[{"serial_port": "/dev/ttyUSB0"}],
        )
    )

    return launch.LaunchDescription(actions)
