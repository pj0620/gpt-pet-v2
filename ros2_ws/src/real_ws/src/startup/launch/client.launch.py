import launch_ros
import launch
import launch_ros.actions
import subprocess


def is_node_running(node_name):
    """Check if a ROS 2 node with the given name is already running."""
    try:
        output = subprocess.check_output(["ros2", "node", "list"]).decode("utf-8")
        return node_name in output
    except Exception as e:
        print(f"Error checking node status: {e}")
        return False


def generate_launch_description():
    actions = []

    # Only launch kinect_ros2_node if it's not already running
    if not is_node_running("/kinect/kinect_ros2"):
      actions.append(
          launch_ros.actions.Node(
              package="kinect_ros2",
              executable="kinect_ros2_node",
              name="kinect_ros2",
              namespace="kinect",
          )
      )

    actions.append(
        launch_ros.actions.Node(
            package="motors",
            executable="service",
            name="motor_control_service",
            parameters=[{"serial_port": "/dev/ttyUSB0"}],
        )
    )

    return launch.LaunchDescription(actions)
