import launch_ros
import launch
import launch_ros.actions

def generate_launch_description():
  actions = []

  # Launch the kinect node in its own process with a unique node name.
  actions.append(
    launch_ros.actions.Node(
      package="kinect_ros2",
      executable="kinect_ros2_node",
      name="kinect_ros2",
      namespace="kinect"
    )
  )

  # Launch motor control service node.
  actions.append(
    launch_ros.actions.Node(
      package="motors",
      executable="service",
      name="motor_control_service",
      parameters=[{"serial_port": "/dev/ttyACM0"}]
    )
  )
  
  actions.append(
     launch_ros.actions.Node(
        package='motors',
        executable='motor_speed_publisher',
        name='motor_speed_publisher'
      )
  )

  return launch.LaunchDescription(actions)
