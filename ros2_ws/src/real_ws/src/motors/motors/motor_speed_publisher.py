import rclpy
from rclpy.node import Node
from gptpet_common.msg import Velocities
import serial


class MotorSpeedPublisher(Node):

    def __init__(self):
        super().__init__('motor_speed_publisher')
        # self.declare_parameter('serial_port', '/dev/ttyACM0')
        # serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        # self.serial_port = serial.Serial(serial_port, 115200)
        self.publisher_ = self.create_publisher(Velocities, 'motor_speed_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Velocities(
          velocity_left_1=1.0,
          velocity_left_2=2.0,
          velocity_right_1=3.0,
          velocity_right_2=4.0
        )
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    motor_speed_publisher = MotorSpeedPublisher()

    rclpy.spin(motor_speed_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_speed_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()