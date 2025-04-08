import rclpy
from rclpy.node import Node
from gptpet_common.msg import Velocities
from gptpet_common.srv import MotorControl
import serial

class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')

        # Get serial port parameter
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value

        # Setup serial
        self.serial_port = serial.Serial(serial_port, 115200, timeout=0.1)

        # Velocities publisher
        self.vel_pub = self.create_publisher(Velocities, 'motor_speeds', 10)
        self.timer = self.create_timer(0.01, self.read_serial_data)  # 10ms

        # Motor control service
        self.create_service(MotorControl, 'motor_control', self.motor_control_callback)

        self.get_logger().info('Arduino motor interface running.')

    def float_to_byte(self, f: float) -> bytes:
        value = max(-127, min(127, int(round(f * 127 / 2))))
        sign_bit = 0x80 if value < 0 else 0x00
        magnitude = abs(value) & 0x7F
        return bytes([sign_bit | magnitude])

    def byte_to_float(self, b: int) -> float:
        val = 2.0 * float(b & 0x7F) / 127.0
        if b & 0x80:
            val = -val
        return val

    def read_serial_data(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline()
            if line.startswith(b'M') and len(line) >= 5:
                try:
                    vel_msg = Velocities(
                        velocity_left_1=self.byte_to_float(line[1]),
                        velocity_left_2=self.byte_to_float(line[2]),
                        velocity_right_1=self.byte_to_float(line[3]),
                        velocity_right_2=self.byte_to_float(line[4])
                    )
                    self.vel_pub.publish(vel_msg)
                except Exception as e:
                    self.get_logger().error(f'Failed to parse serial data: {e}')

    def motor_control_callback(self, request, response):
        self.get_logger().info(f'Received motor control command: {request.motor_speeds}')
        speeds = request.motor_speeds
        command = b'V'
        for vel in [
            speeds.velocity_left_1,
            speeds.velocity_left_2,
            speeds.velocity_right_1,
            speeds.velocity_right_2
        ]:
            command += self.float_to_byte(vel)

        self.serial_port.write(command)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MotorControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()