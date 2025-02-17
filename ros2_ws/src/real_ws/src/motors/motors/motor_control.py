import rclpy
from rclpy.node import Node
from common_interfaces.srv import MotorControl  # Importing a service
import serial

class MotorControlService(Node):
    def __init__(self):
        super().__init__('motor_control_service')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.serial_port = serial.Serial(serial_port, 115200)
        self.srv = self.create_service(MotorControl, 'motor_control', self.motor_control_callback)
        self.get_logger().info('Service is ready to send mototr control commands')

    def motor_control_callback(self, request, response):
        self.get_logger().info(f'Received request: {request.motor_speeds}')
        speeds = request.motor_speeds
        serial_command = f"SET_VS {speeds.velocity_left_1},{speeds.velocity_left_2},{speeds.velocity_right_1},{speeds.velocity_right_2}\n"
        self.serial_port.write(serial_command.encode('utf-8'))
        return response

def main(args=None):
    rclpy.init(args=args)
    service = MotorControlService()
    rclpy.spin(service)
    service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
