import rclpy
from rclpy.node import Node
from gptpet_common.msg import Velocities
from gptpet_common.srv import MotorControl, MotorSpeed
import serial
import time

class MotorControlService(Node):
    def __init__(self):
        super().__init__('motor_control_service')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.serial_port = serial.Serial(serial_port, 115200, timeout=1)
        self.srv = self.create_service(MotorControl, 'motor_control', self.motor_control_callback)
        self.srv = self.create_service(MotorSpeed, 'motor_speed', self.motor_speed_callback)
        self.get_logger().info('Service is ready to send mototr control commands')
        
    def float_to_byte(self, f: float) -> bytes:
      # Clamp and convert to int
      value = max(-127, min(127, int(round(f * 127 / 2))))
      
      sign_bit = 0x80 if value < 0 else 0x00  # Set the S bit (MSB)
      magnitude = abs(value) & 0x7F           # Ensure only lower 7 bits
      encoded_byte = sign_bit | magnitude     # Combine S and XXXXXXX
      
      return bytes([encoded_byte])
    
    def byte_to_float(self, b: int) -> bytes:
      val = 2.0 * float(b & 0x7F) / 127.0
      if b & 0x80:
        val = -val
      return val

    def motor_control_callback(self, request, response):
        self.get_logger().info(f'Received request: {request.motor_speeds}')
        speeds = request.motor_speeds
        serial_command = b"V"
        for vel in [speeds.velocity_left_1, speeds.velocity_left_2, speeds.velocity_right_1, speeds.velocity_right_2]:
          serial_command += self.float_to_byte(vel)
        self.serial_port.write(serial_command)
        return response
    
    def motor_speed_callback(self, request, response):
      self.get_logger().info(f'Received speed request')
      speeds = self.serial_port.readline()
      if speeds == b'':
        self.get_logger().error(f'No serial data recieved!')
        response.motor_speeds = Velocities(
          velocity_left_1=0.0,
          velocity_left_2=0.0,
          velocity_right_1=0.0,
          velocity_right_2=0.0
        )
      else:
        response.motor_speeds = Velocities(
          velocity_left_1=self.byte_to_float(speeds[1]),
          velocity_left_2=self.byte_to_float(speeds[2]),
          velocity_right_1=self.byte_to_float(speeds[3]),
          velocity_right_2=self.byte_to_float(speeds[4])
        )
      return response

def main(args=None):
    rclpy.init(args=args)
    service = MotorControlService()
    rclpy.spin(service)
    service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
