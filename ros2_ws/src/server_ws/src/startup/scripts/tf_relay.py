#!/usr/bin/env python3
"""Relay TF messages from one topic to /tf.

Usage (standalone):
    ros2 run startup tf_relay --ros-args -p source_topic:=/mecanum_drive_controller/tf_odometry

Or launched via launch file (see v4.launch.py).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfRelay(Node):
    def __init__(self):
        super().__init__("tf_relay")
        self.declare_parameter("source_topic", "/mecanum_drive_controller/tf_odometry")
        src = self.get_parameter("source_topic").get_parameter_value().string_value

        qos = QoSProfile(depth=100)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE

        self.pub = self.create_publisher(TFMessage, "/tf", qos)
        self.sub = self.create_subscription(TFMessage, src, self._cb, qos)
        self.get_logger().info(f"Relaying {src} -> /tf")

    def _cb(self, msg: TFMessage):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
