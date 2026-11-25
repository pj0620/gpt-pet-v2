#!/usr/bin/env python3
"""Clamp /cmd_vel_teleop to safe limits and republish it on /cmd_vel.

Used by teleop.sh. Publishes BEST_EFFORT like forward.sh (matches the controller, avoids reliable
retransmit stalls over WiFi) and sends zeros on exit so the robot never keeps a stale command.
Signals are handled here rather than by rclpy, so the stop can still be published after Ctrl-C
or SIGTERM (rclpy's own handler would invalidate the context first).
"""
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions


def clamp(value, limit):
    return max(-limit, min(limit, value))


class CmdVelLimiter(Node):
    def __init__(self, max_linear, max_angular):
        super().__init__("cmd_vel_limiter")
        self.max_linear = max_linear
        self.max_angular = max_angular
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Twist, "/cmd_vel", qos)
        self.sub = self.create_subscription(Twist, "/cmd_vel_teleop", self.on_cmd, 10)
        self.relayed = 0
        self.clamped = 0
        self.get_logger().info(
            f"relaying /cmd_vel_teleop -> /cmd_vel, clamped to {max_linear} m/s and {max_angular} rad/s")

    def on_cmd(self, msg):
        out = Twist()
        out.linear.x = clamp(msg.linear.x, self.max_linear)
        out.linear.y = clamp(msg.linear.y, self.max_linear)
        out.angular.z = clamp(msg.angular.z, self.max_angular)
        if (out.linear.x, out.linear.y, out.angular.z) != (msg.linear.x, msg.linear.y, msg.angular.z):
            self.clamped += 1
        self.relayed += 1
        self.pub.publish(out)

    def send_stop(self):
        for _ in range(5):
            self.pub.publish(Twist())
            time.sleep(0.05)


def main():
    max_linear = float(sys.argv[1]) if len(sys.argv) > 1 else 0.1
    max_angular = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5

    stop_requested = []

    def on_signal(signum, _frame):
        stop_requested.append(signum)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    node = CmdVelLimiter(max_linear, max_angular)
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.send_stop()
        node.get_logger().info(
            f"stop sent; relayed {node.relayed} commands, clamped {node.clamped}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
