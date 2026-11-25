#!/usr/bin/env python3
"""Relay /cmd_vel_nav (Nav2) to /cmd_vel, rewriting commands the robot cannot execute.

The front-right motor (R1) has no reverse drive (2026-11-15: its reverse PWM path, ESP32 GPIO 5, is
dead; the motor runs fine forward). Any command whose R1 wheel speed would be negative therefore
stalls the robot: strafes yaw it, clockwise spins barely turn, backing up does nothing. With
r1_no_reverse true (default) a command is moved to the nearest one with R1 >= 0:
  - clockwise rotation gains forward speed until R1 is exactly zero (a forward arc pivoting about
    the right side); if that needs more than max_vx, the turn rate is reduced instead,
  - pure reverse becomes a stop.
Nav2 closes the loop on odometry, so it copes with the altered motion. Set r1_no_reverse false
(launch arg of server.launch.py) once the motor is repaired; the node is then a plain relay.
Publishes BEST_EFFORT like the teleop limiter, and zeros on exit.
"""
import signal
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions


class CmdVelShaper(Node):
    def __init__(self):
        super().__init__("cmd_vel_shaper")
        self.declare_parameter("r1_no_reverse", True)
        # Same geometry as mecanum_drive_controller in bot_ws controllers.yaml
        self.declare_parameter("wheel_lever", 0.185)  # sum_of_robot_center_projection_on_X_Y_axis
        self.declare_parameter("max_vx", 0.15)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Twist, "/cmd_vel", qos)
        self.sub = self.create_subscription(Twist, "/cmd_vel_nav", self.on_cmd, 10)
        self.shaped = 0
        self.get_logger().info(
            "relaying /cmd_vel_nav -> /cmd_vel, r1_no_reverse=%s"
            % self.get_parameter("r1_no_reverse").value)

    def on_cmd(self, msg):
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        if self.get_parameter("r1_no_reverse").value:
            lever = self.get_parameter("wheel_lever").value
            max_vx = self.get_parameter("max_vx").value
            r1 = vx + vy + lever * wz  # front-right rim speed, m/s (kinematics of the controller)
            if r1 < 0.0:
                new_vx = -vy - lever * wz  # the forward speed that brings R1 to a standstill
                if new_vx > max_vx:
                    new_vx = max_vx
                    wz = -(new_vx + vy) / lever  # keep the turn direction, shrink the rate
                if vx < 0.0 and abs(wz) < 1e-3 and abs(vy) < 1e-3:
                    new_vx = 0.0  # pure reverse: nothing sensible to drive, stop
                self.shaped += 1
                if self.shaped % 20 == 1:
                    self.get_logger().info(
                        f"shaped ({msg.linear.x:+.2f}, {vy:+.2f}, {msg.angular.z:+.2f}) -> "
                        f"({new_vx:+.2f}, {vy:+.2f}, {wz:+.2f}), {self.shaped} so far")
                vx = new_vx
        out = Twist()
        out.linear.x, out.linear.y, out.angular.z = float(vx), float(vy), float(wz)
        self.pub.publish(out)

    def stop(self):
        for _ in range(5):
            self.pub.publish(Twist())


def main():
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    node = CmdVelShaper()
    stopping = []
    signal.signal(signal.SIGINT, lambda *_: stopping.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stopping.append(1))
    while rclpy.ok() and not stopping:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
