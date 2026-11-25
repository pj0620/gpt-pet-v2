#!/usr/bin/env python3
"""Cancel all Nav2 goals and zero cmd_vel to stop the robot immediately."""
import rclpy
from rclpy.node import Node
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
import time


def main():
    rclpy.init()
    node = Node("stop_robot_node")

    # Zero cmd_vel directly — stops motors regardless of Nav2 state
    cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)
    stop = Twist()
    for _ in range(10):
        cmd_pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.02)

    # Call the navigate_to_pose action cancel service to cancel ALL active goals.
    # Sending CancelGoal with a zeroed goal_id cancels every goal.
    cancel_cli = node.create_client(
        CancelGoal, "/navigate_to_pose/_action/cancel_goal"
    )
    if cancel_cli.wait_for_service(timeout_sec=3.0):
        req = CancelGoal.Request()
        # goal_info left default (zeroed goal_id = cancel all)
        future = cancel_cli.call_async(req)
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info("Nav2 goals cancelled and cmd_vel zeroed.")
    else:
        node.get_logger().warn(
            "navigate_to_pose action not reachable — only cmd_vel zeroed."
        )

    # A few more zeros so the controller doesn't resume before Nav2 notices
    for _ in range(10):
        cmd_pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.02)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
