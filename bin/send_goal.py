#!/usr/bin/env python3
"""
Send a Nav2 goal relative to the robot's current pose in the map frame.

Usage:
  send_goal.py                  # 0.5 m straight ahead (default)
  send_goal.py 1.0              # 1.0 m straight ahead
  send_goal.py 1.0 0.5          # 1.0 m ahead, 0.5 m to the left
  send_goal.py 1.0 0.0 1.57     # 1.0 m ahead, rotate 90° at goal
"""
import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


def quaternion_from_yaw(yaw: float) -> tuple:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def main():
    # Parse args: forward_m [left_m [goal_yaw_offset_rad]]
    forward_m   = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
    left_m      = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    yaw_offset  = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

    rclpy.init()
    node = Node("send_goal_node")

    tf_buf = Buffer()
    TransformListener(tf_buf, node)

    pub = node.create_publisher(PoseStamped, "/goal_pose", 10)

    # Wait for map→base_link to exist (up to 10 s)
    deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=10)
    transform = None
    while rclpy.ok() and node.get_clock().now() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            transform = tf_buf.lookup_transform(
                "map", "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            break
        except Exception:
            continue

    if transform is None:
        node.get_logger().error(
            "Could not get map→base_link transform. "
            "Is slam_toolbox running and the robot localised?"
        )
        rclpy.shutdown()
        sys.exit(1)

    # Robot pose in map frame
    tx = transform.transform.translation.x
    ty = transform.transform.translation.y
    qz = transform.transform.rotation.z
    qw = transform.transform.rotation.w
    robot_yaw = 2.0 * math.atan2(qz, qw)

    # Goal position: forward_m ahead, left_m to the left in robot's frame
    goal_x = tx + forward_m * math.cos(robot_yaw) - left_m * math.sin(robot_yaw)
    goal_y = ty + forward_m * math.sin(robot_yaw) + left_m * math.cos(robot_yaw)
    goal_yaw = robot_yaw + yaw_offset

    qx, qy, qz_g, qw_g = quaternion_from_yaw(goal_yaw)

    msg = PoseStamped()
    msg.header.stamp = rclpy.time.Time().to_msg()  # time=0 → use latest TF
    msg.header.frame_id = "map"
    msg.pose.position.x = goal_x
    msg.pose.position.y = goal_y
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz_g
    msg.pose.orientation.w = qw_g

    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info(
        f"Goal sent: ({goal_x:.2f}, {goal_y:.2f}) yaw={math.degrees(goal_yaw):.1f}°  "
        f"[{forward_m}m ahead, {left_m}m left of robot]"
    )
    rclpy.shutdown()


if __name__ == "__main__":
    main()
