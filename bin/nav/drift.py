#!/usr/bin/env python3
"""Sample the EKF yaw (odom->base_link) and the raw gyro z for N seconds while the robot is still.
Usage: drift.py SECONDS"""
import math, sys, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

secs = float(sys.argv[1])
rclpy.init()
node = rclpy.create_node("drift_check")
yaws, gz, gz_raw = [], [], []
def yaw_of(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
node.create_subscription(Odometry, "/odometry/filtered", lambda m: yaws.append((time.time(), yaw_of(m.pose.pose.orientation))), 10)
node.create_subscription(Imu, "/imu/combined", lambda m: gz.append(m.angular_velocity.z), 10)
node.create_subscription(Imu, "/imu/data_raw", lambda m: gz_raw.append(m.angular_velocity.z), qos_profile_sensor_data)
end = time.monotonic() + secs
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
if len(yaws) < 2:
    print("no EKF output"); sys.exit(1)
d = math.degrees(math.atan2(math.sin(yaws[-1][1] - yaws[0][1]), math.cos(yaws[-1][1] - yaws[0][1])))
dt = yaws[-1][0] - yaws[0][0]
mean = lambda v: sum(v) / len(v) if v else float("nan")
print(f"EKF yaw drift {d:+.2f} deg over {dt:.1f} s = {d / dt * 60:+.2f} deg/min; "
      f"gyro z mean: combined {math.degrees(mean(gz)):+.3f} deg/s ({len(gz)} msgs), raw {math.degrees(mean(gz_raw)):+.3f} deg/s ({len(gz_raw)} msgs)")
