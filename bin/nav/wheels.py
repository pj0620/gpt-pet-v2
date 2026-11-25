#!/usr/bin/env python3
"""Drive a timed /cmd_vel command while recording /joint_states; print each wheel's mean measured
velocity during the steady part of the move against the controller's kinematic expectation.
Usage: wheels.py VX VY WZ SECONDS"""
import math, sys, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

vx, vy, wz, secs = (float(a) for a in sys.argv[1:5])
vx, vy = (max(-0.15, min(0.15, v)) for v in (vx, vy)); wz = max(-0.6, min(0.6, wz)); secs = min(secs, 6.0)
R, L = 0.0485, 0.185   # controllers.yaml: wheels_radius, sum_of_robot_center_projection_on_X_Y_axis
expect = {"velocity_left_1_joint": (vx - vy - L * wz) / R,   # front left
          "velocity_right_1_joint": (vx + vy + L * wz) / R,  # front right
          "velocity_left_2_joint": (vx + vy - L * wz) / R,   # rear left
          "velocity_right_2_joint": (vx - vy + L * wz) / R}  # rear right
rclpy.init()
node = rclpy.create_node("wheel_check")
samples = []
node.create_subscription(JointState, "/joint_states", lambda m: samples.append((time.monotonic(), dict(zip(m.name, m.velocity)))), 10)
pub = node.create_publisher(Twist, "/cmd_vel", QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))
def spin(s):
    end = time.monotonic() + s
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
deadline = time.monotonic() + 20
while (pub.get_subscription_count() < 1 or not samples) and time.monotonic() < deadline:
    spin(0.1)
if pub.get_subscription_count() < 1 or not samples:
    print(f"not ready: cmd_vel subs={pub.get_subscription_count()} joint_states={len(samples)}; robot NOT moved"); sys.exit(1)
t = Twist(); t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
t0 = time.monotonic()
try:
    while time.monotonic() < t0 + secs:
        pub.publish(t); spin(0.05)
finally:
    t1 = time.monotonic(); z = Twist()
    while time.monotonic() < t1 + 1.5:
        pub.publish(z); spin(0.05)
steady = [s for s in samples if t0 + 0.7 < s[0] < t1 - 0.1]
print(f"cmd vx={vx} vy={vy} wz={wz} for {secs}s; {len(steady)} joint_state samples in the steady window")
for j, e in expect.items():
    v = [s[1][j] for s in steady if j in s[1]]
    m = sum(v) / len(v) if v else float("nan")
    print(f"  {j:24s} expected {e:+6.2f} rad/s  measured {m:+6.2f} rad/s  ({m / e * 100 if e else float('nan'):5.0f}%)")
