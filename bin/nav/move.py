#!/usr/bin/env python3
"""Drive a timed /cmd_vel command and report how far each estimate says the robot moved.
Usage: move.py VX VY WZ SECONDS     (m/s, m/s, rad/s; caps 0.15 m/s, 0.6 rad/s, 10 s)
Prints wall-clock start/stop and the pose change from wheel odometry (controller), EKF (odom->base_link)
and SLAM (map->base_link), all expressed in the robot's starting frame."""
import json, math, sys, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf2_ros

vx, vy, wz, secs = (float(a) for a in sys.argv[1:5])
vx, vy = (max(-0.15, min(0.15, v)) for v in (vx, vy))
wz, secs = max(-0.6, min(0.6, wz)), min(secs, 10.0)

def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

rclpy.init()
node = rclpy.create_node("move_test")
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, node)
wheel = []
node.create_subscription(Odometry, "/mecanum_drive_controller/odometry", lambda m: wheel.append(m), 10)
pub = node.create_publisher(Twist, "/cmd_vel", QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                                           history=HistoryPolicy.KEEP_LAST))

def spin(secs):
    end = time.monotonic() + secs
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)

def poses():
    out = {}
    for name, parent in (("ekf", "odom"), ("slam", "map")):
        try:
            t = buf.lookup_transform(parent, "base_link", rclpy.time.Time())
            out[name] = (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))
        except Exception as e:
            out[name] = None
    if wheel:
        p = wheel[-1].pose.pose
        out["wheel"] = (p.position.x, p.position.y, yaw_of(p.orientation))
    return out

def rel(a, b):  # b expressed in a's frame
    if a is None or b is None:
        return None
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(-a[2]), math.sin(-a[2])
    dyaw = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
    return {"fwd": round(c * dx - s * dy, 3), "left": round(s * dx + c * dy, 3), "yaw_deg": round(math.degrees(dyaw), 1)}

spin(2.0)
deadline = time.monotonic() + 20
while (pub.get_subscription_count() < 1 or not wheel or "slam" not in poses() or poses()["slam"] is None) \
        and time.monotonic() < deadline:
    spin(0.1)
if pub.get_subscription_count() < 1:
    print("no /cmd_vel subscriber, robot NOT moved"); sys.exit(1)
if not wheel:
    print("no wheel odometry yet (continuing)")
before = poses()
t = Twist(); t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
t0 = time.time()
try:
    end = time.monotonic() + secs
    while time.monotonic() < end:
        pub.publish(t); spin(0.05)
finally:
    t1 = time.time()
    z = Twist()
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        pub.publish(z); spin(0.05)
spin(1.5)
after = poses()
res = {"cmd": [vx, vy, wz, secs], "start": t0, "stop": t1,
       "expected": {"fwd": round(vx * secs, 3), "left": round(vy * secs, 3), "yaw_deg": round(math.degrees(wz * secs), 1)}}
for k in ("wheel", "ekf", "slam"):
    res[k] = rel(before.get(k), after.get(k))
res["slam_abs"] = after.get("slam")
print(json.dumps(res))
