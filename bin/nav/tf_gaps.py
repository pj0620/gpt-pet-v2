#!/usr/bin/env python3
"""Watch map->odom and /scan for SECONDS; report stamp freshness dips and arrival gaps."""
import sys, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
rclpy.init(); node = rclpy.create_node("tf_gaps")
tf_fresh, tf_arrivals, scan_arrivals, scan_stamps = [], [], [], []
def tf_cb(msg):
    now = time.time()
    for t in msg.transforms:
        if t.header.frame_id == "map" and t.child_frame_id == "odom":
            tf_fresh.append((now, t.header.stamp.sec + t.header.stamp.nanosec * 1e-9 - now)); tf_arrivals.append(now)
def scan_cb(m):
    now = time.time(); scan_arrivals.append(now); scan_stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
node.create_subscription(TFMessage, "/tf", tf_cb, 100)
node.create_subscription(LaserScan, "/scan", scan_cb, qos_profile_sensor_data)
t0 = time.monotonic()
while time.monotonic() - t0 < secs:
    rclpy.spin_once(node, timeout_sec=0.05)
gaps = lambda a: [(round(b - a[0], 1) if False else round(a[i] - a[i - 1], 2), round(a[i] - a[0], 1)) for i in range(1, len(a)) if a[i] - a[i - 1] > 0.5]
worst = min(tf_fresh, key=lambda x: x[1]) if tf_fresh else None
print(f"map->odom: {len(tf_fresh)} msgs; stamp-now min {worst[1]:+.2f} s at +{worst[0] - tf_fresh[0][0]:.1f} s; arrival gaps >0.5 s: {gaps(tf_arrivals)[:10]}")
print(f"/scan: {len(scan_arrivals)} msgs in {secs:.0f} s ({len(scan_arrivals) / secs:.1f} Hz); arrival gaps >0.5 s (gap, at): {gaps(scan_arrivals)[:10]}")
if scan_stamps:
    d = [scan_stamps[i] - scan_stamps[i - 1] for i in range(1, len(scan_stamps))]
    print(f"/scan stamp intervals: max {max(d):.2f} s, mean {sum(d) / len(d):.3f} s")
