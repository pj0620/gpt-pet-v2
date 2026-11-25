#!/usr/bin/env python3
"""Print the nearest lidar return in 12 sectors of one /scan (laser frame, 0 deg = laser +x, CCW positive)."""
import math, sys
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

rclpy.init()
node = rclpy.create_node("scan_sectors")
got = []
node.create_subscription(LaserScan, "/scan", lambda m: got.append(m), qos_profile_sensor_data)
end = node.get_clock().now().nanoseconds + 25e9
while not got and node.get_clock().now().nanoseconds < end:
    rclpy.spin_once(node, timeout_sec=0.2)
if not got:
    print("no scan in 25 s"); sys.exit(1)
m = got[0]
sectors = {}
for i, r in enumerate(m.ranges):
    if not (m.range_min < r < m.range_max) or math.isinf(r):
        continue
    a = math.degrees(m.angle_min + i * m.angle_increment) % 360
    s = int(((a + 15) % 360) // 30) * 30
    sectors[s] = min(sectors.get(s, 99), r)
print(f"frame {m.header.frame_id}, {len(m.ranges)} rays, angle {math.degrees(m.angle_min):.0f}..{math.degrees(m.angle_max):.0f} deg")
for s in range(0, 360, 30):
    print(f"  {s:3d} deg: {sectors.get(s, float('nan')):.2f} m")
