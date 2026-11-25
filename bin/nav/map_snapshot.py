#!/usr/bin/env python3
"""Save /map, the robot pose (map->base_link) and the latest /scan in map coordinates to a JSON file.
Usage: map_snapshot.py OUT.json"""
import base64, json, math, sys, time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf2_ros

def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

rclpy.init()
node = rclpy.create_node("map_snapshot")
buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf, node)
maps, scans = [], []
node.create_subscription(OccupancyGrid, "/map", maps.append,
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                    reliability=ReliabilityPolicy.RELIABLE))
node.create_subscription(LaserScan, "/scan", scans.append, qos_profile_sensor_data)
deadline = time.monotonic() + 30
pose = laser = None
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    if maps and scans:
        try:
            t = buf.lookup_transform("map", "base_link", rclpy.time.Time())
            l = buf.lookup_transform("map", scans[-1].header.frame_id, rclpy.time.Time())
            pose, laser = t, l
            break
        except Exception:
            pass
if not (maps and scans and pose):
    print(f"incomplete: map={bool(maps)} scan={bool(scans)} tf={bool(pose)}"); sys.exit(1)
m, s = maps[-1], scans[-1]
lx, ly, lyaw = laser.transform.translation.x, laser.transform.translation.y, yaw_of(laser.transform.rotation)
pts = []
for i, r in enumerate(s.ranges):
    if s.range_min < r < s.range_max and math.isfinite(r):
        a = lyaw + s.angle_min + i * s.angle_increment
        pts.append((round(lx + r * math.cos(a), 3), round(ly + r * math.sin(a), 3)))
out = {
    "stamp": time.time(),
    "map": {"w": m.info.width, "h": m.info.height, "res": m.info.resolution,
            "ox": m.info.origin.position.x, "oy": m.info.origin.position.y,
            "data": base64.b64encode(bytes((v + 256) % 256 for v in m.data)).decode()},
    "robot": {"x": pose.transform.translation.x, "y": pose.transform.translation.y,
              "yaw": yaw_of(pose.transform.rotation)},
    "laser": {"x": lx, "y": ly, "yaw": lyaw},
    "scan": pts,
}
json.dump(out, open(sys.argv[1], "w"))
r = out["robot"]
print(f"map {m.info.width}x{m.info.height} @ {m.info.resolution} m, robot ({r['x']:.3f}, {r['y']:.3f}) "
      f"yaw {math.degrees(r['yaw']):.1f} deg, {len(pts)} scan points")
