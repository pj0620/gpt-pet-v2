#!/usr/bin/env python3
"""Send a Nav2 NavigateToPose goal in the map frame and record the run.
Usage: nav_goal.py X Y YAW_DEG [TIMEOUT_S] [OUT.json]
Records map->base_link every 0.2 s plus the last /cmd_vel, prints feedback once a second, and ends with
the action result. Writes the trail to OUT.json (default /tmp/nav_run.json)."""
import json, math, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
import tf2_ros

x, y, yaw = float(sys.argv[1]), float(sys.argv[2]), math.radians(float(sys.argv[3]))
timeout = float(sys.argv[4]) if len(sys.argv) > 4 else 120.0
out = sys.argv[5] if len(sys.argv) > 5 else "/tmp/nav_run.json"

def yaw_of(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

rclpy.init()
node = rclpy.create_node("nav_goal_runner")
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, node)
last_cmd = [0.0, 0.0, 0.0]
node.create_subscription(Twist, "/cmd_vel", lambda m: last_cmd.__setitem__(slice(0, 3), [m.linear.x, m.linear.y, m.angular.z]),
                         QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))
client = ActionClient(node, NavigateToPose, "navigate_to_pose")
if not client.wait_for_server(timeout_sec=15.0):
    print("navigate_to_pose action server not available"); sys.exit(1)

def pose():
    try:
        t = buf.lookup_transform("map", "base_link", rclpy.time.Time())
        return [round(t.transform.translation.x, 3), round(t.transform.translation.y, 3), round(yaw_of(t.transform.rotation), 3)]
    except Exception:
        return None

deadline = time.monotonic() + 10
while pose() is None and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
start_pose = pose()
goal = NavigateToPose.Goal()
goal.pose = PoseStamped()
goal.pose.header.frame_id = "map"
goal.pose.header.stamp = rclpy.time.Time().to_msg()  # 0 = latest TF, like bin/send_goal.py
goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
feedback = {}
def on_feedback(msg):
    f = msg.feedback
    feedback.update(dist=f.distance_remaining, recoveries=f.number_of_recoveries,
                    t=f.navigation_time.sec + f.navigation_time.nanosec * 1e-9)
t_start = time.time()
send = client.send_goal_async(goal, feedback_callback=on_feedback)
rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
handle = send.result()
if handle is None or not handle.accepted:
    print("goal rejected"); sys.exit(1)
print(f"goal accepted: ({x:.2f}, {y:.2f}) yaw {math.degrees(yaw):.0f} deg, from {start_pose}")
result_future = handle.get_result_async()
trail, last_print, last_rec = [], 0.0, 0.0
status_names = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
while not result_future.done() and time.time() - t_start < timeout:
    rclpy.spin_once(node, timeout_sec=0.05)
    now = time.time()
    if now - last_rec >= 0.2:
        last_rec = now
        trail.append({"t": round(now - t_start, 2), "pose": pose(), "cmd": [round(v, 3) for v in last_cmd]})
    if now - last_print >= 1.0:
        last_print = now
        p = pose()
        print(f"  t={now - t_start:5.1f}s pose={p} cmd={[round(v, 2) for v in last_cmd]} "
              f"remaining={feedback.get('dist', float('nan')):.2f} m recoveries={feedback.get('recoveries', 0)}", flush=True)
if not result_future.done():
    print("TIMEOUT: cancelling goal")
    cancel = handle.cancel_goal_async(); rclpy.spin_until_future_complete(node, cancel, timeout_sec=5.0)
    status = "TIMEOUT"
else:
    status = status_names.get(result_future.result().status, str(result_future.result().status))
end_pose = pose()
err = None
if end_pose:
    err = {"dist_m": round(math.hypot(end_pose[0] - x, end_pose[1] - y), 3),
           "yaw_deg": round(math.degrees(math.atan2(math.sin(end_pose[2] - yaw), math.cos(end_pose[2] - yaw))), 1)}
summary = {"goal": [x, y, round(yaw, 3)], "start": start_pose, "end": end_pose, "status": status, "error": err,
           "t_start": t_start, "t_end": time.time(), "recoveries": feedback.get("recoveries", 0), "trail": trail}
json.dump(summary, open(out, "w"))
print(f"RESULT {status} after {time.time() - t_start:.1f} s, end pose {end_pose}, error {err}, trail -> {out}")
