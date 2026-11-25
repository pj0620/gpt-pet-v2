#!/bin/bash
# Drive the robot straight forward a small distance, then stop — reliably.
#
# Uses ONE long-lived rclpy publisher instead of two fire-and-forget
# `ros2 topic pub` calls. It waits for the controller's /cmd_vel subscription to
# be discovered (over WiFi this takes a few seconds), streams a forward command
# for distance/speed seconds, then streams zeros over the SAME connection so the
# STOP is actually delivered. Aborting (Ctrl-C) also sends zeros. If no
# subscriber ever appears, it refuses to move.
#
# The mecanum_drive_controller subscribes to /cmd_vel as plain
# geometry_msgs/msg/Twist (use_stamped_vel:false) with BEST_EFFORT QoS — we match
# that. NOTE: the robot-side controller holds the last command for
# `reference_timeout` seconds; keep that short (see bot_ws controllers.yaml) so a
# missed stop can't run the robot into a wall.
#
# Usage: ./forward.sh [distance_m] [speed_mps]   (defaults: 0.2 m at 0.1 m/s)
# Run from a shell with the server workspace sourced.

DIST=${1:-0.2}     # metres
SPEED=${2:-0.1}    # m/s — keep above the motor deadband so it actually moves

exec python3 - "$DIST" "$SPEED" <<'PY'
import sys, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist

# --- args + safety caps (reject fat-finger runaways) ---
dist = abs(float(sys.argv[1]))
speed = float(sys.argv[2])
MAX_SPEED, MAX_DIST = 0.25, 2.0          # m/s, m
if abs(speed) > MAX_SPEED:
    print(f"[forward] speed {speed} m/s exceeds cap {MAX_SPEED} — clamping.")
    speed = MAX_SPEED if speed > 0 else -MAX_SPEED
if dist > MAX_DIST:
    print(f"[forward] distance {dist} m exceeds cap {MAX_DIST} — clamping.")
    dist = MAX_DIST
if speed == 0.0 or dist == 0.0:
    print("[forward] speed or distance is 0 — nothing to do.")
    sys.exit(1)

duration = dist / abs(speed)
RATE, STOP_SECS, WAIT_SUB_SECS = 20.0, 1.5, 8.0

rclpy.init()
node = rclpy.create_node("forward_nudge")
# Match the controller: BEST_EFFORT avoids RELIABLE retransmit stalls over WiFi.
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST)
pub = node.create_publisher(Twist, "/cmd_vel", qos)

def cmd(x):
    t = Twist(); t.linear.x = float(x); return t

def stream(x, secs):
    end = time.monotonic() + secs
    while time.monotonic() < end:
        pub.publish(cmd(x))
        time.sleep(1.0 / RATE)

# Don't move until the controller's subscription is actually discovered.
deadline = time.monotonic() + WAIT_SUB_SECS
while pub.get_subscription_count() < 1 and time.monotonic() < deadline:
    time.sleep(0.1)
if pub.get_subscription_count() < 1:
    print(f"[forward] no /cmd_vel subscriber after {WAIT_SUB_SECS:.0f}s "
          f"— aborting, robot NOT moved.")
    node.destroy_node(); rclpy.shutdown(); sys.exit(1)

print(f"[forward] connected. Moving {dist:.2f} m at {speed:.2f} m/s "
      f"(~{duration:.1f}s), then stopping.")
try:
    stream(speed, duration)
finally:
    # Always stop — normal end, Ctrl-C, or exception.
    stream(0.0, STOP_SECS)
    print("[forward] stopped.")
    node.destroy_node()
    rclpy.shutdown()
PY
