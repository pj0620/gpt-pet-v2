#!/bin/bash
# Keyboard teleop from the Mac, speed-clamped. teleop_twist_keyboard starts at 0.5 m/s and 1 rad/s,
# well beyond what this robot should do indoors (Nav2 runs it at 0.1 m/s), so its output goes to
# /cmd_vel_teleop and cmd_vel_limiter.py clamps it before republishing on /cmd_vel.
# The robot stops by itself 0.5 s after the last command (controller reference_timeout), and the
# limiter publishes zeros when this script exits.
# Usage: ./bin/motors/teleop.sh [max_linear_m_s] [max_angular_rad_s]   (defaults 0.1 and 0.5)
cd "$(dirname "$0")/../.." || exit 1
source bin/mac_env.sh
MAX_LIN=${1:-0.1}
MAX_ANG=${2:-0.5}
python3 bin/motors/cmd_vel_limiter.py "$MAX_LIN" "$MAX_ANG" &
LIMITER=$!
# SIGTERM, not SIGINT: background jobs of a script ignore SIGINT, and Ctrl-C already goes to teleop
trap 'kill -TERM $LIMITER 2>/dev/null; wait $LIMITER 2>/dev/null' EXIT
sleep 2
echo "Keys: i/k forward/back, j/l turn, u/o/m/, diagonals, J/L strafe (shift), q/z speed +-10%, Ctrl-C to quit."
echo "Commands are clamped to $MAX_LIN m/s and $MAX_ANG rad/s before they reach the robot."
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop
