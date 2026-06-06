#!/bin/bash
# Cancel the current Nav2 goal and zero the velocity command.
cd ~/gpt-pet-v2/ros2_ws/src/server_ws && . ./install/setup.bash

# Send goal at current position (Nav2 immediately considers goal reached and stops)
python3 ~/gpt-pet-v2/bin/send_goal.py 0.0 &

# Also zero cmd_vel directly so motors stop even if Nav2 takes a moment
ros2 topic pub --times 5 --wait-matching-publishers /cmd_vel geometry_msgs/msg/Twist '{}'
