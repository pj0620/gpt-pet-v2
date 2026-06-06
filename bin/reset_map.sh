#!/bin/bash
# Resets the SLAM map by killing slam_toolbox, deleting stale posegraph files,
# and relaunching slam_toolbox within the existing server session.
# async_slam_toolbox_node does not expose a /clear service in mapping mode.

set -e

echo "Stopping slam_toolbox..."
pkill -f async_slam_toolbox_node || true
sleep 1

echo "Deleting stale posegraph files..."
rm -f ~/.ros/*.posegraph ~/.ros/*.data /tmp/*.posegraph /tmp/*.data

echo "Relaunching slam_toolbox..."
. ~/gpt-pet-v2/ros2_ws/src/server_ws/install/setup.bash

SLAM_PARAMS=$(ros2 pkg prefix startup)/share/startup/config/slam_toolbox_parameters.yaml

ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --remap __node:=slam_toolbox \
  --params-file "$SLAM_PARAMS" \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p map_frame:=map \
  -p scan_topic:=/scan \
  -p mode:=mapping \
  -p use_map_saver:=false \
  -p transform_publish_period:=0.05 \
  -p transform_timeout:=2.0 \
  -p minimum_travel_distance:=0.02 &

echo "slam_toolbox restarted (PID $!). Map reset complete."
