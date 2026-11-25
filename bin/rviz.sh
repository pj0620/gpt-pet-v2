#!/bin/bash
# RViz2 on the Mac, connected to the robot (see bin/mac_env.sh).
# The saved config uses Fixed Frame "map", which exists once the server's SLAM/EKF is running.
# With only the robot running, set Fixed Frame to "laser" (lidar) or "base_link" in Global Options.
cd "$(dirname "$0")/.." || exit 1
source bin/mac_env.sh
exec rviz2 -d rviz2_config.rviz "$@"
