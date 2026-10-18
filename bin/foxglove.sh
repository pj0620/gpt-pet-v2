#!/bin/bash
# Foxglove on the Mac. Two ways to connect:
#  1. The robot runs foxglove_bridge itself (client.launch.py, port 8765). In the Foxglove desktop
#     app: Open connection -> Foxglove WebSocket -> ws://gptpetclient.local:8765. No ROS needed here.
#  2. This script runs a foxglove_bridge locally, on top of this Mac's own DDS peer (bin/mac_env.sh),
#     at ws://localhost:<port>. localhost is the only kind of address the Foxglove *web* app
#     (https://app.foxglove.dev) is allowed to use, so this is the path for the browser version.
# Usage: ./bin/foxglove.sh [port]   (default 8765)
cd "$(dirname "$0")/.." || exit 1
source bin/mac_env.sh
PORT=${1:-8765}
echo "Foxglove: connect to ws://localhost:$PORT (web app or desktop app)."
echo "          The robot's own bridge is at ws://gptpetclient.local:8765 (desktop app)."
exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=$PORT
