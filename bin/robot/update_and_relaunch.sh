#!/usr/bin/env bash
# Run ON the robot (the Jetson): bring the checkout to a branch, update the kinect_ros2 submodule,
# install the image transport plugins the robot's MCP server reads (compressed topics), rebuild the
# bot workspace and relaunch the client launch in tmux session `client`.
#
#   bin/robot/update_and_relaunch.sh [branch]      # default: stay on the current branch
#
# Needs: ssh key auth as gptpetclient, passwordless sudo (/etc/sudoers.d/90-gptpetclient), the
# robot's internet (IPv4 only: apt-get -o Acquire::ForceIPv4=true). Deploys by git, as CLAUDE.md says.
set -euo pipefail

REPO="${GPTPET_REPO:-$HOME/gpt-pet-v2}"
BRANCH="${1:-}"
BOT_WS="$REPO/ros2_ws/src/bot_ws"
SUBMODULE="ros2_ws/src/bot_ws/src/kinect_ros2"

cd "$REPO"
git fetch --quiet origin
if [ -n "$BRANCH" ]; then
  git checkout --quiet "$BRANCH"
fi
git pull --quiet --ff-only
git submodule sync --quiet -- "$SUBMODULE"
git submodule update --init -- "$SUBMODULE"
echo "checkout: $(git rev-parse --abbrev-ref HEAD) @ $(git rev-parse --short HEAD); kinect_ros2 @ $(git -C "$SUBMODULE" rev-parse --short HEAD)"

if ! dpkg -s ros-humble-image-transport-plugins >/dev/null 2>&1; then
  echo "installing ros-humble-image-transport-plugins (compressed image topics)"
  sudo -n apt-get update -o Acquire::ForceIPv4=true
  sudo -n apt-get install -y -o Acquire::ForceIPv4=true ros-humble-image-transport-plugins
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
cd "$BOT_WS"
colcon build --packages-select kinect_ros2 startup --cmake-args -DCMAKE_BUILD_TYPE=Release

if tmux has-session -t client 2>/dev/null; then
  echo "stopping the running client launch"
  tmux send-keys -t client C-c "" || true
  sleep 6
  tmux kill-session -t client 2>/dev/null || true
fi
tmux new-session -d -s client \
  "bash -ic 'cd $BOT_WS && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch startup client.launch.py 2>&1 | tee ~/client_launch.log'"
echo "client launch started in tmux session client (log ~/client_launch.log); the Kinect node waits for the device and respawns"
