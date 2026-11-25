This repo contains two ROS2 Humble workspaces: one for the server and one for the robot.

**Both workspaces must be running simultaneously.** Use Foxglove to debug the robot.

## Shell Setup

Always keep **two shells open**:

| Shell | Location | How to open |
|-------|----------|-------------|
| **Server shell** | This machine (VS Code terminal) | Already open |
| **Robot shell** | Jetson Nano on the robot | `./bin/ssh/gptpetclient.sh` (user: `gptpetclient`, password: `gptpetclient`) |

## Launching the Full System

### 1. Start server nodes (server shell)

```bash
cd ~/gpt-pet-v2/ros2_ws/src/server_ws
colcon build
. ./install/setup.bash
ros2 launch startup server.launch.py
```

### 2. Start robot nodes (robot shell)

```bash
# SSH into the robot first
./bin/ssh/gptpetclient.sh

# Then on the robot:
cd ~/gpt-pet-v2/ros2_ws/src/bot_ws
colcon build
. ./install/setup.bash
ros2 launch startup client.launch.py
```

Both launches must be running before using Foxglove to debug.

## Server Workspace

**Location:** `ros2_ws/src/server_ws`

Runs on the server. Handles compute-intensive tasks (SLAM, etc.). Launch files are in `ros2_ws/src/server_ws/src/startup/launch`.

### Making changes

Run these commands in the **server shell**:

```bash
cd ~/gpt-pet-v2/ros2_ws/src/server_ws
# 1. Edit files
colcon build
. ./install/setup.bash
ros2 launch startup server.launch.py
```

## Bot Workspace

**Location:** `ros2_ws/src/bot_ws`

Runs on the Jetson Nano. Only handles tasks that must run on the robot (motor control, sensor publishing, etc.). Main launch file: `ros2_ws/src/bot_ws/src/startup/launch/client.launch.py`.

### Making changes

Changes are deployed to the robot via **git** — edit locally, commit and push, then pull on the robot.

**Server shell** (commit and push):
```bash
# 1. Edit files in ros2_ws/src/bot_ws
git add --all && git commit -m "<message>" && git push
```

**Robot shell** (pull, build, and launch):
```bash
cd ~/gpt-pet-v2/ros2_ws/src/bot_ws
git pull
colcon build && . ./install/setup.bash
ros2 launch startup client.launch.py
```

## Mac Dev Machine (this checkout)

This Mac is not the server: it has a RoboStack ROS 2 Humble env (mamba `ros_env`) and talks to the
robot directly. Multicast discovery does not cross the WiFi and the robot runs Fast DDS, so always:

```bash
source bin/mac_env.sh      # Fast DDS, domain 0, unicast peers for gptpetclient.local / pj-ubuntu.local
ros2 topic list            # robot topics now visible; ros2 param set /gptpet_motors ... works too
./bin/rviz.sh              # RViz2 with rviz2_config.rviz (Fixed Frame map needs the server running)
./bin/foxglove.sh          # local foxglove_bridge on ws://localhost:8765 for the Foxglove web app
./bin/motors/teleop.sh     # keyboard teleop clamped to 0.1 m/s, 0.5 rad/s
./bin/motors/forward.sh 0.2 0.1
```

The robot also runs `foxglove_bridge` from `client.launch.py` on `ws://gptpetclient.local:8765`
(Foxglove desktop app, no ROS needed). `~/.zshrc` exports CycloneDDS, which `mac_env.sh` overrides.

## One-time Robot Setup

`bin/robot/install_udev_rules.sh` (run on the robot) names the ESP32 motor driver
`/dev/gptpet_motors`, keeps ModemManager off it, and lets `gptpet_hardware` reset its USB device
when the serial link stalls. Start the client launch with `nohup setsid ... &` or inside tmux: a
dropped SSH session kills it otherwise. `client.launch.py` takes `foxglove:=false` and `kinect:=false`.

## About the Robot

- Xbox 360 Kinect camera
- ICM20948 IMU
- 4 motors with encoders
- RPiLIDAR

Full URDF: `ros2_ws/src/bot_ws/src/startup/urdf/gptpet.xacro`
