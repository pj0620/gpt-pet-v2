This repo contains two ROS2 Humble workspaces: one for the server and one for the robot.

## Shell Setup

Always keep **three shells open**:

| Shell | Location | How to open |
|-------|----------|-------------|
| **Server shell** | This machine (VS Code terminal) | Already open |
| **Robot shell** | Jetson Nano on the robot | `./bin/ssh/gptpetclient.sh` (user: `gptpetclient`, password: `gptpetclient`) |

## Server Workspace

**Location:** `ros2_ws/src/server_ws`

Runs on the server. Handles compute-intensive tasks (SLAM, etc.). Launch files are in `ros2_ws/src/server_ws/src/startup/launch`.

### Making changes

Run these commands in the **server shell**:

```bash
# 1. Edit files in ros2_ws/src/server_ws
colcon build
. ./install/setup.bash
# 2. Launch or test
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

**Robot shell** (pull and build):
```bash
cd /home/gptpetclient/gpt-pet-v2/ros2_ws/src/bot_ws
git pull
colcon build && . ./install/setup.bash
# Launch or test
```

## About the Robot

- Xbox 360 Kinect camera
- ICM20948 IMU
- 4 motors with encoders
- RPiLIDAR

Full URDF: `ros2_ws/src/bot_ws/src/startup/urdf/gptpet.xacro`
