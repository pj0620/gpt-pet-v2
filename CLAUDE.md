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

## About the Robot

- Xbox 360 Kinect camera
- ICM20948 IMU
- 4 motors with encoders
- RPiLIDAR

Full URDF: `ros2_ws/src/bot_ws/src/startup/urdf/gptpet.xacro`
