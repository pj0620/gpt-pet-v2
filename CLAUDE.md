# GPT Pet v2 — ROS2 Workspace Guide

This project has two ROS2 Humble workspaces with distinct roles:

| Workspace | Location | Runs on | Purpose |
|-----------|----------|---------|---------|
| **Server** | `ros2_ws/src/server_ws` | Ubuntu server (this machine) | Compute-heavy nodes (SLAM, etc.) |
| **Bot** | `ros2_ws/src/bot_ws` | Jetson Nano (on the robot) | Hardware-facing nodes (motor control, sensor publishing) |

---

## Server Workspace

**Path:** `ros2_ws/src/server_ws`
**Launch files:** `ros2_ws/src/server_ws/src/startup/launch/`

Changes are built and run directly on this machine.

### Workflow

```bash
# 1. Make edits in ros2_ws/src/server_ws
# 2. Build
colcon build
# 3. Source the workspace
. ./install/setup.bash
# 4. Run/test (launch files, nodes, etc.)
```

---

## Bot Workspace

**Path:** `ros2_ws/src/bot_ws`
**Main launch file:** `ros2_ws/src/bot_ws/src/startup/launch/client.launch.py`
**Test launch files:** `ros2_ws/src/bot_ws/src/startup/launch/` (add extras here)

Changes must be committed and pulled onto the robot — you cannot build directly on the Jetson from here.

### Workflow

```bash
# 1. Make edits in ros2_ws/src/bot_ws
# 2. Commit and push
git add --all && git commit -m "<descriptive message>" && git push
```

Then, in a separate terminal, SSH into the robot:

```bash
./bin/ssh/gptpetclient.sh
```

On the robot:

```bash
cd /home/gptpetclient/gpt-pet-v2/ros2_ws/src/bot_ws
git pull
colcon build
# Launch a file or run commands
```

---

## Robot Hardware

**URDF:** `ros2_ws/src/bot_ws/src/startup/urdf/gptpet.xacro`

- Xbox 360 Kinect camera
- ICM20948 IMU
- 4 motors with encoders
