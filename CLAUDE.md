This code has two different ros2 workspaces both using ros2 humble.

## Server Workspace

The Server ros2 workspace can be found under ros2_ws/src/server_ws. This workspace will be run on a large ubuntu server which this vscode terminal is running on. This will be running the more compute intensive ros2 nodes (slam, etc). The launch files for this package can be found under ros2_ws/src/server_ws/src/startup/launch. 

### Changes to server workspace

To make a change in the server workspace, run the following.

1. Make edits in ros2_ws/src/server_ws
2. If launch file will be used, update launch files in ros2_ws/src/server_ws/src/startup/launch
3. run `colcon build`
4. run ` . ./install/setup.bash` to source updatd code
5. launch launch files, or do whatever you need to run/test new code

## Bot Workspace

The bot ros2 workspace can be found under ros2_ws/src/bot_ws. This workspace is run on a small Jetson Nano on the actual robot. Being a small compute this workspace should only be running things that need to be run on the robot itself (ros control nodes, kinect data publishing, etc) with everything else being left to the server workspace. 

The main launch file to run on the robot is ros2_ws/src/bot_ws/src/startup/launch/client.launch.py. However feel free to add other test launch files in ros2_ws/src/bot_ws/src/startup/launch to test things.

### Changes to bot workspace

To make changes in the bot workspace, you must make the changes locally commit the changes, then pull
them on the robot through git. Steps outlines below.

1. Make edits in ros2_ws/src/bot_ws
2. If a launch file is changed update ros2_ws/src/bot_ws/src/startup/launch/client.launch.py or a new launch file in ros2_ws/src/bot_ws/src/startup/launch
3. commit your changes `git add --all && git commit -m "[insert a good message]" && git push`
4. In a new terminal (or in a dedicated shell you keep open), ssh into the robot. `./bin/ssh/gptpetclient.sh` 
5. Goto the bot workspace. `cd /home/gptpetclient/gpt-pet-v2/ros2_ws/src/bot_ws`
6. Pull the changes you commited `git pull` (You should see your changes getting pulled in)
7. `colcon build && . ./install/setup.bash`
8. Launch a launch file, run commands etc. 

## About the robot

The robot features

- A Xbox 360 kinect camera
- A ICM20948 imu
- 4 motors with encoders
- RPiLIDAR unit

More details can be found at ros2_ws/src/bot_ws/src/startup/urdf/gptpet.xacro
