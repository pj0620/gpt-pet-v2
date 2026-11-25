# bin/nav — Nav2 / SLAM test tools (2026-11-15)

Run the ROS ones on the server (`ssh pj@pj-ubuntu.local`, `source /opt/ros/humble/setup.bash`,
`. ~/gpt-pet-v2/ros2_ws/src/server_ws/install/setup.bash`) while both launches are up.
`render_map.py` runs anywhere with matplotlib (the Mac's `ros_env`).

| Script | What it does |
|---|---|
| `nav_goal.py X Y YAW_DEG [TIMEOUT] [OUT.json]` | Sends a NavigateToPose goal in the map frame, prints pose / cmd_vel / distance once a second, records the trail to JSON, reports the result. |
| `move.py VX VY WZ SECS` | Timed open-loop move (caps 0.15 m/s, 0.6 rad/s, 10 s); prints the displacement seen by wheel odometry, EKF and SLAM in the robot's starting frame. Disagreement between them points at a TF or kinematics problem. |
| `wheels.py VX VY WZ SECS` | Same move while recording `/joint_states`; prints each wheel's measured speed against the mecanum kinematics. Found the dead reverse on the front-right motor. |
| `map_snapshot.py OUT.json` | Saves `/map`, the robot pose and the latest scan in map coordinates. |
| `render_map.py SNAP.json OUT.png [--nav run.json] [--trail a.json ...] [--goal x y]` | Renders a snapshot to PNG with the scan, robot arrow, 0.5 m grid and an optional Nav2 trail. |
| `scan_sectors.py` | Nearest lidar return in 12 sectors of one scan (laser frame; robot forward is the lidar's 90 deg). |
| `tf_gaps.py [SECS]` | Reports map->odom freshness dips and `/scan` arrival gaps; use it to check WiFi scan loss. |
| `drift.py SECS` | EKF yaw drift and gyro bias while the robot is still. |

Camera validation: `bin/video/ipad_record.sh --bg` records the iPad, `bin/video/ipad_clip.sh START SECS OUT.mp4`
cuts a clip (START from the JSON's `t_start`), `bin/video/watch.sh` or `bin/video/sheet.py` turn it into images.
