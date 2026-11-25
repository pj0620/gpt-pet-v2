---
name: robot-video
description: Watch a video of the robot from the test-rig camera (iPad on a tripod, any static camera) by turning it into images and numbers with bin/video - timestamped contact sheet, motion trail, ArUco metric track. Use whenever a test leaves an .mp4/.mov to review or when asked what the robot did on camera.
---

# Watching robot test videos

The Read tool cannot open video. Turn the clip into images plus numbers with the tools in
`bin/video/`, then Read the images. All tools run with the ROS env python (`source bin/mac_env.sh`
or the mamba `ros_env` python3; they need cv2, numpy, matplotlib).

## One command

    bin/video/watch.sh CLIP.mp4 [floor.json]

Creates `CLIP_watch/` next to the clip and prints what to Read:

- `sheet.jpg`: 12 timestamped frames, read left to right, top to bottom. Qualitative check: where
  the robot is, what changed, anything unexpected in the scene. Works for any camera.
- `trail.png` + `motion.png` (static camera only, no markers needed): onion skin of the moving
  object coloured blue (early) to red (late) with its centroid path; `motion.png` shows when
  motion happened (% of pixels changed versus 0.5 s earlier). The printed line gives start/stop.
- `track.png` + `summary.json` (only with `floor.json`): metric path from the ArUco markers with
  path length, net displacement, net heading change (positive = counter-clockwise), motion
  start/stop, max speed, max yaw rate and heading wobble.

## Individual tools

- `bin/video/sheet.py CLIP -n 12 --cols 4 --width 480 [--start S --end E]`: zoom into an interval.
- `bin/video/trail.py CLIP -o DIR [--samples 24 --thresh 35]`
- `bin/video/track.py CLIP --floor floor.json -o DIR [--hfov 60]`: hfov is the camera's horizontal
  field of view in degrees; it only feeds the marker-height correction.
- `bin/video/make_synthetic.py OUT --seed N [--camera angled] [--scenario NAME] [--reveal]`:
  renders a test clip with `truth.json` and `floor.json`. Re-verify the tools with it after
  changing them; for a blind check judge the clip before reading `truth.json`.

## Marker setup for metric tracking

- Floor: four ArUco 4x4_50 markers (ids 10 to 13) taped at measured positions; put their centres
  in `floor.json` in metres (`{"markers": {"10": [x, y], ...}}`). Any origin works; the track
  comes out in that frame.
- Robot: marker id 0 flat on top with the marker image's top edge facing forward, and its height
  above the floor in `floor.json` as `robot_marker_height_m`.
- Each marker needs a white border and at least 8 px per marker cell in the video.

## Reading the numbers

- Verified 2026-10-28 on synthetic clips with known truth: path, net displacement and heading
  within about 1 %, start and stop within 0.5 s (the detectors use a 0.5 s lag, so printed
  motion bounds are conservative).
- Compare with ROS: net displacement against /odom, heading change against the map to base_link
  yaw, motion start against the command timestamp for latency.
- The camera must not move during the clip. A following camera breaks trail and track; the
  sheet still works.
