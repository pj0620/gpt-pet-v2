#!/bin/bash
# Records the iPad camera (RTSP from the OctoStream app) with mediamtx, using bin/video/mediamtx.yml.
# Usage: bin/video/ipad_record.sh          (foreground; Ctrl-C stops)
#        bin/video/ipad_record.sh --bg     (background, log in ~/robot-rig/mediamtx.log)
# IPAD_URL overrides the stream address. Recordings: ~/robot-rig/recordings/ipad, kept 72 h.
# Pull a clip:  bin/video/ipad_clip.sh START_TIME SECONDS OUT.mp4

IPAD_URL="${IPAD_URL:-rtsp://192.168.1.121:554/stream}"
DIR="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$HOME/robot-rig/recordings"
cd "$HOME/robot-rig"   # mediamtx writes its MoQ certificate (auto.crt/auto.key) into the working directory
export MTX_PATHS_IPAD_SOURCE="$IPAD_URL"
export MTX_PATHS_IPAD_RECORDPATH="$HOME/robot-rig/recordings/%path/%Y-%m-%d_%H-%M-%S-%f%z"
pkill -f "mediamtx $DIR/mediamtx.yml" 2>/dev/null && sleep 0.5
if [ "$1" = "--bg" ]; then
  nohup mediamtx "$DIR/mediamtx.yml" >>"$HOME/robot-rig/mediamtx.log" 2>&1 &
  echo "recording $IPAD_URL in the background (pid $!), log ~/robot-rig/mediamtx.log"
else
  exec mediamtx "$DIR/mediamtx.yml"
fi
