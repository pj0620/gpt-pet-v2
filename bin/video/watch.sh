#!/bin/bash
# "Watch" a video from the test-rig camera: contact sheet + motion trail (+ ArUco track when a floor
# layout is given). Afterwards Read the listed images; the printed numbers are the quantitative part.
# Usage: bin/video/watch.sh VIDEO [floor.json]
set -e
cd "$(dirname "$0")/../.." || exit 1
PY="${GPTPET_PY:-$HOME/.local/share/mamba/envs/ros_env/bin/python3}"
[ -x "$PY" ] || PY=python3
VIDEO="$1"; FLOOR="$2"
[ -f "$VIDEO" ] || { echo "usage: $0 VIDEO [floor.json]" >&2; exit 1; }
OUT="${VIDEO%.*}_watch"; mkdir -p "$OUT"
echo "== sheet";  "$PY" bin/video/sheet.py "$VIDEO" -o "$OUT/sheet.jpg"
echo "== trail";  "$PY" bin/video/trail.py "$VIDEO" -o "$OUT"
if [ -n "$FLOOR" ]; then echo "== track"; "$PY" bin/video/track.py "$VIDEO" --floor "$FLOOR" -o "$OUT"; fi
echo "Read: $OUT/sheet.jpg $OUT/trail.png $OUT/motion.png${FLOOR:+ $OUT/track.png}"
