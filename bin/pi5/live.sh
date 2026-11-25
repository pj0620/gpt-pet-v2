#!/bin/bash
# Live view of the Raspberry Pi 5 camera on pi5.local, in a window on this Mac.
# The Pi encodes H.264 with rpicam-vid (software x264 on a Pi 5) and streams it over SSH into ffplay:
# no ports to open, and closing the window stops the camera.
#
# Usage: bin/pi5/live.sh [--width W] [--height H] [--fps N] [--rotate] [--test] [-- rpicam-vid args]
#   --rotate  turn the picture 180 degrees (camera mounted upside down)
#   --test    stream a test pattern showing the Pi's clock instead of the camera (checks SSH + viewer)
#   -- ...    everything after -- goes to rpicam-vid, e.g.  -- --hflip --autofocus-mode continuous
# Defaults 1280x720 at 30 fps. PI5_HOST=user@host overrides pj@pi5.local.
# In the window: q or Esc quits, f toggles full screen. One viewer at a time (a new run stops the old one).

HOST="${PI5_HOST:-pj@pi5.local}"
W=1280; H=720; FPS=30; ROT=0; TEST=0; EXTRA=""
while [ $# -gt 0 ]; do
  case "$1" in
    --width)  W="$2"; shift 2 ;;
    --height) H="$2"; shift 2 ;;
    --fps)    FPS="$2"; shift 2 ;;
    --rotate) ROT=180; shift ;;
    --test)   TEST=1; shift ;;
    --)       shift; [ $# -gt 0 ] && EXTRA=$(printf '%q ' "$@"); break ;;
    -h|--help) sed -n '2,11p' "$0"; exit 0 ;;
    *) echo "unknown option $1 (see --help)" >&2; exit 2 ;;
  esac
done

FFPLAY=$(command -v ffplay)
for p in "$HOME/.local/share/mamba/envs/ros_env/bin/ffplay" /opt/homebrew/bin/ffplay /usr/local/bin/ffplay; do
  [ -z "$FFPLAY" ] && [ -x "$p" ] && FFPLAY="$p"
done
[ -n "$FFPLAY" ] || { echo "ffplay not found (brew install ffmpeg)" >&2; exit 1; }

SSH=(ssh -o BatchMode=yes -o ConnectTimeout=5 -o ServerAliveInterval=5 -o ServerAliveCountMax=3)
TAG=pi5-live-view                       # argv[0] of the streaming process on the Pi
STOP="pkill -f '^$TAG ' ; sleep 0.3"    # stops a stream a previous run left holding the camera

if [ "$TEST" = 1 ]; then
  "${SSH[@]}" "$HOST" "$STOP" || { echo "cannot reach $HOST over ssh" >&2; exit 1; }
  SRC="exec -a $TAG ffmpeg -hide_banner -loglevel error -re -f lavfi -i testsrc2=size=${W}x${H}:rate=$FPS \
-vf drawtext=text=%{localtime}:fontsize=$((H / 10)):fontcolor=white:box=1:boxcolor=black@0.6:x=20:y=20 \
-c:v libx264 -preset ultrafast -tune zerolatency -g $FPS -pix_fmt yuv420p -f h264 -"
else
  CAMS=$("${SSH[@]}" "$HOST" "$STOP; rpicam-hello --list-cameras 2>&1")
  [ $? = 255 ] && { echo "cannot reach $HOST over ssh" >&2; exit 1; }
  if ! grep -q "Available cameras" <<<"$CAMS"; then
    cat >&2 <<MSG
No camera detected on $HOST (rpicam-hello --list-cameras: $(head -1 <<<"$CAMS"))
Shut the Pi down and reseat the camera ribbon at both ends with the latches pushed closed. On a Pi 5
the narrow 22-pin end goes into a CAM/DISP socket with its gold contacts facing the Ethernet port;
the wider 15-pin cable that comes with most camera modules does not fit a Pi 5. The Pi looks for
the camera at boot. Run with --test to check the stream and the viewer without a camera.
MSG
    exit 1
  fi
  grep -E '^ *[0-9]+ : ' <<<"$CAMS" | head -1
  # A Pi 5 has no H.264 hardware: rpicam-vid encodes with x264 through libav, which cannot write to
  # stdout unless the format is given. --low-latency = x264 ultrafast/zerolatency, no B-frames.
  SRC="export LIBCAMERA_LOG_LEVELS='*:ERROR'; exec -a $TAG rpicam-vid -t 0 -n -v 0 \
--width $W --height $H --framerate $FPS --rotation $ROT \
--codec h264 --libav-format h264 --low-latency --inline --intra $FPS $EXTRA -o -"
fi

trap '"${SSH[@]}" "$HOST" "$STOP" >/dev/null 2>&1' EXIT
echo "Live view ${W}x${H} @ $FPS fps from $HOST$([ "$TEST" = 1 ] && echo ' (test pattern)'). q or Esc closes it."
# setpts=N/FPS gives every frame a near-zero timestamp, so ffplay shows each one the moment it decodes
# instead of scheduling it; -framedrop then skips any backlog, keeping the view at the newest frame.
# rpicam-vid's libav encoder always prints its x264 setup to stderr; the grep passes only real errors.
"${SSH[@]}" -n -T "$HOST" "$SRC" 2> >(grep -vE '^\[libx264 @|^Output #0|^ +(Stream #0|Side data:|cpb:)' >&2) \
  | "$FFPLAY" -hide_banner -loglevel error -nostats -autoexit \
  -fflags nobuffer -flags low_delay -framedrop -probesize 32 -analyzeduration 0 \
  -f h264 -vf "setpts=N/$FPS" -window_title "$HOST camera" -
