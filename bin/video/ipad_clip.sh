#!/bin/bash
# Cuts a clip out of the iPad recording (bin/video/ipad_record.sh must be running or have run).
# Usage: bin/video/ipad_clip.sh START SECONDS [OUT.mp4]
#   START: "last" (the last SECONDS), a time today like 15:40:05, or RFC3339 like 2026-11-15T15:40:05-05:00
# Then: bin/video/watch.sh OUT.mp4
START="$1"; SECS="${2:-30}"; OUT="${3:-$HOME/robot-rig/clip_$(/bin/date +%H%M%S).mp4}"
[ -n "$START" ] || { sed -n '2,5p' "$0"; exit 2; }
if [ "$START" = last ]; then
  START=$(/bin/date -v-"$((SECS + 2))"S +%Y-%m-%dT%H:%M:%S%z)
elif [[ "$START" =~ ^[0-9]{1,2}:[0-9]{2}(:[0-9]{2})?$ ]]; then
  [ ${#START} -le 5 ] && START="$START:00"
  START="$(/bin/date +%Y-%m-%d)T$START$(/bin/date +%z)"
fi
[[ "$START" =~ [+-][0-9]{4}$ ]] && START="${START:0:${#START}-2}:${START: -2}"   # +hhmm -> +hh:mm for the playback server
curl -sSf -G "http://localhost:9996/get" --data-urlencode "path=ipad" --data-urlencode "start=$START" \
  --data-urlencode "duration=$SECS" --data-urlencode "format=mp4" -o "$OUT" || {
  echo "no recording for $START (+$SECS s); available:" >&2; curl -s "http://localhost:9996/list?path=ipad" >&2; echo >&2; exit 1; }
echo "$OUT"
