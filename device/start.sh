#!/bin/bash
# Start the whole device (Stages 1 to 4) for a test session, after clearing leftovers.
#   bash start.sh            top speed 0.2 m/s (since 23 Nov)
#   bash start.sh 0.5        top speed 0.5 m/s
#   bash start.sh 0.3 map:=esb117_device     any extra launch options after the speed
# Settings that stay the same from run to run (the campus PC's address, the voice, the map,
# the camera's tilt ...) are read from ~/apnvi_device.conf: one launch option per line,
# lines starting with # are skipped (see apnvi_device.conf.example here). Options given on
# the command line come after them and win.
# Started so that Ctrl+C (and stop.sh) stops every program.
# Log: /tmp/apnvi_session.log
HERE="$(cd "$(dirname "$0")" && pwd)"
SPEED="${1:-0.2}"                   # 23 Nov: 0.3 -> 0.2 (the LiDAR and grip dropped at 75-80% power)
shift
source /opt/ros/humble/setup.bash
source ~/apnvi_ws/install/setup.bash
bash "$HERE/stop.sh" quiet

# The patched tf2 library goes first (17 Nov): with ROS's own libtf2.so Nav2's programs lock
# up a few seconds after start and guided trips never move (lab row A1). Built once by
# jetson_setup/tf2_fix/build_tf2_fix.sh.
TF2_FIX=~/tf2_fix_ws/install/tf2/lib
if [ -f "$TF2_FIX/libtf2.so" ]; then
  export LD_LIBRARY_PATH="$TF2_FIX:$LD_LIBRARY_PATH"
else
  echo "WARNING: no patched tf2 library ($TF2_FIX): guided trips will not move."
  echo "         Run jetson_setup/tf2_fix/build_tf2_fix.sh once, then start again."
fi

# Stage 1's ESP32: any USB serial board that is not the master (..._0001) or the LiDAR (CP2102N).
STAGE1=$(ls /dev/serial/by-id/ 2>/dev/null | grep -v '_0001-' | grep -v 'CP2102N' | head -1)
if [ -n "$STAGE1" ]; then
  STAGE1_PORT="/dev/serial/by-id/$STAGE1"
  echo "Stage 1 ESP32: $STAGE1_PORT"
else
  STAGE1_PORT="/dev/serial/by-id/stage1-esp32-not-plugged"
  echo "Stage 1 ESP32: not plugged in (Stage 1 runs on the camera alone)"
fi

CONF=()
if [ -f ~/apnvi_device.conf ]; then
  while IFS= read -r line; do
    line="${line%%#*}"; line="$(echo "$line" | xargs)"
    [ -n "$line" ] && CONF+=("$line")
  done < ~/apnvi_device.conf
  echo "settings from ~/apnvi_device.conf: ${CONF[*]}"
else
  echo "no ~/apnvi_device.conf: Stage 2 will not find the campus PC unless pc:=<address> is given"
fi

cd ~
( trap - INT; exec setsid ros2 launch apnvi_stage4 apnvi.launch.py port:="$STAGE1_PORT" \
    max_speed_mps:="$SPEED" "${CONF[@]}" "$@" > /tmp/apnvi_session.log 2>&1 ) &
echo "starting, top speed $SPEED m/s ... (45 s)"
sleep 45
# Did Nav2's programs really load the patched tf2 library?
for PROGRAM in nav2_controller/controller_server nav2_planner/planner_server; do
  PID=$(pgrep -f "lib/$PROGRAM" | head -1)
  if [ -n "$PID" ] && ! grep -q "$TF2_FIX/libtf2.so" "/proc/$PID/maps" 2>/dev/null; then
    echo "WARNING: $PROGRAM runs with the old tf2 library: guided trips will not move."
  fi
done
python3 "$HERE/session.py" health
