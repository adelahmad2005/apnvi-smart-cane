#!/bin/bash
# Replay the recorded lap through the LiDAR filter + icp_odometry with one set of settings, and
# report where it thinks the lap ended (it really ended back on START: 0 0 0).
#   bash replay.sh NAME DOMAIN FILTER(on|off) PARAMS_YAML
NAME=$1; export ROS_DOMAIN_ID=$2; FILTER=$3; PARAMS=$4
source /opt/ros/humble/setup.bash
source ~/apnvi_ws/install/setup.bash
B=$(ls -td ~/lap_bags/lap_* | head -1)
OUT=/tmp/replay_$NAME; mkdir -p $OUT
if [ "$FILTER" = "on" ]; then
  ros2 run apnvi_stage3 scan_filter > $OUT/filter.log 2>&1 &
  SCAN=/scan
else
  SCAN=/scan_raw
fi
ros2 run rtabmap_odom icp_odometry --ros-args --params-file $PARAMS -r scan:=$SCAN -r odom:=/odom \
  > $OUT/odom.log 2>&1 &
python3 /tmp/odom_eval.py $OUT/track.txt > $OUT/eval.txt 2>&1 &
EVAL=$!
sleep 4
ros2 bag play $B > $OUT/play.log 2>&1
wait $EVAL
pkill -f "ROS_DOMAIN_ID=$2" 2>/dev/null
for p in $(pgrep -f "icp_odometry|scan_filter"); do
  if tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$2"; then kill -INT $p; fi
done
sleep 1
echo "$NAME: $(tail -1 $OUT/eval.txt)  | lost: $(grep -ciE 'lost' $OUT/odom.log)"
