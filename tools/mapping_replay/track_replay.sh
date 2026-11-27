#!/bin/bash
# Replay the lap through the filter and the movement tracker (today's settings) and keep the whole
# path and some sweeps, then draw it.   bash track_replay.sh [CFG] [DOMAIN]
CFG=${1:-/tmp/cfg_now.yaml}; export ROS_DOMAIN_ID=${2:-61}
source /opt/ros/humble/setup.bash
source ~/apnvi_ws/install/setup.bash
B=${BAG:-$(ls -td ~/lap_bags/lap_* | head -1)}     # BAG=folder picks another recording
OUT=/tmp/track_$(basename $CFG .yaml); rm -rf $OUT; mkdir -p $OUT
ros2 run apnvi_stage3 scan_filter $FILTER_ARGS > $OUT/filter.log 2>&1 &   # FILTER_ARGS="--ros-args -p ..."
ros2 run rtabmap_odom icp_odometry --ros-args --params-file $CFG -r scan:=/scan -r odom:=/odom -r odom_info:=/odom_info \
  > $OUT/odom.log 2>&1 &
python3 /tmp/track_rec.py $OUT/track.npz > $OUT/rec.txt 2>&1 &
REC=$!
sleep 4
ros2 bag play $B > $OUT/play.log 2>&1
wait $REC
for p in $(pgrep -f "icp_odometry|scan_filter"); do
  if tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"; then kill -INT $p; fi
done
cat $OUT/rec.txt
python3 /tmp/plot_track.py $OUT/track.npz $OUT/lap_path.png
