#!/bin/bash
# Replay one or more recordings one after the other into ONE scan filter + movement tracker, so the
# tracker keeps its memory from one to the next (e.g. a lap, then a spin on START). Plays only
# /scan_raw and /tf_static, so bags that also hold the live /odom and /tf can be used (16 Nov).
#   bash seq_replay.sh NAME DOMAIN CFG_YAML BAG1 [BAG2 ...]
# Output in /tmp/seq_NAME: track.npz (every pose), marks.txt (the time each bag started), rec.txt.
NAME=$1; export ROS_DOMAIN_ID=$2; CFG=$3; shift 3
source /opt/ros/humble/setup.bash
source ~/apnvi_ws/install/setup.bash
OUT=/tmp/seq_$NAME; rm -rf $OUT; mkdir -p $OUT
if [ "$RESTAMP" = "1" ]; then           # RESTAMP=1: even sweep stamps first (restamp.py)
  python3 /tmp/restamp.py > $OUT/restamp.log 2>&1 &
  ros2 run apnvi_stage3 scan_filter --ros-args -r /scan_raw:=/scan_rs > $OUT/filter.log 2>&1 &
else
  ros2 run apnvi_stage3 scan_filter > $OUT/filter.log 2>&1 &
fi
ros2 run rtabmap_odom icp_odometry --ros-args --params-file $CFG -r scan:=/scan -r odom:=/odom \
  -r odom_info:=/odom_info > $OUT/odom.log 2>&1 &
python3 /tmp/track_rec.py $OUT/track.npz > $OUT/rec.txt 2>&1 &
REC=$!
sleep 4
for B in "$@"; do
  echo "$(date +%s.%N) $B" >> $OUT/marks.txt
  ros2 bag play $B --topics /scan_raw /tf_static > $OUT/play_$(basename $B).log 2>&1
done
wait $REC
for p in $(pgrep -f "icp_odometry|scan_filter|restamp.py"); do
  if tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"; then kill -INT $p; fi
done
cat $OUT/rec.txt | tail -3
