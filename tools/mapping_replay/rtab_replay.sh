#!/bin/bash
# Replay the lap through filter + icp_odometry (today's settings) + rtabmap with one set of map
# settings, then report where the MAP thinks the lap ended (really START: 0 0 0).
#   bash rtab_replay.sh NAME DOMAIN MAP_PARAMS_YAML      (ODOM_CFG=/tmp/cfg_fix.yaml picks the tracker's settings)
NAME=$1; export ROS_DOMAIN_ID=$2; PARAMS=$3
source /opt/ros/humble/setup.bash
source ~/apnvi_ws/install/setup.bash
B=${BAG:-$(ls -td ~/lap_bags/lap_* | head -1)}
OUT=/tmp/rreplay_$NAME; rm -rf $OUT; mkdir -p $OUT
ros2 run apnvi_stage3 scan_filter > $OUT/filter.log 2>&1 &
ros2 run rtabmap_odom icp_odometry --ros-args --params-file ${ODOM_CFG:-/tmp/cfg_now.yaml} -r scan:=/scan \
  -r odom:=/odom > $OUT/odom.log 2>&1 &
ros2 run rtabmap_slam rtabmap --ros-args --params-file $PARAMS -p database_path:=$OUT/map.db \
  -r scan:=/scan -r odom:=/odom -r map:=/rtabmap/live_map > $OUT/rtab.log 2>&1 &
python3 /tmp/rtab_eval.py > $OUT/eval.txt 2>&1 &
EVAL=$!
sleep 6
ros2 bag play $B > $OUT/play.log 2>&1
wait $EVAL
for p in $(pgrep -f "icp_odometry|scan_filter|rtabmap_slam/rtabmap"); do
  if tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$2"; then kill -INT $p; fi
done
sleep 3
echo "$NAME: $(tail -1 $OUT/eval.txt) | links: $(python3 /tmp/db_links.py $OUT/map.db)"
