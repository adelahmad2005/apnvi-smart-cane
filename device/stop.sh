#!/bin/bash
# Stop the whole device: Ctrl+C to every launch, then clear anything left over.
#   bash stop.sh          (bash stop.sh quiet: print nothing)
PAT='apnvi_stage|rtabmap|controller_server|planner_server|realsense2_camera|sllidar|lifecycle_manager|icp_odometry|static_transform'
pkill -INT -f "ros2 launch" 2>/dev/null
for i in 1 2 3 4 5 6 7 8; do sleep 1; [ "$(pgrep -fc "$PAT")" = "0" ] && break; done
LEFT=$(pgrep -fc "$PAT")
for p in apnvi_stage rtabmap controller_server planner_server realsense2_camera sllidar \
         lifecycle_manager icp_odometry static_transform "ros2 launch"; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 1
[ "$1" = "quiet" ] || echo "stopped ($LEFT needed a hard stop); now running: $(pgrep -fc "$PAT")"
