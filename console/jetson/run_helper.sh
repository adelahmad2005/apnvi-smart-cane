#!/bin/bash
# Start the console helper with ROS ready. Used by apnvi-console.service;
# by hand: bash run_helper.sh   (Ctrl+C ends it; the device keeps running).
HERE="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/humble/setup.bash
source ~/apnvi_ws/install/setup.bash
exec python3 "$HERE/helper.py" "$@"
