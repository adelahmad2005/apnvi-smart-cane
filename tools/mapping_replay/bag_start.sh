source /opt/ros/humble/setup.bash
source ~/apnvi_ws/install/setup.bash
rtabmap --version 2>/dev/null | grep -iE "version|pointmatcher|libpointmatcher|g2o|gtsam" | head -6
mkdir -p ~/lap_bags
NAME=~/lap_bags/lap_$(date +%H%M%S)
( trap - INT; exec setsid nohup ros2 bag record -o $NAME /scan_raw /tf_static /odom /tf > /tmp/bag_record.out 2>&1 ) &
sleep 4
echo "recording to $NAME"
tail -3 /tmp/bag_record.out | cut -c1-160
