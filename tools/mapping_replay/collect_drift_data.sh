#!/bin/bash
# Run ON THE JETSON (16 Nov). Collects what the cloud fixing chat needs for the map drift (todo
# 137) into ~/drift_export/: the two recordings of 15 Nov as they are, and seven replays of the
# lap through the movement tracker (today's settings and six one-change variants), each with the
# tracker's own log and its whole path. Takes about 25 minutes, two replays at a time.
# Stop the device software first (bash stop.sh). Nothing here touches the wheels.
#   bash collect_drift_data.sh
T=$(dirname "$(readlink -f "$0")")
OUT=~/drift_export; rm -rf $OUT; mkdir -p $OUT
export BAG=~/lap_bags/lap_134450
for f in "$T"/*.py "$T"/*.sh; do cp "$f" /tmp/; sed -i 's/\r$//' "/tmp/$(basename "$f")"; done
python3 /tmp/cfgs.py

source /opt/ros/humble/setup.bash
source ~/apnvi_ws/install/setup.bash
{ rtabmap --version 2>&1 | head -40; dpkg -l 2>/dev/null | grep -iE "rtabmap|pointmatcher|sllidar|laser-geometry";
  uname -a; uptime; } > $OUT/versions.txt 2>&1
for b in lap_134450 push_ABA_15nov; do
  [ -d ~/lap_bags/$b ] && tar czf $OUT/$b.tgz -C ~/lap_bags $b
done

run_pair() {            # two replays side by side, each in its own domain
  bash /tmp/track_replay.sh /tmp/cfg_$1.yaml 61 > $OUT/run_$1.txt 2>&1 &
  if [ -n "$2" ]; then bash /tmp/track_replay.sh /tmp/cfg_$2.yaml 62 > $OUT/run_$2.txt 2>&1 & fi
  wait
  for n in $1 $2; do
    cp /tmp/track_cfg_$n/track.npz $OUT/track_$n.npz 2>/dev/null
    gzip -c /tmp/track_cfg_$n/odom.log > $OUT/odom_$n.log.gz 2>/dev/null
  done
  echo "done: $1 $2"
}
run_pair now big20k
run_pair mcd06 p2l
run_pair vox0 f2f
run_pair noguess
ls -la $OUT
du -sh $OUT
echo "Now copy ~/drift_export to the campus PC and push it (see drift-data-request-16nov.md)."
