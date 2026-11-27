#!/bin/bash
# on the Jetson: bash /tmp/runpair.sh cfgA [cfgB]   (two replays side by side, results in ~/drift_exp/)
sed -i 's/\r$//' /tmp/cfgs.py /tmp/track_rec.py /tmp/track_replay.sh
python3 /tmp/cfgs.py > /dev/null
mkdir -p ~/drift_exp; export BAG=${BAG:-$HOME/lap_bags/lap_134450}
bash /tmp/track_replay.sh /tmp/cfg_$1.yaml 61 > ~/drift_exp/run_$1.txt 2>&1 &
if [ -n "$2" ]; then bash /tmp/track_replay.sh /tmp/cfg_$2.yaml 62 > ~/drift_exp/run_$2.txt 2>&1 & fi
wait
for n in $1 $2; do cp /tmp/track_cfg_$n/track.npz ~/drift_exp/track_$n.npz; gzip -c /tmp/track_cfg_$n/odom.log > ~/drift_exp/odom_$n.log.gz; done
echo "done: $1 $2"
