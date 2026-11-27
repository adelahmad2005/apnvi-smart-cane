#!/bin/bash
# on the Jetson: bash /tmp/queue.sh "a,b" "c,d" ...   runs each pair one after the other
for pair in "$@"; do
  a=${pair%%,*}; b=${pair#*,}; [ "$b" = "$a" ] && b=""
  bash /tmp/runpair.sh $a $b
done
echo "queue finished"
