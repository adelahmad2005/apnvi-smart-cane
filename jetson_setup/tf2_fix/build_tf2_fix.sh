#!/bin/bash
# Build the patched tf2 library on the Jetson (17 Nov). No sudo needed. About 2 minutes.
#   bash build_tf2_fix.sh
#
# Why: tf2 0.25.23 (the version apt installed on the Jetson) can lock itself up inside any
# program that waits for frames with a message filter, which Nav2's obstacle maps do. The thread
# that receives frames holds tf2's request lock and asks for the buffer's timer lock; the thread
# that receives LiDAR sweeps holds the timer lock and asks for the request lock. Both then wait
# for ever: the program's frames stop at that moment. In the route follower this made every trip
# end at once ("Transform data too old when converting from map to odom", "Reached the goal!",
# 17 Nov). The tf2 authors fixed it later on their humble branch: the callbacks are
# called after the lock is released. The patch here is that one function and nothing else, on
# the exact version the Jetson runs, so every other program keeps working with it unchanged.
#
# What it makes: ~/tf2_fix_ws/install/tf2/lib/libtf2.so
# device/start.sh puts that folder first in LD_LIBRARY_PATH, so every program of the device
# loads the patched library instead of /opt/ros/humble/lib/libtf2.so, and checks that it did.
# To go back: move ~/tf2_fix_ws away (start.sh then warns and runs with the old library).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
WS=~/tf2_fix_ws
WANT=0.25.23
HAVE=$(dpkg-query -W -f='${Version}' ros-humble-tf2 2>/dev/null | cut -d- -f1)
if [ "$HAVE" != "$WANT" ]; then
  echo "The installed tf2 is '$HAVE', the patch is for $WANT. Not building."
  echo "A newer tf2 may already hold the fix: look for 'pending' in testTransformableRequests"
  echo "(tf2/src/buffer_core.cpp of that version) before changing this script."
  exit 1
fi
if [ -e "$WS" ]; then
  echo "$WS is already there. Move it away first if you want to build again."
  exit 1
fi
source /opt/ros/humble/setup.bash
mkdir -p "$WS/src"
git clone --quiet --depth 1 --branch "$WANT" https://github.com/ros2/geometry2.git "$WS/src/geometry2"
patch "$WS/src/geometry2/tf2/src/buffer_core.cpp" < "$HERE/tf2_callbacks_outside_lock.patch"
cd "$WS"
OVERRIDE=""
colcon build --help 2>/dev/null | grep -q -- --allow-overriding && OVERRIDE="--allow-overriding tf2"
colcon build --packages-select tf2 $OVERRIDE \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF 2>&1 | tail -4
ls -la "$WS/install/tf2/lib/libtf2.so"
echo "Built. The next start.sh uses it."
