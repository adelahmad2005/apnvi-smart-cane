#!/usr/bin/env bash
# APNVI Jetson setup for Stage 1, Stage 2 and Stage 3 (map and route). Safe to run again: it only adds what is missing,
# downloads the newest code and builds it. Ubuntu 22.04 (the Jetson on JetPack 6.2).
#
# First time, paste these two lines:
#   git clone https://github.com/adelahmad2005/apnvi-smart-cane.git ~/apnvi_demo/src/apnvi-smart-cane
#   bash ~/apnvi_demo/src/apnvi-smart-cane/jetson_setup/jetson_setup.sh
# After that, to update and rebuild, only the last line is needed.

REPO_URL="https://github.com/adelahmad2005/apnvi-smart-cane.git"
WS="$HOME/apnvi_demo"                    # everything goes in this folder
REPO="$WS/src/apnvi-smart-cane"              # the downloaded repo
OLD="$WS/src/apnvi-stage1"               # the old code repo, from before 11 Oct 2026
say () { echo; echo "========== $* =========="; }
fail () { echo; echo "XXXXXXXXXX  PROBLEM: $*  XXXXXXXXXX"; echo "Read the problem above, fix it, then run this script again."; exit 1; }

# Everything is inside main() so the whole file is read before anything runs.
main () {
  say "APNVI SETUP STARTED - this can take 10 to 30 minutes. Do not close this window."

  # 1. Right Ubuntu?
  . /etc/os-release
  [ "$VERSION_ID" = "22.04" ] || fail "This needs Ubuntu 22.04 (JetPack 6.2). This computer has Ubuntu $VERSION_ID."

  # 2. Internet?
  wget -q --spider https://github.com || fail "No internet. Connect to wifi and run the line again."

  # 3. Password once, at the start
  echo "Type your password below and press Enter (nothing shows while you type, that is normal):"
  sudo -v || fail "Wrong password."

  # 4. ROS 2 Humble (skipped if already installed)
  if [ ! -f /opt/ros/humble/setup.bash ]; then
    say "Installing ROS 2 Humble (the longest part)"
    sudo apt-get update -y && sudo apt-get install -y software-properties-common curl locales \
      || fail "Could not install basic tools."
    sudo locale-gen en_US en_US.UTF-8 >/dev/null
    sudo add-apt-repository -y universe >/dev/null
    V=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F tag_name | awk -F\" '{print $4}')
    curl -fsL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.jammy_all.deb" \
      || fail "Could not download the ROS 2 source file."
    sudo dpkg -i /tmp/ros2-apt-source.deb >/dev/null || fail "Could not add the ROS 2 source."
    sudo apt-get update -y
    sudo apt-get install -y ros-humble-ros-base || fail "Could not install ROS 2 Humble."
  else
    say "ROS 2 Humble is already installed - good"
  fi

  # 5. Everything else the demo needs
  say "Installing the camera driver, sound and tools"
  sudo apt-get install -y ros-dev-tools ros-humble-realsense2-camera ros-humble-launch-ros \
    espeak-ng python3-serial pulseaudio-utils alsa-utils git python3-numpy python3-zstd \
    pulseaudio-module-bluetooth libasound2-plugins \
    || fail "Could not install the camera driver or tools."
  python3 -c "import cv2" 2>/dev/null || sudo apt-get install -y python3-opencv \
    || fail "Could not install OpenCV."
  sudo usermod -aG dialout "$USER"         # permission to read the ESP32's USB port

  # 5a. Stage 3: the map (RTAB-Map) and the route planner (Nav2), added 2026-10-29.
  # About 480 packages, 380 MB to download, 1.7 GB on disk.
  say "Installing RTAB-Map and Nav2 (Stage 3)"
  sudo apt-get install -y ros-humble-rtabmap-ros ros-humble-navigation2 ros-humble-nav2-bringup \
    || fail "Could not install RTAB-Map and Nav2."

  # 5a2. The LiDAR driver (SLAMTEC RPLIDAR), built in its own folder ~/lidar_ws. Skipped if built.
  LIDAR_WS="$HOME/lidar_ws"
  if [ ! -d "$LIDAR_WS/install/sllidar_ros2" ]; then
    say "Building the LiDAR driver in $LIDAR_WS"
    mkdir -p "$LIDAR_WS/src"
    [ -d "$LIDAR_WS/src/sllidar_ros2" ] || git clone https://github.com/Slamtec/sllidar_ros2.git \
      "$LIDAR_WS/src/sllidar_ros2" || fail "Could not download the LiDAR driver."
    git -C "$LIDAR_WS/src/sllidar_ros2" checkout 3430009 || fail "Could not pick the LiDAR driver version."
    ( source /opt/ros/humble/setup.bash && cd "$LIDAR_WS" && colcon build 2>&1 | tail -3 ) \
      || fail "Could not build the LiDAR driver."
    [ -d "$LIDAR_WS/install/sllidar_ros2" ] || fail "The LiDAR driver build did not work."
  else
    say "The LiDAR driver is already built - good"
  fi

  # 5b. Run the Jetson at full clock speed from every start-up (Adel, 2026-10-12).
  # With it, Qwen answers in about 2.3 s instead of about 6.3 s. jetson_clocks alone is
  # undone by a restart, so a small start-up service runs it each time.
  say "Setting the Jetson to full speed at every start-up"
  printf '%s\n' '[Unit]' 'Description=APNVI: run jetson_clocks at start-up' \
    'After=nvpmodel.service' '' '[Service]' 'Type=oneshot' 'ExecStart=/usr/bin/jetson_clocks' \
    '' '[Install]' 'WantedBy=multi-user.target' \
    | sudo tee /etc/systemd/system/apnvi-jetson-clocks.service >/dev/null
  sudo systemctl daemon-reload && sudo systemctl enable --now apnvi-jetson-clocks.service \
    || fail "Could not set the Jetson to full speed at start-up."

  # 6. Get the latest code
  say "Downloading the APNVI code"
  mkdir -p "$WS/src"
  # The old code repo holds the same packages. Two copies would stop the build, so move it
  # out of the workspace (kept in the home folder, not deleted).
  if [ -d "$OLD" ]; then
    mv "$OLD" "$HOME/old-apnvi-stage1-$(date +%Y%m%d%H%M)" || fail "Could not move the old code folder."
  fi
  if [ -d "$REPO/.git" ]; then
    git -C "$REPO" pull --ff-only || fail "Could not update the code."
  else
    git clone "$REPO_URL" "$REPO" || fail "Could not download the code. Check the internet connection."
  fi

  # 7. Build Stage 1 and Stage 2 (colcon finds both packages inside the repo's folders)
  say "Building"
  source /opt/ros/humble/setup.bash
  cd "$WS" && colcon build 2>&1 | tail -3
  [ -d "$WS/install/apnvi_stage1" ] || fail "The build did not work (Stage 1)."
  [ -d "$WS/install/apnvi_stage2" ] || fail "The build did not work (Stage 2)."
  [ -d "$WS/install/apnvi_stage3" ] || fail "The build did not work (Stage 3)."

  # 7b. The patched tf2 library (17 Nov): without it Nav2 locks up and guided trips never move.
  #     device/start.sh loads it. Skipped when it is already built.
  [ -d "$HOME/tf2_fix_ws" ] || bash "$REPO/jetson_setup/tf2_fix/build_tf2_fix.sh" \
    || fail "Could not build the patched tf2 library."

  # 8. Short command to start the Stage 1 demo: apnvi
  sed -i '/apnvi-stage1\/tools\/start_demo.sh/d' ~/.bashrc   # remove the old shortcut
  LINE="alias apnvi='bash $REPO/tools/stage1/start_demo.sh'"
  grep -qxF "$LINE" ~/.bashrc || echo "$LINE" >> ~/.bashrc

  say "SETUP FINISHED - ALL GOOD"
  echo "Now: 1) RESTART the Jetson once (needed for the USB permission)."
  echo "     2) Open a terminal and type:  apnvi   then press Enter."
}

main "$@"
