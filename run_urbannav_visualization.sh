#!/usr/bin/env bash
# Exit on Ctrl+C (SIGINT)
trap cleanup INT

cleanup() {
    echo ""
    echo "Cleaning up ROS 2 processes..."
    kill $PID_RVIZ 2>/dev/null || true
    kill $PID_RQT 2>/dev/null || true
    killall -9 static_transform_publisher ekf_fusion gps_monitor monocular_vio landmark_ghost urbannav_converter yolo_detector lane_detector uturn_detector rviz2 rqt_image_view 2>/dev/null || true
    echo "Done. Exiting."
    exit 0
}

# Go to workspace root
cd /home/tranleduy/GPS-Degraded-Localization

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Clean up any residual ROS 2 processes
echo "Cleaning up any old ROS 2 processes..."
killall -9 static_transform_publisher ekf_fusion gps_monitor monocular_vio landmark_ghost urbannav_converter yolo_detector lane_detector uturn_detector rviz2 rqt_image_view ros2 2>/dev/null || true

# 1. Khởi chạy RViz2 ở nửa bên trái màn hình
echo "Launching RViz2..."
rviz2 -geometry 960x1080+0+0 -d ev_localization.rviz --ros-args -p use_sim_time:=true > /tmp/rviz2_urbannav.log 2>&1 &
PID_RVIZ=$!

# 2. Khởi chạy rqt_image_view ở góc trên bên phải màn hình
echo "Launching rqt_image_view..."
rqt_image_view -geometry 960x500+960+0 /lane/debug_image --ros-args -p use_sim_time:=true > /tmp/rqt_image_view_urbannav.log 2>&1 &
PID_RQT=$!

# 3. Khởi chạy Windows Terminal ở góc dưới bên phải màn hình (chia làm 4 ô)
echo "Launching Windows Terminal Dashboard..."
wt.exe --pos 960,520 --size 115,25 \
  new-tab -p "Ubuntu-24.04" -d "\\\\wsl.localhost\\Ubuntu-24.04\\home\\tranleduy\\GPS-Degraded-Localization" wsl.exe -d Ubuntu-24.04 -e bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch ev_localization ev_localization_urbannav.launch.py; exec bash" \; \
  split-pane -V -p "Ubuntu-24.04" -d "\\\\wsl.localhost\\Ubuntu-24.04\\home\\tranleduy\\GPS-Degraded-Localization" wsl.exe -d Ubuntu-24.04 -e bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run ev_localization yolo_detector --ros-args -p use_sim_time:=true --remap /camera/image_raw:=/zed2/camera/left/image_raw; exec bash" \; \
  split-pane -H -p "Ubuntu-24.04" -d "\\\\wsl.localhost\\Ubuntu-24.04\\home\\tranleduy\\GPS-Degraded-Localization" wsl.exe -d Ubuntu-24.04 -e bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && sleep 5 && ros2 topic hz /ekf/pose; exec bash" \; \
  focus-pane -t 0 \; \
  split-pane -H -p "Ubuntu-24.04" -d "\\\\wsl.localhost\\Ubuntu-24.04\\home\\tranleduy\\GPS-Degraded-Localization" wsl.exe -d Ubuntu-24.04 -e bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && sleep 7 && ros2 bag play data/UrbanNav_dataset/whampoa_ros2_bag --clock --rate 1.0 --start-offset 535; exec bash"

# Chờ tiến trình nền kết thúc
wait $PID_RVIZ
