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

# 3. Khởi chạy các tiến trình nền (YOLO và Bag Play)
echo "Launching yolo_detector in the background..."
ros2 run ev_localization yolo_detector --ros-args -p use_sim_time:=true --remap /camera/image_raw:=/zed2/camera/left/image_raw > /tmp/yolo_detector_urbannav.log 2>&1 &
PID_YOLO=$!

echo "Launching ros2 bag play in the background (with 7s delay)..."
(sleep 7 && ros2 bag play data/UrbanNav_dataset/whampoa_ros2_bag --clock --rate 1.0 --start-offset 110 > /tmp/bag_play_urbannav.log 2>&1) &
PID_BAG=$!

# 4. Khởi chạy 2 cửa sổ Windows Terminal riêng biệt (cho Launch và Hz)
echo "Launching 2 separate Windows Terminal windows..."
wt.exe -w new -p "Ubuntu-24.04" wsl.exe -d Ubuntu-24.04 -e bash -c "/home/tranleduy/GPS-Degraded-Localization/run_launch.sh"
sleep 1
wt.exe -w new -p "Ubuntu-24.04" wsl.exe -d Ubuntu-24.04 -e bash -c "/home/tranleduy/GPS-Degraded-Localization/run_hz.sh"

# Chờ tiến trình nền kết thúc
wait $PID_RVIZ
