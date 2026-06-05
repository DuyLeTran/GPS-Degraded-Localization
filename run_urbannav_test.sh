#!/usr/bin/env bash
set -e

# Go to workspace root
cd /home/tranleduy/GPS-Degraded-Localization

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Clean up any residual ROS 2 processes
echo "Cleaning up any old ROS 2 processes..."
killall -9 static_transform_publisher ekf_fusion gps_monitor monocular_vio landmark_ghost urbannav_converter yolo_detector lane_detector uturn_detector 2>/dev/null || true

# Remove old results bag if it exists
echo "Cleaning up old recording directories..."
rm -rf data/urbannav_ekf_result

# 1. Launch ev_localization with UrbanNav configuration
echo "Launching ev_localization system for UrbanNav Whampoa..."
ros2 launch ev_localization ev_localization_urbannav.launch.py > /tmp/ev_localization_urbannav.log 2>&1 &
PID_LAUNCH=$!

# 2. Launch yolo_detector with remapping
echo "Launching yolo_detector..."
ros2 run ev_localization yolo_detector --ros-args -p use_sim_time:=true --remap /camera/image_raw:=/zed2/camera/left/image_raw > /tmp/yolo_detector_urbannav.log 2>&1 &
PID_YOLO=$!

# 3. Publish simulated clock at 10Hz during startup to prevent clock jump
# Bag starts at UTC time 1621578524
echo "Publishing initial simulated clock to initialize node times..."
python3 -c "
import rclpy
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
import time
rclpy.init()
node = rclpy.create_node('start_clock')
pub = node.create_publisher(Clock, '/clock', 10)
msg = Clock()
msg.clock = Time(sec=1621578634, nanosec=0)
for _ in range(40):
    pub.publish(msg)
    time.sleep(0.1)
node.destroy_node()
rclpy.shutdown()
" &
PID_START_CLOCK=$!

# Wait for nodes to initialize
echo "Waiting 5 seconds for system initialization..."
sleep 5

# 4. Start recording EKF results
echo "Starting ros2 bag record..."
ros2 bag record /ekf/pose /ekf/trajectory /gps/status /vehicle/lane_status /vehicle/u_turn_event /gps/fix /vehicle/odom -o data/urbannav_ekf_result > /tmp/ros2_record_urbannav.log 2>&1 &
PID_RECORD=$!

# Wait 2 seconds for record to start
sleep 2

# 5. Play UrbanNav bag (generates /clock)
echo "Playing UrbanNav Whampoa bag for 160 seconds starting from 110s..."
ros2 bag play data/UrbanNav_dataset/whampoa_ros2_bag \
  --clock --rate 1.0 \
  --start-offset 110 \
  --playback-duration 160

# Wait 2 seconds after play finishes to flush recording
echo "Bag play finished. Flushing recording..."
sleep 2

# 6. Clean up everything
echo "Cleaning up ROS 2 processes..."
kill $PID_RECORD 2>/dev/null || true

echo "Waiting 2 seconds for recorder to write metadata safely..."
sleep 2

kill $PID_YOLO 2>/dev/null || true
kill $PID_LAUNCH 2>/dev/null || true
killall -9 static_transform_publisher ekf_fusion gps_monitor monocular_vio landmark_ghost urbannav_converter yolo_detector lane_detector uturn_detector 2>/dev/null || true

echo "=== EKF Result Bag Info ==="
ros2 bag info data/urbannav_ekf_result
echo "==========================="

echo "=== EKF Fusion Log Sample ==="
grep -E "ekf_fusion|Transitioned" /tmp/ev_localization_urbannav.log | tail -n 25 || true
echo "============================="
