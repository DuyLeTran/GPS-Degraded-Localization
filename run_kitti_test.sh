#!/usr/bin/env bash
set -e

# Go to workspace root
cd /home/tranleduy/GPS-Degraded-Localization

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Clean up any residual ROS 2 processes
echo "Cleaning up any old ROS 2 processes..."
killall -9 static_transform_publisher ekf_fusion gps_monitor monocular_vio landmark_ghost kitti_odom_converter yolo_detector lane_detector uturn_detector 2>/dev/null || true

# Remove old results bag if it exists
echo "Cleaning up old recording directories..."
rm -rf data/ekf_result

# 1. Start Static TF Publishers
echo "Launching Static TF Publishers..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom &
PID_TF1=$!
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 1.5 --roll -1.5708 --pitch 0 --yaw -1.5708 --frame-id base_link --child-frame-id camera_link &
PID_TF2=$!
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.5 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id gps_link &
PID_TF3=$!

# 2. Launch ev_localization (uses params.yaml with use_sim_time: true)
echo "Launching ev_localization system..."
ros2 launch ev_localization ev_localization.launch.py > /tmp/ev_localization_kitti.log 2>&1 &
PID_LAUNCH=$!

# 3. Launch helper nodes with use_sim_time:=true
echo "Launching kitti_odom_converter..."
ros2 run ev_localization kitti_odom_converter --ros-args --params-file install/ev_localization/share/ev_localization/config/params.yaml -p use_sim_time:=true > /tmp/kitti_odom_converter.log 2>&1 &
PID_ODOM=$!

echo "Launching yolo_detector..."
ros2 run ev_localization yolo_detector --ros-args -p use_sim_time:=true > /tmp/yolo_detector.log 2>&1 &
PID_YOLO=$!

# Publish simulated clock at 10Hz during startup to prevent clock jump
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
msg.clock = Time(sec=1317017304, nanosec=0)
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
ros2 bag record /ekf/pose /ekf/trajectory /gps/status /vehicle/lane_status /vehicle/u_turn_event -o data/ekf_result > /tmp/ros2_record.log 2>&1 &
PID_RECORD=$!

# Wait 2 seconds for record to start
sleep 2

# 5. Play KITTI bag (generates /clock)
echo "Playing KITTI bag..."
ros2 bag play data/kitti_dataset/kitti_ros2_bag \
  --clock --rate 1.0 \
  --remap /kitti/oxts/gps/fix:=/gps/fix \
          /kitti/camera_color_left/image_raw:=/camera/image_raw \
          /kitti/oxts/imu:=/imu/data

# Wait 2 seconds after play finishes to flush recording
echo "Bag play finished. Flushing recording..."
sleep 2

# 6. Clean up everything
echo "Cleaning up ROS 2 processes..."
kill $PID_RECORD 2>/dev/null || true

echo "Waiting 2 seconds for recorder to write metadata safely..."
sleep 2

kill $PID_ODOM 2>/dev/null || true
kill $PID_YOLO 2>/dev/null || true
kill $PID_LAUNCH 2>/dev/null || true
kill $PID_TF1 2>/dev/null || true
kill $PID_TF2 2>/dev/null || true
kill $PID_TF3 2>/dev/null || true
killall -9 static_transform_publisher ekf_fusion gps_monitor monocular_vio landmark_ghost kitti_odom_converter yolo_detector lane_detector uturn_detector 2>/dev/null || true

echo "=== EKF Result Bag Info ==="
ros2 bag info data/ekf_result
echo "==========================="

echo "=== YOLO Detector Log Sample ==="
tail -n 15 /tmp/yolo_detector.log
echo "================================"

echo "=== EKF Fusion Log Sample ==="
grep -E "ekf_fusion|Transitioned" /tmp/ev_localization_kitti.log | tail -n 15 || true
echo "============================="
