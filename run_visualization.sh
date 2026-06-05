#!/usr/bin/env bash
# Exit on Ctrl+C (SIGINT)
trap cleanup INT

cleanup() {
    echo ""
    echo "Cleaning up ROS 2 processes..."
    kill $PID_RVIZ 2>/dev/null || true
    kill $PID_ODOM 2>/dev/null || true
    kill $PID_YOLO 2>/dev/null || true
    kill $PID_LAUNCH 2>/dev/null || true
    kill $PID_TF1 2>/dev/null || true
    kill $PID_TF2 2>/dev/null || true
    kill $PID_TF3 2>/dev/null || true
    killall -9 static_transform_publisher ekf_fusion gps_monitor monocular_vio landmark_ghost kitti_odom_converter yolo_detector lane_detector uturn_detector rviz2 2>/dev/null || true
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
killall -9 static_transform_publisher ekf_fusion gps_monitor monocular_vio landmark_ghost kitti_odom_converter yolo_detector lane_detector uturn_detector rviz2 ros2 2>/dev/null || true

# 1. Start Static TF Publishers
echo "Launching Static TF Publishers..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom --ros-args -p use_sim_time:=true &
PID_TF1=$!
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 1.5 --roll -1.5708 --pitch 0 --yaw -1.5708 --frame-id base_link --child-frame-id camera_link --ros-args -p use_sim_time:=true &
PID_TF2=$!
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.5 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id gps_link --ros-args -p use_sim_time:=true &
PID_TF3=$!

# 2. Launch ev_localization
echo "Launching ev_localization system..."
ros2 launch ev_localization ev_localization.launch.py > /tmp/ev_localization_viz.log 2>&1 &
PID_LAUNCH=$!

# 3. Launch helper nodes with use_sim_time:=true
echo "Launching kitti_odom_converter..."
ros2 run ev_localization kitti_odom_converter --ros-args --params-file install/ev_localization/share/ev_localization/config/params.yaml -p use_sim_time:=true > /tmp/kitti_odom_converter_viz.log 2>&1 &
PID_ODOM=$!

echo "Launching yolo_detector..."
ros2 run ev_localization yolo_detector --ros-args -p use_sim_time:=true > /tmp/yolo_detector_viz.log 2>&1 &
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

# 4. Launch RViz2 with pre-configured ev_localization.rviz layout
echo "Launching RViz2..."
rviz2 -d ev_localization.rviz --ros-args -p use_sim_time:=true > /tmp/rviz2.log 2>&1 &
PID_RVIZ=$!

# Wait 2 seconds for RViz2 to open
sleep 2

# 5. Play KITTI bag (generates /clock)
echo "Playing KITTI bag..."
echo "Press Ctrl+C to stop the visualization and cleanup."
ros2 bag play data/kitti_dataset/kitti_ros2_bag \
  --clock --rate 1.0 \
  --remap /kitti/oxts/gps/fix:=/gps/fix \
          /kitti/camera_color_left/image_raw:=/camera/image_raw \
          /kitti/oxts/imu:=/imu/data
