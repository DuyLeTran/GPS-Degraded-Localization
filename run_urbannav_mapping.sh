#!/usr/bin/env bash
set -e

# Go to workspace root
cd /home/tranleduy/GPS-Degraded-Localization

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Source python virtual environment
source /home/tranleduy/GPS-Degraded-Localization/venv/bin/activate

# Build the package to register new nodes and launch files
echo "Building ev_localization package..."
colcon build --packages-select ev_localization

# Source local workspace setup
source install/setup.bash

# Clean up any residual ROS 2 processes
echo "Cleaning up any old ROS 2 processes..."
killall -9 static_transform_publisher ekf_fusion gps_monitor monocular_vio landmark_ghost urbannav_converter yolo_detector lane_detector uturn_detector landmark_builder 2>/dev/null || true

# Initialize landmarks file as empty list if it doesn't exist
# This avoids JSON loading errors on first run
mkdir -p ev_localization/config
if [ ! -f ev_localization/config/landmarks_urbannav.json ] || [ ! -s ev_localization/config/landmarks_urbannav.json ]; then
  echo "[]" > ev_localization/config/landmarks_urbannav.json
fi

# 1. Launch mapping system (with sim_gps_loss disabled to maintain perfect pose)
echo "Launching mapping nodes..."
ros2 launch ev_localization ev_localization_urbannav_mapping.launch.py > /tmp/urbannav_mapping.log 2>&1 &
PID_LAUNCH=$!

# 2. Publish simulated clock at 10Hz to prevent clock jump at startup
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

# 3. Play the UrbanNav bag for the test segment (from 110s, duration 160s)
echo "Playing UrbanNav Whampoa bag for mapping..."
ros2 bag play data/UrbanNav_dataset/whampoa_ros2_bag \
  --clock --rate 1.0 \
  --start-offset 110 \
  --playback-duration 160

# Wait 5 seconds to allow final landmarks to process and save
echo "Bag play finished. Waiting for final landmark processing..."
sleep 5

# 4. Clean up ROS 2 processes
echo "Cleaning up ROS 2 processes..."
kill $PID_LAUNCH 2>/dev/null || true
killall -9 static_transform_publisher ekf_fusion gps_monitor monocular_vio landmark_ghost urbannav_converter yolo_detector lane_detector uturn_detector landmark_builder 2>/dev/null || true

echo "=== Mapping Complete ==="
if [ -f ev_localization/config/landmarks_urbannav.json ]; then
  python3 -c "import json; data=json.load(open('ev_localization/config/landmarks_urbannav.json')); print(f'Total landmarks mapped: {len(data.get(\"landmarks\", data))}')"
fi
echo "========================"
