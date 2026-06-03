#!/usr/bin/env bash
set -e

export PATH="/home/tranleduy/GPS-Degraded-Localization/venv/bin:$PATH"

# Go to workspace root
cd /home/tranleduy/GPS-Degraded-Localization

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# 1. Backup original params.yaml
echo "Backing up params.yaml..."
cp ev_localization/config/params.yaml /tmp/params.yaml.bak

# 2. Update params.yaml coordinates to 21.0 and 105.0, and disable use_sim_time for the smoke test
echo "Updating params.yaml coordinates and use_sim_time for smoke test..."
sed -i 's/gps_lat0: 49.011758/gps_lat0: 21.0/' ev_localization/config/params.yaml
sed -i 's/gps_lon0: 8.422249/gps_lon0: 105.0/' ev_localization/config/params.yaml
sed -i 's/use_sim_time: true/use_sim_time: false/g' ev_localization/config/params.yaml

# 3. Build package to apply smoke test parameters
echo "Building workspace for smoke test..."
colcon build --packages-select ev_localization --symlink-install
source install/setup.bash

# Clean up any residual ROS 2 processes
echo "Cleaning up any old ROS 2 processes..."
killall -9 static_transform_publisher ekf_fusion gps_monitor monocular_vio landmark_ghost 2>/dev/null || true

# 4. Start Static TF Publishers
echo "Launching Static TF Publishers..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom &
PID_TF1=$!
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 1.5 --roll -1.5708 --pitch 0 --yaw -1.5708 --frame-id base_link --child-frame-id camera_link &
PID_TF2=$!
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.5 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id gps_link &
PID_TF3=$!

# 5. Launch ev_localization
echo "Launching ev_localization system..."
ros2 launch ev_localization ev_localization.launch.py > /tmp/ev_localization.log 2>&1 &
PID_LAUNCH=$!

# Wait for system to initialize
echo "Waiting 5 seconds for system to initialize..."
sleep 5

# 6. Run Smoke Test client and capture its output
echo "Running smoke_test.py..."
rm -rf data/smoke_ekf_result
ros2 bag record /ekf/pose /ekf/trajectory /gps/status -o data/smoke_ekf_result > /tmp/ros2_record_smoke.log 2>&1 &
PID_RECORD_SMOKE=$!
python3 ev_localization/test/smoke_test.py > /tmp/smoke_test_run.log 2>&1 &
PID_SMOKE=$!

# Wait for smoke test to complete (it runs for ~30 seconds)
echo "Waiting 35 seconds for smoke test to complete..."
sleep 35

# 7. Clean up background tasks
echo "Cleaning up ROS 2 processes..."
kill $PID_RECORD_SMOKE 2>/dev/null || true
sleep 2
kill $PID_SMOKE 2>/dev/null || true
kill $PID_LAUNCH 2>/dev/null || true
kill $PID_TF1 2>/dev/null || true
kill $PID_TF2 2>/dev/null || true
kill $PID_TF3 2>/dev/null || true
killall -9 static_transform_publisher ekf_fusion gps_monitor monocular_vio landmark_ghost 2>/dev/null || true

# 8. Restore original params.yaml
echo "Restoring original params.yaml..."
mv /tmp/params.yaml.bak ev_localization/config/params.yaml

# 9. Rebuild workspace to restore KITTI parameters
echo "Rebuilding workspace to restore KITTI parameters..."
colcon build --packages-select ev_localization --symlink-install

echo "=== SMOKE TEST RUN LOG ==="
cat /tmp/smoke_test_run.log
echo "=========================="

echo "=== ROS 2 LAUNCH LOG (Last 20 lines) ==="
tail -n 20 /tmp/ev_localization.log
echo "========================================"
