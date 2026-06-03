#!/usr/bin/env bash
trap cleanup INT

cleanup() {
    echo ""
    echo "Cleaning up..."
    kill $PID_RVIZ 2>/dev/null || true
    killall -9 rviz2 2>/dev/null || true
    exit 0
}

# Go to workspace root
cd /home/tranleduy/GPS-Degraded-Localization

# Source ROS 2
source /opt/ros/jazzy/setup.bash

echo "Launching RViz2 with EKF config..."
rviz2 -d ev_localization.rviz --ros-args -p use_sim_time:=true > /tmp/rviz2_play.log 2>&1 &
PID_RVIZ=$!

sleep 2

echo "Playing recorded EKF output bag in loop mode..."
echo "Press Ctrl+C to stop."
ros2 bag play data/ekf_result --clock --loop
