#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source /home/tranleduy/GPS-Degraded-Localization/install/setup.bash
sleep 5
ros2 topic hz /ekf/pose
exec bash
