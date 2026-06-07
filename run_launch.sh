#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source /home/tranleduy/GPS-Degraded-Localization/install/setup.bash
ros2 launch ev_localization ev_localization_urbannav.launch.py
exec bash
