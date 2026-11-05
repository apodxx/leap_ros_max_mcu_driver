#!/bin/bash
# Start Orbbec Astra Pro (depth + RGB + point cloud) on the Jetson.
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
exec ros2 launch astra_camera astra_pro.launch.py "$@"
