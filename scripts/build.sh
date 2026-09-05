#!/bin/bash

# Build script for ROS2 Autonomous Warehouse Robot

echo "Building ROS2 Autonomous Warehouse Robot..."

# Source ROS2
source /opt/ros/humble/setup.bash

# Build workspace
colcon build --symlink-install

echo "Build complete!"
echo "Source the workspace with: source install/setup.bash"
