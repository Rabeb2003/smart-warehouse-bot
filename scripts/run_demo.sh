#!/bin/bash

# Demo script for ROS2 Autonomous Warehouse Robot

echo "Starting ROS2 Autonomous Warehouse Robot Demo..."

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace
source install/setup.bash

# Set robot model
export TURTLEBOT3_MODEL=burger_cam

# Launch complete system
echo "Launching Gazebo simulation, Nav2, and perception..."
ros2 launch warehouse_bringup warehouse_robot_system_camera.launch.py
