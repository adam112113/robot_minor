#!/bin/bash
# Clean start script for SLAM system
# Kills all ROS processes and restarts cleanly

echo "Stopping all ROS 2 processes..."
pkill -9 -f "ros2|rplidar|slam_toolbox|odometry|rviz|serial_driver|joy|teleop" 2>/dev/null
sleep 2

echo "Clearing DDS cache..."
rm -rf ~/.ros/log/* 2>/dev/null

echo "Sourcing workspace..."
cd /home/mimi/ros2_ws
source install/setup.bash

echo "Starting SLAM system..."
ros2 launch robot_slam robot_slam.launch.py
