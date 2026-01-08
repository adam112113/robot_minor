#!/bin/bash
# All-in-one launch script with proper cleanup

echo "=========================================="
echo "Robot SLAM System - Clean Launch"
echo "=========================================="

# Kill old processes
echo "Cleaning up old processes..."
killall -9 rplidar_composition joy_node teleop_node odometry serial_driver async_slam_toolbox_node rviz2 2>/dev/null
sleep 1

# Clear cache
rm -f /tmp/launch_params_* 2>/dev/null

# Set USB permissions
if [ -e /dev/ttyUSB0 ]; then
    sudo chmod 777 /dev/ttyUSB0
fi

# Source and launch
cd /home/mimi/ros2_ws
source install/setup.bash

echo ""
echo "Launching system..."
echo "NOTE: RPlidar may show errors for ~10 seconds while initializing."
echo "This is normal - it will auto-retry and succeed."
echo "=========================================="
echo ""

ros2 launch robot_slam robot_slam.launch.py
