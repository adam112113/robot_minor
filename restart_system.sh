#!/bin/bash
# Proper restart script for robot_slam system

echo "=========================================="
echo "Restarting ROS2 SLAM System"
echo "=========================================="
echo ""

echo "1. Killing all ROS2 processes..."
killall -9 rplidar_composition joy_node teleop_node odometry serial_driver async_slam_toolbox_node rviz2 2>/dev/null
sleep 1
echo "✓ Processes killed"
echo ""

echo "2. Resetting USB device..."
# Reset the USB device by toggling permissions (simulates reconnect)
if [ -e /dev/ttyUSB0 ]; then
    sudo chmod 666 /dev/ttyUSB0
    echo "✓ USB device ready"
else
    echo "⚠ Warning: /dev/ttyUSB0 not found!"
fi
echo ""

echo "3. Clearing parameter cache..."
rm -f /tmp/launch_params_* 2>/dev/null
echo "✓ Cache cleared"
echo ""

echo "4. Sourcing workspace..."
cd /home/mimi/ros2_ws
source install/setup.bash
echo "✓ Workspace sourced"
echo ""

echo "=========================================="
echo "Ready to launch!"
echo "Run: ros2 launch robot_slam robot_slam.launch.py"
echo "=========================================="
