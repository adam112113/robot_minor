#!/bin/bash
# Diagnostic script to check ROS topic publishing rates

echo "=== ROS 2 Topic Rate Diagnostics ==="
echo ""
echo "Checking publishing rates (press Ctrl+C after a few seconds for each)..."
echo ""

source /home/mimi/ros2_ws/install/setup.bash

echo "1. Checking /fb_speed rate (Arduino feedback → odometry):"
echo "   Expected: 50 Hz if Arduino sends fast enough"
timeout 5 ros2 topic hz /fb_speed 2>/dev/null || echo "   Topic not publishing or no data"
echo ""

echo "2. Checking /odom rate (odometry publishing):"
echo "   Expected: 50 Hz"
timeout 5 ros2 topic hz /odom 2>/dev/null || echo "   Topic not publishing"
echo ""

echo "3. Checking /scan rate (RPLidar):"
echo "   Expected: ~5-10 Hz"
timeout 5 ros2 topic hz /scan 2>/dev/null || echo "   Topic not publishing"
echo ""

echo "4. Checking TF transform rate (odom → base_link):"
echo "   Expected: 50 Hz"
timeout 5 ros2 topic hz /tf 2>/dev/null || echo "   Topic not publishing"
echo ""

echo "=== Latest /fb_speed message ==="
timeout 2 ros2 topic echo /fb_speed --once 2>/dev/null
echo ""

echo "=== Diagnostics Complete ===" 
echo ""
echo "If /fb_speed rate is low (<20 Hz), the Arduino is the bottleneck!"
echo "Check your Arduino code for:"
echo "  - Serial.print() rate in loop()"
echo "  - PID controller update frequency"
echo "  - Encoder reading frequency"
