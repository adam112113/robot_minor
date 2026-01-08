#!/bin/bash
# TF Tree Diagnostic Script

echo "=========================================="
echo "TF Tree Diagnostic"
echo "=========================================="
echo ""

echo "Expected TF chain: map -> odom -> base_link -> laser"
echo ""

echo "1. Checking /tf topic..."
timeout 2 ros2 topic hz /tf 2>&1 | head -3
echo ""

echo "2. Checking available frames..."
timeout 3 ros2 run tf2_ros tf2_monitor 2>&1 | head -20 &
MON_PID=$!
sleep 2
kill $MON_PID 2>/dev/null
wait $MON_PID 2>/dev/null
echo ""

echo "3. Checking specific transforms..."
echo "--- map -> odom (from SLAM Toolbox) ---"
timeout 2 ros2 run tf2_ros tf2_echo map odom 2>&1 | head -6
echo ""

echo "--- odom -> base_link (from odometry node) ---"
timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -6
echo ""

echo "--- base_link -> laser (static TF) ---"
timeout 2 ros2 run tf2_ros tf2_echo base_link laser 2>&1 | head -6
echo ""

echo "--- map -> base_link (full chain) ---"
timeout 2 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -6
echo ""

echo "=========================================="
echo "4. Generating TF tree PDF..."
timeout 6 ros2 run tf2_tools view_frames 2>&1
if [ -f frames.pdf ]; then
    echo "✓ TF tree saved to: $(pwd)/frames.pdf"
    echo "  View with: evince frames.pdf"
else
    echo "✗ Failed to generate TF tree"
fi
echo ""

echo "=========================================="
echo "5. Checking SLAM Toolbox parameters..."
ros2 param get /slam_toolbox map_frame 2>&1 | head -1
ros2 param get /slam_toolbox odom_frame 2>&1 | head -1
ros2 param get /slam_toolbox base_frame 2>&1 | head -1
ros2 param get /slam_toolbox publish_tf 2>&1 | head -1
echo ""

echo "=========================================="
echo "Diagnostic complete!"
echo "=========================================="
