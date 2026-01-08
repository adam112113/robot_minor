#!/bin/bash
# Verify the correct TF tree structure

echo "=========================================="
echo "TF Tree Verification"
echo "=========================================="
echo ""
echo "Expected TF chain:"
echo "  map -> odom -> base_footprint -> base_link -> taitc_lidar_link"
echo ""
echo "=========================================="
echo ""

echo "Checking individual transforms..."
echo ""

echo "1. map -> odom (from SLAM Toolbox)"
timeout 2 ros2 run tf2_ros tf2_echo map odom 2>&1 | head -8
echo ""

echo "2. odom -> base_footprint (from odometry node)"
timeout 2 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | head -8
echo ""

echo "3. base_footprint -> base_link (static TF)"
timeout 2 ros2 run tf2_ros tf2_echo base_footprint base_link 2>&1 | head -8
echo ""

echo "4. base_link -> taitc_lidar_link (static TF)"
timeout 2 ros2 run tf2_ros tf2_echo base_link taitc_lidar_link 2>&1 | head -8
echo ""

echo "5. Full chain: map -> taitc_lidar_link"
timeout 2 ros2 run tf2_ros tf2_echo map taitc_lidar_link 2>&1 | head -8
echo ""

echo "=========================================="
echo "Generating TF tree visualization..."
timeout 6 ros2 run tf2_tools view_frames 2>&1
if [ -f frames.pdf ]; then
    echo "✓ TF tree saved to: $(pwd)/frames.pdf"
    echo "  Open with: evince frames.pdf"
else
    echo "✗ Failed to generate TF tree PDF"
fi
echo ""

echo "=========================================="
echo "Checking scan topic frame..."
timeout 1 ros2 topic echo /scan --once 2>&1 | grep "frame_id" | head -1
echo ""

echo "=========================================="
echo "Summary of nodes:"
ros2 node list 2>/dev/null | grep -E "odometry|slam|rplidar|static"
echo ""
echo "=========================================="
