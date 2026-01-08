#!/bin/bash
# System verification script after launch

echo "=========================================="
echo "ROS2 SLAM System Verification"
echo "=========================================="
echo ""

echo "1. Checking if nodes are running..."
ros2 node list 2>/dev/null | grep -E "rplidar|slam|odometry" && echo "✓ Core nodes running" || echo "✗ Nodes not found"
echo ""

echo "2. Checking /scan topic..."
if ros2 topic list 2>/dev/null | grep -q "^/scan$"; then
    echo "✓ /scan topic exists"
    timeout 3 ros2 topic hz /scan 2>&1 | head -5 &
    SCAN_PID=$!
    sleep 2
    kill $SCAN_PID 2>/dev/null
    wait $SCAN_PID 2>/dev/null
else
    echo "✗ /scan topic not found"
fi
echo ""

echo "3. Checking /map topic..."
if ros2 topic list 2>/dev/null | grep -q "^/map$"; then
    echo "✓ /map topic exists"
    echo "Checking if map is being published..."
    timeout 2 ros2 topic hz /map 2>&1 | head -3
else
    echo "✗ /map topic not found"
fi
echo ""

echo "4. Checking TF transforms..."
ros2 run tf2_ros tf2_echo base_link laser 2>&1 | head -8 &
TF_PID=$!
sleep 2
kill $TF_PID 2>/dev/null
wait $TF_PID 2>/dev/null
echo ""

echo "=========================================="
echo "5. Quick scan data test..."
timeout 1 ros2 topic echo /scan --once 2>&1 | head -15
echo ""

echo "=========================================="
echo "To visualize in RViz, run:"
echo "  ros2 run rviz2 rviz2"
echo ""
echo "Then in RViz:"
echo "  - Set Fixed Frame to: map"
echo "  - Add > LaserScan > Topic: /scan"
echo "  - Add > Map > Topic: /map"
echo "=========================================="
