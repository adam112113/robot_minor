#!/bin/bash
# Launch Navigation with Nav2 using a saved map

echo "========================================="
echo "  Starting Nav2 Navigation"
echo "========================================="
echo ""

cd /home/mimi/ros2_ws
source install/setup.bash

# Check if map exists
MAP_FILE="/home/mimi/ros2_ws/src/robot_slam/maps/my_map.yaml"
if [ ! -f "$MAP_FILE" ]; then
    echo "ERROR: Map file not found at $MAP_FILE"
    echo ""
    echo "You need to create a map first!"
    echo "Run: ./save_map.sh (while robot_slam.launch.py is running)"
    echo ""
    exit 1
fi

echo "Using map: $MAP_FILE"
echo ""
echo "Starting Nav2 navigation..."
echo ""
echo "In RViz:"
echo "  1. Set '2D Pose Estimate' to initialize robot position"
echo "  2. Use 'Nav2 Goal' or click 'Publish Point' to navigate"
echo ""

ros2 launch robot_slam navigation.launch.py
