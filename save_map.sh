#!/bin/bash
# Save the current SLAM map for Nav2 navigation

echo "========================================="
echo "  Saving SLAM Map"
echo "========================================="
echo ""

cd /home/mimi/ros2_ws
source install/setup.bash

MAP_DIR="/home/mimi/ros2_ws/src/robot_slam/maps"
MAP_NAME="my_map"

echo "Saving map to: ${MAP_DIR}/${MAP_NAME}"
echo ""

# Save the map using map_saver from nav2_map_server
ros2 run nav2_map_server map_saver_cli -f ${MAP_DIR}/${MAP_NAME}

echo ""
echo "========================================="
echo "Map saved successfully!"
echo ""
echo "Files created:"
echo "  - ${MAP_DIR}/${MAP_NAME}.pgm  (image)"
echo "  - ${MAP_DIR}/${MAP_NAME}.yaml (metadata)"
echo ""
echo "You can now use this map for Nav2 navigation!"
echo "========================================="
