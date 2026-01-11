#!/bin/bash
# Launch Nav2 navigation with custom launch (no route_server dependency)

echo "==========================================="
echo "  Starting Nav2 Navigation (Custom)"
echo "==========================================="
echo ""
echo "This uses a custom Nav2 launch that bypasses"
echo "the route_server dependency issue."
echo ""
echo "Prerequisites:"
echo "  1. Map saved in install/robot_slam/share/robot_slam/maps/"
echo "  2. Robot powered on with Arduino connected"
echo "  3. RPLidar connected to /dev/ttyUSB0"
echo ""
echo "In RViz (on host machine):"
echo "  1. Run: ./host_scripts/launch_nav_rviz.sh"
echo "  2. Set 2D Pose Estimate to initialize AMCL"
echo "  3. Use Nav2 Goal to navigate"
echo ""

# Source ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch navigation
ros2 launch robot_slam navigation_custom.launch.py
