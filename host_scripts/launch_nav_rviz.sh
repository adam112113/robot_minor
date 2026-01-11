#!/bin/bash
# Run RViz for Nav2 on HOST machine
# This connects to ROS2 nodes running on the Pi5

# ROS2 Network Configuration
# Change this to match your Pi5's IP address
export ROS_DOMAIN_ID=0
export PI5_IP="192.168.1.100"  # ⚠️ CHANGE THIS to your Pi5's IP!

echo "========================================="
echo "  RViz for Nav2 - Host Machine"
echo "========================================="
echo ""
echo "Connecting to Pi5 at: $PI5_IP"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo ""
echo "Make sure:"
echo "  1. Pi5 is running: ./launch_navigation.sh"
echo "  2. Both machines on same network"
echo "  3. ROS_DOMAIN_ID matches on both machines"
echo ""
echo "In RViz:"
echo "  1. Set '2D Pose Estimate' to initialize robot"
echo "  2. Use 'Nav2 Goal' to navigate"
echo ""

# Check if Nav2 RViz plugins are installed
if ! dpkg -l | grep -q "ros-jazzy-navigation2"; then
    echo "⚠️  WARNING: Nav2 RViz plugins not found!"
    echo ""
    echo "Installing required packages..."
    echo "This is needed for Nav2 Goal Tool and panels."
    echo ""
    sudo apt update
    sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup
    echo ""
    echo "✅ Nav2 packages installed!"
    echo ""
fi

# Path to RViz config (adjust if your workspace is elsewhere)
RVIZ_CONFIG="$(dirname "$0")/../src/robot_slam/config/nav2_rviz.rviz"

if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "ERROR: RViz config not found at: $RVIZ_CONFIG"
    echo ""
    echo "Using default RViz config instead..."
    ros2 run rviz2 rviz2
else
    echo "Starting RViz with Nav2 configuration..."
    echo ""
    ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG"
fi
