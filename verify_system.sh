#!/bin/bash
# Complete system timing verification for SLAM v2.0
# This checks ALL components are synchronized at 50 Hz

echo "========================================="
echo "  SLAM SYSTEM TIMING VERIFICATION v2.0"
echo "========================================="
echo ""

source /home/mimi/ros2_ws/install/setup.bash

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "Expected Timing Configuration:"
echo "  Arduino Serial Output:  50 Hz (20ms interval)"
echo "  Pi Serial Reader:       50 Hz (20ms interval)"
echo "  Odometry Publisher:     50 Hz (20ms interval)"
echo "  TF Transform:           50 Hz (20ms interval)"
echo "  SLAM TF Publish:        50 Hz (20ms interval)"
echo ""
echo "========================================="
echo ""

# Function to check rate
check_rate() {
    local topic=$1
    local expected=$2
    local tolerance=$3
    
    echo -n "Checking $topic rate... "
    
    # Get rate (run for 5 seconds)
    local output=$(timeout 5 ros2 topic hz $topic 2>/dev/null | grep "average rate" | awk '{print $3}')
    
    if [ -z "$output" ]; then
        echo -e "${RED}FAILED - No data${NC}"
        return 1
    fi
    
    # Extract just the number
    local rate=$(echo $output | cut -d'.' -f1)
    
    # Check if within tolerance
    local min=$((expected - tolerance))
    local max=$((expected + tolerance))
    
    if [ "$rate" -ge "$min" ] && [ "$rate" -le "$max" ]; then
        echo -e "${GREEN}OK - ${rate} Hz${NC}"
        return 0
    else
        echo -e "${RED}FAILED - ${rate} Hz (expected ${expected} ±${tolerance} Hz)${NC}"
        return 1
    fi
}

echo "1. Checking /fb_speed (Arduino → Pi Serial)"
check_rate "/fb_speed" 50 5
echo ""

echo "2. Checking /odom (Odometry Publishing)"
check_rate "/odom" 50 5
echo ""

echo "3. Checking /tf (Transform Broadcasting)"
check_rate "/tf" 50 10
echo ""

echo "4. Checking /scan (RPLidar)"
check_rate "/scan" 8 3
echo ""

echo "========================================="
echo ""
echo "Checking TF Tree Structure..."
echo ""

# Check if map frame exists
if timeout 2 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -q "Translation"; then
    echo -e "${GREEN}✓ map → odom transform EXISTS${NC}"
else
    echo -e "${RED}✗ map → odom transform MISSING (SLAM not publishing TF!)${NC}"
fi

if timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | grep -q "Translation"; then
    echo -e "${GREEN}✓ odom → base_link transform EXISTS${NC}"
else
    echo -e "${RED}✗ odom → base_link transform MISSING${NC}"
fi

if timeout 2 ros2 run tf2_ros tf2_echo base_link laser 2>/dev/null | grep -q "Translation"; then
    echo -e "${GREEN}✓ base_link → laser transform EXISTS${NC}"
else
    echo -e "${RED}✗ base_link → laser transform MISSING${NC}"
fi

echo ""
echo "========================================="
echo ""
echo "Latest /fb_speed message (Arduino feedback):"
timeout 2 ros2 topic echo /fb_speed --once 2>/dev/null || echo "No message received"
echo ""

echo "========================================="
echo " SUMMARY"
echo "========================================="
echo ""
echo "All components should be at 50 Hz for optimal SLAM performance."
echo ""
echo "If any component shows:"
echo "  • < 20 Hz → Check that component (Arduino/Pi code)"
echo "  • > 100 Hz → Wasting CPU, reduce rate"
echo "  • No data → Check if node is running"
echo ""
echo "TF tree must be: map → odom → base_link → laser"
echo "If 'map' frame missing, check slam_toolbox publish_tf setting"
echo ""

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
