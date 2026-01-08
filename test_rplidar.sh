#!/bin/bash
# RPlidar A1 Diagnostic Script

echo "======================================"
echo "RPlidar A1 Hardware Diagnostic"
echo "======================================"
echo ""

echo "1. Checking USB device..."
if [ -e /dev/ttyUSB0 ]; then
    ls -la /dev/ttyUSB0
    echo "✓ Device exists"
else
    echo "✗ /dev/ttyUSB0 not found!"
    echo "Available USB devices:"
    ls -la /dev/ttyUSB* 2>/dev/null || echo "None found"
    exit 1
fi
echo ""

echo "2. Checking USB device info..."
lsusb | grep -i "CP210\|Silicon"
echo ""

echo "3. Checking permissions..."
if [ -r /dev/ttyUSB0 ] && [ -w /dev/ttyUSB0 ]; then
    echo "✓ Read/Write permissions OK"
else
    echo "✗ Permission issue - run: sudo chmod 666 /dev/ttyUSB0"
fi
echo ""

echo "4. Testing serial communication..."
python3 << 'PYEOF'
import serial
import time

try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    print(f"✓ Port opened successfully")
    
    # Send RPlidar RESET command
    ser.write(bytes([0xA5, 0x40]))
    time.sleep(0.2)
    
    # Send GET_INFO command  
    ser.write(bytes([0xA5, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14]))
    time.sleep(0.5)
    
    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        print(f"✓ RPlidar responded! ({len(response)} bytes)")
        print(f"  Response: {response.hex()}")
    else:
        print("✗ NO RESPONSE from RPlidar")
        print("")
        print("PROBABLE CAUSE: Motor is not spinning!")
        print("")
        print("CHECK:")
        print("  1. Is the motor spinning? (should hear whirring)")
        print("  2. Is 5V power connected to motor pins?")
        print("  3. Red laser dot visible when spinning?")
    
    ser.close()
except Exception as e:
    print(f"✗ ERROR: {e}")
PYEOF
echo ""

echo "======================================"
echo "5. Testing with ros2..."
echo "Starting rplidar_composition for 5 seconds..."
timeout 5 ros2 run rplidar_ros rplidar_composition --ros-args \
    -p serial_port:=/dev/ttyUSB0 \
    -p serial_baudrate:=115200 \
    -p frame_id:=laser 2>&1 | grep -E "INFO|ERROR|scan"

echo ""
echo "======================================"
echo "Diagnostic complete!"
echo "======================================"
