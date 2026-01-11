#!/bin/bash
# Synchronize system time on Pi5 to avoid TF extrapolation errors
# Run this on Pi5 before starting navigation

echo "========================================="
echo "  Time Synchronization for ROS2"
echo "========================================="
echo ""
echo "Current system time:"
date

echo ""
echo "Synchronizing with NTP servers..."
sudo timedatectl set-ntp true
sudo systemctl restart systemd-timesyncd
sleep 2

echo ""
echo "Time sync status:"
timedatectl status

echo ""
echo "If time is still not synchronized, you can manually set it:"
echo "  sudo date -s 'YYYY-MM-DD HH:MM:SS'"
echo ""
echo "For distributed ROS2, both Pi5 and host should use NTP."
echo "On host machine, run: sudo timedatectl set-ntp true"
