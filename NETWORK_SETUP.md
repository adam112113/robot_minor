# ROS2 Network Setup - Pi5 (Headless) + Host Machine (GUI)

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Pi5 (Ubuntu Server - No GUI)                               │
│  IP: 192.168.1.100 (example)                                │
│  ────────────────────────────────────                       │
│  • Robot hardware (motors, lidar)                           │
│  • Odometry node                                            │
│  • SLAM Toolbox / Nav2                                      │
│  • All computation                                          │
│                                                             │
│  ROS2 Topics published ──────────┐                          │
└──────────────────────────────────│──────────────────────────┘
                                   │
                          Network (WiFi/Ethernet)
                                   │
┌──────────────────────────────────▼──────────────────────────┐
│  Host Machine (Ubuntu/Windows/Mac with GUI)                 │
│  IP: 192.168.1.50 (example)                                 │
│  ────────────────────────────────────                       │
│  • RViz2 (visualization only)                               │
│  • Subscribes to topics from Pi5                            │
│  • Sends goals to Pi5                                       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## Setup Guide

### 1. Network Configuration

**Both machines MUST be on the same network!**

Common options:
- Same WiFi network
- Connected via Ethernet switch
- Direct Ethernet connection (with manual IP config)

**Find Pi5 IP address:**
```bash
# On Pi5:
ip addr show
# Look for inet address (e.g., 192.168.1.100)
# In this case: 192.168.1.35
```

---

### 2. Pi5 Setup (Already Done ✅)

**Set ROS_DOMAIN_ID in .bashrc:**
```bash
# On Pi5:
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc
```

**Configure network for multicast (if needed):**
```bash
# On Pi5 - Allow ROS2 discovery traffic
sudo ufw allow from 192.168.1.0/24  # Adjust subnet to match your network
# OR disable firewall for testing:
# sudo ufw disable
```

**Verify it works:**
```bash
# On Pi5:
source /home/mimi/ros2_ws/install/setup.bash
ros2 topic list
# Should see topics from running nodes
```

---

### 3. Host Machine Setup

#### Option A: Linux (Ubuntu 22.04/24.04)

**Install ROS2 Jazzy:**
```bash
# Add ROS2 repository
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-rviz2 ros-jazzy-navigation2 ros-jazzy-nav2-bringup -y
```

**Set ROS_DOMAIN_ID:**
```bash
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc
```

#### Option B: Windows + WSL2

1. Install WSL2 with Ubuntu 22.04
2. Follow Linux setup above
3. Install VcXsrv or similar X server for GUI

#### Option C: macOS (Not recommended - limited ROS2 support)

Use Docker with X11 forwarding instead.

---

### 4. Copy Configuration Files to Host

**Transfer the RViz configs and scripts from Pi5 to your host:**

```bash
# On host machine:
mkdir -p ~/ros2_ws/src/robot_slam/config
mkdir -p ~/ros2_ws/host_scripts

# Copy files from Pi5 (replace with your Pi5's IP):
PI5_IP="192.168.1.100"
scp mimi@$PI5_IP:/home/mimi/ros2_ws/src/robot_slam/config/slam_rviz.yaml ~/ros2_ws/src/robot_slam/config/
scp mimi@$PI5_IP:/home/mimi/ros2_ws/src/robot_slam/config/nav2_rviz.rviz ~/ros2_ws/src/robot_slam/config/
scp mimi@$PI5_IP:/home/mimi/ros2_ws/host_scripts/*.sh ~/ros2_ws/host_scripts/
chmod +x ~/ros2_ws/host_scripts/*.sh
```

**Edit the scripts to set your Pi5's IP:**
```bash
# Edit both scripts:
nano ~/ros2_ws/host_scripts/launch_slam_rviz.sh
nano ~/ros2_ws/host_scripts/launch_nav_rviz.sh

# Change this line in each:
export PI5_IP="192.168.1.100"  # ⚠️ Use your actual Pi5 IP!
```

---

### 5. Test the Connection

**On Pi5:**
```bash
cd /home/mimi/ros2_ws
./clean_start.sh
```

**On Host:**
```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0

# Check if you can see Pi5's topics
ros2 topic list

# You should see:
#   /scan
#   /odom
#   /tf
#   /map
#   etc.
```

If you see the topics ✅ = Network is working!

If not ❌ = Check:
- Both on same network?
- Same ROS_DOMAIN_ID?
- Firewall blocking?
- Multicast enabled?

---

### 6. Launch RViz

**For SLAM mapping:**
```bash
# On host machine:
cd ~/ros2_ws
./host_scripts/launch_slam_rviz.sh
```

**For Nav2 navigation:**
```bash
# On host machine:
cd ~/ros2_ws
./host_scripts/launch_nav_rviz.sh
```

---

## Troubleshooting

### "No topics visible in RViz"

**Check ROS_DOMAIN_ID matches:**
```bash
# On Pi5:
echo $ROS_DOMAIN_ID

# On Host:
echo $ROS_DOMAIN_ID

# Should be the same (usually 0)
```

**Check network connectivity:**
```bash
# From host, ping Pi5:
ping 192.168.1.100  # Use actual Pi5 IP

# Test ROS2 discovery:
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

**Check multicast:**
```bash
# On both machines:
sudo apt install avahi-daemon
sudo systemctl enable avahi-daemon
sudo systemctl start avahi-daemon
```

### "Topics appear but no data"

**Check topic rates:**
```bash
# On host:
ros2 topic hz /scan
ros2 topic hz /odom

# Should show data flowing at expected rates
```

**Check Fixed Frame in RViz:**
- For SLAM: Should be `map`
- For Nav2: Should be `map`
- If `map` doesn't exist yet, temporarily use `odom`

### "RViz crashes or freezes"

**Reduce display rate in RViz:**
- Click on topic (e.g., LaserScan)
- Adjust "Reliability Policy" to "Best Effort"
- Reduce "Queue Size" to 1-5

**Lower quality settings:**
- In LaserScan, set "Style" to "Points" (not "Flat Squares")
- Reduce TF "Update Interval" to 0.5-1.0

### "Connection drops frequently"

**Use Ethernet instead of WiFi**

**Or configure WiFi for better ROS2:**
```bash
# On both machines - disable power saving:
sudo iwconfig wlan0 power off
```

**Increase ROS2 timeouts:**
Add to `~/.bashrc` on both machines:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/mimi/cyclonedds.xml
```

Create `/home/mimi/cyclonedds.xml`:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
    </General>
  </Domain>
</CycloneDDS>
```

---

## Advanced: Using Different ROS_DOMAIN_IDs

If you have multiple robots:

**Robot 1:**
```bash
# Pi5 and Host:
export ROS_DOMAIN_ID=0
```

**Robot 2:**
```bash
# Different Pi5 and Host:
export ROS_DOMAIN_ID=1
```

Each domain is isolated - topics won't cross-talk.

---

## Performance Optimization

**For WiFi connections:**
- Use 5GHz WiFi (less interference)
- Place Pi5 close to WiFi access point
- Limit other WiFi devices

**For large maps:**
- Set map update rate lower: `map_update_interval: 2.0` in nav2_params.yaml
- Reduce map resolution if acceptable: `resolution: 0.10` (was 0.05)

**For better RViz performance:**
- Disable unused displays (TF, Particle Cloud)
- Set laser scan "Queue Size" to 1
- Use "Points" style instead of "Spheres"

---

## Quick Reference

**Pi5 Commands:**
```bash
./clean_start.sh           # SLAM mapping
./launch_navigation.sh     # Nav2 navigation
./save_map.sh              # Save map
```

**Host Commands:**
```bash
./host_scripts/launch_slam_rviz.sh    # RViz for SLAM
./host_scripts/launch_nav_rviz.sh     # RViz for Nav2
```

**Check connection:**
```bash
ros2 topic list            # See topics from Pi5
ros2 topic hz /scan        # Check lidar rate
ros2 node list             # See nodes running
```

---

## Files on Host Machine

You need these files from Pi5:
- `config/slam_rviz.yaml` - SLAM RViz config
- `config/nav2_rviz.rviz` - Nav2 RViz config  
- `host_scripts/launch_slam_rviz.sh` - SLAM RViz launcher
- `host_scripts/launch_nav_rviz.sh` - Nav2 RViz launcher

**That's it! No need for full workspace on host - just RViz configs.**
