# Nav2 Autonomous Navigation Setup Guide

## ✅ Setup Complete! All files created.

---

## 📋 Step-by-Step Usage Guide

### Phase 1: Create a Map (If you haven't already)

**1. Start SLAM system:**
```bash
./clean_start.sh
```

**2. Drive robot around to map the room**
- Use joystick to explore the environment
- Make sure map covers all areas you want to navigate

**3. Save the map:**
```bash
# In a NEW terminal (keep SLAM running)
cd /home/mimi/ros2_ws
./save_map.sh
```

This creates:
- `src/robot_slam/maps/my_map.pgm` (image)
- `src/robot_slam/maps/my_map.yaml` (metadata)

**4. Stop SLAM system** (Ctrl+C)

---

### Phase 2: Navigate with Nav2

**1. Start Navigation system:**
```bash
./launch_navigation.sh
```

This launches:
- ✅ Robot hardware (motors, lidar, odometry)
- ✅ Nav2 stack (AMCL localization + navigation)
- ✅ Map server (loads your saved map)
- ✅ RViz with navigation tools

**2. In RViz - Initialize Robot Position:**

When RViz opens, the robot needs to know where it is on the map:

a) Click **"2D Pose Estimate"** button (top toolbar)
b) Click on the map where the robot currently is
c) Drag to set the robot's orientation
d) You should see green particles appear around the robot

**Tip:** If particles don't converge, try:
- Setting initial pose more accurately
- Driving the robot a bit (helps AMCL localize)

**3. Send Navigation Goals:**

**Method 1: Nav2 Goal Tool**
- Click **"Nav2 Goal"** button in RViz
- Click destination on map
- Drag to set final orientation
- Robot automatically plans path and navigates!

**Method 2: Publish Point (for your nav2_pose.py)**
- Click **"Publish Point"** button
- Click destination on map
- Robot navigates there (default orientation)

**4. Monitor Navigation:**

In RViz you'll see:
- **Red line**: Global plan (overall path to goal)
- **Green line**: Local plan (immediate trajectory)
- **Pink area**: Local costmap (obstacles nearby)
- **Blue area**: Global costmap (full map obstacles)
- **Yellow arrow**: Current robot pose
- **Green particles**: AMCL localization estimates

---

## 🎯 Nav2 Features Available

### Manual Control
Joystick still works! Use it to:
- Override autonomous navigation (safety)
- Manually position robot for better localization
- Test hardware

### Click-to-Navigate
Your `nav2_pose.py` enables simple point-and-click navigation

### Path Planning
Nav2 automatically:
- Finds optimal path around obstacles
- Avoids walls and furniture
- Re-plans if obstacles block the path
- Smooths trajectories for mecanum wheels

### Localization (AMCL)
- Continuously tracks robot position on map
- Uses lidar scans + odometry
- Corrects odometry drift

---

## 🛠️ Configuration Files Created

| File | Purpose |
|------|---------|
| [nav2_params.yaml](src/robot_slam/config/nav2_params.yaml) | Nav2 tuning parameters |
| [navigation.launch.py](src/robot_slam/launch/navigation.launch.py) | Launch file for Nav2 |
| [nav2_rviz.rviz](src/robot_slam/config/nav2_rviz.rviz) | RViz configuration |
| [nav2_pose.py](src/robot_slam/robot_slam/nav2_pose.py) | Click-to-navigate node |
| [save_map.sh](save_map.sh) | Save SLAM map script |
| [launch_navigation.sh](launch_navigation.sh) | Start navigation script |

---

## 🔧 Tuning Parameters

If robot behaves oddly, adjust these in [nav2_params.yaml](src/robot_slam/config/nav2_params.yaml):

### Speed Limits
```yaml
controller_server:
  FollowPath:
    max_vel_x: 0.8          # Max forward speed (m/s)
    max_vel_y: 0.8          # Max strafe speed (m/s)
    max_vel_theta: 1.0      # Max rotation speed (rad/s)
    min_vel_x: -0.5         # Max backward speed
```

### Goal Tolerance
```yaml
controller_server:
  general_goal_checker:
    xy_goal_tolerance: 0.15    # How close to goal (meters)
    yaw_goal_tolerance: 0.2    # How close to orientation (radians)
```

### Obstacle Avoidance
```yaml
local_costmap:
  inflation_layer:
    inflation_radius: 0.55   # Safety distance from obstacles (m)
```

### Robot Size
```yaml
local_costmap:
  robot_radius: 0.22   # Robot radius for collision checking (m)
```

---

## 🐛 Troubleshooting

### Robot doesn't localize (green particles everywhere)
**Fix:**
- Set initial pose more accurately
- Drive robot around a bit
- Check if map matches environment
- Verify lidar is working: `ros2 topic echo /scan`

### Robot won't navigate / "No path found"
**Check:**
- Is goal position on the map (not in obstacle)?
- Is inflation radius too large?
- Run: `ros2 topic echo /plan` - should show path

### Robot drives erratically
**Check:**
- Odometry working: `ros2 topic hz /odom` (should be 50 Hz)
- TF tree complete: `ros2 run tf2_tools view_frames`
- Reduce max velocities in nav2_params.yaml

### Map doesn't load
**Check:**
- Map file exists: `ls src/robot_slam/maps/my_map.*`
- Map path in launch file is correct
- Map server started: `ros2 node list | grep map_server`

### Click-to-navigate doesn't work
**Check:**
- Nav2 action server running: `ros2 action list`
- Should see: `/navigate_to_pose`
- Check logs: `ros2 node info /click_to_nav_goal`

---

## 📊 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     NAV2 NAVIGATION STACK                   │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Hardware Layer:                                            │
│    Arduino → Serial Driver → Odometry → TF                 │
│    RPLidar → /scan topic                                    │
│                                                             │
│  ↓                                                          │
│                                                             │
│  Localization (AMCL):                                       │
│    /scan + /odom + /map → Robot pose on map                │
│    Publishes: /amcl_pose, /particle_cloud                  │
│                                                             │
│  ↓                                                          │
│                                                             │
│  Global Planner:                                            │
│    Finds path from robot → goal using map                  │
│    Publishes: /plan (red line in RViz)                     │
│                                                             │
│  ↓                                                          │
│                                                             │
│  Local Planner (DWB):                                       │
│    Follows global plan while avoiding obstacles            │
│    Uses: Local costmap, velocity limits                    │
│    Publishes: /cmd_vel → Serial Driver → Arduino           │
│                                                             │
│  ↓                                                          │
│                                                             │
│  Robot moves autonomously! 🎯                               │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 🚀 Quick Reference

**Create map:**
```bash
./clean_start.sh      # Start SLAM
# Drive around...
./save_map.sh         # Save map
```

**Navigate:**
```bash
./launch_navigation.sh   # Start Nav2
# In RViz: Set initial pose → Send goal
```

**Check system:**
```bash
./verify_system.sh    # Check all rates
ros2 node list        # See active nodes
ros2 topic list       # See active topics
ros2 action list      # See navigation actions
```

---

**Your robot is now ready for autonomous navigation!** 🎉
