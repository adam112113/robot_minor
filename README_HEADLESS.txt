╔═══════════════════════════════════════════════════════════════╗
║          HEADLESS PI5 + HOST MACHINE RVIZ SETUP              ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║  ✅ FILES UPDATED FOR DISTRIBUTED ROS2                        ║
║                                                               ║
╠═══════════════════════════════════════════════════════════════╣
║  ARCHITECTURE:                                                ║
║  ─────────────                                                ║
║                                                               ║
║  ┌──────────────────────────────────┐                        ║
║  │ Pi5 (Ubuntu Server - No GUI)     │                        ║
║  │ • All hardware & computation     │                        ║
║  │ • SLAM / Nav2                    │                        ║
║  │ • Publishes ROS topics           │                        ║
║  └──────────────┬───────────────────┘                        ║
║                 │                                             ║
║          Network (WiFi/Ethernet)                              ║
║                 │                                             ║
║  ┌──────────────▼───────────────────┐                        ║
║  │ Host (Your laptop/desktop)       │                        ║
║  │ • RViz only                      │                        ║
║  │ • Subscribes to Pi5 topics       │                        ║
║  └──────────────────────────────────┘                        ║
║                                                               ║
╠═══════════════════════════════════════════════════════════════╣
║  QUICK SETUP:                                                 ║
║  ────────────                                                 ║
║                                                               ║
║  1. ON PI5 (Already done ✅):                                 ║
║     $ echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc              ║
║     $ source ~/.bashrc                                        ║
║                                                               ║
║  2. FIND PI5 IP ADDRESS:                                      ║
║     $ ip addr show                                            ║
║     Look for: inet 192.168.1.XXX (example)                    ║
║                                                               ║
║  3. ON HOST MACHINE:                                          ║
║     a) Install ROS2 Jazzy Desktop:                            ║
║        Follow: NETWORK_SETUP.md                               ║
║                                                               ║
║     b) Set ROS_DOMAIN_ID:                                     ║
║        $ echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc           ║
║        $ source ~/.bashrc                                     ║
║                                                               ║
║     c) Copy files from Pi5:                                   ║
║        $ PI5_IP="192.168.1.XXX"  # Your Pi5's IP              ║
║        $ mkdir -p ~/ros2_ws/src/robot_slam/config             ║
║        $ mkdir -p ~/ros2_ws/host_scripts                      ║
║        $ scp mimi@$PI5_IP:/home/mimi/ros2_ws/host_scripts/*.sh ~/ros2_ws/host_scripts/
║        $ scp mimi@$PI5_IP:/home/mimi/ros2_ws/src/robot_slam/config/*.{yaml,rviz} ~/ros2_ws/src/robot_slam/config/
║        $ chmod +x ~/ros2_ws/host_scripts/*.sh                 ║
║                                                               ║
║     d) Edit scripts with Pi5 IP:                              ║
║        $ nano ~/ros2_ws/host_scripts/launch_slam_rviz.sh      ║
║        Change: export PI5_IP="192.168.1.XXX"                  ║
║                                                               ║
║        $ nano ~/ros2_ws/host_scripts/launch_nav_rviz.sh       ║
║        Change: export PI5_IP="192.168.1.XXX"                  ║
║                                                               ║
╠═══════════════════════════════════════════════════════════════╣
║  USAGE:                                                       ║
║  ──────                                                       ║
║                                                               ║
║  FOR SLAM MAPPING:                                            ║
║  ─────────────────                                            ║
║  Pi5:   $ ./clean_start.sh                                    ║
║  Host:  $ cd ~/ros2_ws                                        ║
║         $ ./host_scripts/launch_slam_rviz.sh                  ║
║                                                               ║
║  FOR NAVIGATION:                                              ║
║  ───────────────                                              ║
║  Pi5:   $ ./launch_navigation.sh                              ║
║  Host:  $ cd ~/ros2_ws                                        ║
║         $ ./host_scripts/launch_nav_rviz.sh                   ║
║                                                               ║
╠═══════════════════════════════════════════════════════════════╣
║  TEST CONNECTION:                                             ║
║  ────────────────                                             ║
║  On Host:                                                     ║
║  $ source /opt/ros/jazzy/setup.bash                           ║
║  $ export ROS_DOMAIN_ID=0                                     ║
║  $ ros2 topic list                                            ║
║                                                               ║
║  Should see: /scan, /odom, /tf, /map, etc.                    ║
║  ✅ Topics visible = Network works!                           ║
║  ❌ No topics = Check NETWORK_SETUP.md troubleshooting        ║
║                                                               ║
╠═══════════════════════════════════════════════════════════════╣
║  FILES CHANGED:                                               ║
║  ──────────────                                               ║
║  Pi5 Side:                                                    ║
║  • robot_slam.launch.py - RViz removed                        ║
║  • navigation.launch.py - RViz removed                        ║
║  • host_scripts/launch_slam_rviz.sh - NEW                     ║
║  • host_scripts/launch_nav_rviz.sh - NEW                      ║
║                                                               ║
║  Host Side (copy these):                                      ║
║  • host_scripts/launch_slam_rviz.sh                           ║
║  • host_scripts/launch_nav_rviz.sh                            ║
║  • config/slam_rviz.yaml                                      ║
║  • config/nav2_rviz.rviz                                      ║
║                                                               ║
╠═══════════════════════════════════════════════════════════════╣
║  DOCUMENTATION:                                               ║
║  ──────────────                                               ║
║  • NETWORK_SETUP.md  - Complete network setup guide           ║
║  • NAV2_GUIDE.md     - Navigation guide (updated)             ║
║                                                               ║
╠═══════════════════════════════════════════════════════════════╣
║  TROUBLESHOOTING:                                             ║
║  ────────────────                                             ║
║  No topics visible:                                           ║
║    → Check both machines on same network                      ║
║    → Check ROS_DOMAIN_ID=0 on both                            ║
║    → Try: ping <pi5_ip> from host                             ║
║                                                               ║
║  RViz slow/laggy:                                             ║
║    → Use Ethernet instead of WiFi                             ║
║    → Reduce queue sizes in RViz                               ║
║    → Disable unused displays                                  ║
║                                                               ║
║  Full troubleshooting: NETWORK_SETUP.md                       ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
