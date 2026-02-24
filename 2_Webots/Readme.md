# 🤖 Webots Simulation Tutorials: ROS 2 Integration

> **Complete Learning Path | From Basic Control to Autonomous Navigation | 14 Progressive Tutorials**

---

## 📌 Overview

This comprehensive collection contains **14 progressive robot simulation tutorials** using **Webots** and **ROS 2 Jazzy**. Each tutorial demonstrates real robotics concepts through hands-on simulation, building from basic motor control to advanced autonomous navigation with SLAM.

### ✨ What You'll Learn

- ✅ Webots simulation environment setup
- ✅ ROS 2 subscriber/publisher patterns
- ✅ Sensor integration (encoders, touch, distance, IMU, cameras, LIDAR)
- ✅ Differential drive kinematics
- ✅ Computer vision (color detection, line following)
- ✅ Autonomous navigation and SLAM
- ✅ Real robotics principles through simulation

### 🎯 Target Audience

- 🎓 Robotics students and beginners
- 🔬 ROS 2 learners
- 🤖 Autonomous systems enthusiasts
- 👨‍💻 Developers transitioning from simulation to real robots

---

## 📚 Complete Tutorial Roadmap

### **Level 0️⃣ - Installation & Setup**

| # | Name | Focus | Topics |
|---|------|-------|--------|
| **000** | [Install Webots](000_Install_webots/) | Environment Setup | Webots installation, ROS 2 integration, package creation |

### **Level 1️⃣ - Basic Motor Control**

| # | Name | Focus | Sensors | Topics |
|---|------|-------|---------|--------|
| **001** | [Simple Robot Controller](001_simple_robot_controller/) | Velocity commands | None | ROS 2 Twist, motor control, kinematics |
| **002** | [Keyboard Teleoperation](002_keyboard_teleop/) | Manual control | None | Real-time keyboard input, non-blocking I/O |

### **Level 2️⃣ - Proprioceptive Sensing (Proprioceptive)**

| # | Name | Focus | Sensors | Topics |
|---|------|-------|---------|--------|
| **011** | [Wheel Encoder Mission](011_wheel_encoder_mission/) | Distance control | ✅ Encoders | Closed-loop control, 5.4m precise movement |
| **012** | [Keyboard with Distance](012_keyboard_with_distance/) | Manual + feedback | ✅ Encoders | Real-time distance tracking, manual control |

### **Level 3️⃣ - Reactive Obstacle Avoidance**

| # | Name | Focus | Sensors | Topics |
|---|------|-------|---------|--------|
| **021** | [Touch Sensor Controller](021_touch_sensor_controller/) | Collision detection | ✅ Bumpers | Reactive behavior, state machine, collision handling |
| **022** | [Distance Sensor Controller](022_distance_sensor_controller/) | Proactive avoidance | ✅ Distance sensors | Multi-sensor fusion, threat levels, 5-sensor array |

### **Level 4️⃣ - Inertial & Vision Sensors**

| # | Name | Focus | Sensors | Topics |
|---|------|-------|---------|--------|
| **031** | [IMU Controller](031_imu_controller/) | Heading control | ✅ IMU/Compass | Quaternion conversion, heading navigation |
| **032** | [Accelerometer Controller](032_accelerometer_controller/) | Motion detection | ✅ Accelerometer | Impact detection, gravity compensation, motion analysis |

### **Level 5️⃣ - Computer Vision**

| # | Name | Focus | Sensors | Topics |
|---|------|-------|---------|--------|
| **041** | [Camera Controller](041_camera_controller/) | Ball tracking | ✅ RGB Camera | Color detection, HSV filtering, centroid calculation |
| **042** | [Line Follower](042_line_follower_controller/) | Autonomous line following | ✅ Camera (ROI) | Image processing, brightness thresholding, adaptive control |

### **Level 6️⃣ - Range Sensing & Mapping**

| # | Name | Focus | Sensors | Topics |
|---|------|-------|---------|--------|
| **051** | [LIDAR Controller](051_lidar_controller/) | 360° obstacle detection | ✅ LIDAR | Sector analysis, range mapping, obstacle detection |
| **052** | [SLAM Controller](052_slam_controller/) | Simultaneous localization & mapping | ✅ Odometry + LIDAR | Loop closure, particle filtering, map building |

### **Level 7️⃣ - Advanced 3D Perception**

| # | Name | Focus | Sensors | Topics |
|---|------|-------|---------|--------|
| **061** | [Depth Camera Controller](061_depth_camera_controller/) | 3D point clouds | ✅ RGB-D Camera | Backprojection, plane fitting, 3D perception |
| **062** | [3D Mapping](062_3d_mapping/) | Environment reconstruction | ✅ Point clouds | Voxel grids, outlier removal, surface reconstruction |

### **Level 8️⃣ - Autonomous Navigation**

| # | Name | Focus | Sensors | Topics |
|---|------|-------|---------|--------|
| **071** | [Go-to-Goal Navigation](071_go_to_goal/) | Waypoint navigation | ✅ Odometry + IMU + LIDAR | Path planning, collision avoidance, stuck detection |

---

## 🚀 Quick Start

### Prerequisites

```bash
# Ubuntu 22.04 LTS or later
# ROS 2 Jazzy installed
# Webots simulator
```

### Step 1: Install Webots

Follow [000_Install_webots](000_Install_webots/) guide for complete setup.

### Step 2: Clone & Setup Repository

```bash
cd ~/ros2_ws/src
git clone <repo-url> ce_webots
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Step 3: Run Any Tutorial

```bash
# Example: Run 001 Simple Robot Controller
webots ~/ros2_ws/src/ce_webots/worlds/001_basic_control.wbt &
ros2 run ce_webots 001_simple_robot_controller
```

### Step 4: Send Commands (New Terminal)

```bash
# Test with velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

---

## 🚀 How to Run Each Lab

### General Running Pattern

Most tutorials follow this pattern:

1. **Terminal 1:** Start Webots with the world file
2. **Terminal 2:** Run the controller
3. **Terminal 3 (optional):** Monitor topics or visualize in RViz

### Labs Without RViz (001-051)

```bash
# Terminal 1: Start Webots simulation
webots ~/ros2_ws/src/ce_webots/worlds/<lab_world_file>.wbt

# Terminal 2: Run the controller
ros2 run ce_webots <controller_name>

# Terminal 3 (optional): Monitor topics
ros2 topic list
ros2 topic echo /cmd_vel
```

### Labs With RViz (052, 061, 062, 071)

For labs that use RViz visualization:

```bash
# Terminal 1: Start Webots simulation
webots ~/ros2_ws/src/ce_webots/worlds/<lab_world_file>.wbt

# Terminal 2: Run the controller
ros2 run ce_webots <controller_name>

# Terminal 3: Launch RViz2
rviz2
```

#### RViz Configuration Steps:

1. **Set Fixed Frame:**
   - In RViz left panel, set `Fixed Frame` to `odom` or `map`

2. **Add Visualization Topics:**
   - Click "Add" button (bottom left)
   - Select appropriate display types:
     - **052 SLAM:** Add `LaserScan` (/scan), `Map` (/map), `Odometry` (/odom)
     - **061 Depth Camera:** Add `PointCloud2` (/camera/points), `Image` (/camera/rgb/image_raw)
     - **062 3D Mapping:** Add `PointCloud2` (/camera/points), `Marker` (/robot_marker)
     - **071 Go-to-Goal:** Add `LaserScan` (/scan), `Odometry` (/odom), `Marker` (/waypoint_markers)

3. **Configure Topics:**
   - For each display, set the correct topic name
   - Adjust visualization parameters (size, color, decay time)

4. **Save Configuration:**
   - File → Save Config As → `<lab_name>_config.rviz`
   - Load next time: `rviz2 -d <lab_name>_config.rviz`

### Quick Reference by Lab Number

| Lab | Webots World File | Controller | RViz Required |
|-----|-------------------|------------|---------------|
| 001 | `001_basic_control.wbt` | `001_simple_robot_controller` | ❌ |
| 002 | `002_keyboard_teleop.wbt` | `002_keyboard_teleop` | ❌ |
| 011 | `011_wheel_encoder.wbt` | `011_wheel_encoder_mission` | ❌ |
| 012 | `012_keyboard_with_distance.wbt` | `012_keyboard_with_distance` | ❌ |
| 021 | `021_touch_sensor.wbt` | `021_touch_sensor_controller` | ❌ |
| 022 | `022_distance_sensor.wbt` | `022_distance_sensor_controller` | ❌ |
| 031 | `031_imu.wbt` | `031_imu_controller` | ❌ |
| 032 | `032_accelerometer.wbt` | `032_accelerometer_controller` | ❌ |
| 041 | `041_camera.wbt` | `041_camera_controller` | ❌ |
| 042 | `042_line_follower.wbt` | `042_line_follower_controller` | ❌ |
| 051 | `051_lidar.wbt` | `051_lidar_controller` | ❌ |
| 052 | `052_slam.wbt` | `052_slam_controller` | ✅ |
| 061 | `061_depth_camera.wbt` | `061_depth_camera_controller` | ✅ |
| 062 | `062_3d_map.wbt` | `062_3d_mapping` | ✅ |
| 071 | `071_go_goal.wbt` | `071_go_to_goal` | ✅ |

---

## 📁 Directory Structure

```
2_Webots/
├── 000_Install_webots/           # Webots + ROS 2 setup
│   └── Readme.md
│
├── 001_simple_robot_controller/  # Level 1: Basic control
│   ├── 001_basic_control.wbt
│   ├── 001_simple_robot_controller.py
│   ├── 001_simple_robot_controller.md
│   └── Readme.md
│
├── 002_keyboard_teleop/          # Level 1: Manual control
│   ├── 002_keyboard_teleop.wbt
│   ├── 002_keyboard_teleop.py
│   ├── 002_keyboard_teleop.md
│   └── Readme.md
│
├── 011_wheel_encoder_mission/    # Level 2: Encoder feedback
├── 012_keyboard_with_distance/
│
├── 021_touch_sensor_controller/  # Level 3: Bumper sensors
├── 022_distance_sensor_controller/
│
├── 031_imu_controller/           # Level 4: Inertial sensors
├── 032_accelerometer_controller/
│
├── 041_camera_controller/        # Level 5: Vision
├── 042_line_follower_controller/
│
├── 051_lidar_controller/         # Level 6: LIDAR mapping
├── 052_slam_controller/
│
├── 061_depth_camera_controller/  # Level 7: 3D perception
├── 062_3d_mapping/
│
├── 071_go_to_goal/               # Level 8: Autonomous nav
│
└── ce_webots/                    # ROS 2 package root
    ├── setup.py
    ├── setup.cfg
    ├── package.xml
    └── ...
```

---

## 📊 Learning Progression

```
Beginner Path (001-002):
    Basic motor control → Manual teleoperation

Intermediate Path (011-022):
    Proprioceptive sensing → Reactive obstacle avoidance

Advanced Path (031-042):
    IMU navigation → Computer vision

Expert Path (051-071):
    LIDAR mapping → SLAM → Autonomous navigation

Specialized Topics:
    - 061-062: 3D perception and point clouds
    - 052: SLAM fundamentals
```

---

## 🎓 Learning Outcomes

### After Tutorial 001-002 (Basic)
- ✅ Understand Webots-ROS 2 integration
- ✅ Work with motor controllers
- ✅ Implement ROS 2 subscribers
- ✅ Apply differential drive kinematics

### After Tutorial 011-022 (Intermediate)
- ✅ Read and interpret sensor data
- ✅ Implement closed-loop control
- ✅ Design state machines for behaviors
- ✅ Fuse multiple sensors

### After Tutorial 031-042 (Advanced)
- ✅ Process IMU/compass data
- ✅ Implement computer vision
- ✅ Color detection and image processing
- ✅ Autonomous tracking behaviors

### After Tutorial 051-071 (Expert)
- ✅ Build environmental maps
- ✅ Implement SLAM algorithms
- ✅ Design autonomous navigation
- ✅ Handle real-world constraints

---

## 🔧 Common Commands

### Run Webots with World File
```bash
webots ~/ros2_ws/src/ce_webots/worlds/<world_name>.wbt
```

### Run ROS 2 Controller
```bash
ros2 run ce_webots <controller_name>
```

### Monitor Topics
```bash
ros2 topic list                    # List all topics
ros2 topic echo /cmd_vel           # Monitor specific topic
ros2 topic hz /cmd_vel             # Check publication rate
```

### Record/Playback Bag Files
```bash
ros2 bag record /cmd_vel /robot/odom
ros2 bag play rosbag2_<timestamp>
```

### Build Workspace
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 📚 Sensor Reference

| Sensor Type | Use Case | Range | Accuracy | Tutorial |
|-------------|----------|-------|----------|----------|
| **Encoders** | Distance tracking | Unlimited | ±0.001 rad | 011, 012 |
| **Touch/Bumpers** | Collision detection | ~1-2cm | Binary | 021 |
| **Distance Sensors** | Obstacle detection | 0.05-2.0m | ±5cm | 022 |
| **IMU/Compass** | Heading control | 360° | ±0.1° | 031 |
| **Accelerometer** | Motion detection | ±50 m/s² | ±0.1 m/s² | 032 |
| **RGB Camera** | Color tracking | 640×480 | Pixel-level | 041 |
| **Camera (ROI)** | Line detection | Full image | Edge-based | 042 |
| **LIDAR** | 360° mapping | 0.05-2.0m | ±5cm | 051, 052 |
| **RGB-D Camera** | 3D perception | 0.1-6.0m | ±1-2% | 061 |
| **Point Cloud** | Environment mapping | Unlimited | Variable | 062 |

---

## 🐛 Troubleshooting

### Webots Won't Start
```bash
# Check installation
which webots

# Install missing dependencies
sudo apt install libxrender1 libxrandr2

# Try direct launch
webots &
```

### ROS 2 Package Not Found
```bash
# Rebuild workspace
cd ~/ros2_ws && colcon build
source install/setup.bash

# Verify package
ros2 pkg list | grep ce_webots
```

### Simulation Too Slow
```bash
# Check timestep in .wbt file
# Increase basicTimeStep value (32 → 64)
# Reduce graphics quality in Webots settings
# Close unnecessary background processes
```

### Topic Not Publishing
```bash
# Check controller is running
ps aux | grep <controller_name>

# Verify ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID

# Check topic data
ros2 topic echo /cmd_vel
```

---

## 📖 Resources

### Official Documentation
- 📘 [Webots Documentation](https://cyberbotics.com/doc/)
- 📗 [ROS 2 Docs](https://docs.ros.org/en/jazzy/)
- 📕 [ROS 2 Geometry Messages](https://docs.ros.org/en/jazzy/p/geometry_msgs/)

### Related Topics
- 🤖 [Differential Drive Kinematics](https://en.wikipedia.org/wiki/Differential_wheeled_robot)
- 🔍 [Computer Vision Basics](https://docs.opencv.org/master/d9/df8/tutorial_root.html)
- 📍 [SLAM Fundamentals](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)
- 🧭 [IMU and Quaternions](https://en.wikipedia.org/wiki/Quaternion)

### Recommended Tutorials
1. Start with **001** to understand basic concepts
2. Progress through **011-022** for sensor integration
3. Try **031-042** for advanced sensing
4. Finish with **051-071** for autonomous systems

---

## 💻 System Requirements

| Component | Requirement |
|-----------|------------|
| **OS** | Ubuntu 22.04 LTS (recommended) |
| **ROS 2** | Jazzy or later |
| **Webots** | 2023b or later |
| **Python** | 3.10+ |
| **RAM** | 4GB minimum (8GB+ recommended) |
| **Disk Space** | 5GB (with simulation files) |
| **GPU** | Optional (improves graphics) |

---

## 🤝 Contributing

Found a bug or want to improve a tutorial?

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/improvement`)
3. Make your changes
4. Commit with clear messages (`git commit -m "Add: improved documentation"`)
5. Push to branch (`git push origin feature/improvement`)
6. Open a Pull Request

---

## 📝 Notes

### For Instructors
- Each tutorial can be completed independently (after prerequisites)
- Expected time per tutorial: 1-2 hours
- Recommended sequence: 000 → 001 → 002 → 011 → 012 → ... → 071

### For Self-Learners
- Review the "Learning Outcomes" for each level
- Run the simulation multiple times with different parameters
- Modify the controller code to experiment
- Combine concepts from multiple tutorials

### For Real Robot Transfer
- Concepts directly apply to real TurtleBot3, E-puck, etc.
- Webots physics closely matches real-world behavior
- Main differences: sensor noise, computational delays
- Use the same ROS 2 code structure with real robots

---

## **👥 Authors & Contributors**

- 🚀 [@alfaXphoori](https://www.github.com/alfaXphoori) - Original Creator

---

## 📜 License

This project is licensed under the **MIT License** - see LICENSE file for details.

---

## 🔗 Quick Links

| Tutorial | Level | Sensors | Link |
|----------|-------|---------|------|
| Installation | 0 | - | [000_Install_webots](000_Install_webots/) |
| Simple Robot | 1 | - | [001_simple_robot_controller](001_simple_robot_controller/) |
| Keyboard Teleop | 1 | - | [002_keyboard_teleop](002_keyboard_teleop/) |
| Encoder Mission | 2 | Encoders | [011_wheel_encoder_mission](011_wheel_encoder_mission/) |
| Keyboard + Distance | 2 | Encoders | [012_keyboard_with_distance](012_keyboard_with_distance/) |
| Touch Sensor | 3 | Bumpers | [021_touch_sensor_controller](021_touch_sensor_controller/) |
| Distance Sensor | 3 | IR/Ultrasonic | [022_distance_sensor_controller](022_distance_sensor_controller/) |
| IMU Controller | 4 | Compass | [031_imu_controller](031_imu_controller/) |
| Accelerometer | 4 | Accelerometer | [032_accelerometer_controller](032_accelerometer_controller/) |
| Camera Ball Chaser | 5 | RGB Camera | [041_camera_controller](041_camera_controller/) |
| Line Follower | 5 | Camera (ROI) | [042_line_follower_controller](042_line_follower_controller/) |
| LIDAR Scanner | 6 | LIDAR | [051_lidar_controller](051_lidar_controller/) |
| SLAM Mapping | 6 | LIDAR + Odometry | [052_slam_controller](052_slam_controller/) |
| Depth Camera | 7 | RGB-D | [061_depth_camera_controller](061_depth_camera_controller/) |
| 3D Mapping | 7 | Point Cloud | [062_3d_mapping](062_3d_mapping/) |
| Go-to-Goal | 8 | Multi-sensor | [071_go_to_goal](071_go_to_goal/) |

---

## 🔗 Work

| ROS | 7 | [Ros #7 Webots](https://docs.google.com/forms/d/e/1FAIpQLSdMTO5NYQL9SISZ-GZsaH18elBoaJ_03zJS-PVpblwBLvQBJA/viewform?usp=dialog) |
| ROS | 8 | [Ros #8 Webots_Encoder/Touch/Distance](https://docs.google.com/forms/d/e/1FAIpQLSfVfoMu13B3RZ0FPwuqnIksftYeLMky_nbmJtvMPvIgWFsICA/viewform?usp=dialog) |
| ROS | 9 | [Ros #9 Webots_imu_lidar](https://docs.google.com/forms/d/e/1FAIpQLScXNWqrW5sqfYRCcqr50B26U0oim8wnhzjIpL8VJS709KwnyQ/viewform?usp=publish-editor) |

---

<div align="center">

### 🎯 Start Learning Now! Pick Any Tutorial Above 👆

**Made with ❤️ for the Robotics Community**

</div>
