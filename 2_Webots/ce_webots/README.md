# CE Webots - ROS2 + Webots Learning Package

Learn robotics through **sensor-based progression** - from basic proprioceptive sensors to advanced 3D perception.

## 🗺️ Sensor-Based Learning Roadmap

Progressive learning path following how real robots are developed:

| Level | Sensor Type | Status | Tutorial | Time |
|-------|-------------|--------|----------|------|
| **1.1** | 🔄 Wheel Encoders & Odometry | ✅ Complete | [Tutorial 04](docs/04_odometry.md) | 2-3h |
| **2.1** | 👆 Touch/Bumper Sensors | 🔜 Planned | - | 1-2h |
| **2.2** | 📏 Distance Sensors (IR) | ✅ Complete | [Tutorial 03](docs/03_obstacle_avoidance.md) | 2-3h |
| **3.1** | 🧭 IMU (Inertial Unit) | 🔜 Planned | - | 2-3h |
| **4.1** | 👁️ RGB Camera & Vision | 🔜 Planned | - | 4-6h |
| **5.1** | 📡 Lidar & SLAM | 🔜 Planned | - | 6-8h |
| **6.1** | 🧊 Depth/RGB-D (3D) | 🔜 Planned | - | 6-8h |
| **7.1** | 🛰️ GPS (Global Position) | 🔜 Planned | - | 4-6h |

**Supporting Tutorials:**
- Tutorial 01: Basic differential drive control (foundation)
- Tutorial 02: Keyboard teleop (interactive testing)

**📖 Complete Guide:** [LEARNING_GUIDE.md](docs/LEARNING_GUIDE.md)

## 🚀 Quick Start

### Prerequisites
- ROS2 Humble or Jazzy
- Webots R2025a
- Python 3.8+

### Build Package

```bash
cd ~/ros2_ws
colcon build --packages-select ce_webots --symlink-install
source install/setup.bash
```

---

## 🔰 Level 1: Proprioceptive Sensors

### Level 1.1 - Wheel Encoders & Odometry (Tutorial 04)

**Mission:** Move forward exactly 1 meter using odometry feedback (no time delays)

**Terminal 1 - Launch Webots:**
```bash
webots ~/ros2_ws/src/ce_webots/worlds/04_odometry.wbt
```

**Terminal 2 - Run Odometry Controller:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run ce_webots 04_odometry_controller
```

**Terminal 3 - Send Commands:**
```bash
# Move forward
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"

# Monitor position (stop when x or y ≈ 1.0m)
ros2 topic echo /odom/pose/pose/position

# Auto-align to Y-axis
ros2 topic pub --once /align_axis std_msgs/msg/String "{data: 'y'}"
```

**What you'll learn:** Forward kinematics, position integration, odometry publishing

---

## 🚧 Level 2: Proximity & Touch Sensors

### Level 2.2 - Distance Sensors (Tutorial 03)

**Mission:** Navigate autonomously without collisions

**Terminal 1 - Launch Webots:**
```bash
webots ~/ros2_ws/src/ce_webots/worlds/03_obstacle_avoidance.wbt
```

**Terminal 2 - Run Autonomous Controller:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run ce_webots 03_obstacle_avoidance_controller
```

Robot explores autonomously, avoiding obstacles using 3x distance sensors!

**What you'll learn:** Reactive behaviors, state machines, sensor-based navigation

### Level 2.1 - Touch/Bumper Sensors

**Status:** 🔜 Coming soon

---

## 🧰 Supporting Tutorials

### Tutorial 01 - Basic Control (Foundation)

**Purpose:** Learn differential drive kinematics (inverse kinematics)

```bash
# Terminal 1:
webots ~/ros2_ws/src/ce_webots/worlds/base_robot.wbt

# Terminal 2:
ros2 run ce_webots 01_simple_robot_controller

# Terminal 3 - Control robot:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

### Tutorial 02 - Keyboard Teleop (Interactive Testing)

**Purpose:** Manual control for testing sensor implementations

```bash
# Terminal 1: Webots (any world file)
# Terminal 2:
ros2 run ce_webots 01_keyboard_teleop
```

Control with: **W/A/S/D/Q/E**, **+/-** for speed, **X** to stop

## 📁 Package Structure

```
ce_webots/
├── ce_webots/                      # Python controllers
│   ├── 01_simple_robot_controller.py
│   ├── 02_keyboard_teleop.py
│   ├── 03_obstacle_avoidance_controller.py
│   └── 04_odometry_controller.py
├── worlds/                         # Webots world files
│   ├── base_robot.wbt              # Foundation robot (used by Tutorial 01)
│   ├── 03_obstacle_avoidance.wbt
│   └── 04_odometry.wbt
├── docs/                           # Documentation
│   ├── LEARNING_GUIDE.md           # Complete learning path
│   ├── 01_simple_robot.md
│   ├── 01_keyboard_teleop.md
│   ├── 03_obstacle_avoidance.md
│   └── 04_odometry.md
├── package.xml
├── setup.py
└── README.md
```

## 🎯 Learning Path

1. **Start Here:** [Tutorial 01](docs/01_simple_robot.md) - Learn differential drive basics
2. **Control:** [Tutorial 02](docs/01_keyboard_teleop.md) - Drive with keyboard
3. **Sensors:** [Tutorial 03](docs/03_obstacle_avoidance.md) - Autonomous navigation
4. **Localization:** [Tutorial 04](docs/04_odometry.md) - Track position
5. **Complete Guide:** [LEARNING_GUIDE.md](docs/LEARNING_GUIDE.md) - Full curriculum

## 🔧 Available Commands

```bash
# List all executables
ros2 pkg executables ce_webots

# Output:
# ce_webots 01_simple_robot_controller
# ce_webots 01_keyboard_teleop
# ce_webots 03_obstacle_avoidance_controller
# ce_webots 04_odometry_controller
```

## 🎓 What's Next?

Following the sensor-based progression:

**Next Recommended:** Level 2.1 - Touch/Bumper Sensors (1-2 hours)
- Simple reactive collision response
- Emergency stop behaviors
- Safety-first approach

**Alternative:** Level 3.1 - IMU (2-3 hours)
- Improve odometry accuracy
- Precise rotation control
- Sensor fusion basics

**Later:**
- Level 4.1: Camera & vision (line following, object tracking)
- Level 5.1: Lidar & SLAM (mapping, navigation)
- Level 6.1: 3D perception (depth cameras, point clouds)
- Level 7.1: GPS (outdoor navigation)

See [LEARNING_GUIDE.md](docs/LEARNING_GUIDE.md) for complete roadmap with missions and checkpoints.

---

## 📝 License

MIT License - Feel free to use, modify, and share!

---

**🤖 Happy Learning!** Follow the sensor-based progression from [LEARNING_GUIDE](docs/LEARNING_GUIDE.md)
