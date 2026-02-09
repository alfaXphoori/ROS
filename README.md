# 🤖 ROS 2 Robotics Learning Platform

A comprehensive educational platform for learning **ROS 2 Jazzy** robotics programming with hands-on exercises, simulations, and real-world applications.

## 📋 Project Structure

This repository is organized into two main sections:

### 1️⃣ **1_ROS2/** - Core ROS 2 Concepts
Foundational ROS 2 programming tutorials covering:

- **00_Install** - ROS 2 Jazzy installation guide and setup
- **01_Publisher_Subscriber** - Message passing between nodes
- **02_Server_Client** - Request-response communication patterns
- **03_Message** - Custom message definition and usage
- **04_Service** - Service definition and implementation
- **05_Parameters** - Dynamic parameter management
- **06_Action** - Long-running task execution
- **07_Launch** - Launch file configuration and execution

[📖 Explore ROS 2 Tutorials →](1_ROS2/README.md)

### 2️⃣ **2_Webots/** - Robot Simulation & Sensor Integration
Advanced robotics tutorials using Webots simulator with ROS 2 integration:

- **8 Progressive Learning Levels** (001-071)
- **Level 0:** Installation and basic setup
- **Level 1:** Motor control fundamentals
- **Level 2:** Proprioceptive sensing (encoders)
- **Level 3:** Obstacle avoidance (bumpers, distance sensors)
- **Level 4:** Inertial measurement and motion detection
- **Level 5:** Computer vision and image processing
- **Level 6:** Range sensing and SLAM mapping
- **Level 7:** 3D perception and point clouds
- **Level 8:** Autonomous navigation and waypoint tracking

[🚀 Explore Webots Tutorials →](2_Webots/README.md)

---

## ⚡ Quick Start

### Prerequisites
- **OS**: Ubuntu 22.04 LTS (or compatible Linux distribution)
- **Python**: 3.10+
- **ROS 2**: Jazzy distribution
- **Webots**: 2023b or later (for simulation exercises)

### Installation
```bash
# Clone repository
git clone https://github.com/alfaXphoori/ROS.git
cd ROS

# Navigate to desired section
cd 1_ROS2/00_Install  # For ROS 2 setup
# OR
cd 2_Webots           # For robot simulation
```

### Running Tutorials

**ROS 2 Core Concepts:**
```bash
cd 1_ROS2/01_Publisher_Subscriber/src
python3 simple_publisher.py    # In terminal 1
python3 simple_subscriber.py   # In terminal 2
```

**Webots Simulations:**
```bash
cd 2_Webots/001_simple_robot_controller
# Open in Webots IDE or run controller
python3 robot_controller.py
```

---

## 📚 Learning Path

### Beginner (Start Here)
1. **ROS 2 Basics** - Understand pub/sub, services, and messages
   - 1_ROS2/01_Publisher_Subscriber
   - 1_ROS2/02_Server_Client

2. **Basic Robot Control** - Motor and actuator commands
   - 2_Webots/001_simple_robot_controller
   - 2_Webots/002_keyboard_teleop

### Intermediate
3. **Sensor Integration** - Read and process sensor data
   - 2_Webots/011_wheel_encoder_mission (Encoders)
   - 2_Webots/021_touch_sensor_controller (Bumpers)
   - 2_Webots/022_distance_sensor_controller (Distance)

4. **Motion & Orientation** - IMU and motion detection
   - 2_Webots/031_imu_controller (Orientation)
   - 2_Webots/032_accelerometer_controller (Acceleration)

### Advanced
5. **Computer Vision** - Image processing and object detection
   - 2_Webots/041_camera_controller (Color tracking)
   - 2_Webots/042_line_follower_controller (Line detection)

6. **Mapping & Navigation** - SLAM and autonomous navigation
   - 2_Webots/051_lidar_controller (Distance mapping)
   - 2_Webots/052_slam_controller (SLAM implementation)
   - 2_Webots/071_go_to_goal (Waypoint navigation)

7. **3D Perception** - Depth sensing and point clouds
   - 2_Webots/061_depth_camera_controller (RGB-D)
   - 2_Webots/062_3d_mapping (Point cloud processing)

---

## 🛠 Technology Stack

| Component | Version | Purpose |
|-----------|---------|---------|
| ROS 2 | Jazzy | Middleware framework |
| Python | 3.10+ | Programming language |
| Webots | 2023b+ | Robot simulator |
| Ubuntu | 22.04 LTS | Operating system |

---

## 📖 Documentation Structure

Each tutorial includes:
- **🎯 Overview** - Learning objectives and key concepts
- **🚀 Quick Start** - Get running in 5 minutes
- **🔧 How It Works** - Deep dive into implementation
- **📚 Sensor Knowledge** - Theory and practical considerations
- **💡 Usage Tips** - Best practices and optimization
- **📝 Customization** - Modify and extend examples
- **⚠️ Troubleshooting** - Common issues and solutions

---

## 🔗 Key Resources

### Official Documentation
- [ROS 2 Official Documentation](https://docs.ros.org/en/jazzy/)
- [Webots Documentation](https://cyberbotics.com/documentation)
- [Python 3 Documentation](https://docs.python.org/3/)

### Learning Materials
- ROS 2 Concepts: Actions, Services, Parameters
- Sensor Physics: LIDAR, Camera, IMU principles
- Robot Kinematics: Motion planning and control
- Computer Vision: Image processing fundamentals

---

## 📝 Course Structure

| Level | Focus | Tutorials | Duration |
|-------|-------|-----------|----------|
| 0 | Installation | 000_Install_webots | 30 min |
| 1 | Basic Control | 001, 002 | 2 hours |
| 2 | Encoders | 011, 012 | 3 hours |
| 3 | Touch/Distance | 021, 022 | 3 hours |
| 4 | IMU/Acceleration | 031, 032 | 3 hours |
| 5 | Vision | 041, 042 | 4 hours |
| 6 | LIDAR/SLAM | 051, 052 | 5 hours |
| 7 | 3D Perception | 061, 062 | 4 hours |
| 8 | Navigation | 071 | 3 hours |

**Total: ~27 hours of comprehensive robotics education**

---

## 🚀 Getting Started

### Step 1: Install ROS 2 Jazzy
Follow the detailed guide in [1_ROS2/00_Install/Readme.md](1_ROS2/00_Install/Readme.md)

### Step 2: Install Webots
Download from [https://cyberbotics.com/download](https://cyberbotics.com/download)

### Step 3: Choose Your Path
- **ROS 2 Fundamentals** → Start with `1_ROS2/` folder
- **Robot Simulation** → Start with `2_Webots/000_Install_webots/`

### Step 4: Run First Example
```bash
# Simple ROS 2 example
cd 1_ROS2/01_Publisher_Subscriber/src
python3 simple_publisher.py &
python3 simple_subscriber.py

# Simple Webots example
cd 2_Webots/001_simple_robot_controller/
# Open robot_controller.py in Webots
```

---

## 🎓 Learning Outcomes

After completing this course, you will be able to:

✅ Understand ROS 2 architecture and key concepts  
✅ Create publishers and subscribers for message passing  
✅ Implement services and actions in ROS 2  
✅ Define and use custom messages and services  
✅ Integrate multiple sensors into a single application  
✅ Process sensor data in real-time  
✅ Perform image processing and computer vision  
✅ Implement SLAM and autonomous navigation  
✅ Create launch files and configure complex systems  
✅ Debug and optimize robotics applications  

---

## 🤝 Contributing

This is an educational repository. Feel free to:
- Report issues and suggest improvements
- Create pull requests with enhancements
- Share your own robot projects and examples
- Contribute translated documentation

---

## 📜 License

This project is open source and available under the MIT License.

---

## 👤 Authors & Contributors

**Original Author:**
- **alfaXphoori** - Repository maintainer

**Contributors:**
- KSU Robotics Team
- Community contributors

---

## ❓ FAQ

**Q: What are the system requirements?**  
A: Ubuntu 22.04 LTS, Python 3.10+, ROS 2 Jazzy, and 4GB RAM minimum.

**Q: Can I run this on Windows/macOS?**  
A: ROS 2 runs best on Linux. For Windows, use WSL2. For macOS, some features may not work.

**Q: Do I need a physical robot?**  
A: No! All tutorials use Webots simulator. You can extend them to physical robots later.

**Q: How long does the course take?**  
A: Approximately 27 hours for complete coverage. You can learn at your own pace.

**Q: Are there prerequisites?**  
A: Basic Python programming knowledge helpful but not required.

---

## 📞 Support

- 📖 [Read the documentation](2_Webots/README.md)
- 🐛 [Report issues](https://github.com/alfaXphoori/ROS/issues)
- 💬 [Join discussions](https://github.com/alfaXphoori/ROS/discussions)

---

## 🎯 Project Roadmap

- ✅ Core ROS 2 tutorials (01-07)
- ✅ Webots simulation integration (001-071)
- ✅ Sensor knowledge documentation
- 🔄 Real robot integration (TurtleBot3)
- 🔄 Advanced manipulation (arm control)
- 🔄 Multi-robot coordination
- 🔄 ROS 2 deployment and containerization

---

**Happy Learning! 🚀 Start with [1_ROS2/00_Install/Readme.md](1_ROS2/00_Install/Readme.md) or [2_Webots/Readme.md](2_Webots/Readme.md)**

