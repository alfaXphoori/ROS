# 🤖 Install Webots for ROS 2 Jazzy

<div align="center">

**A comprehensive guide to set up Webots with ROS 2 Jazzy on Ubuntu 22.04**

[![Webots](https://img.shields.io/badge/Webots-2023b+-blue?style=flat-square&logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAyNCAyNCI+PC9zdmc+)]()
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-34aadc?style=flat-square)]()
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04%20LTS-E95420?style=flat-square&logo=ubuntu)]()

</div>

---

## 📋 Prerequisites

Before you begin, ensure you have:

| Item | Status | Link |
|------|--------|------|
| 🐧 **Ubuntu 22.04 LTS** | Required | [Download](https://ubuntu.com/download/desktop) |
| ✅ **ROS 2 Jazzy** | Required | [Installation Guide](https://docs.ros.org/en/jazzy/Installation.html) |
| 💾 **~2GB Disk Space** | Required | — |

> **Tip:** To verify your ROS 2 installation:
> ```bash
> ros2 --version
> ```

---

## 🚀 Installation Steps

### Step 1️⃣ Add Cyberbotics Repository

Add the official Webots package repository to your system:

```bash
# Add GPG key
wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo tee /etc/apt/trusted.gpg.d/cyberbotics.asc

# Add repository
echo "deb https://cyberbotics.com/debian/ binary-amd64/" | sudo tee /etc/apt/sources.list.d/cyberbotics.list

# Update package list
sudo apt update
```

### Step 2️⃣ Install Webots

Choose one of the following installation methods:

**Option A: Via Package Manager (Recommended)** ✨

```bash
sudo apt install webots
```

**Option B: Manual Download** 📥

Download from the official source: [Webots Download](https://drive.google.com/file/d/1Tfv30zlxfBkMNNqX0jkhqE105ZEgcDNq/view?usp=sharing)

Then extract and install:
```bash
tar -xzf webots-*.tar.gz -C ~/
~/webots/install.sh
```

### Step 3️⃣ (Optional) Install ROS 2 Webots Bridge

For seamless ROS 2 integration, install the Webots ROS 2 interface package:

```bash
sudo apt install ros-jazzy-webots-ros2
```

This enables:
- 🔗 Direct topic communication between Webots and ROS 2
- 🎮 Native ROS 2 message types in simulations
- 📦 Pre-built Webots ROS 2 examples

---

## ✅ Verify Installation

Confirm everything is installed correctly:

```bash
# Check Webots installation
webots --version

# Check ROS 2 installation
ros2 --version

# List available Webots packages (if using ROS 2 bridge)
ros2 pkg list | grep webots
```

Expected output:
```
Webots: 2023b or later
ROS 2: Jazzy (or later)
```

---

## 🎮 Launch Webots

Start the Webots simulator:

```bash
webots
```

Or launch with specific world file:
```bash
webots ~/path/to/world/file.wbt
```

---

---

## 📦 Create Your First ROS 2 Webots Package

After installation, set up your ROS 2 workspace for Webots projects:

### Step 1️⃣ Navigate to ROS 2 Workspace

```bash
cd ~/ros2_ws/src
```

### Step 2️⃣ Create New Package

```bash
ros2 pkg create --build-type ament_python ce_webots
cd ce_webots
```

### Step 3️⃣ Set Up Directory Structure

```bash
# Create essential directories
mkdir -p ce_webots/controllers
mkdir -p worlds
mkdir -p launch
mkdir -p docs
mkdir -p config
```

Directory structure:
```
ce_webots/
├── ce_webots/
│   ├── __init__.py
│   └── controllers/          # Webots robot controllers
├── worlds/                   # .wbt world files
├── launch/                   # ROS 2 launch files
├── config/                   # Configuration files
├── package.xml
├── setup.py
└── README.md
```

### Step 4️⃣ Update Dependencies in `package.xml`

Add these essential dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ce_webots</name>
  <version>0.0.1</version>
  <description>ROS 2 Webots Integration Package</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake_python</buildtool_depend>
  <build_depend>rclpy</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>nav_msgs</build_depend>
  <build_depend>tf2</build_depend>
  <build_depend>tf2_ros</build_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>tf2</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>webots-ros2</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Step 5️⃣ Build the Package

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build the package
colcon build --packages-select ce_webots

# Source the setup script
source install/setup.bash
```

### Step 6️⃣ Verify Setup

```bash
# Check if package is available
ros2 pkg list | grep ce_webots

# Expected output: ce_webots
```

---

## 🔗 Additional Resources

<table>
  <tr>
    <td><a href="https://cyberbotics.com/doc/">📖 Webots Documentation</a></td>
    <td><a href="https://github.com/cyberbotics/webots_ros2">🤖 Webots ROS 2</a></td>
  </tr>
  <tr>
    <td><a href="https://docs.ros.org/en/jazzy/">🔗 ROS 2 Jazzy Docs</a></td>
    <td><a href="https://ubuntu.com/tutorials">🐧 Ubuntu Tutorials</a></td>
  </tr>
</table>

---

## ⚠️ Troubleshooting

| Problem | Solution |
|---------|----------|
| **Webots won't start** | Update drivers: `sudo apt update && sudo apt upgrade` |
| **ROS 2 not found** | Source setup: `source /opt/ros/jazzy/setup.bash` |
| **Permission denied** | Use `sudo` for apt commands or check file permissions |
| **Out of disk space** | Free up space: Webots needs ~2GB |
| **Package not found** | Rebuild: `colcon build --packages-select ce_webots` |

**Need more help?** Check:
- 🐛 [Webots Issues](https://github.com/cyberbotics/webots/issues)
- 💬 [ROS Discourse](https://discourse.ros.org/)
- 📚 [ROS 2 Troubleshooting](https://docs.ros.org/en/jazzy/Installation.html)

---

## 🎯 Next Steps

Once installation is complete:

1. ✅ **Explore Examples** — Check out the Webots tutorials
2. ✅ **Create Controllers** — Write your first Python robot controller
3. ✅ **Build Worlds** — Design custom Webots environments
4. ✅ **ROS 2 Integration** — Publish/Subscribe to robot topics
5. ✅ **Advanced Projects** — Implement SLAM, navigation, and more

**Ready to start?** 👉 Check out [1_ROS2 Tutorials](../../1_ROS2/README.md) and the [Webots Controllers Guide](../Readme.md)

---

<div align="center">

### 📞 Support

| Channel | Link |
|---------|------|
| **Webots Community** | [https://github.com/cyberbotics/webots](https://github.com/cyberbotics/webots) |
| **ROS 2 Support** | [https://discourse.ros.org/](https://discourse.ros.org/) |
| **GitHub Issues** | [Report Problems](https://github.com/alfaXphoori/ROS/issues) |

---

**Made with ❤️ by the ROS Community**

</div>
