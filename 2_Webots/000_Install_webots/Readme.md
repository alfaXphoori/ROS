# 🤖 Install Webots (ROS 2 Jazzy, Ubuntu)

> **A comprehensive guide to install Webots for use with ROS 2 Jazzy on Ubuntu**

---

## 📋 Prerequisites

- 🐧 **Ubuntu 22.04** (recommended for ROS 2 Jazzy)
- ✅ **ROS 2 Jazzy** installed ([ROS 2 Installation Guide](https://docs.ros.org/en/ros2/Installation.html))

---

## 📦 Install Webots

### Step 1️⃣ Add the Cyberbotics Repository

```bash
wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo tee /etc/apt/trusted.gpg.d/cyberbotics.asc
echo "deb https://cyberbotics.com/debian/ binary-amd64/" | sudo tee /etc/apt/sources.list.d/cyberbotics.list
```

### Step 2️⃣ Install Webots

```bash
sudo apt install webots
```
#### OR 
- Mauanl Install ([Webots Download]([https://docs.ros.org/en/ros2/Installation.html](https://drive.google.com/file/d/1Tfv30zlxfBkMNNqX0jkhqE105ZEgcDNq/view?usp=sharing)))
---

## 🔧 (Optional) Install Webots ROS 2 Interface

To use Webots seamlessly with ROS 2 Jazzy:

```bash
sudo apt update
sudo apt install ros-jazzy-webots-ros2
```

---

## 🚀 Launch Webots

```bash
webots
```

---

## 📦 Create ROS 2 Package `ce_webots`

Once Webots is installed, create the ROS 2 package for your Webots controllers:

### Step 1️⃣ Navigate to Your Workspace

```bash
cd ~/ros2_ws/src
```

### Step 2️⃣ Create the Package

```bash
ros2 pkg create --build-type ament_python ce_webots
```

### Step 3️⃣ Set Up Package Structure

```bash
cd ce_webots

# Create necessary directories
mkdir -p ce_webots/controllers
mkdir -p worlds
mkdir -p launch
mkdir -p docs
```

### Step 4️⃣ Update `package.xml`

Add the following dependencies to your `package.xml`:

```xml
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>tf2</depend>
<depend>tf2_ros</depend>
```

### Step 5️⃣ Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select ce_webots
source install/setup.bash
```

### Verify Installation

```bash
ros2 pkg list | grep ce_webots
```

---

##  Additional Resources

- 📖 [Webots Official Documentation](https://cyberbotics.com/doc/)
- 🔗 [Webots ROS 2 Integration](https://github.com/cyberbotics/webots_ros2)
- 🤝 [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)

---

## ⚠️ Troubleshooting

> **Encountering issues?** Try the following:
> - Ensure your system is fully up to date: `sudo apt update && sudo apt upgrade`
> - Verify ROS 2 Jazzy installation: `ros2 --version`
> - Check Webots version: `webots --version`
> - Consult the [Webots Community](https://github.com/cyberbotics/webots/issues) for known issues


---

## **👤 Authors**

- 🚀 [@alfaXphoori](https://www.github.com/alfaXphoori)

---

<div align="center">

**Made with ❤️ for the ROS 2 Community**

</div>
