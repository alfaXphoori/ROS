# 🚀 ROS 2 Gazebo Simulation Lab

This lab will guide you through setting up **Gazebo** with ROS 2 for simulating robots in a physics-based environment.

---

## 📦 Installing Gazebo for ROS 2

### 🛠️ Install Gazebo for ROS 2
For **Gazebo Classic**:
```bash
sudo apt update
sudo apt install ros-jazzy-gazebo-ros-pkgs
```

For **Ignition Gazebo (Gazebo Sim)**:
```bash
sudo apt install ros-jazzy-ros-gz
```

Verify installation:
```bash
gz sim --help
```

Check the installed Gazebo version:
```bash
gz --version
```

---

## 🚀 Running a Simple Robot Simulation

### 1️⃣ Launch Gazebo with an Empty World
```bash
ros2 launch gazebo_ros gazebo.launch.py
```

### 2️⃣ Spawn a TurtleBot4 in Gazebo
Install the TurtleBot4 package:
```bash
sudo apt install ros-jazzy-turtlebot4*
```

Launch TurtleBot4 simulation:
```bash
ros2 launch turtlebot4_gazebo turtlebot4_playground.launch.py
```

Check if the robot model is loaded:
```bash
ros2 topic list | grep /model
```

---

## 🤖 Controlling the Simulated Robot

### 1️⃣ Control the Robot via Teleoperation
Use the keyboard teleop node to send movement commands:
```bash
ros2 run turtlebot4_teleop teleop_keyboard
```

### 2️⃣ Send Velocity Commands Programmatically
Publish velocity commands using ROS 2 topics:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{ linear: {x: 0.5}, angular: {z: 0.3} }" -r 10
```
This command makes the robot **move forward and turn** simultaneously.

### 3️⃣ Retrieve Sensor Data
To check **LiDAR sensor readings**:
```bash
ros2 topic echo /scan
```
To check **camera feed**:
```bash
ros2 topic echo /camera/image_raw
```

### 4️⃣ Monitor Robot State
List all active nodes:
```bash
ros2 node list
```
Check the robot’s transform frames:
```bash
ros2 run tf2_tools view_frames
```

---

### 📌 Updating `package.xml` & `CMakeLists.txt`
Modify `package.xml` by adding:
```xml
<exec_depend>gazebo_ros</exec_depend>
```

Modify `CMakeLists.txt` by adding:
```cmake
find_package(gazebo_ros REQUIRED)
```

---

### 🔨 Building the Package with Colcon
Compile the package:
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_gazebo --symlink-install
```

Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

---

### 🚀 Running and Testing the Simulation

Open a terminal and launch Gazebo:
```bash
source ~/.bashrc
ros2 launch ce_robot_gazebo robot_world.launch.py
```

To check available nodes:
```bash
ros2 node list
```

To verify topics being published:
```bash
ros2 topic list
```

To debug simulation issues:
```bash
gz log -v 4   # View detailed Gazebo logs
```

To restart a simulation cleanly:
```bash
killall -9 gzserver gzclient
ros2 launch gazebo_ros gazebo.launch.py
```

---

### 🗂️ Directory Structure

```bash
|--ros2_ws
   |--build
   |--install
   |--log
   |--src
      |--ce_robot_gazebo
         |--launch
            |--robot_world.launch.py
```

✅ **Setup Complete!** 🚀✨
