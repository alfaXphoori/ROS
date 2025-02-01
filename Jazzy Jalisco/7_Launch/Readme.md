## 🚀 Create Launch in ROS 2

Using **Launch Files** in ROS 2 allows users to start multiple nodes simultaneously, simplifying the execution of complex applications.

---

## 📦 Creating a Launch Package

### 🛠️ Creating the `ce_robot_bootup` Package
Navigate to the `src` folder and create a new package:
```bash
cd ~/ros2_ws/src
ros2 pkg create ce_robot_bootup
```

Remove unnecessary directories and create a `launch` folder:
```bash
cd ce_robot_bootup
rm -rf include src
mkdir launch
```

Create a Python launch file:
```bash
cd launch
touch ce_boot_launch.py
chmod +x ce_boot_launch.py
```

---

### 📌 Updating `package.xml` & `CMakeLists.txt`
Modify `package.xml` by adding the following dependency:
```xml
<exec_depend>ce_robot</exec_depend>
```

Modify `CMakeLists.txt` by adding:
```cmake
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)
```

---

### 🔨 Building the Package with Colcon
Compile the package:
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_bootup --symlink-install
```

---

### 🚀 Running and Testing the Launch File

Open a terminal and launch all nodes:
```bash
source ~/.bashrc
ros2 launch ce_robot_bootup ce_boot_launch.py
```

Verify the running nodes:
```bash
source ~/.bashrc
ros2 node list
```
Expected nodes:
- `CalRect_sv`
- `HWStatus_para`
- `HWStatus_pub`

Verify the topics being published:
```bash
ros2 topic list
ros2 topic echo /hw_status
ros2 topic echo /hw_parameter
```

Verify the active services:
```bash
ros2 service list
ros2 service call /cal_rectangle ce_robot_interfaces/srv/CalRectangle "{length: 12.13, width: 4.9}"
```

---

### 🗂️ Directory Structure

```bash
|--ros2_ws
   |--build
   |--install
   |--log
   |--src
      |--ce_robot_bootup
         |--launch
            |--ce_boot_launch.py
      |--ce_robot_interfaces
         |--msg
            |--HardwareStatus.msg
         |--srv
            |--CalRectangle.srv
      |--ce_robot
         |--ce_robot
            |--HwStatus_para_publish.py
```

✅ **Setup Complete!** 🚀✨
