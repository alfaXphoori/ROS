## 🚀 Create Custom Message (Msg) in ROS 2

Setting up a **custom message** allows communication between nodes using user-defined data structures.

---

## 📦 Creating a Custom Msg Package

### 🛠️ Creating the `ce_robot_interfaces` Package

Navigate to the `src` folder and create a new package:

```bash
cd ~/ros2_ws/src
ros2 pkg create ce_robot_interfaces
```

Remove unnecessary folders:

```bash
cd ce_robot_interfaces
rm -rf include/
rm -rf src
```

Create a folder for storing message definitions:

```bash
mkdir msg
code .
```

---

### 📌 Updating `package.xml` & `CMakeLists.txt`

Modify `package.xml` by adding the following lines under `<buildtool_depend>`:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Modify `CMakeLists.txt` by adding:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
)
```

---

## 📝 Defining the Custom Message

Navigate to the `msg` folder and create a new message file:

```bash
cd msg
touch HardwareStatus.msg
```

Define message variables inside `HardwareStatus.msg`, such as:

```plaintext
int32 temperature
bool is_operational
string message
```

---

### 🔨 Building the Package with Colcon

Compile the custom message package:

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
```

Verify the custom message structure:

```bash
ros2 interface show ce_robot_interfaces/msg/HardwareStatus
```

---

## 🚀 Using Custom Message in a Publisher

### 📡 Creating the Publisher Node

Navigate to the `ce_robot` package folder:

```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
```

Create a Python file for the publisher:

```bash
touch HardwareStatus_publish.py
chmod +x HardwareStatus_publish.py
```

Write the necessary Python code and test the file using:

```bash
./HardwareStatus_publish.py
```

---

### 📌 Updating `package.xml` & `setup.py`

Modify `package.xml` to include necessary dependencies ✏️
Update `setup.py` by adding the following under `console_scripts`:

```python
entry_points={
    'console_scripts': [
        "hw_status = ce_robot.HardwareStatus_publish:main",
    ],
},
```

---

### 🔨 Building the Package with Colcon

Compile the package:

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
```

---

### 🚀 Running and Testing the Publisher

Open a terminal and run the **Publisher**:

```bash
source ~/.bashrc
ros2 run ce_robot hw_status
```

Open a new terminal and echo the topic to verify message transmission:

```bash
ros2 topic echo /hardware_status
```

---

### 🗂️ Directory Structure

```bash
|--ros2_ws
   |--build
   |--install
   |--log
   |--src
      |--ce_robot_interfaces
         |--msg
            |--HardwareStatus.msg
      |--ce_robot
         |--ce_robot
            |--HardwareStatus_publish.py
```

✅ **Setup Complete!** 🚀✨
