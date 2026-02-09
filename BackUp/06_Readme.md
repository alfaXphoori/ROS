# 🚀 ROS 2 Action and Parameterized Client/Server

This guide provides a comprehensive setup for using **ROS 2 Actions** with dynamic topic names and parameters. You will learn how to create, build, and run an action server and client that allow dynamic topic configuration using ROS 2 parameters.

---

## � Creating an Action in `ce_robot_interfaces`

### 🛠️ Creating the `action` Directory

```bash
cd ~/ros2_ws/src/ce_robot_interfaces
mkdir action
```

Create an action definition file:
```bash
cd action
touch CountUntil.action
code CountUntil.action
```

Define the action structure in `CountUntil.action`:

---

### 📌 Updating `CMakeLists.txt`
Modify `CMakeLists.txt` by adding:
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CountUntil.action"
  DEPENDENCIES action_msgs
)
```

---

### 🔨 Building the Package with Colcon
Compile the package:
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces --symlink-install
source install/setup.bash
```

Verify the custom action structure:
```bash
ros2 interface show ce_robot_interfaces/action/CountUntil
```

---

## 🚀 Using Custom Action in a Server/Client with Dynamic Topic Names

### ⚙️ Creating the Action Server with Dynamic Topic Name

Create a Python file for the server:
```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
touch count_until_server.py
chmod +x count_until_server.py
```

---

### 🔄 Creating the Action Client with Dynamic Topic Name

Create a Python file for the client:
```bash
touch count_until_client.py
chmod +x count_until_client.py
```

---

### 📌 Updating `package.xml` & `setup.py`
Modify `package.xml` to include necessary dependencies ✏️  
Update `setup.py` by adding the following under `console_scripts`:
```python
entry_points={
    'console_scripts': [
        "count_until_server = ce_robot.count_until_server:main",
        "count_until_client = ce_robot.count_until_client:main",
    ],
},
```

---

### 🔨 Building the Package with Colcon
Compile the package:
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

---

### 🚀 Running and Testing the Action Server/Client

Run the **server** with the default topic:
```bash
ros2 run ce_robot count_until_server
```

Run the **server** with a custom topic:
```bash
ros2 run ce_robot count_until_server --ros-args -p topic_name:="action_count_until"
```

Run the **client** with the default topic:
```bash
ros2 run ce_robot count_until_client 10 1.0
```

Run the **client** with a custom topic:
```bash
ros2 run ce_robot count_until_client 10 1.0 --ros-args -p topic_name:="action_count_until"
```

To verify parameters:
```bash
ros2 param list
```

---

### �️ Directory Structure

```bash
|--ros2_ws
   |--build
   |--install
   |--log
   |--src
      |--ce_robot_interfaces
         |--action
            |--CountUntil.action
      |--ce_robot
         |--ce_robot
            |--count_until_server.py
            |--count_until_client.py
```

✅ **Setup Complete!** 🚀✨
