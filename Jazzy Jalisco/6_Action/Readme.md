## 🚀 Create Action in ROS 2

Using **Actions** in ROS 2 allows nodes to perform long-running tasks asynchronously, such as robotic arm movements or navigation.

---

## 📦 Creating an Action Package

### 🛠️ Creating the `ce_robot_actions` Package
Navigate to the `src` folder and create a new package:
```bash
cd ~/ros2_ws/src
ros2 pkg create ce_robot_actions --build-type ament_python --dependencies rclpy action_msgs
```

Remove unnecessary directories and create an `action` folder:
```bash
cd ce_robot_actions
rm -rf include src
mkdir action
```

Create an action definition file:
```bash
cd action
touch MoveRobot.action
code MoveRobot.action
```

Define the request, feedback, and response structure inside `MoveRobot.action`:
```plaintext
# Request
float64 distance
float64 speed
---
# Feedback
float64 current_distance
---
# Response
bool success
```

---

### 📌 Updating `CMakeLists.txt`
Modify `CMakeLists.txt` by adding:
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveRobot.action"
  DEPENDENCIES action_msgs
)
```

---

### 🔨 Building the Package with Colcon
Compile the package:
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_actions --symlink-install
```

Verify the custom action structure:
```bash
ros2 interface show ce_robot_actions/action/MoveRobot
```

---

## 🚀 Using Custom Action in a Server/Client

### ⚙️ Creating the Action Server
Navigate to the `ce_robot_actions` package folder:
```bash
cd ~/ros2_ws/src/ce_robot_actions
```

Create a Python file for the server:
```bash
touch move_robot_server.py
chmod +x move_robot_server.py
```

Write the necessary Python code and test the file using:
```bash
./move_robot_server.py
```

---

### 🔄 Creating the Action Client
Navigate to the `ce_robot_actions` package folder:
```bash
cd ~/ros2_ws/src/ce_robot_actions
```

Create a Python file for the client:
```bash
touch move_robot_client.py
chmod +x move_robot_client.py
```

Write the necessary Python code and test the file using:
```bash
./move_robot_client.py
```

---

### 📌 Updating `package.xml` & `setup.py`
Modify `package.xml` to include necessary dependencies ✏️
Update `setup.py` by adding the following under `console_scripts`:
```python
entry_points={
    'console_scripts': [
        "move_robot_server = ce_robot_actions.move_robot_server:main",
        "move_robot_client = ce_robot_actions.move_robot_client:main",
    ],
},
```

---

### 🔨 Building the Package with Colcon
Compile the package:
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_actions --symlink-install
```

---

### 🚀 Running and Testing the Action Server/Client

Open a terminal and run the **Action Server**:
```bash
source ~/.bashrc
ros2 run ce_robot_actions move_robot_server
```

Open another terminal and send a request using the **Action Client**:
```bash
source ~/.bashrc
ros2 run ce_robot_actions move_robot_client 5.0 1.0
```

To verify action communication, list the available actions:
```bash
ros2 action list
```

---

### 🗂️ Directory Structure

```bash
|--ros2_ws
   |--build
   |--install
   |--log
   |--src
      |--ce_robot_actions
         |--action
            |--MoveRobot.action
         |--move_robot_server.py
         |--move_robot_client.py
```

✅ **Setup Complete!** 🚀✨
