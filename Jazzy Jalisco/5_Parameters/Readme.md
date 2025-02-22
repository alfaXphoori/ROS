## 🚀 Create Action in ROS 2

Using **Actions** in ROS 2 allows nodes to perform long-running tasks asynchronously, such as robotic arm movements or navigation.

---

## 📦 Creating an Action in `ce_robot_interfaces` Package

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
```

Verify the custom action structure:
```bash
ros2 interface show ce_robot_interfaces/action/CountUntil
```

---

## 🚀 Using Custom Action in a Server/Client

### ⚙️ Creating the Action Server
Navigate to the `ce_robot` package folder:
```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
```

Create a Python file for the server:
```bash
touch count_until_server.py
chmod +x count_until_server.py
```

Write the necessary Python code and test the file using:
```bash
./count_until_server.py
```

---

### 🔄 Creating the Action Client
Navigate to the `ce_robot` package folder:
```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
```

Create a Python file for the client:
```bash
touch count_until_client.py
chmod +x count_until_client.py
```

Write the necessary Python code and test the file using:
```bash
./count_until_client.py
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

Open a terminal and run the **Action Server**:
```bash
ros2 run ce_robot count_until_server
```

Open another terminal and send a request using the **Action Client**:
```bash
ros2 run ce_robot count_until_client 10 1.0
```

To verify action communication, list the available actions:
```bash
ros2 action list
```

Check feedback messages:
```bash
ros2 action info /count_until
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
         |--action
            |--CountUntil.action
      |--ce_robot
         |--count_until_server.py
         |--count_until_client.py
```

✅ **Setup Complete!** 🚀✨
