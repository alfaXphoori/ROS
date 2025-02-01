## 🚀 Create Custom Service (Srv) in ROS 2

Setting up a **custom service** allows communication between nodes using user-defined request-response structures.

---

## 📦 Creating a Custom Srv Package

### 🛠️ Creating the `ce_robot_interfaces` Package
Navigate to the `src` folder and create a service folder:
```bash
cd ~/ros2_ws/src/ce_robot_interfaces
mkdir srv
```

Create a new service definition file:
```bash
cd srv
touch CalRectangle.srv
code .
```

Define the request and response structure inside `CalRectangle.srv`:
```plaintext
float64 length
float64 width
---
float64 area
```

---

### 📌 Updating `CMakeLists.txt`
Modify `CMakeLists.txt` by adding:
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CalRectangle.srv"
)
```

---

### 🔨 Building the Package with Colcon
Compile the custom service package:
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
```

Verify the custom service structure:
```bash
ros2 interface show ce_robot_interfaces/srv/CalRectangle
```

---

## 🚀 Using Custom Service in a Server/Client

### ⚙️ Creating the Server Node
Navigate to the `ce_robot` package folder:
```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
```

Create a Python file for the server:
```bash
touch CalRect_server.py
chmod +x CalRect_server.py
```

Write the necessary Python code and test the file using:
```bash
./CalRect_server.py
```

---

### 🔄 Creating the Client Node
Navigate to the `ce_robot` package folder:
```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
```

Create a Python file for the client:
```bash
touch CalRect_client.py
chmod +x CalRect_client.py
```

Write the necessary Python code and test the file using:
```bash
./CalRect_client.py
```

---

### 📌 Updating `package.xml` & `setup.py`
Modify `package.xml` to include necessary dependencies ✏️
Update `setup.py` by adding the following under `console_scripts`:
```python
entry_points={
    'console_scripts': [
        "cal_rect_server = ce_robot.CalRect_server:main",
        "cal_rect_client = ce_robot.CalRect_client:main",
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

### 🚀 Running and Testing the Server/Client

Open a terminal and run the **Server**:
```bash
source ~/.bashrc
ros2 run ce_robot cal_rect_server
```

Open another terminal and send a request using the **Client**:
```bash
source ~/.bashrc
ros2 run ce_robot cal_rect_client 22.22 33.34
```

To verify service communication, call it manually:
```bash
ros2 service call /cal_rect ce_robot_interfaces/srv/CalRectangle "{length: 5.20, width: 3.12}"
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
         |--srv
            |--CalRectangle.srv
      |--ce_robot
         |--ce_robot
            |--CalRect_server.py
            |--CalRect_client.py
```

✅ **Setup Complete!** 🚀✨
