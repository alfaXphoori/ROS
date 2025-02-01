## 🚀 Create Parameter in ROS 2

Using **parameters** in ROS 2 allows nodes to store and retrieve configurable values at runtime.

---

## 📦 Creating a Parameterized Publisher Node

### 🛠️ Creating the Parameter Node
Navigate to the `ce_robot` package folder:
```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
```

Create a Python file for the parameterized publisher:
```bash
touch HwStatus_para_publish.py
chmod +x HwStatus_para_publish.py
```

Write the necessary Python code and test the file using:
```bash
./HwStatus_para_publish.py
```

---

### 📌 Updating `package.xml` & `setup.py`
Modify `package.xml` to include necessary dependencies ✏️
Update `setup.py` by adding the following under `console_scripts`:
```python
entry_points={
    'console_scripts': [
        "hw_para = ce_robot.HwStatus_para_publish:main",
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

### 🚀 Running and Testing the Parameterized Node

Open a terminal and run the **Publisher with Parameters**:
```bash
source ~/.bashrc
ros2 run ce_robot hw_para
```

Open another terminal and check the available parameters:
```bash
source ~/.bashrc
ros2 param list
```

To modify a parameter, first stop the running node, then restart it with new values:
```bash
ros2 run ce_robot hw_para --ros-args -p rb_name:="rb-ce" -p rb_no:=1789
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
         |--srv
            |--CalRectangle.srv
      |--ce_robot
         |--ce_robot
            |--HwStatus_para_publish.py
```

✅ **Setup Complete!** 🚀✨
