## 🚀 Create Publisher / Subscriber Node in ROS 2

Setting up a **Publisher Node** involves creating a node to send data 📡, while a **Subscriber Node** listens for incoming data.

### 🛠️ Setting Up the Publisher Node
Open a terminal and navigate to the `ce_robot` folder inside the `ce_robot` package:
```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
```

Create a Python file for the Publisher:
```bash
touch first_publisher.py
chmod +x first_publisher.py
```

Write the necessary Python code and test the file using:
```bash
./first_publisher.py
```

---

### 📥 Setting Up the Subscriber Node
Navigate to the `ce_robot` folder:
```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
```

Create a Python file for the Subscriber:
```bash
touch first_subscriber.py
chmod +x first_subscriber.py
```

Write the necessary Python code and test the file using:
```bash
./first_subscriber.py
```

---

### 📌 Updating `package.xml` & `setup.py`
Modify the `package.xml` file to include necessary dependencies ✏️
Then, update the `setup.py` file by adding the following lines under `console_scripts`:
```python
entry_points={
    'console_scripts': [
        "first_pub = ce_robot.first_publisher:main",
        "first_sub = ce_robot.first_subscriber:main",
    ],
},
```

---

### 🔨 Building the Package with Colcon
Once the code is error-free, compile the package using `colcon build`:
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
```

---

### 🚀 Running and Testing the Package

Open a terminal and run the **Publisher**:
```bash
ros2 run ce_robot first_pub
```

Open another terminal and run the **Subscriber**:
```bash
ros2 run ce_robot first_sub
```

To visualize the node connections, open another terminal and run:
```bash
rqt_graph
```

---

### 🗂️ Directory Structure

```bash
|--ros2_ws
   |--build
   |--install
   |--log
   |--src
      |--ce_robot
         |--ce_robot
            |--first_node.py
            |--first_publisher.py
            |--first_subscriber.py
```

✅ **Setup Complete!** 🚀✨
