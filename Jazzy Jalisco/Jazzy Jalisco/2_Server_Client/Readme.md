
## 🚀 Create Server / Client Node in ROS 2

Setting up a **Server Node** involves creating a node to handle requests, while a **Client Node** sends requests to the server.

---

## 🔗 Server / Client Node

### ⚙️ Setting Up the Server Node
Navigate to the `ce_robot` folder:
```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
```

Create a Python file for the Server:
```bash
touch add_two_ints_server.py
chmod +x add_two_ints_server.py
```

Write the necessary Python code and test the file using:
```bash
./add_two_ints_server.py
```

---

### 🔄 Setting Up the Client Node
Navigate to the `ce_robot` folder:
```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
```

Create a Python file for the Client:
```bash
touch add_two_ints_client.py
chmod +x add_two_ints_client.py
```

Write the necessary Python code and test the file using:
```bash
./add_two_ints_client.py
```

---

### 📌 Updating `package.xml` & `setup.py`
Modify the `package.xml` file to include necessary dependencies ✏️
Then, update the `setup.py` file by adding the following lines under `console_scripts`:
```python
entry_points={
    'console_scripts': [
        "add_two_server = ce_robot.add_two_ints_server:main",
        "add_two_client = ce_robot.add_two_ints_client:main",
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

Open a terminal and run the **Server**:
```bash
source ~/.bashrc
ros2 run ce_robot add_two_server
```

Open another terminal and send a request using the **Client**:
```bash
source ~/.bashrc
ros2 run ce_robot add_two_client 10 20
```

To visualize the node connections, open another terminal and run:
```bash
source ~/.bashrc
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
            |--add_two_ints_server.py
            |--add_two_ints_client.py
```

✅ **Setup Complete!** 🚀✨
