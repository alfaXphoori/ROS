# **ROS2 Publisher & Subscriber Nodes**

## **📌 Project Title**

Create Publisher and Subscriber Nodes in ROS 2 for Asynchronous Communication

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

## **🛠 Overview**

This guide demonstrates **Topic-based communication** in ROS 2:
- **Publisher Node** - Sends data continuously to a topic 📡
- **Subscriber Node** - Listens to incoming data from a topic 👂
- Asynchronous communication pattern (one-to-many)
- Ideal for continuous data streaming

---

## **📊 Architecture**

```
┌──────────────────────────────────────┐
│      Publisher Node                  │
│   (Publishes Data to Topic)          │
└────────────┬─────────────────────────┘
             │
        Topic: /chatter
             │
     ┌───────┴────────┐
     │                │
┌────▼──────────┐  ┌─▼──────────────┐
│ Subscriber 1  │  │ Subscriber 2   │
│  (Listener)   │  │  (Listener)    │
└───────────────┘  └────────────────┘
```

---

## **🛠️ Setting Up the Publisher Node**

Navigate to the `ce_robot` package directory:

```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
```

Create a Python file for the Publisher:

```bash
touch first_publisher.py
chmod +x first_publisher.py
```

**Directory Structure:**
```
📁 ros2_ws/
└── 📁 src/
    └── 📁 ce_robot/
        └── 📁 ce_robot/
            ├── 📄 __init__.py
            └── 🐍 first_publisher.py    ← Create this file
```

![Publisher File](imgs/1_pub.png)

Write the Python code for the publisher:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

![Publisher Code](imgs/1_pub_code.png)

Test the publisher:

```bash
./first_publisher.py
```

![Publisher Running](imgs/1_pub_run.png)

---

## **📥 Setting Up the Subscriber Node**

Navigate to the `ce_robot` folder:

```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
```

Create a Python file for the Subscriber:

```bash
touch first_subscriber.py
chmod +x first_subscriber.py
```

**Directory Structure:**
```
📁 ros2_ws/
└── 📁 src/
    └── 📁 ce_robot/
        └── 📁 ce_robot/
            ├── 📄 __init__.py
            ├── 🐍 first_publisher.py
            └── 🐍 first_subscriber.py    ← Create this file
```

Write the Python code for the subscriber:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Test the subscriber:

```bash
./first_subscriber.py
```

![Subscriber Running](imgs/1_sub_run.png)

---

## **📌 Updating `package.xml` & `setup.py`**

### **1. Modify `package.xml`**

Add required dependencies:

```xml
<exec_depend>std_msgs</exec_depend>
<build_depend>std_msgs</build_depend>
```

### **2. Update `setup.py`**

Add the following lines under `console_scripts`:

```python
entry_points={
    'console_scripts': [
        '00_first_node = ce_robot.first_node:main',
        "01_first_pub = ce_robot.first_publisher:main",
        "01_first_sub = ce_robot.first_subscriber:main",
        "02_add_two_server = ce_robot.add_two_ints_server:main",
        "02_add_two_client = ce_robot.add_two_ints_client:main",
        "01_simple_publisher = ce_robot.simple_publisher:main",
        "01_simple_subscriber = ce_robot.simple_subscriber:main",
        "01_counter_pubslisher = ce_robot.counter_publisher:main",
        "01_counter_processor = ce_robot.counter_processor:main",
        "01_counter_logger = ce_robot.counter_logger:main",
        "01_temperature_publisher = ce_robot.temperature_publisher:main",
        "01_temperature_subsciber = ce_robot.temperature_subscriber:main",
        "01_sensor_monitor = ce_robot.sensor_monitor:main",
        "01_pressure_publisher = ce_robot.pressure_publisher:main",
        "01_humidity_publisher = ce_robot.humidity_publisher:main",
        "02_temp_converter_server = ce_robot.temp_converter_server:main",
        "02_temp_converter_client = ce_robot.temp_converter_client:main",
        "02_database_server = ce_robot.database_server:main",
        "02_database_client = ce_robot.database_client:main",
        "02_robot_controller_server = ce_robot.robot_controller_server:main",
        "02_robot_controller_client = ce_robot.robot_controller_client:main",
        "03_hw_status_publisher = ce_robot.HardwareStatus_publish:main",
        "03_hw_status_subscriber = ce_robot.HardwareStatus_subscribe:main", 
        "03_hw_status_aggregator = ce_robot.HardwareStatus_aggregate:main",
        "03_robot_status_publisher = ce_robot.RobotStatus_publisher:main",
        "03_robot_status_safety_monitor = ce_robot.RobotStatus_safety_monitor:main",
        "04_CalRect_server = ce_robot.CalRect_server:main",
        "04_CalRect_client = ce_robot.CalRect_client:main",
        "04_navigate_server = ce_robot.navigate_to_position_server:main",
        "04_navigate_client = ce_robot.navigate_to_position_client:main",
        "04_gripper_server = ce_robot.gripper_control_server:main",
        "04_gripper_client = ce_robot.gripper_control_client:main",
        "05_robot_tag = ce_robot.robot_tag_publisher:main",
        "05_robot_tag_param = ce_robot.robot_tag_param_pub:main",
        "05_robot_tag_callback = ce_robot.robot_tag_callback_pub:main",
        "05_robot_tag_validated = ce_robot.robot_tag_validated_pub:main",
        "06_count_until_server = ce_robot.count_until_server:main",
        "06_count_until_client = ce_robot.count_until_client:main",
        "06_battery_charging_server = ce_robot.battery_charging_server:main",
        "06_battery_charging_client = ce_robot.battery_charging_client:main",
        "06_navigate_server = ce_robot.navigate_server:main",
        "06_navigate_client = ce_robot.navigate_client:main",
        "06_gripper_server = ce_robot.gripper_server:main",
        "06_gripper_client = ce_robot.gripper_client:main",
    ],
},
```

---

## **🔨 Building the Package with Colcon**

Once the code is error-free, compile the package:

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
```

![Colcon Build](imgs/1_build.png)

---

## **🚀 Running and Testing the Package**

### **Step 1: Start the Publisher Node**

Open a terminal and run:

```bash
ros2 run ce_robot first_pub
```

You should see:
```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
```

### **Step 2: Start the Subscriber Node**

Open another terminal and run:

```bash
ros2 run ce_robot first_sub
```

You should see:
```
[INFO] [minimal_subscriber]: I heard: "Hello World: 0"
[INFO] [minimal_subscriber]: I heard: "Hello World: 1"
```

![Publisher & Subscriber Running](imgs/1_run_pub_sub.png)

### **Step 3: Monitor Node Connections**

Open another terminal to visualize the node graph:

```bash
rqt_graph
ros2 topic list
ros2 topic info /topic
```

![RQT Graph](imgs/1_rqt_graph.png)

---

## **🔍 Topic Inspection Commands**

### **List Available Topics**
```bash
ros2 topic list
```

### **View Topic Type**
```bash
ros2 topic type /topic
```

### **View Topic Data**
```bash
ros2 topic echo /topic
```

### **View Topic Information**
```bash
ros2 topic info /topic --verbose
```

### **Measure Topic Bandwidth**
```bash
ros2 topic bw /topic
```

---

## **📂 Directory Structure**

```
📁 ROS2_WS/
├── 📁 .vscode/
├── 📁 build/
├── 📁 install/
├── 📁 log/
└── 📁 src/
    ├── 📁 .vscode/
    └── 📁 ce_robot/
        ├── 📁 ce_robot/
        │   ├── 📄 __init__.py
        │   ├── 🐍 first_node.py
        │   ├── 🐍 first_publisher.py
        │   └── 🐍 first_subscriber.py
        ├── 📁 resource/
        │   └── 📄 ce_robot
        ├── 📁 test/
        ├── 📄 package.xml
        ├── 📄 setup.cfg
        └── 📄 setup.py
```

---

## **🎯 Key Concepts**

### **Publish-Subscribe Pattern**
- Publisher sends data to a topic (producer)
- Subscriber receives data from a topic (consumer)
- Asynchronous communication (non-blocking)
- One publisher, multiple subscribers supported
- Decoupled: publisher and subscriber don't know about each other

### **Quality of Service (QoS)**
- History: Keep last N messages (10 in examples)
- Reliability: Best effort or reliable delivery
- Durability: Volatile or transient data

### **Advantages**
- ✅ Asynchronous communication
- ✅ Loose coupling between nodes
- ✅ One-to-many communication
- ✅ Easy to scale
- ✅ Continuous data streaming

### **Disadvantages**
- ❌ Subscribers may miss messages if not listening
- ❌ No confirmation that data was received
- ❌ Harder to debug

---

## **🛠 Customization Ideas**

### **1. Different Message Types**
```python
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
```

### **2. Multiple Publishers on Same Topic**
```python
self.publisher_ = self.create_publisher(String, 'shared_topic', 10)
```

### **3. Multiple Subscribers**
```python
self.sub1 = self.create_subscription(String, 'topic1', self.callback1, 10)
self.sub2 = self.create_subscription(String, 'topic2', self.callback2, 10)
```

### **4. Message Filtering**
```python
if msg.data.startswith('important'):
    self.process_message(msg)
```

### **5. Data Transformation**
```python
def listener_callback(self, msg):
    transformed = msg.data.upper()
    self.publish_transformed(transformed)
```

---

## **🔗 Related Topics**

- Services & Clients (synchronous communication)
- Actions (long-running tasks with feedback)
- Custom Message Types (`.msg` files)
- Parameter Server
- ROS 2 Launch Files
- Advanced Pub/Sub Patterns

---

**✅ Publisher & Subscriber Setup Complete!** 🚀✨
