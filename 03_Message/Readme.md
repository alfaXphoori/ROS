# **ROS 2 Custom Messages (Msg)**

## **📌 Project Title**

Create Custom Message Types in ROS 2 for Structured Data Communication

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

## **🛠 Overview**

This guide demonstrates **custom message creation and usage** in ROS 2:
- **Message Package** - Dedicated package for message definitions
- **Custom Types** - User-defined data structures for specialized communication
- **Publisher/Subscriber** - Asynchronous communication using custom messages
- **Data Aggregation** - Collecting and analyzing multi-field messages
- **Validation** - Error handling and input validation

---

## **📊 Architecture**

```
┌──────────────────────────────────────────────────────┐
│         Message Definition (HardwareStatus.msg)      │
│  - name_robot: string                                │
│  - number_robot: int64                               │
│  - temperature: int64                                │
│  - motor_ready: bool                                 │
│  - debug_message: string                             │
└──────────────────┬───────────────────────────────────┘
                   │
        ┌──────────┴──────────┐
        │                     │
        ▼                     ▼
┌────────────────┐    ┌────────────────┐
│ Publisher Node │    │ Subscriber Node│
│  (Sends Data)  │───▶│  (Receives)    │
└────────────────┘    └────────────────┘
```

![Architecture Diagram](./imgs/03_Robot.png)

---

## **Understanding Message Types 📖**

ROS 2 provides built-in message types through standard packages like `std_msgs`, `geometry_msgs`, and `sensor_msgs`. These cover common data structures such as strings, integers, floats, and geometric data.

### 📝 Task
1. List all available built-in messages:
```bash
ros2 interface list | grep "std_msgs"
```

2. Examine message structures:
```bash
ros2 interface show std_msgs/msg/String
ros2 interface show std_msgs/msg/Int32
ros2 interface show geometry_msgs/msg/Point
```

3. Create a comparison document listing 5 built-in message types with their use cases

### 💡 Key Concepts
- **Built-in messages** provided by standard ROS 2 packages
- **Message types**: basic types (int, float, bool), collections, and specialized types
- **When to use custom messages**: when built-in types don't match your data structure
- **Naming conventions**: PascalCase for message names (e.g., `HardwareStatus`)

### 🔍 Expected Output
```
Comparison of Message Types:
- std_msgs/String: Simple text data (use for logging, notifications)
- std_msgs/Int32: 32-bit integer (use for counters, IDs)
- geometry_msgs/Point: 3D point with x, y, z (use for positions, coordinates)
- sensor_msgs/Temperature: Temperature reading with timestamp (use for sensor data)
- diagnostic_msgs/DiagnosticStatus: System diagnostics (use for health monitoring)
```

---

## **⚙️ Creating the Message Package**

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

## **📝 Defining the Custom Message**

Navigate to the `msg` folder and create a new message file:

```bash
cd ~/ros2_ws/src/ce_robot_interfaces/msg
touch HardwareStatus.msg
```

Define message variables inside `HardwareStatus.msg`:

```msg
string name_robot          # Robot identifier/name
int64 number_robot         # Robot ID number (1-1000)
int64 temperature          # Temperature in Celsius (0-100)
bool motor_ready           # Motor operational status
string debug_message       # Status message for debugging
```

**Key Points:**
- Message files use the `.msg` extension
- Each line defines a data member with type and name
- Comments explain field purpose
- Common types: `uint8`, `uint16`, `uint32`, `int32`, `float32`, `float64`, `string`, `bool`
- Arrays: use `type[size]` syntax (e.g., `uint8[3]` for 3-byte array)

---

## **🔨 Building the Package with Colcon**

Once the package is configured, compile it:

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
source install/setup.bash
```

Verify the message structure:

```bash
ros2 interface show ce_robot_interfaces/msg/HardwareStatus
```

---

## **🚀 Using Custom Messages in Publisher/Subscriber**

## **🔄 Publisher Implementation**

Navigate to the `ce_robot` package folder:

```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
```

Create the publisher file `HardwareStatus_publish.py`:

```bash
chmod +x HardwareStatus_publish.py
```

**File: HardwareStatus_publish.py**

```python
#!/usr/bin/env python3
"""
Hardware Status Publisher
Publishes custom HardwareStatus messages at regular intervals
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus


class HwStatusNode(Node):
    def __init__(self):
        super().__init__("hardwarestatus_publish")
        self.robot_name_ = "CE-ROBOT"
        self.robot_number_ = 1001
        self.hw_status_publish_ = self.create_publisher(
            HardwareStatus, "hardware_status", 10
        )
        self.timer_ = self.create_timer(1.0, self.publish_hw_status)
        self.get_logger().info("Hw_Status_Publish_Node Start Now!")

    def publish_hw_status(self):
        """Publish hardware status message"""
        import random
        msg = HardwareStatus()
        msg.name_robot = self.robot_name_
        msg.number_robot = self.robot_number_
        msg.temperature = random.randint(35, 60)  # Random temperature 35-60°C
        msg.motor_ready = True
        msg.debug_message = "Motor 1"
        self.hw_status_publish_.publish(msg)
        self.get_logger().info(
            f"Published: Robot={msg.name_robot}, "
            f"Number={msg.number_robot}, "
            f"Temp={msg.temperature}°C, "
            f"Motor={msg.motor_ready}, "
            f"Message={msg.debug_message}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = HwStatusNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## **📌 Updating `package.xml` & `setup.py`**

### **1. Modify `package.xml`**

Add the required dependencies:

```xml
<depend>ce_robot_interfaces</depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
```

### **2. Update `setup.py`**

Add the following lines under `console_scripts`:

```python
entry_points={
    'console_scripts': [
        "03_hw_status_publisher = ce_robot.HardwareStatus_publish:main",
        "03_hw_status_subscriber = ce_robot.HardwareStatus_subscribe:main",
        "03_hw_status_aggregator = ce_robot.HardwareStatus_aggregate:main",
    ],
},
```

---

## **🔨 Building the Package with Colcon**

Compile the package:

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
```

---

## **🚀 Running and Testing**

### **Step 1: Terminal 1 - Start Publisher**

```bash
source ~/.bashrc
ros2 run ce_robot 03_hw_status_publisher
```

You should see:
```
[INFO] [hardware_status_publisher]: Published: Robot=CE-ROBOT, Number=1001, Temp=47°C, Motor=True, Message=Motor 1
```
(Temperature will vary between 35-60°C on each publication)

![Publisher Output](./imgs/01_Convert_Temp.png)

---

## **🔄 Subscriber Implementation**

**File: HardwareStatus_subscribe.py**

```python
#!/usr/bin/env python3
"""
Hardware Status Subscriber
Subscribes to hardware_status and displays received messages
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus


class HardwareStatusSubscriber(Node):
    def __init__(self):
        super().__init__('hardware_status_subscriber')
        self.subscriber = self.create_subscription(
            HardwareStatus, 'hardware_status', self.status_callback, 10
        )
        self.received_count = 0
        self.get_logger().info('Hardware Status Subscriber started!')

    def status_callback(self, msg):
        """Callback function to process received messages"""
        self.received_count += 1
        self.get_logger().info(
            f'[{self.received_count}] Received: {msg.name_robot} | '
            f'Number: {msg.number_robot} | '
            f'Temp: {msg.temperature}°C | '
            f'Motor: {msg.motor_ready} | '
            f'Message: {msg.debug_message}'
        )


def main(args=None):
    rclpy.init(args=args)
    subscriber = HardwareStatusSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

### **Step 2: Terminal 2 - Start Subscriber**

```bash
source ~/.bashrc
ros2 run ce_robot 03_hw_status_subscriber
```

### **Step 3: Terminal 3 - Monitor Topic**

```bash
ros2 topic echo /hardware_status
```

You should see messages displayed:
```
name_robot: CE-ROBOT
number_robot: 1001
temperature: 47
motor_ready: true
debug_message: Motor 1
---
name_robot: CE-ROBOT
number_robot: 1001
temperature: 52
motor_ready: true
debug_message: Motor 1
---
```
(Temperature values will vary between 35-60°C)

![Topic Echo Output](./imgs/02_Database.png)

### **Step 4: Advanced - Run Aggregator**

**File: HardwareStatus_aggregate.py**

```python
#!/usr/bin/env python3
"""
Hardware Status Aggregator
Collects and aggregates hardware status messages
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus


class HardwareStatusAggregator(Node):
    def __init__(self):
        super().__init__('hardware_status_aggregator')
        self.subscriber = self.create_subscription(
            HardwareStatus, 'hardware_status', self.status_callback, 10
        )
        self.received_count = 0
        self.total_temperature = 0
        self.get_logger().info('Hardware Status Aggregator started!')

    def status_callback(self, msg):
        """Callback function to aggregate messages"""
        self.received_count += 1
        self.total_temperature += msg.temperature
        avg_temp = self.total_temperature / self.received_count
        
        self.get_logger().info(
            f'[Aggregated] Count: {self.received_count} | '
            f'Robot: {msg.name_robot} | '
            f'Avg Temp: {avg_temp:.1f}°C | '
            f'Current Temp: {msg.temperature}°C'
        )


def main(args=None):
    rclpy.init(args=args)
    aggregator = HardwareStatusAggregator()
    rclpy.spin(aggregator)
    aggregator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Terminal 2 alternative - Run aggregator instead:
```bash
ros2 run ce_robot 03_hw_status_aggregator
```

You should see periodic aggregated statistics output:
```
[INFO] [hardware_status_aggregator]: [Aggregated] Count: 1 | Robot: CE-ROBOT | Avg Temp: 50.0°C | Current Temp: 50°C
[INFO] [hardware_status_aggregator]: [Aggregated] Count: 2 | Robot: CE-ROBOT | Avg Temp: 50.0°C | Current Temp: 50°C
[INFO] [hardware_status_aggregator]: [Aggregated] Count: 3 | Robot: CE-ROBOT | Avg Temp: 50.0°C | Current Temp: 50°C
```

---

## **📂 Directory Structure**

```
📁 ROS2_WS/
├── 📁 src/
│   ├── 📁 ce_robot_interfaces/
│   │   ├── 📁 msg/
│   │   │   └── 📄 HardwareStatus.msg
│   │   ├── 📄 package.xml
│   │   └── 📄 CMakeLists.txt
│   └── 📁 ce_robot/
│       ├── 📁 ce_robot/
│       │   ├── 📄 __init__.py
│       │   ├── 🐍 HardwareStatus_publish.py
│       │   ├── 🐍 HardwareStatus_subscribe.py
│       │   ├── 🐍 HardwareStatus_aggregate.py
│       │   ├── 🐍 HardwareStatus_validated.py
│       │   └── 🐍 HardwareStatus_validated_sub.py
│       ├── 📄 package.xml
│       ├── 📄 setup.cfg
│       └── 📄 setup.py
└── 📁 install/
```

---

## **🔍 Message Inspection Commands**

### **List All Messages**
```bash
ros2 interface list | grep "ce_robot"
```

### **Show Message Structure**
```bash
ros2 interface show ce_robot_interfaces/msg/HardwareStatus
```

### **Check Active Topics**
```bash
ros2 topic list
```

### **View Topic Type**
```bash
ros2 topic type /hardware_status
```

### **Echo Topic Messages**
```bash
ros2 topic echo /hardware_status
```

---

## **🎯 Key Concepts**

### **Message Architecture**
- **Message Definition** (`.msg` files): Declarative schema for data structures
- **Package Organization**: Custom messages in dedicated `*_interfaces` packages
- **Type System**: Primitive types (int, float, bool, string) and collections
- **Field Naming**: snake_case for clarity and consistency

### **Communication Pattern**
- **Asynchronous**: Publisher and subscriber operate independently
- **One-to-Many**: Single publisher, multiple subscribers
- **Many-to-One**: Multiple publishers, single subscriber (aggregation)
- **Quality of Service (QoS)**: Message queue depth, reliability settings

### **Advantages**
- ✅ Structured data communication
- ✅ Type-safe message passing
- ✅ Decoupled, asynchronous communication
- ✅ Flexible data aggregation
- ✅ Built-in validation support

### **Disadvantages**
- ❌ Requires message package compilation
- ❌ Schema changes require rebuild
- ❌ Fixed message structure
- ❌ Slightly higher overhead than built-in types

---

## **⚠️ Troubleshooting**

### **Issue: Cannot find module ce_robot_interfaces**
**Solution:** Build and source the message package
```bash
colcon build --packages-select ce_robot_interfaces
source ~/ros2_ws/install/setup.bash
```

### **Issue: CMake Error: rosidl_generate_interfaces not found**
**Solution:** Ensure `package.xml` has proper dependencies
```xml
<build_depend>rosidl_default_generators</build_depend>
```

### **Issue: No module named 'ce_robot_interfaces'**
**Solution:** Verify package is built and dependencies are set
```bash
ls ~/ros2_ws/install/ce_robot_interfaces/
```

### **Issue: Subscriber doesn't receive messages**
**Solution:** Check topic name and QoS settings
```bash
ros2 topic list
ros2 topic info /hardware_status
```

---

## **📚 Resources**

- [ROS 2 Custom Interfaces Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [ROS 2 Message Types Documentation](https://docs.ros.org/en/jazzy/Concepts/About-ROS-Interfaces.html)
- [ROS 2 Message Type Reference](https://docs.ros.org/en/jazzy/Concepts/About-ROS-Interfaces.html#builtin-types)
- [ROS 2 Publishers and Subscribers](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

---

## **✅ Verification Checklist**

- [ ] Message package created (`ce_robot_interfaces`)
- [ ] HardwareStatus.msg defined with 5 fields
- [ ] package.xml configured with rosidl dependencies
- [ ] CMakeLists.txt configured with message generation
- [ ] Message package built successfully
- [ ] Message structure verified with `ros2 interface show`
- [ ] Publisher node created and executable
- [ ] Subscriber node created and executable
- [ ] Aggregator node created and executable
- [ ] Validated publisher/subscriber created
- [ ] All entry points added to setup.py
- [ ] Package built successfully with colcon
- [ ] Publisher sends messages without errors
- [ ] Subscriber receives messages correctly
- [ ] Topics visible with `ros2 topic list`
- [ ] Message structure correct with `ros2 topic echo`

---

## **🔗 Related Topics**

- Publishers & Subscribers (asynchronous communication)
- Services (synchronous request-reply)
- Actions (long-running tasks with feedback)
- Parameter Server
- ROS 2 Launch Files
- Message Quality of Service (QoS)

---

**✅ Custom Message Setup Complete!** 🚀✨


