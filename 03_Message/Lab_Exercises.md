# **🚀 Custom Messages Lab Exercises**

Master custom message creation and usage in ROS 2 through progressive hands-on exercises.

---

## **📌 Project Title**

Create and Use Custom Message Types in ROS 2

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🛠 Lab Overview**

This lab provides comprehensive hands-on exercises to master custom message creation and usage in ROS 2. Starting from understanding built-in messages, you'll progress through creating message packages, implementing publishers and subscribers, aggregating data, and finally implementing production-quality validation and error handling.

**Duration:** ~90 minutes
**Level:** Beginner to Advanced
**Prerequisites:** ROS 2 Jazzy installed, basic Publisher/Subscriber knowledge

---

## **🎯 Learning Objectives**

By completing this lab, you will be able to:

- ✅ Understand built-in message types and when to create custom messages
- ✅ Create custom message packages following ROS 2 conventions
- ✅ Define message types with appropriate fields and types
- ✅ Build and generate message interfaces with `colcon`
- ✅ Publish and subscribe to custom messages in Python
- ✅ Aggregate data from multiple messages for analysis
- ✅ Implement message validation and error handling
- ✅ Use logging levels appropriately (INFO, WARN, ERROR)
- ✅ Track reliability metrics and success rates
- ✅ Write defensive code that handles invalid data
- ✅ Debug and monitor custom message communication

---

## **📊 Lab Architecture**

```
┌─────────────────────────────────────────────┐
│ Exercise 1: Understanding Message Types     │
│ (Explore built-in messages)                 │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 2: Message Package Creation        │
│ (Create ce_robot_interfaces package)        │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 3: Publisher Implementation        │
│ (Send HardwareStatus messages)              │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 4: Subscriber Implementation       │
│ (Receive and display messages)              │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 5: Data Aggregation                │
│ (Collect & analyze multiple messages)       │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 6: Validation & Error Handling     │
│ (Production-quality implementation)         │
└─────────────────────────────────────────────┘
```

---

## **📚 Learning Path Overview**

| Exercise | Title | Level | Duration |
|----------|-------|-------|----------|
| 1 | Understanding Message Types | Beginner | 10 min |
| 2 | Message Package Creation | Beginner | 15 min |
| 3 | Publisher Implementation | Beginner | 15 min |
| 4 | Subscriber Implementation | Beginner | 15 min |
| 5 | Data Aggregation | Intermediate | 20 min |
| 6 | Validation & Error Handling | Advanced | 25 min |

---

## **Exercise 1: Understanding Message Types 📖**

### 🎯 Objective
Explore built-in ROS 2 message types and understand when to create custom messages.

### 📝 Task

**Step 1: List all available built-in messages**

```bash
ros2 interface list | grep "std_msgs"
```

**Step 2: Examine message structures**

```bash
ros2 interface show std_msgs/msg/String
ros2 interface show std_msgs/msg/Int32
ros2 interface show geometry_msgs/msg/Point
```

**Step 3: Create a comparison document**

List 5 built-in message types with their use cases:

1. **std_msgs/String**: Simple text data (logging, notifications)
2. **std_msgs/Int32**: 32-bit integer (counters, IDs)
3. **geometry_msgs/Point**: 3D point with x, y, z (positions, coordinates)
4. **sensor_msgs/Temperature**: Temperature reading with timestamp (sensor data)
5. **diagnostic_msgs/DiagnosticStatus**: System diagnostics (health monitoring)

### 🔍 Expected Output
```bash
$ ros2 interface show std_msgs/msg/String
string data

$ ros2 interface show geometry_msgs/msg/Point
float64 x
float64 y
float64 z
```

### 💡 Key Learning Points
- **Built-in messages** are provided by standard ROS 2 packages
- **Message types**: basic types (int, float, bool), collections, and specialized types
- **When to use custom messages**: when built-in types don't match your data structure
- **Naming conventions**: PascalCase for message names (e.g., `HardwareStatus`)

### ✅ Completion Criteria
- [ ] Listed all std_msgs message types
- [ ] Examined at least 3 different message structures
- [ ] Identified when custom messages are needed
- [ ] Understood message naming conventions

---

## **Exercise 2: Creating the Message Package 🔧**

### 🎯 Objective
Create a dedicated message package for custom message definitions.

### 📝 Task

**Step 1: Create message package**

```bash
cd ~/ros2_ws/src
ros2 pkg create ce_robot_interfaces
cd ce_robot_interfaces
rm -rf include/ src/
mkdir msg
```

**Step 2: Define HardwareStatus.msg**

Create `msg/HardwareStatus.msg`:
```msg
string name_robot          # Robot identifier/name
int64 number_robot         # Robot ID number (1-1000)
int64 temperature          # Temperature in Celsius (0-100)
bool motor_ready           # Motor operational status
string debug_message       # Status message for debugging
```

**Step 3: Update package.xml**

Add these lines after `<buildtool_depend>`:
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

**Step 4: Update CMakeLists.txt**

Add these lines:
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
)
```

**Step 5: Build and verify**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
source install/setup.bash
ros2 interface show ce_robot_interfaces/msg/HardwareStatus
```

### 🔍 Expected Output
```bash
$ ros2 interface show ce_robot_interfaces/msg/HardwareStatus
string name_robot
int64 number_robot
int64 temperature
bool motor_ready
string debug_message
```

### 💡 Key Learning Points
- **Message packages**: dedicated packages (typically named `*_interfaces`)
- **CMake integration**: `rosidl_generate_interfaces()` auto-generates Python/C++ classes
- **Field types**: Common types include uint8, uint16, int32, float32, float64, string, bool
- **Field comments**: Document each field's purpose and valid ranges

### ✅ Completion Criteria
- [ ] Created `ce_robot_interfaces` package
- [ ] Defined `HardwareStatus.msg` with 5 fields
- [ ] Updated `package.xml` with rosidl dependencies
- [ ] Updated `CMakeLists.txt` with message generation
- [ ] Built package successfully without errors
- [ ] Verified message structure with `ros2 interface show`

---

## **Exercise 3: Publisher Implementation 📡**

### 🎯 Objective
Create a publisher node that sends `HardwareStatus` messages with realistic robot data.

### 📝 Task

**Step 1: Create publisher file**

Navigate to `ce_robot` package:
```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
```

Create `HardwareStatus_publish.py`:
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

Make it executable:
```bash
chmod +x HardwareStatus_publish.py
```

**Step 2: Update package.xml in `ce_robot`**

Add dependency:
```xml
<depend>ce_robot_interfaces</depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
```

**Step 3: Update setup.py**

Add entry point:
```python
entry_points={
    'console_scripts': [
        "03_hw_status_publisher = ce_robot.HardwareStatus_publish:main",
    ],
},
```

**Step 4: Build and run**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
ros2 run ce_robot 03_hw_status_publisher
```

### 🔍 Expected Output
```
[INFO] [hardwarestatus_publish]: Hw_Status_Publish_Node Start Now!
[INFO] [hardwarestatus_publish]: Published: Robot=CE-ROBOT, Number=1001, Temp=47°C, Motor=True, Message=Motor 1
[INFO] [hardwarestatus_publish]: Published: Robot=CE-ROBOT, Number=1001, Temp=52°C, Motor=True, Message=Motor 1
```

### 💡 Key Learning Points
- **Importing custom messages**: `from ce_robot_interfaces.msg import HardwareStatus`
- **Message instantiation**: Create message object with `HardwareStatus()`
- **Field assignment**: Set each field with appropriate types
- **Timer callbacks**: Execute code at fixed intervals (1.0s = 1 Hz)
- **Publishing**: Send messages to named topics with specified QoS depth

### ✅ Completion Criteria
- [ ] Created `HardwareStatus_publish.py`
- [ ] Publisher node initializes without errors
- [ ] Timer callback executes every 1 second
- [ ] All 5 message fields populated correctly
- [ ] Random temperature between 35-60°C
- [ ] Entry point configured in setup.py as `03_hw_status_publisher`
- [ ] Build completes without errors
- [ ] Publisher runs and sends messages

---

## **Exercise 4: Subscriber Implementation 📥**

### 🎯 Objective
Create a subscriber node that receives and displays `HardwareStatus` messages.

### 📝 Task

**Step 1: Create subscriber file**

Create `HardwareStatus_subscribe.py` in `ce_robot/ce_robot/`:
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

Make it executable:
```bash
chmod +x HardwareStatus_subscribe.py
```

**Step 2: Update setup.py**

Add entry point:
```python
entry_points={
    'console_scripts': [
        "03_hw_status_publisher = ce_robot.HardwareStatus_publish:main",
        "03_hw_status_subscriber = ce_robot.HardwareStatus_subscribe:main",
    ],
},
```

**Step 3: Build and run**

Build:
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

Terminal 1 (Publisher):
```bash
ros2 run ce_robot 03_hw_status_publisher
```

Terminal 2 (Subscriber):
```bash
ros2 run ce_robot 03_hw_status_subscriber
```

Terminal 3 (Monitor):
```bash
ros2 topic echo /hardware_status
```

### 🔍 Expected Output

**Terminal 2 (Subscriber):**
```
[INFO] [hardware_status_subscriber]: Hardware Status Subscriber started!
[INFO] [hardware_status_subscriber]: [1] Received: CE-ROBOT | Number: 1001 | Temp: 47°C | Motor: True | Message: Motor 1
[INFO] [hardware_status_subscriber]: [2] Received: CE-ROBOT | Number: 1001 | Temp: 52°C | Motor: True | Message: Motor 1
```

**Terminal 3 (Topic Echo):**
```
name_robot: CE-ROBOT
number_robot: 1001
temperature: 47
motor_ready: true
debug_message: Motor 1
---
```

### 💡 Key Learning Points
- **Message subscription**: Use `create_subscription()` to receive messages
- **Callback functions**: Data processing happens when messages arrive
- **Message counter**: Track received messages for statistics
- **Asynchronous communication**: Publisher and subscriber run independently

### ✅ Completion Criteria
- [ ] Created `HardwareStatus_subscribe.py`
- [ ] Subscriber successfully receives messages
- [ ] Callback function processes all message fields
- [ ] Message counter increments correctly
- [ ] Entry point configured in setup.py as `03_hw_status_subscriber`
- [ ] Build completes without errors
- [ ] Subscriber and publisher run simultaneously
- [ ] Messages displayed with proper formatting

---

## **Exercise 5: Data Aggregation 🔄**

### 🎯 Objective
Create an aggregator node that collects and analyzes `HardwareStatus` messages over time.

### 📝 Task

**Step 1: Create aggregator file**

Create `HardwareStatus_aggregate.py` in `ce_robot/ce_robot/`:
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

Make it executable:
```bash
chmod +x HardwareStatus_aggregate.py
```

**Step 2: Update setup.py**

Add entry point:
```python
entry_points={
    'console_scripts': [
        "03_hw_status_publisher = ce_robot.HardwareStatus_publish:main",
        "03_hw_status_subscriber = ce_robot.HardwareStatus_subscribe:main",
        "03_hw_status_aggregator = ce_robot.HardwareStatus_aggregate:main",
    ],
},
```

**Step 3: Build and run**

Build:
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

Terminal 1 (Publisher):
```bash
ros2 run ce_robot 03_hw_status_publisher
```

Terminal 2 (Aggregator):
```bash
ros2 run ce_robot 03_hw_status_aggregator
```

### 🔍 Expected Output
```
[INFO] [hardware_status_aggregator]: Hardware Status Aggregator started!
[INFO] [hardware_status_aggregator]: [Aggregated] Count: 1 | Robot: CE-ROBOT | Avg Temp: 56.0°C | Current Temp: 56°C
[INFO] [hardware_status_aggregator]: [Aggregated] Count: 2 | Robot: CE-ROBOT | Avg Temp: 52.5°C | Current Temp: 49°C
[INFO] [hardware_status_aggregator]: [Aggregated] Count: 3 | Robot: CE-ROBOT | Avg Temp: 52.3°C | Current Temp: 52°C
```

### 💡 Key Learning Points
- **Data aggregation**: Collecting multiple messages for analysis
- **Running averages**: Computing statistics from message streams
- **Many-to-one pattern**: Multiple data sources, single analyzer
- **Real-time statistics**: Live data processing and reporting

### ✅ Completion Criteria
- [ ] Created `HardwareStatus_aggregate.py`
- [ ] Aggregator receives all published messages
- [ ] Message count tracked correctly
- [ ] Average temperature calculated accurately
- [ ] Current and average temperatures displayed
- [ ] Entry point configured in setup.py as `03_hw_status_aggregator`
- [ ] Publisher and aggregator run simultaneously
- [ ] Statistics update with each new message

---

## **Exercise 6: Message Validation & Error Handling ⚠️**

### 🎯 Objective
Implement robust message validation and error handling for production-quality code.

### 📝 Task

**Step 1: Create validated publisher**

Create `HardwareStatus_validated.py` in `ce_robot/ce_robot/`:
```python
#!/usr/bin/env python3
"""
Exercise 4: Validated Hardware Status Publisher
Publishes messages with input validation and error handling
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
import random


class ValidatedHardwarePublisher(Node):
    def __init__(self):
        super().__init__('validated_hw_publisher')
        self.publisher_ = self.create_publisher(HardwareStatus, 'hardware_status', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0
        self.error_count = 0
        self.success_count = 0

    def validate_message(self, msg):
        """Validate message fields before publishing"""
        errors = []
        
        # Validate name_robot
        if not msg.name_robot or len(msg.name_robot) == 0:
            errors.append("name_robot cannot be empty")
        
        # Validate number_robot
        if msg.number_robot < 1 or msg.number_robot > 1000:
            errors.append(f"number_robot must be 1-1000, got {msg.number_robot}")
        
        # Validate temperature
        if msg.temperature < -40 or msg.temperature > 100:
            errors.append(f"temperature out of range: {msg.temperature}°C")
        
        # Validate debug_message
        if len(msg.debug_message) > 200:
            errors.append("debug_message exceeds 200 characters")
        
        return errors

    def timer_callback(self):
        msg = HardwareStatus()
        msg.name_robot = 'CE-ROBOT'
        msg.number_robot = 1001
        msg.temperature = random.randint(35, 60)
        msg.motor_ready = True
        msg.debug_message = f'Cycle #{self.count}: All systems nominal'
        
        # Validate before publishing
        errors = self.validate_message(msg)
        
        if errors:
            self.error_count += 1
            self.get_logger().error(f'❌ Validation failed: {", ".join(errors)}')
            return
        
        try:
            self.publisher_.publish(msg)
            self.success_count += 1
            self.get_logger().info(
                f'✓ Published [{self.count}]: {msg.name_robot} | '
                f'Temp: {msg.temperature}°C'
            )
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'❌ Publishing error: {str(e)}')
        
        self.count += 1
        
        # Report stats every 10 cycles
        if self.count % 10 == 0:
            success_rate = (self.success_count / self.count) * 100
            self.get_logger().info(
                f'📊 Stats: {self.success_count}/{self.count} published '
                f'({success_rate:.1f}% success)'
            )


def main(args=None):
    rclpy.init(args=args)
    publisher = ValidatedHardwarePublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 2: Create validated subscriber**

Create `HardwareStatus_validated_sub.py` in `ce_robot/src/`:
```python
#!/usr/bin/env python3
"""
Exercise 4: Validated Hardware Status Subscriber
Subscribes with error handling and data validation
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus


class ValidatedSubscriber(Node):
    def __init__(self):
        super().__init__('validated_hw_subscriber')
        self.subscriber = self.create_subscription(
            HardwareStatus, 'hardware_status', self.status_callback, 10
        )
        self.received_count = 0
        self.error_count = 0
        self.alert_count = 0

    def status_callback(self, msg):
        try:
            self.received_count += 1
            
            # Check for high temperature alert
            if msg.temperature > 75:
                self.alert_count += 1
                self.get_logger().warn(
                    f'🔥 HIGH TEMPERATURE ALERT: {msg.name_robot} '
                    f'reached {msg.temperature}°C (Alert #{self.alert_count})'
                )
            
            # Check for motor offline
            if not msg.motor_ready:
                self.get_logger().warn(
                    f'🛑 MOTOR OFFLINE: {msg.name_robot} motors are not ready'
                )
            
            # Normal operation log
            self.get_logger().info(
                f'[{self.received_count}] {msg.name_robot} - '
                f'Temp: {msg.temperature}°C, Motor: {msg.motor_ready}'
            )
        
        except AttributeError as e:
            self.error_count += 1
            self.get_logger().error(f'❌ Missing message field: {str(e)}')
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'❌ Processing error: {str(e)}')
        
        # Report status every 20 messages
        if self.received_count % 20 == 0:
            self.get_logger().info(
                f'📈 Received {self.received_count} messages, '
                f'{self.error_count} errors, {self.alert_count} alerts'
            )


def main(args=None):
    rclpy.init(args=args)
    subscriber = ValidatedSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Make files executable:
```bash
chmod +x HardwareStatus_validated.py
chmod +x HardwareStatus_validated_sub.py
```

**Step 3: Add entry points to setup.py**

Update entry points:
```python
entry_points={
    'console_scripts': [
        "03_hw_status_publisher = ce_robot.HardwareStatus_publish:main",
        "03_hw_status_subscriber = ce_robot.HardwareStatus_subscribe:main",
        "03_hw_status_aggregator = ce_robot.HardwareStatus_aggregate:main",
        "03_hw_status_validated = ce_robot.HardwareStatus_validated:main",
        "03_hw_status_validated_sub = ce_robot.HardwareStatus_validated_sub:main",
    ],
},
```

**Step 4: Build and run**

Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

In Terminal 1 (Validated Publisher):
```bash
ros2 run ce_robot 03_hw_status_validated
```

In Terminal 2 (Validated Subscriber):
```bash
ros2 run ce_robot 03_hw_status_validated_sub
```

### 🔍 Expected Output
```
# Terminal 1 (Publisher):
[INFO] ✓ Published [0]: CE-ROBOT | Temp: 45°C
[INFO] ✓ Published [1]: CE-ROBOT | Temp: 58°C
[INFO] ✓ Published [2]: CE-ROBOT | Temp: 39°C
...
[INFO] 📊 Stats: 10/10 published (100.0% success)

# Terminal 2 (Subscriber):
[INFO] [validated_hw_subscriber-1]: [1] CE-ROBOT - Temp: 45°C, Motor: True
[INFO] [validated_hw_subscriber-1]: [2] CE-ROBOT - Temp: 58°C, Motor: True
[INFO] [validated_hw_subscriber-1]: [3] CE-ROBOT - Temp: 39°C, Motor: True
...
[INFO] [validated_hw_subscriber-1]: 📈 Received 20 messages, 0 errors, 0 alerts
```

### 💡 Key Learning Points
- **Input validation**: Check all message fields for valid ranges before processing
- **Error handling**: Use try-catch blocks to gracefully handle exceptions
- **Logging levels**: Different levels (INFO, WARN, ERROR) for different severity
- **Exceptions**: Catch specific exceptions (AttributeError) and generic ones
- **Success metrics**: Track and report reliability statistics
- **Defensive programming**: Assume data might be invalid or incomplete
- **Alerting mechanisms**: Notify about critical conditions (high temp, motor offline)
- **Production-quality code**: Robust error handling prevents crashes

### ✅ Completion Criteria
- [ ] Created `HardwareStatus_validated.py` with validation logic
- [ ] Implemented `validate_message()` function with all checks
- [ ] Range checks for all message fields:
  - [ ] name_robot: not empty
  - [ ] number_robot: 1-1000
  - [ ] temperature: -40 to 100°C
  - [ ] debug_message: ≤ 200 characters
- [ ] Added error handling (try-catch) in publisher
- [ ] Created `HardwareStatus_validated_sub.py` with error handling
- [ ] Added temperature alert (>75°C) in subscriber
- [ ] Added motor status alert (motor offline)
- [ ] Entry points configured in setup.py
- [ ] Build completes without errors
- [ ] Both publisher and subscriber run without crashes
- [ ] Success rate statistics calculated and logged
- [ ] Validation correctly identifies out-of-range values
- [ ] Alerts displayed for critical conditions

---

## 🔑 Key Concepts Summary

### Message Architecture
- **Message Definition** (`.msg` files): Declarative schema for data structures
- **Package Organization**: Custom messages in dedicated `*_interfaces` packages
- **Type System**: Primitive types (int, float, bool, string) and collections
- **Field Naming**: snake_case for clarity and consistency

### Communication Patterns
- **One-to-Many**: Single publisher, multiple subscribers
- **Many-to-One**: Multiple publishers, single subscriber (aggregation)
- **Publish-Subscribe**: Asynchronous, decoupled communication
- **Quality of Service (QoS)**: Message queue depth, reliability settings

### Best Practices
- **Validation**: Always validate received messages
- **Error Handling**: Use try-catch and logging for resilience
- **Performance**: Use appropriate field types to minimize message size
- **Documentation**: Include units and constraints in field comments
- **Testing**: Test with edge cases and invalid data

---

## 🛠️ Troubleshooting Guide

### Issue: "Cannot find module ce_robot_interfaces"
**Cause**: Message package not built or not sourced  
**Solution**:
```bash
colcon build --packages-select ce_robot_interfaces
source ~/ros2_ws/install/setup.bash
```

### Issue: "CMake Error: rosidl_generate_interfaces not found"
**Cause**: Missing rosidl build dependency  
**Solution**: Ensure `package.xml` has `<build_depend>rosidl_default_generators</build_depend>`

### Issue: Publisher message shows as different type
**Cause**: Old build artifacts not cleared  
**Solution**:
```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select ce_robot_interfaces ce_robot
```

### Issue: Subscriber doesn't receive messages
**Cause**: Topic names don't match or QoS incompatible  
**Solution**:
```bash
# Check active topics
ros2 topic list
# Check message type
ros2 topic info /hardware_status
# Echo messages to verify
ros2 topic echo /hardware_status
```

---

## 📚 Additional Resources

### Official ROS 2 Documentation
- [Creating Custom Interfaces](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [Interfaces (Messages, Services, Actions)](https://docs.ros.org/en/jazzy/Concepts/About-ROS-Interfaces.html)
- [ROS 2 Message Type Reference](https://docs.ros.org/en/jazzy/Concepts/About-ROS-Interfaces.html#builtin-types)

### ROS 2 CLI Commands
```bash
# List all message types
ros2 interface list

# Show message structure
ros2 interface show package_name/msg/MessageName

# Find message definitions
find ~/ros2_ws -name "*.msg"

# Check topic information
ros2 topic info /topic_name
ros2 topic type /topic_name
```

### Message Type Best Practices
- Use appropriate types: `int32` vs `int64` vs `float32` vs `float64`
- Include field documentation with units: `int32 temperature_celsius`
- Avoid deeply nested messages for beginner projects
- Use consistent naming: `name_robot` not `robot_name`
- Consider versioning comments for future compatibility

---

---

## **📂 Final Directory Structure**

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

## **🔍 Useful ROS 2 Commands**

### **Message Inspection**
```bash
# List all message interfaces
ros2 interface list | grep "ce_robot"

# Show message structure
ros2 interface show ce_robot_interfaces/msg/HardwareStatus

# Check active topics
ros2 topic list

# View topic type
ros2 topic type /hardware_status

# Echo topic messages
ros2 topic echo /hardware_status

# Show topic info
ros2 topic info /hardware_status
```

### **Node Management**
```bash
# List running nodes
ros2 node list

# Show node info
ros2 node info /hardwarestatus_publish

# View node graph
rqt_graph
```

### **Build and Clean**
```bash
# Build specific package
colcon build --packages-select ce_robot_interfaces

# Build with symlink install
colcon build --packages-select ce_robot --symlink-install

# Clean build artifacts
rm -rf build install log

# Rebuild everything
colcon build
```

---

## **🎯 Key Concepts Summary**

### **Message Architecture**
- **Message Definition** (`.msg` files): Declarative schema for data structures
- **Package Organization**: Custom messages in dedicated `*_interfaces` packages
- **Type System**: Primitive types (int, float, bool, string) and collections
- **Field Naming**: snake_case for clarity and consistency

### **Communication Patterns**
- **Asynchronous**: Publisher and subscriber operate independently
- **One-to-Many**: Single publisher, multiple subscribers
- **Many-to-One**: Multiple publishers, single subscriber (aggregation)
- **Quality of Service (QoS)**: Message queue depth, reliability settings

### **Best Practices**
- ✅ Create dedicated message packages (`*_interfaces`)
- ✅ Document all message fields with comments
- ✅ Use appropriate data types for fields
- ✅ Validate data before publishing
- ✅ Handle exceptions gracefully
- ✅ Use descriptive entry point names
- ✅ Log important events and errors
- ✅ Test with edge cases and invalid data

---

## ✅ Lab Completion Checklist

### Exercise 1: Understanding Message Types
- [ ] Listed all std_msgs message types
- [ ] Examined at least 3 message structures
- [ ] Identified when custom messages are needed
- [ ] Understood message naming conventions

### Exercise 2: Message Package Creation
- [ ] Created `ce_robot_interfaces` package
- [ ] Defined `HardwareStatus.msg` with 5 fields
- [ ] Updated `package.xml` with rosidl dependencies
- [ ] Updated `CMakeLists.txt` with message generation
- [ ] Built package successfully
- [ ] Verified message with `ros2 interface show`

### Exercise 3: Publisher Implementation
- [ ] Created `HardwareStatus_publish.py`
- [ ] Publisher sends messages every 1 second
- [ ] All message fields populated correctly
- [ ] Random temperature between 35-60°C
- [ ] Entry point configured in setup.py
- [ ] Build completes without errors
- [ ] Publisher runs successfully

### Exercise 4: Subscriber Implementation
- [ ] Created `HardwareStatus_subscribe.py`
- [ ] Subscriber receives all published messages
- [ ] Callback function processes messages correctly
- [ ] Message counter tracks received messages
- [ ] Entry point configured in setup.py
- [ ] Publisher and subscriber run simultaneously
- [ ] Messages displayed with proper formatting

### Exercise 5: Data Aggregation
- [ ] Created `HardwareStatus_aggregate.py`
- [ ] Aggregator receives all messages
- [ ] Message count tracked correctly
- [ ] Average temperature calculated accurately
- [ ] Entry point configured in setup.py
- [ ] Publisher and aggregator run simultaneously
- [ ] Statistics update with each message

### Exercise 6: Validation & Error Handling
- [ ] Created `HardwareStatus_validated.py`
- [ ] Implemented validation logic for all fields
- [ ] Range checks work correctly
- [ ] Created `HardwareStatus_validated_sub.py`
- [ ] Error handling implemented in both nodes
- [ ] Temperature alert (>75°C) working
- [ ] Motor status alert working
- [ ] Entry points configured in setup.py
- [ ] Success rate statistics calculated
- [ ] Both nodes run without crashes

### Overall Completion
- [ ] All 6 exercises completed
- [ ] All Python files created and tested
- [ ] Custom message package working
- [ ] Publisher/subscriber communication verified
- [ ] Aggregation working correctly
- [ ] Error handling implemented
- [ ] Validation working correctly

---

## **⚠️ Troubleshooting Guide**

### **Issue: Cannot find module ce_robot_interfaces**
**Cause**: Message package not built or not sourced  
**Solution**:
```bash
colcon build --packages-select ce_robot_interfaces
source ~/ros2_ws/install/setup.bash
```

### **Issue: CMake Error: rosidl_generate_interfaces not found**
**Cause**: Missing rosidl build dependency  
**Solution**: Ensure `package.xml` has:
```xml
<build_depend>rosidl_default_generators</build_depend>
```

### **Issue: Publisher message shows as different type**
**Cause**: Old build artifacts not cleared  
**Solution**:
```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select ce_robot_interfaces ce_robot
```

### **Issue: Subscriber doesn't receive messages**
**Cause**: Topic names don't match or QoS incompatible  
**Solution**:
```bash
# Check active topics
ros2 topic list
# Check message type
ros2 topic info /hardware_status
# Echo messages to verify
ros2 topic echo /hardware_status
```

### **Issue: Entry point not found**
**Cause**: setup.py not updated or package not rebuilt  
**Solution**:
```bash
# Update setup.py with correct entry point
# Rebuild with symlink install
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

---

## **📚 Additional Resources**

### Official ROS 2 Documentation
- [Creating Custom Interfaces](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [Interfaces (Messages, Services, Actions)](https://docs.ros.org/en/jazzy/Concepts/About-ROS-Interfaces.html)
- [ROS 2 Message Type Reference](https://docs.ros.org/en/jazzy/Concepts/About-ROS-Interfaces.html#builtin-types)
- [ROS 2 Publishers and Subscribers](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

### ROS 2 CLI Commands
```bash
# List all message types
ros2 interface list

# Show message structure
ros2 interface show package_name/msg/MessageName

# Find message definitions
find ~/ros2_ws -name "*.msg"

# Check topic information
ros2 topic info /topic_name
ros2 topic type /topic_name
```

### Message Type Best Practices
- Use appropriate types: `int32` vs `int64` vs `float32` vs `float64`
- Include field documentation with units: `int64 temperature  # Celsius`
- Avoid deeply nested messages for beginner projects
- Use consistent naming: `name_robot` not `robot_name`
- Consider versioning comments for future compatibility

---

## **🔗 Related Topics**

- Publishers & Subscribers (asynchronous communication)
- Services (synchronous request-reply)
- Actions (long-running tasks with feedback)
- Parameter Server
- ROS 2 Launch Files
- Message Quality of Service (QoS)

---

**🎓 Congratulations!** You've mastered custom message creation and usage in ROS 2! 🚀✨
