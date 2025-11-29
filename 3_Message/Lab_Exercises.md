# 🚀 Custom Messages Lab Exercises

Master custom message creation and usage in ROS 2 through progressive hands-on exercises.

---

## 📋 Lab Overview

This laboratory provides a comprehensive introduction to creating and using custom ROS 2 message types. You'll progress from understanding message fundamentals through building production-quality applications with validation and error handling.

**What You'll Learn:**
- How ROS 2 messages work and when to create custom types
- Structuring message packages and defining message schemas
- Publishing and subscribing to custom messages
- Aggregating data from multiple messages for analysis
- Validating messages and implementing robust error handling

**What You'll Build:**
- A custom message package (`ce_robot_interfaces`) containing the `HardwareStatus` message type
- Publisher nodes that broadcast robot hardware status information
- Subscriber nodes that process and aggregate hardware status data
- Validated publisher/subscriber pairs with error handling and monitoring

**Duration:** ~90 minutes total (4 exercises, 15-25 minutes each)

**Prerequisites:**
- Completed "Publisher & Subscriber" and "Server & Client" labs
- ROS 2 Jazzy Jalisco installed and configured
- Basic Python and ROS 2 node development knowledge
- Familiarity with `colcon` build system

---

## 🎓 Learning Objectives

By completing this lab, you will be able to:

**Message Definition & Structure**
- Define custom message types using ROS 2 message syntax
- Understand primitive types (int, float, bool, string) and when to use each
- Organize fields with clear naming conventions and documentation
- Validate message field ranges and constraints

**Package Organization**
- Create message interface packages following ROS 2 conventions
- Configure `package.xml` and `CMakeLists.txt` for message generation
- Build custom message packages with `colcon`
- Verify generated message interfaces with ROS 2 CLI tools

**Message Communication**
- Import and instantiate custom messages in Python nodes
- Publish custom messages with appropriate data types
- Subscribe to custom messages and process received data
- Coordinate multiple publishers and subscribers on the same topic

**Data Aggregation & Analysis**
- Collect message data over time using efficient data structures
- Calculate statistics (average, min, max) from multiple messages
- Implement rolling-window history using deques
- Display periodic reports from aggregated data

**Error Handling & Validation**
- Implement pre-publication validation for message fields
- Handle exceptions gracefully in message callbacks
- Use appropriate logging levels (INFO, WARN, ERROR)
- Track and report reliability metrics and success rates

**Production Practices**
- Write defensive code that handles invalid data
- Implement try-catch blocks for robustness
- Test edge cases and boundary conditions
- Document code with clear docstrings and comments

---

## 📚 Learning Path Overview

| Exercise | Level | Topic | Time |
|----------|-------|-------|------|
| 1 | Beginner | Creating Custom Message Package | 20 min |
| 2 | Intermediate | Publisher with Custom Messages | 20 min |
| 3 | Intermediate | Multi-Field Message Aggregation | 25 min |
| 4 | Advanced | Message Validation & Error Handling | 25 min |

---

## Exercise 1: Creating Custom Message Package 🔧

### 🎯 Objective
Create the `ce_robot_interfaces` package and define the `HardwareStatus` custom message.

### 📝 Task

**Step 1: Create Message Package**
```bash
cd ~/ros2_ws/src
ros2 pkg create ce_robot_interfaces
cd ce_robot_interfaces
rm -rf include/ src/
mkdir msg
```

**Step 2: Define HardwareStatus.msg**

Create `msg/HardwareStatus.msg` with the following fields:
```msg
string name_robot          # Robot identifier/name
int64 number_robot         # Robot ID number (1-1000)
int64 temperature          # Temperature in Celsius (0-100)
bool motor_ready           # Motor operational status
string debug_message       # Status message for debugging
```

**Step 3: Configure package files**

Update `package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ce_robot_interfaces</name>
  <version>0.0.0</version>
  <description>Custom ROS 2 message definitions for CE Robot</description>
  <maintainer email="student@ksu.edu">Student</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

</package>
```

Update `CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(ce_robot_interfaces)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
)

ament_package()
```

**Step 4: Build and verify**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
source install/setup.bash
ros2 interface show ce_robot_interfaces/msg/HardwareStatus
```

### 🔍 Expected Output
```
$ ros2 interface show ce_robot_interfaces/msg/HardwareStatus
string name_robot
int64 number_robot
int64 temperature
bool motor_ready
string debug_message
```

### 💡 Key Concepts
- **Message packages**: dedicated packages (typically named `*_interfaces`)
- **CMake integration**: `rosidl_generate_interfaces()` auto-generates Python/C++ classes
- **Field naming**: snake_case for field names, clear type annotations
- **Package dependencies**: `rosidl_default_generators` for build, `rosidl_default_runtime` for execution

### ✅ Completion Criteria
- [ ] Created `ce_robot_interfaces` package
- [ ] Defined `HardwareStatus.msg` with 5 fields
- [ ] Updated `package.xml` with rosidl dependencies
- [ ] Updated `CMakeLists.txt` with message generation
- [ ] Built package successfully without errors
- [ ] Verified message structure with `ros2 interface show`

---

## Exercise 2: Publisher with Custom Messages 📡

### 🎯 Objective
Create a publisher node that sends `HardwareStatus` messages with realistic robot data.

### 📝 Task

**Step 1: Create publisher file**

Create `HardwareStatus_publish.py`:
```python
#!/usr/bin/env python3
"""
Exercise 3: Hardware Status Publisher
Publishes simulated robot hardware status using custom message type
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
import random


class HardwareStatusPublisher(Node):
    def __init__(self):
        super().__init__('hardware_status_publisher')
        self.publisher = self.create_publisher(HardwareStatus, 'hardware_status', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.count = 0
        self.robot_id = 1

    def timer_callback(self):
        msg = HardwareStatus()
        msg.name_robot = f'Robot-{self.robot_id}'
        msg.number_robot = self.robot_id
        msg.temperature = random.randint(20, 80)
        msg.motor_ready = random.choice([True, False])
        msg.debug_message = f'Status check #{self.count}: All systems nominal' if self.count % 2 == 0 else f'Warning: High temperature detected'
        
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published: Robot={msg.name_robot}, Temp={msg.temperature}°C, '
            f'Motor={msg.motor_ready}, Message={msg.debug_message}'
        )
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    publisher = HardwareStatusPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 2: Update package.xml in `ce_robot`**
```xml
<depend>ce_robot_interfaces</depend>
```

**Step 3: Update setup.py in `ce_robot`**
```python
entry_points={
    'console_scripts': [
        'hw_status = ce_robot.HardwareStatus_publish:main',
    ],
},
install_requires=['setuptools', 'ce_robot_interfaces'],
```

**Step 4: Build and run**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
ros2 run ce_robot hw_status
```

**Step 5: Verify in separate terminal**
```bash
source ~/.bashrc
ros2 topic echo /hardware_status
```

### 🔍 Expected Output
```
$ ros2 run ce_robot hw_status
[INFO] [hardware_status_publisher]: Published: Robot=Robot-1, Temp=45°C, Motor=True, Message=Status check #0: All systems nominal
[INFO] [hardware_status_publisher]: Published: Robot=Robot-1, Temp=62°C, Motor=False, Message=Warning: High temperature detected
[INFO] [hardware_status_publisher]: Published: Robot=Robot-1, Temp=38°C, Motor=True, Message=Status check #2: All systems nominal

# In separate terminal:
$ ros2 topic echo /hardware_status
name_robot: Robot-1
number_robot: 1
temperature: 45
motor_ready: true
debug_message: Status check #0: All systems nominal
---
```

### 💡 Key Concepts
- **Importing custom messages**: `from ce_robot_interfaces.msg import HardwareStatus`
- **Message instantiation**: Create message object with `HardwareStatus()`
- **Field assignment**: Set each field independently
- **Publishing frequency**: Use timer callbacks for periodic publication
- **Type safety**: ROS 2 enforces correct types at publish time

### ✅ Completion Criteria
- [ ] Created `HardwareStatus_publish.py` in `ce_robot` package
- [ ] Updated `package.xml` and `setup.py` with dependencies and entry point
- [ ] Built package successfully
- [ ] Publisher runs and sends messages every 2 seconds
- [ ] Messages visible with `ros2 topic echo`
- [ ] All 5 fields populated with reasonable data

---

## Exercise 3: Multi-Field Message Aggregation 🔄

### 🎯 Objective
Create subscriber that collects and analyzes `HardwareStatus` messages, aggregating multiple field values.

### 📝 Task

**Step 1: Create subscriber with aggregation**

Create `HardwareStatus_aggregate.py`:
```python
#!/usr/bin/env python3
"""
Exercise 4: Hardware Status Aggregator
Subscribes to hardware_status and aggregates statistics
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
from collections import deque


class HardwareStatusAggregator(Node):
    def __init__(self):
        super().__init__('hardware_status_aggregator')
        self.subscriber = self.create_subscription(
            HardwareStatus, 'hardware_status', self.status_callback, 10
        )
        self.timer = self.create_timer(5.0, self.print_stats)
        
        # Keep history of last 10 messages
        self.temperature_history = deque(maxlen=10)
        self.motor_statuses = []
        self.message_count = 0

    def status_callback(self, msg):
        self.temperature_history.append(msg.temperature)
        self.motor_statuses.append(msg.motor_ready)
        self.message_count += 1
        
        self.get_logger().info(
            f'Received: {msg.name_robot} | '
            f'Temp: {msg.temperature}°C | '
            f'Motor: {msg.motor_ready} | '
            f'Message: {msg.debug_message}'
        )

    def print_stats(self):
        if len(self.temperature_history) == 0:
            return
        
        avg_temp = sum(self.temperature_history) / len(self.temperature_history)
        max_temp = max(self.temperature_history)
        min_temp = min(self.temperature_history)
        motor_on_count = sum(1 for status in self.motor_statuses if status)
        
        self.get_logger().info(
            f'\n=== Statistics (Last {len(self.temperature_history)} messages) ===\n'
            f'Total Messages: {self.message_count}\n'
            f'Temperature - Avg: {avg_temp:.1f}°C, Max: {max_temp}°C, Min: {min_temp}°C\n'
            f'Motor Status - On: {motor_on_count}/{len(self.motor_statuses)}\n'
            f'================================\n'
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

**Step 2: Add entry point to setup.py**
```python
'hw_agg = ce_robot.HardwareStatus_aggregate:main',
```

**Step 3: Build and run**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash

# Terminal 1: Run publisher
ros2 run ce_robot hw_status

# Terminal 2: Run aggregator
ros2 run ce_robot hw_agg
```

### 🔍 Expected Output
```
# Terminal 2:
[INFO] [hardware_status_aggregator]: Received: Robot-1 | Temp: 45°C | Motor: True | Message: Status check #0: All systems nominal
[INFO] [hardware_status_aggregator]: Received: Robot-1 | Temp: 62°C | Motor: False | Message: Warning: High temperature detected
...
[INFO] [hardware_status_aggregator]: 
=== Statistics (Last 10 messages) ===
Total Messages: 10
Temperature - Avg: 51.2°C, Max: 78°C, Min: 22°C
Motor Status - On: 5/10
================================
```

### 💡 Key Concepts
- **Message subscription**: Use `create_subscription()` to receive messages
- **Callback functions**: Processing happens in `status_callback()`
- **Data aggregation**: Collecting and analyzing multiple messages over time
- **Deque for history**: Efficient rolling window for statistics
- **Timer-based reporting**: Periodic statistics output

### ✅ Completion Criteria
- [ ] Created subscriber that receives `HardwareStatus` messages
- [ ] Stores temperature history in deque (max 10 items)
- [ ] Tracks motor status changes
- [ ] Calculates average, max, min temperature every 5 seconds
- [ ] Publisher and subscriber running simultaneously
- [ ] Statistics display shows correct aggregations

---

## Exercise 4: Message Validation & Error Handling ⚠️

### 🎯 Objective
Implement robust message validation and error handling for production-quality code.

### 📝 Task

**Step 1: Create validated publisher**

Create `HardwareStatus_validated.py`:
```python
#!/usr/bin/env python3
"""
Exercise 5: Validated Hardware Status Publisher
Publishes messages with input validation and error handling
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
import random


class ValidatedHardwarePublisher(Node):
    def __init__(self):
        super().__init__('validated_hw_publisher')
        self.publisher = self.create_publisher(HardwareStatus, 'hardware_status', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.robot_id = 1
        self.count = 0
        self.error_count = 0

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
        msg.name_robot = f'Robot-{self.robot_id}'
        msg.number_robot = self.robot_id
        msg.temperature = random.randint(20, 80)
        msg.motor_ready = random.choice([True, False])
        msg.debug_message = f'Cycle #{self.count}: {"Motors running" if msg.motor_ready else "Motors stopped"}'
        
        # Validate before publishing
        errors = self.validate_message(msg)
        
        if errors:
            self.error_count += 1
            self.get_logger().error(f'Validation failed: {", ".join(errors)}')
            return
        
        try:
            self.publisher.publish(msg)
            self.get_logger().info(
                f'✓ Published [{self.count}]: {msg.name_robot} | '
                f'Temp: {msg.temperature}°C | Motor: {msg.motor_ready}'
            )
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Publishing error: {str(e)}')
        
        self.count += 1
        
        # Report stats every 10 cycles
        if self.count % 10 == 0:
            success_rate = ((self.count - self.error_count) / self.count) * 100
            self.get_logger().info(
                f'Stats: Published {self.count - self.error_count}/{self.count} '
                f'({success_rate:.1f}% success rate)'
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

Create `HardwareStatus_validated_sub.py`:
```python
#!/usr/bin/env python3
"""
Exercise 5: Validated Hardware Status Subscriber
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

    def status_callback(self, msg):
        try:
            # Basic validation check
            if msg.temperature > 75:
                self.get_logger().warn(
                    f'⚠️  HIGH TEMPERATURE ALERT: {msg.name_robot} reached {msg.temperature}°C'
                )
            
            if not msg.motor_ready:
                self.get_logger().info(f'🛑 {msg.name_robot} motors are offline')
            
            self.received_count += 1
            self.get_logger().info(
                f'[{self.received_count}] Received: {msg.name_robot} - '
                f'Temp: {msg.temperature}°C, Motor: {msg.motor_ready}'
            )
        
        except AttributeError as e:
            self.error_count += 1
            self.get_logger().error(f'Missing message field: {str(e)}')
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Processing error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    subscriber = ValidatedSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 3: Add entry points to setup.py**
```python
'hw_validated = ce_robot.HardwareStatus_validated:main',
'hw_validated_sub = ce_robot.HardwareStatus_validated_sub:main',
```

**Step 4: Build and run**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash

# Terminal 1: Run validated publisher
ros2 run ce_robot hw_validated

# Terminal 2: Run validated subscriber
ros2 run ce_robot hw_validated_sub
```

### 🔍 Expected Output
```
# Publisher Terminal:
[INFO] ✓ Published [0]: Robot-1 | Temp: 45°C | Motor: True
[INFO] ✓ Published [1]: Robot-1 | Temp: 78°C | Motor: False
...
[INFO] Stats: Published 10/10 (100.0% success rate)

# Subscriber Terminal:
[INFO] [1] Received: Robot-1 - Temp: 45°C, Motor: True
[WARN] ⚠️  HIGH TEMPERATURE ALERT: Robot-1 reached 78°C
[INFO] [2] Received: Robot-1 - Temp: 78°C, Motor: False
[INFO] 🛑 Robot-1 motors are offline
```

### 💡 Key Concepts
- **Input validation**: Check all message fields for valid ranges
- **Error handling**: Try-catch blocks for robust operation
- **Warning levels**: Different log levels (INFO, WARN, ERROR) for severity
- **Exception handling**: Graceful handling of unexpected errors
- **Success metrics**: Track and report reliability statistics
- **Defensive programming**: Assume data might be invalid

### ✅ Completion Criteria
- [ ] Created validated publisher with range checks
- [ ] Implemented error handling in publisher
- [ ] Created validated subscriber with error handling
- [ ] Added warning alerts for high temperature
- [ ] Added alert for motor offline status
- [ ] Both publisher and subscriber run without crashes
- [ ] Validation catches at least one invalid value
- [ ] Success rate statistics calculated and displayed

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

## ✅ Lab Completion Checklist

### Exercise 1: Message Package
- [ ] Created `ce_robot_interfaces` package
- [ ] Defined `HardwareStatus.msg` with 5 fields
- [ ] Updated `package.xml` and `CMakeLists.txt`
- [ ] Built package without errors
- [ ] Verified message with `ros2 interface show`

### Exercise 2: Publisher
- [ ] Created `HardwareStatus_publish.py`
- [ ] Updated package dependencies
- [ ] Built and ran publisher successfully
- [ ] Messages visible with `ros2 topic echo`
- [ ] All fields populated correctly

### Exercise 3: Aggregation
- [ ] Created subscriber with aggregation logic
- [ ] Implemented deque-based history tracking
- [ ] Calculated statistics (avg, min, max)
- [ ] Publisher and subscriber run together
- [ ] Statistics output displays correctly

### Exercise 4: Validation
- [ ] Created validated publisher with checks
- [ ] Implemented error handling in publisher
- [ ] Created validated subscriber with error handling
- [ ] Added temperature and motor status alerts
- [ ] Tracked and reported success metrics
- [ ] No crashes on invalid data

### Overall Completion
- [ ] All 4 exercises completed
- [ ] All Python files created and tested
- [ ] Custom message package working
- [ ] Publisher/subscriber communication verified
- [ ] Error handling implemented
- [ ] Statistics and validation working

---

**🎓 Congratulations!** You've mastered custom message creation and usage in ROS 2! 🚀✨
