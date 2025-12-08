# **🚀 Custom Messages Lab Exercises**

Master custom message creation and usage in ROS 2 through progressive hands-on exercises.

---

## **📌 Project Title**

Create and Use Custom Message Types in ROS 2

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🛠 Lab Overview**

This lab focuses on implementing production-quality message validation and error handling in ROS 2. You'll create robust publisher and subscriber nodes with comprehensive input validation, exception handling, and reliability metrics tracking.

**Duration:** ~25 minutes
**Level:** Advanced
**Prerequisites:** ROS 2 Jazzy installed, custom messages package created, basic publisher/subscriber knowledge

---

## **🎯 Learning Objectives**

By completing this lab, you will be able to:

- ✅ Implement message validation and error handling
- ✅ Validate all message fields for correct types and ranges
- ✅ Use logging levels appropriately (INFO, WARN, ERROR)
- ✅ Track reliability metrics and success rates
- ✅ Write defensive code that handles invalid data
- ✅ Create alert mechanisms for critical conditions
- ✅ Handle exceptions gracefully in publishers and subscribers
- ✅ Implement production-quality error handling patterns

---

## **📊 Lab Architecture**

```
┌──────────────────────────────────────────────────┐
│ Exercise 1: Validation & Error Handling          │
│ (Production-quality implementation)              │
│                                                  │
│  ┌─────────────────────────────────────────┐   │
│  │ Validated Publisher                     │   │
│  │ ✓ Input Validation                      │   │
│  │ ✓ Range Checking                        │   │
│  │ ✓ Error Handling & Metrics              │   │
│  └────────────┬────────────────────────────┘   │
│               │ HardwareStatus messages        │
│               ▼                                 │
│  ┌─────────────────────────────────────────┐   │
│  │ Validated Subscriber                    │   │
│  │ ✓ Error Handling                        │   │
│  │ ✓ Critical Alerts                       │   │
│  │ ✓ Statistics Tracking                   │   │
│  └─────────────────────────────────────────┘   │
└──────────────────────────────────────────────────┘
```

---

## **📚 Learning Path Overview**

| Exercise | Title | Level | Duration |
|----------|-------|-------|----------|
| 1 | Validation & Error Handling | Advanced | 25 min |

---

## **Exercise 1: Message Validation & Error Handling ⚠️**

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

### Exercise 1: Validation & Error Handling
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
- [ ] Exercise 1 completed
- [ ] Validation Python files created and tested
- [ ] Error handling implemented
- [ ] Validation working correctly
- [ ] Success metrics tracked accurately

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
