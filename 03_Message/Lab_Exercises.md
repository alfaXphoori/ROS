# **🚀 Custom Messages Lab Exercises**

Master custom message creation and usage in ROS 2 through progressive hands-on exercises.

---

## **📌 Project Title**

Create and Use Custom Message Types in ROS 2

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🛠 Lab Overview**

This lab provides hands-on exercises to master custom message creation and usage in ROS 2. Each exercise builds upon the previous one, progressing from basic message definition through production-quality validation and error handling.

**Duration:** ~90 minutes
**Level:** Beginner to Intermediate
**Prerequisites:** ROS 2 Jazzy installed, Publisher/Subscriber lab completed

---

## **🎯 Learning Objectives**

By completing this lab, you will be able to:

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
│ Exercise 1: Message Package Creation        │
│ (Create ce_robot_interfaces package)        │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 2: Publisher Implementation        │
│ (Send HardwareStatus messages)              │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 3: Data Aggregation                │
│ (Collect & analyze multiple messages)       │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 4: Validation & Error Handling     │
│ (Production-quality implementation)         │
└─────────────────────────────────────────────┘
```

---

## **📚 Learning Path Overview**

| Exercise | Title | Level | Duration |
|----------|-------|-------|----------|
| 1 | Message Package Creation | Beginner | 20 min |
| 2 | Publisher Implementation | Beginner | 20 min |
| 3 | Data Aggregation | Intermediate | 25 min |
| 4 | Validation & Error Handling | Advanced | 25 min |

---

## **Exercise 1: Creating Custom Message Package 🔧**

Refer to **Readme.md** for complete instructions on creating the message package, defining HardwareStatus.msg, and configuring package.xml and CMakeLists.txt.

**Quick Summary:**
```bash
cd ~/ros2_ws/src
ros2 pkg create ce_robot_interfaces
cd ce_robot_interfaces
rm -rf include/ src/
mkdir msg
```

Define `msg/HardwareStatus.msg` with the following fields:
```msg
string name_robot          # Robot identifier/name
int64 number_robot         # Robot ID number (1-1000)
int64 temperature          # Temperature in Celsius (0-100)
bool motor_ready           # Motor operational status
string debug_message       # Status message for debugging
```

Then build and verify:
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
source install/setup.bash
ros2 interface show ce_robot_interfaces/msg/HardwareStatus
```

**See Readme.md for:**
- Detailed package.xml configuration
- Detailed CMakeLists.txt configuration
- Complete setup instructions with expected output

### ✅ Completion Criteria
- [ ] Created `ce_robot_interfaces` package
- [ ] Defined `HardwareStatus.msg` with 5 fields
- [ ] Built package successfully
- [ ] Verified message structure with `ros2 interface show`

---

## Exercise 2: Publisher with Custom Messages 📡

### 🎯 Objective
Create a publisher node that sends `HardwareStatus` messages with realistic robot data at 2 Hz (every 0.5 seconds).

### 📝 Task

**Step 1: Create publisher file**

Create `HardwareStatus_publish.py` in `ce_robot/src/`:
```python
#!/usr/bin/env python3
"""
Exercise 2: Hardware Status Publisher
Publishes simulated robot hardware status using custom message type
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
import random


class HardwareStatusPublisher(Node):
    def __init__(self):
        super().__init__('hardware_status_publisher')
        self.publisher_ = self.create_publisher(HardwareStatus, 'hardware_status', 10)
        timer_period = 0.5  # 2 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.message_count = 0

    def timer_callback(self):
        msg = HardwareStatus()
        msg.name_robot = 'CE-ROBOT'
        msg.number_robot = 1001
        msg.temperature = random.randint(35, 60)
        msg.motor_ready = True
        msg.debug_message = f"Status OK - Message #{self.message_count}"
        
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published: Robot={msg.name_robot}, Temp={msg.temperature}°C, '
            f'Motor={msg.motor_ready}'
        )
        self.message_count += 1


def main(args=None):
    rclpy.init(args=args)
    publisher = HardwareStatusPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 2: Update setup.py in `ce_robot`**

Add the entry point in `setup.py`:
```python
entry_points={
    'console_scripts': [
        '03_hw_status_publisher = ce_robot.HardwareStatus_publish:main',
    ],
},
```

**Step 3: Build and run**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
ros2 run ce_robot 03_hw_status_publisher
```

**Step 4: Verify in separate terminal**
```bash
source install/setup.bash
ros2 topic echo /hardware_status
```

### 🔍 Expected Output
```
$ ros2 run ce_robot 03_hw_status_publisher
[INFO] [hardware_status_publisher-1]: Published: Robot=CE-ROBOT, Temp=45°C, Motor=True
[INFO] [hardware_status_publisher-1]: Published: Robot=CE-ROBOT, Temp=58°C, Motor=True
[INFO] [hardware_status_publisher-1]: Published: Robot=CE-ROBOT, Temp=39°C, Motor=True

# In separate terminal:
$ ros2 topic echo /hardware_status
name_robot: CE-ROBOT
number_robot: 1001
temperature: 45
motor_ready: true
debug_message: Status OK - Message #0
---
name_robot: CE-ROBOT
number_robot: 1001
temperature: 58
motor_ready: true
debug_message: Status OK - Message #1
---
```

### 💡 Key Learning Points
- **Importing custom messages**: `from ce_robot_interfaces.msg import HardwareStatus`
- **Message instantiation**: Create message object with `HardwareStatus()`
- **Field assignment**: Set each field with appropriate types
- **Timer callbacks**: Execute code at fixed intervals (0.5s = 2 Hz)
- **Publishing**: Send messages to named topics with specified QoS depth
- **Logging**: Track publisher activity with `self.get_logger().info()`

### ✅ Completion Criteria
- [ ] Created `HardwareStatus_publish.py` in `ce_robot/src/`
- [ ] Publisher node initializes without errors
- [ ] Timer callback executes every 0.5 seconds (2 Hz)
- [ ] All 5 message fields populated correctly:
  - [ ] name_robot = 'CE-ROBOT'
  - [ ] number_robot = 1001
  - [ ] temperature = random 35-60°C
  - [ ] motor_ready = True
  - [ ] debug_message with message number
- [ ] Entry point configured in setup.py as `03_hw_status_publisher`
- [ ] Build completes without errors
- [ ] Publisher runs: `ros2 run ce_robot 03_hw_status_publisher`
- [ ] Messages visible with: `ros2 topic echo /hardware_status`

---

## Exercise 3: Multi-Field Message Aggregation 🔄

### 🎯 Objective
Create a subscriber that collects and analyzes `HardwareStatus` messages, calculating statistics over a rolling window of messages.

### 📝 Task

**Step 1: Create subscriber with aggregation**

Create `HardwareStatus_aggregate.py` in `ce_robot/src/`:
```python
#!/usr/bin/env python3
"""
Exercise 3: Hardware Status Aggregator
Subscribes to hardware_status and aggregates statistics over time
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
            f'Received: {msg.name_robot} (#{self.message_count}) | '
            f'Temp: {msg.temperature}°C | Motor: {msg.motor_ready}'
        )

    def print_stats(self):
        if len(self.temperature_history) == 0:
            self.get_logger().info('No messages received yet')
            return
        
        avg_temp = sum(self.temperature_history) / len(self.temperature_history)
        max_temp = max(self.temperature_history)
        min_temp = min(self.temperature_history)
        motor_on_count = sum(1 for status in self.motor_statuses if status)
        
        self.get_logger().info(
            f'\n╔════════════════════════════════════════╗\n'
            f'║  Statistics (Last {len(self.temperature_history)} messages)     ║\n'
            f'╠════════════════════════════════════════╣\n'
            f'║  Total Messages: {self.message_count:<25} ║\n'
            f'║  Temperature:                          ║\n'
            f'║    • Average: {avg_temp:>6.1f}°C                    ║\n'
            f'║    • Max:     {max_temp:>6}°C                      ║\n'
            f'║    • Min:     {min_temp:>6}°C                      ║\n'
            f'║  Motor Status - On: {motor_on_count}/{len(self.motor_statuses):<28} ║\n'
            f'╚════════════════════════════════════════╝\n'
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

Add to `setup.py`:
```python
'03_hw_status_aggregator = ce_robot.HardwareStatus_aggregate:main',
```

**Step 3: Build and run**

In Terminal 1 (Publisher):
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
ros2 run ce_robot 03_hw_status_publisher
```

In Terminal 2 (Aggregator):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run ce_robot 03_hw_status_aggregator
```

### 🔍 Expected Output
```
# Terminal 2 (Aggregator):
[INFO] [hardware_status_aggregator-1]: Received: CE-ROBOT (#1) | Temp: 45°C | Motor: True
[INFO] [hardware_status_aggregator-1]: Received: CE-ROBOT (#2) | Temp: 58°C | Motor: True
[INFO] [hardware_status_aggregator-1]: Received: CE-ROBOT (#3) | Temp: 39°C | Motor: True
...
[INFO] [hardware_status_aggregator-1]: 
╔════════════════════════════════════════╗
║  Statistics (Last 10 messages)         ║
╠════════════════════════════════════════╣
║  Total Messages: 10                    ║
║  Temperature:                          ║
║    • Average:  47.2°C                  ║
║    • Max:        60°C                  ║
║    • Min:        35°C                  ║
║  Motor Status - On: 10/10              ║
╚════════════════════════════════════════╝
```

### 💡 Key Learning Points
- **Message subscription**: Use `create_subscription()` to receive messages on a topic
- **Callback functions**: Data processing happens in the callback when messages arrive
- **Data aggregation**: Collecting multiple messages to analyze trends
- **Deque for efficiency**: `deque(maxlen=10)` automatically drops old values, keeping only recent ones
- **Statistics calculation**: Computing average, min, max from collected data
- **Timer-based reporting**: Periodic status updates using `create_timer()`
- **Data structure selection**: Choose appropriate collections for your use case

### ✅ Completion Criteria
- [ ] Created `HardwareStatus_aggregate.py` in `ce_robot/src/`
- [ ] Subscriber successfully receives messages on 'hardware_status' topic
- [ ] Temperature history stored in deque with maxlen=10
- [ ] Motor status list tracks all received statuses
- [ ] Statistics calculated correctly:
  - [ ] Average temperature computed
  - [ ] Maximum temperature tracked
  - [ ] Minimum temperature tracked
  - [ ] Motor status count calculated
- [ ] Statistics printed every 5 seconds via timer callback
- [ ] Entry point configured in setup.py as `03_hw_status_aggregator`
- [ ] Publisher and aggregator run simultaneously
- [ ] Statistics display shows correct values
- [ ] No crashes when subscriber receives messages

---

## Exercise 4: Message Validation & Error Handling ⚠️

### 🎯 Objective
Implement robust message validation and error handling for production-quality code. Validate all message fields and handle errors gracefully.

### 📝 Task

**Step 1: Create validated publisher**

Create `HardwareStatus_validated.py` in `ce_robot/src/`:
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

**Step 3: Add entry points to setup.py**

Add to `setup.py`:
```python
'03_hw_status_validated = ce_robot.HardwareStatus_validated:main',
'03_hw_status_validated_sub = ce_robot.HardwareStatus_validated_sub:main',
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
