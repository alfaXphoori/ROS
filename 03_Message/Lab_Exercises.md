# **🚀 Custom Messages Lab Exercises**

Master custom message creation and usage in ROS 2 through progressive hands-on exercises.

---

## **📌 Project Title**

Create and Use Custom Message Types in ROS 2

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🛠 Lab Overview**

This comprehensive lab series covers advanced ROS 2 custom message techniques from validation to complex message handling. Progress through beginner to intermediate exercises that build practical skills in message processing, filtering, recording, multi-robot systems, rate control, and nested message structures.

**Duration:** ~140 minutes (6 exercises)
**Level:** Beginner to Intermediate
**Prerequisites:** ROS 2 Jazzy installed, custom messages package created (`ce_robot_interfaces`), basic publisher/subscriber knowledge

---

## **🎯 Learning Objectives**

By completing this lab series, you will be able to:

- ✅ Implement message validation and error handling
- ✅ Create conditional message filtering and processing
- ✅ Record and playback messages for testing and analysis
- ✅ Handle messages from multiple robots simultaneously
- ✅ Implement dynamic rate control and throttling
- ✅ Design and use complex nested message structures
- ✅ Track reliability metrics and success rates
- ✅ Write defensive code that handles invalid data
- ✅ Create alert mechanisms for critical conditions
- ✅ Implement production-quality error handling patterns

---

## **📊 Lab Architecture**

```
┌─────────────────────────────────────────────────────────┐
│                  Beginner Track                         │
├─────────────────────────────────────────────────────────┤
│ Exercise 1: Validation & Error Handling                 │
│ Exercise 2: Advanced Subscriber with Filtering          │
│ Exercise 3: Message Recording and Playback              │
└──────────────────┬──────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────────┐
│                Intermediate Track                       │
├─────────────────────────────────────────────────────────┤
│ Exercise 4: Multi-Robot Aggregator                      │
│ Exercise 5: Message Throttling and Rate Control         │
│ Exercise 6: Nested Message Types                        │
└─────────────────────────────────────────────────────────┘
```

---

## **📚 Learning Path Overview**

### **Beginner Track**
| Exercise | Title | Level | Duration |
|----------|-------|-------|----------|
| 1 | Validation & Error Handling | Beginner | 25 min |
| 2 | Advanced Subscriber with Filtering | Beginner | 20 min |
| 3 | Message Recording and Playback | Beginner | 25 min |

### **Intermediate Track**
| Exercise | Title | Level | Duration |
|----------|-------|-------|----------|
| 4 | Multi-Robot Aggregator | Intermediate | 25 min |
| 5 | Message Throttling and Rate Control | Intermediate | 20 min |
| 6 | Nested Message Types | Intermediate | 25 min |

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

## **Exercise 2: Advanced Subscriber with Filtering 🔍**

### 🎯 Objective
Create an advanced subscriber that filters messages based on conditions and calculates statistics on filtered data.

### 📝 Task

**Step 1: Create filtered subscriber**

Create `HardwareStatus_filter.py` in `ce_robot/ce_robot/`:
```python
#!/usr/bin/env python3
"""
Hardware Status Filter Subscriber
Filters and processes only high-temperature messages
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
from collections import deque


class HardwareStatusFilter(Node):
    def __init__(self):
        super().__init__('hardware_status_filter')
        self.subscriber = self.create_subscription(
            HardwareStatus, 'hardware_status', self.status_callback, 10
        )
        
        # Statistics tracking
        self.total_messages = 0
        self.high_temp_messages = 0
        self.temp_threshold = 50  # Filter threshold in °C
        self.high_temp_history = deque(maxlen=10)
        
        self.get_logger().info(f'Filter Subscriber started! Threshold: {self.temp_threshold}°C')

    def status_callback(self, msg):
        """Filter and process high-temperature messages"""
        self.total_messages += 1
        
        # Filter: Only process high temperature messages
        if msg.temperature > self.temp_threshold:
            self.high_temp_messages += 1
            self.high_temp_history.append(msg.temperature)
            
            # Calculate statistics on filtered data
            avg_high_temp = sum(self.high_temp_history) / len(self.high_temp_history)
            max_temp = max(self.high_temp_history)
            percentage = (self.high_temp_messages / self.total_messages) * 100
            
            self.get_logger().warn(
                f'🔥 HIGH TEMP DETECTED: {msg.name_robot} | '
                f'Temp: {msg.temperature}°C | '
                f'Count: {self.high_temp_messages}/{self.total_messages} ({percentage:.1f}%) | '
                f'Avg High: {avg_high_temp:.1f}°C | Max: {max_temp}°C'
            )
        else:
            # Log normal temperature messages at info level
            self.get_logger().info(
                f'✓ Normal: {msg.name_robot} | Temp: {msg.temperature}°C '
                f'(Total: {self.total_messages}, High: {self.high_temp_messages})'
            )


def main(args=None):
    rclpy.init(args=args)
    filter_node = HardwareStatusFilter()
    rclpy.spin(filter_node)
    filter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Make it executable:
```bash
chmod +x HardwareStatus_filter.py
```

**Step 2: Update setup.py**

Add entry point:
```python
"03_hw_status_filter = ce_robot.HardwareStatus_filter:main",
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

Terminal 2 (Filter Subscriber):
```bash
ros2 run ce_robot 03_hw_status_filter
```

### 🔍 Expected Output
```
[INFO] [hardware_status_filter]: Filter Subscriber started! Threshold: 50°C
[INFO] [hardware_status_filter]: ✓ Normal: CE-ROBOT | Temp: 45°C (Total: 1, High: 0)
[WARN] [hardware_status_filter]: 🔥 HIGH TEMP DETECTED: CE-ROBOT | Temp: 58°C | Count: 1/2 (50.0%) | Avg High: 58.0°C | Max: 58°C
[INFO] [hardware_status_filter]: ✓ Normal: CE-ROBOT | Temp: 39°C (Total: 3, High: 1)
[WARN] [hardware_status_filter]: 🔥 HIGH TEMP DETECTED: CE-ROBOT | Temp: 55°C | Count: 2/4 (50.0%) | Avg High: 56.5°C | Max: 58°C
```

### 💡 Key Learning Points
- **Conditional filtering**: Process only messages meeting specific criteria
- **Deque for windowed data**: Efficient rolling window for recent high-temp values
- **Statistical analysis**: Calculate percentage, average, and max on filtered data
- **Logging levels**: Use WARN for filtered events, INFO for normal
- **Real-time metrics**: Track and display filtering statistics

### ✅ Completion Criteria
- [ ] Created `HardwareStatus_filter.py`
- [ ] Implemented temperature threshold filtering (>50°C)
- [ ] Track total and filtered message counts
- [ ] Calculate percentage of high-temperature messages
- [ ] Store last 10 high-temperature readings in deque
- [ ] Calculate average and max of high temperatures
- [ ] Use appropriate logging levels (INFO/WARN)
- [ ] Entry point configured in setup.py
- [ ] Filter correctly identifies high-temperature messages
- [ ] Statistics display accurately

---

## **Exercise 3: Message Recording and Playback 💾**

### 🎯 Objective
Record hardware status messages to a file and implement playback functionality for testing and analysis.

### 📝 Task

**Step 1: Create message recorder**

Create `HardwareStatus_recorder.py` in `ce_robot/ce_robot/`:
```python
#!/usr/bin/env python3
"""
Hardware Status Recorder
Records messages to CSV file with timestamps
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
import csv
from datetime import datetime
import os


class HardwareStatusRecorder(Node):
    def __init__(self):
        super().__init__('hardware_status_recorder')
        self.subscriber = self.create_subscription(
            HardwareStatus, 'hardware_status', self.status_callback, 10
        )
        
        # Create recordings directory
        self.recordings_dir = os.path.expanduser('~/ros2_ws/recordings')
        os.makedirs(self.recordings_dir, exist_ok=True)
        
        # Create CSV file with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filename = f'{self.recordings_dir}/hw_status_{timestamp}.csv'
        
        # Initialize CSV file with headers
        with open(self.filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'name_robot', 'number_robot', 
                           'temperature', 'motor_ready', 'debug_message'])
        
        self.message_count = 0
        self.get_logger().info(f'Recorder started! Saving to: {self.filename}')

    def status_callback(self, msg):
        """Record message to CSV file"""
        try:
            timestamp = datetime.now().isoformat()
            
            with open(self.filename, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    timestamp,
                    msg.name_robot,
                    msg.number_robot,
                    msg.temperature,
                    msg.motor_ready,
                    msg.debug_message
                ])
            
            self.message_count += 1
            self.get_logger().info(
                f'📝 Recorded #{self.message_count}: {msg.name_robot} | '
                f'Temp: {msg.temperature}°C'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to record message: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    recorder = HardwareStatusRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info(
            f'Recording stopped. Total messages: {recorder.message_count}'
        )
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 2: Create message playback node**

Create `HardwareStatus_playback.py` in `ce_robot/ce_robot/`:
```python
#!/usr/bin/env python3
"""
Hardware Status Playback
Replays recorded messages from CSV file
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
import csv
import os
import sys


class HardwareStatusPlayback(Node):
    def __init__(self, filename, playback_speed=1.0):
        super().__init__('hardware_status_playback')
        self.publisher_ = self.create_publisher(HardwareStatus, 'hardware_status', 10)
        self.playback_speed = playback_speed
        
        # Load messages from CSV
        self.messages = []
        try:
            with open(filename, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    self.messages.append(row)
            
            self.get_logger().info(
                f'Loaded {len(self.messages)} messages from {filename} | '
                f'Playback speed: {playback_speed}x'
            )
        except FileNotFoundError:
            self.get_logger().error(f'File not found: {filename}')
            sys.exit(1)
        
        # Start playback
        self.current_index = 0
        playback_interval = 1.0 / playback_speed  # Adjust timer based on speed
        self.timer = self.create_timer(playback_interval, self.playback_callback)

    def playback_callback(self):
        """Publish next message from recording"""
        if self.current_index >= len(self.messages):
            self.get_logger().info('Playback completed!')
            self.timer.cancel()
            return
        
        row = self.messages[self.current_index]
        
        msg = HardwareStatus()
        msg.name_robot = row['name_robot']
        msg.number_robot = int(row['number_robot'])
        msg.temperature = int(row['temperature'])
        msg.motor_ready = row['motor_ready'].lower() == 'true'
        msg.debug_message = row['debug_message']
        
        self.publisher_.publish(msg)
        self.current_index += 1
        
        progress = (self.current_index / len(self.messages)) * 100
        self.get_logger().info(
            f'▶️  Playback [{self.current_index}/{len(self.messages)}] ({progress:.1f}%) | '
            f'{msg.name_robot} | Temp: {msg.temperature}°C'
        )


def main(args=None):
    rclpy.init(args=args)
    
    # Get filename from command line or use default
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        recordings_dir = os.path.expanduser('~/ros2_ws/recordings')
        # Find most recent recording
        files = sorted([f for f in os.listdir(recordings_dir) if f.endswith('.csv')])
        if not files:
            print('No recordings found!')
            sys.exit(1)
        filename = os.path.join(recordings_dir, files[-1])
    
    # Get playback speed (default 1.0x)
    playback_speed = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    
    playback = HardwareStatusPlayback(filename, playback_speed)
    rclpy.spin(playback)
    playback.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Make files executable:
```bash
chmod +x HardwareStatus_recorder.py
chmod +x HardwareStatus_playback.py
```

**Step 3: Update setup.py**

Add entry points:
```python
"03_hw_status_recorder = ce_robot.HardwareStatus_recorder:main",
"03_hw_status_playback = ce_robot.HardwareStatus_playback:main",
```

**Step 4: Build and test**

Build:
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

**Record messages:**
```bash
# Terminal 1: Start publisher
ros2 run ce_robot 03_hw_status_publisher

# Terminal 2: Start recorder (let it run for ~30 seconds, then Ctrl+C)
ros2 run ce_robot 03_hw_status_recorder
```

**Playback recorded messages:**
```bash
# Terminal 1: Start playback at 1x speed (default)
ros2 run ce_robot 03_hw_status_playback

# Terminal 2: Subscribe to see replayed messages
ros2 run ce_robot 03_hw_status_subscriber

# Or playback at 2x speed:
ros2 run ce_robot 03_hw_status_playback ~/ros2_ws/recordings/hw_status_YYYYMMDD_HHMMSS.csv 2.0
```

### 🔍 Expected Output

**Recorder:**
```
[INFO] [hardware_status_recorder]: Recorder started! Saving to: /home/user/ros2_ws/recordings/hw_status_20251208_143025.csv
[INFO] [hardware_status_recorder]: 📝 Recorded #1: CE-ROBOT | Temp: 47°C
[INFO] [hardware_status_recorder]: 📝 Recorded #2: CE-ROBOT | Temp: 52°C
[INFO] [hardware_status_recorder]: Recording stopped. Total messages: 30
```

**Playback:**
```
[INFO] [hardware_status_playback]: Loaded 30 messages from .../hw_status_20251208_143025.csv | Playback speed: 1.0x
[INFO] [hardware_status_playback]: ▶️  Playback [1/30] (3.3%) | CE-ROBOT | Temp: 47°C
[INFO] [hardware_status_playback]: ▶️  Playback [2/30] (6.7%) | CE-ROBOT | Temp: 52°C
[INFO] [hardware_status_playback]: Playback completed!
```

### 💡 Key Learning Points
- **Data persistence**: Save ROS messages to files for later analysis
- **CSV format**: Human-readable storage format
- **Timestamp tracking**: Record exact message reception time
- **File I/O operations**: Read/write operations in ROS nodes
- **Playback control**: Variable speed playback (1x, 2x, 0.5x)
- **Progress tracking**: Monitor playback completion percentage
- **Testing tool**: Replay scenarios for debugging

### ✅ Completion Criteria
- [ ] Created `HardwareStatus_recorder.py`
- [ ] Recorder saves messages to CSV with timestamps
- [ ] Recording directory created automatically
- [ ] Each recording has unique timestamped filename
- [ ] All message fields recorded correctly
- [ ] Created `HardwareStatus_playback.py`
- [ ] Playback loads messages from CSV file
- [ ] Playback publishes messages at configurable speed
- [ ] Progress percentage displayed during playback
- [ ] Entry points configured in setup.py
- [ ] Can record at least 30 messages
- [ ] Can playback recorded messages successfully
- [ ] Playback speed parameter works (1x, 2x, etc.)

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

## **Exercise 4: Multi-Robot Aggregator 🤖**

### 🎯 Objective
Handle messages from multiple robots simultaneously and track statistics for each robot separately.

### 📝 Task

**Step 1: Create multi-robot publisher**

Create `HardwareStatus_multi_publish.py` in `ce_robot/ce_robot/`:
```python
#!/usr/bin/env python3
"""
Multi-Robot Hardware Status Publisher
Publishes messages from 5 different robots
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
import random


class MultiRobotPublisher(Node):
    def __init__(self):
        super().__init__('multi_robot_publisher')
        self.publisher_ = self.create_publisher(HardwareStatus, 'hardware_status', 10)
        
        # Define multiple robots
        self.robots = [
            {'name': 'CE-ROBOT-A', 'number': 1001},
            {'name': 'CE-ROBOT-B', 'number': 1002},
            {'name': 'CE-ROBOT-C', 'number': 1003},
            {'name': 'CE-ROBOT-D', 'number': 1004},
            {'name': 'CE-ROBOT-E', 'number': 1005},
        ]
        
        self.current_robot_index = 0
        self.timer = self.create_timer(0.5, self.publish_callback)
        self.get_logger().info(f'Multi-Robot Publisher started! Publishing from {len(self.robots)} robots')

    def publish_callback(self):
        """Publish message from next robot in rotation"""
        robot = self.robots[self.current_robot_index]
        
        msg = HardwareStatus()
        msg.name_robot = robot['name']
        msg.number_robot = robot['number']
        msg.temperature = random.randint(35, 70)  # Wider range for variety
        msg.motor_ready = random.choice([True, False])
        msg.debug_message = f"Status from {robot['name']}"
        
        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published: {msg.name_robot} | Temp: {msg.temperature}°C | Motor: {msg.motor_ready}"
        )
        
        # Rotate to next robot
        self.current_robot_index = (self.current_robot_index + 1) % len(self.robots)


def main(args=None):
    rclpy.init(args=args)
    publisher = MultiRobotPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 2: Create multi-robot aggregator**

Create `HardwareStatus_multi_aggregate.py` in `ce_robot/ce_robot/`:
```python
#!/usr/bin/env python3
"""
Multi-Robot Aggregator
Tracks statistics for each robot separately
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
from collections import defaultdict
from datetime import datetime


class MultiRobotAggregator(Node):
    def __init__(self):
        super().__init__('multi_robot_aggregator')
        self.subscriber = self.create_subscription(
            HardwareStatus, 'hardware_status', self.status_callback, 10
        )
        
        # Track data per robot
        self.robot_data = defaultdict(lambda: {
            'count': 0,
            'total_temp': 0,
            'temps': [],
            'motor_on_count': 0,
            'last_seen': None
        })
        
        # Periodic report timer
        self.timer = self.create_timer(10.0, self.print_report)
        self.get_logger().info('Multi-Robot Aggregator started!')

    def status_callback(self, msg):
        """Aggregate data per robot"""
        robot_name = msg.name_robot
        data = self.robot_data[robot_name]
        
        # Update statistics
        data['count'] += 1
        data['total_temp'] += msg.temperature
        data['temps'].append(msg.temperature)
        if msg.motor_ready:
            data['motor_on_count'] += 1
        data['last_seen'] = datetime.now()
        
        self.get_logger().info(
            f"Received from {robot_name}: Temp={msg.temperature}°C, "
            f"Motor={msg.motor_ready} (Total: {data['count']} messages)"
        )

    def print_report(self):
        """Print aggregated statistics for all robots"""
        if not self.robot_data:
            self.get_logger().info('No data received yet')
            return
        
        self.get_logger().info('\n' + '='*70)
        self.get_logger().info('MULTI-ROBOT STATISTICS REPORT')
        self.get_logger().info('='*70)
        
        # Find robot with highest/lowest temperature
        all_temps = []
        for robot_name, data in self.robot_data.items():
            if data['temps']:
                avg_temp = data['total_temp'] / data['count']
                max_temp = max(data['temps'])
                min_temp = min(data['temps'])
                motor_rate = (data['motor_on_count'] / data['count']) * 100
                
                all_temps.append((robot_name, avg_temp))
                
                self.get_logger().info(
                    f"\n{robot_name}:\n"
                    f"  Messages: {data['count']}\n"
                    f"  Avg Temp: {avg_temp:.1f}°C | Max: {max_temp}°C | Min: {min_temp}°C\n"
                    f"  Motor Ready Rate: {motor_rate:.1f}%\n"
                    f"  Last Seen: {data['last_seen'].strftime('%H:%M:%S')}"
                )
        
        # Identify hottest and coldest robots
        if all_temps:
            hottest = max(all_temps, key=lambda x: x[1])
            coldest = min(all_temps, key=lambda x: x[1])
            
            self.get_logger().info(
                f"\n{'='*70}\n"
                f"🔥 HOTTEST: {hottest[0]} ({hottest[1]:.1f}°C)\n"
                f"❄️  COLDEST: {coldest[0]} ({coldest[1]:.1f}°C)\n"
                f"{'='*70}"
            )


def main(args=None):
    rclpy.init(args=args)
    aggregator = MultiRobotAggregator()
    rclpy.spin(aggregator)
    aggregator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Make files executable:
```bash
chmod +x HardwareStatus_multi_publish.py
chmod +x HardwareStatus_multi_aggregate.py
```

**Step 3: Update setup.py**

Add entry points:
```python
"03_hw_multi_publisher = ce_robot.HardwareStatus_multi_publish:main",
"03_hw_multi_aggregator = ce_robot.HardwareStatus_multi_aggregate:main",
```

**Step 4: Build and run**

Build:
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

Terminal 1 (Multi-Robot Publisher):
```bash
ros2 run ce_robot 03_hw_multi_publisher
```

Terminal 2 (Multi-Robot Aggregator):
```bash
ros2 run ce_robot 03_hw_multi_aggregator
```

### 🔍 Expected Output

**Aggregator Report (every 10 seconds):**
```
======================================================================
MULTI-ROBOT STATISTICS REPORT
======================================================================

CE-ROBOT-A:
  Messages: 20
  Avg Temp: 52.5°C | Max: 68°C | Min: 38°C
  Motor Ready Rate: 55.0%
  Last Seen: 14:32:15

CE-ROBOT-B:
  Messages: 20
  Avg Temp: 48.3°C | Max: 65°C | Min: 36°C
  Motor Ready Rate: 60.0%
  Last Seen: 14:32:14

======================================================================
🔥 HOTTEST: CE-ROBOT-A (52.5°C)
❄️  COLDEST: CE-ROBOT-E (45.2°C)
======================================================================
```

### 💡 Key Learning Points
- **Dictionary-based aggregation**: Track multiple data sources separately
- **defaultdict usage**: Automatically initialize new robot entries
- **Per-source statistics**: Calculate metrics for each robot independently
- **Comparative analysis**: Identify extremes (hottest/coldest robots)
- **Timestamp tracking**: Monitor last message reception time
- **Rotation pattern**: Round-robin publishing from multiple sources

### ✅ Completion Criteria
- [ ] Created `HardwareStatus_multi_publish.py`
- [ ] Publisher rotates through 5 different robots
- [ ] Each robot has unique name and ID
- [ ] Created `HardwareStatus_multi_aggregate.py`
- [ ] Aggregator tracks statistics per robot using dictionary
- [ ] Calculate per-robot: count, avg/max/min temp, motor ready rate
- [ ] Periodic report displays all robot statistics
- [ ] Identify and display hottest and coldest robots
- [ ] Track last seen timestamp for each robot
- [ ] Entry points configured in setup.py
- [ ] Multi-robot publisher cycles through all robots
- [ ] Aggregator correctly separates data by robot name

---

## **Exercise 5: Message Throttling and Rate Control ⏱️**

### 🎯 Objective
Implement dynamic rate control and throttling to manage message publication frequency adaptively.

### 📝 Task

**Step 1: Create adaptive rate publisher**

Create `HardwareStatus_adaptive_publish.py` in `ce_robot/ce_robot/`:
```python
#!/usr/bin/env python3
"""
Adaptive Rate Hardware Status Publisher
Dynamically adjusts publishing rate based on conditions
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
import random


class AdaptiveRatePublisher(Node):
    def __init__(self):
        super().__init__('adaptive_rate_publisher')
        self.publisher_ = self.create_publisher(HardwareStatus, 'hardware_status', 10)
        
        # Rate control parameters
        self.min_rate = 0.5  # Minimum: 1 message per 2 seconds (0.5 Hz)
        self.max_rate = 5.0  # Maximum: 5 messages per second (5 Hz)
        self.current_rate = 1.0  # Start at 1 Hz
        
        # Operating modes
        self.mode = 'NORMAL'  # NORMAL, BURST, THROTTLE
        self.message_count = 0
        
        # Create timer with initial rate
        self.timer = self.create_timer(1.0 / self.current_rate, self.publish_callback)
        
        # Mode change timer (change mode every 15 seconds for demo)
        self.mode_timer = self.create_timer(15.0, self.change_mode)
        
        self.get_logger().info(
            f'Adaptive Rate Publisher started! '
            f'Rate: {self.current_rate} Hz | Mode: {self.mode}'
        )

    def change_mode(self):
        """Cycle through different operating modes"""
        modes = ['NORMAL', 'BURST', 'THROTTLE']
        current_index = modes.index(self.mode)
        self.mode = modes[(current_index + 1) % len(modes)]
        
        # Adjust rate based on mode
        if self.mode == 'NORMAL':
            self.set_rate(1.0)
        elif self.mode == 'BURST':
            self.set_rate(5.0)  # Maximum rate
        elif self.mode == 'THROTTLE':
            self.set_rate(0.5)  # Minimum rate
        
        self.get_logger().info(
            f'🔄 MODE CHANGED: {self.mode} | New Rate: {self.current_rate} Hz'
        )

    def set_rate(self, new_rate):
        """Dynamically change publishing rate"""
        # Clamp rate to min/max bounds
        new_rate = max(self.min_rate, min(new_rate, self.max_rate))
        
        if new_rate != self.current_rate:
            self.current_rate = new_rate
            
            # Cancel old timer and create new one with new rate
            self.timer.cancel()
            self.timer = self.create_timer(1.0 / self.current_rate, self.publish_callback)
            
            self.get_logger().info(f'Rate adjusted to {self.current_rate} Hz')

    def publish_callback(self):
        """Publish hardware status message"""
        msg = HardwareStatus()
        msg.name_robot = 'CE-ROBOT'
        msg.number_robot = 1001
        msg.temperature = random.randint(35, 60)
        msg.motor_ready = True
        msg.debug_message = f"Mode: {self.mode} | Rate: {self.current_rate}Hz"
        
        self.publisher_.publish(msg)
        self.message_count += 1
        
        mode_emoji = {'NORMAL': '➡️', 'BURST': '⚡', 'THROTTLE': '🐢'}
        self.get_logger().info(
            f'{mode_emoji[self.mode]} [{self.mode}] #{self.message_count} | '
            f'Temp: {msg.temperature}°C | Rate: {self.current_rate} Hz'
        )


def main(args=None):
    rclpy.init(args=args)
    publisher = AdaptiveRatePublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 2: Create rate monitoring subscriber**

Create `HardwareStatus_rate_monitor.py` in `ce_robot/ce_robot/`:
```python
#!/usr/bin/env python3
"""
Rate Monitor Subscriber
Monitors and reports message reception rate
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
from collections import deque
from datetime import datetime


class RateMonitor(Node):
    def __init__(self):
        super().__init__('rate_monitor')
        self.subscriber = self.create_subscription(
            HardwareStatus, 'hardware_status', self.status_callback, 10
        )
        
        # Track message timestamps
        self.message_times = deque(maxlen=50)
        self.total_messages = 0
        
        # Report timer
        self.timer = self.create_timer(5.0, self.print_rate_report)
        
        self.get_logger().info('Rate Monitor started!')

    def status_callback(self, msg):
        """Record message reception time"""
        self.message_times.append(datetime.now())
        self.total_messages += 1

    def print_rate_report(self):
        """Calculate and report message reception rate"""
        if len(self.message_times) < 2:
            self.get_logger().info('Waiting for more messages...')
            return
        
        # Calculate rate from recent messages
        time_span = (self.message_times[-1] - self.message_times[0]).total_seconds()
        message_count = len(self.message_times)
        
        if time_span > 0:
            current_rate = (message_count - 1) / time_span
            
            self.get_logger().info(
                f'\n📊 RATE REPORT:\n'
                f'  Current Rate: {current_rate:.2f} Hz\n'
                f'  Total Messages: {self.total_messages}\n'
                f'  Window Size: {message_count} messages over {time_span:.1f}s'
            )


def main(args=None):
    rclpy.init(args=args)
    monitor = RateMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Make files executable:
```bash
chmod +x HardwareStatus_adaptive_publish.py
chmod +x HardwareStatus_rate_monitor.py
```

**Step 3: Update setup.py**

Add entry points:
```python
"03_hw_adaptive_publisher = ce_robot.HardwareStatus_adaptive_publish:main",
"03_hw_rate_monitor = ce_robot.HardwareStatus_rate_monitor:main",
```

**Step 4: Build and run**

Build:
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

Terminal 1 (Adaptive Publisher):
```bash
ros2 run ce_robot 03_hw_adaptive_publisher
```

Terminal 2 (Rate Monitor):
```bash
ros2 run ce_robot 03_hw_rate_monitor
```

### 🔍 Expected Output

**Adaptive Publisher:**
```
[INFO] Adaptive Rate Publisher started! Rate: 1.0 Hz | Mode: NORMAL
[INFO] ➡️ [NORMAL] #1 | Temp: 47°C | Rate: 1.0 Hz
[INFO] ➡️ [NORMAL] #2 | Temp: 52°C | Rate: 1.0 Hz
[INFO] 🔄 MODE CHANGED: BURST | New Rate: 5.0 Hz
[INFO] ⚡ [BURST] #15 | Temp: 45°C | Rate: 5.0 Hz
[INFO] ⚡ [BURST] #16 | Temp: 58°C | Rate: 5.0 Hz
[INFO] 🔄 MODE CHANGED: THROTTLE | New Rate: 0.5 Hz
[INFO] 🐢 [THROTTLE] #90 | Temp: 39°C | Rate: 0.5 Hz
```

**Rate Monitor:**
```
📊 RATE REPORT:
  Current Rate: 1.02 Hz
  Total Messages: 25
  Window Size: 25 messages over 24.5s

📊 RATE REPORT:
  Current Rate: 4.98 Hz
  Total Messages: 115
  Window Size: 50 messages over 10.0s
```

### 💡 Key Learning Points
- **Dynamic timer adjustment**: Cancel and recreate timers for rate changes
- **Operating modes**: Different behaviors for different scenarios
- **Rate clamping**: Enforce minimum and maximum rate limits
- **Rate measurement**: Calculate actual message frequency
- **Deque for time windowing**: Track recent message timestamps
- **Adaptive control**: Adjust behavior based on conditions
- **Burst vs throttle**: High-frequency vs low-frequency modes

### ✅ Completion Criteria
- [ ] Created `HardwareStatus_adaptive_publish.py`
- [ ] Implement three modes: NORMAL (1 Hz), BURST (5 Hz), THROTTLE (0.5 Hz)
- [ ] Dynamically change publishing rate
- [ ] Mode cycles automatically every 15 seconds
- [ ] Rate clamped to min (0.5 Hz) and max (5 Hz)
- [ ] Created `HardwareStatus_rate_monitor.py`
- [ ] Monitor calculates message reception rate
- [ ] Use deque to track recent 50 message timestamps
- [ ] Report actual rate every 5 seconds
- [ ] Entry points configured in setup.py
- [ ] Publisher rate changes correctly in each mode
- [ ] Monitor accurately measures reception rate

---

## **Exercise 6: Nested Message Types 🏗️**

### 🎯 Objective
Create and use complex nested message structures for more sophisticated data organization.

### 📝 Task

**Step 1: Define nested message types**

Create `msg/SensorData.msg` in `ce_robot_interfaces/msg/`:
```msg
# Sensor Data Sub-message
float32 voltage           # Battery voltage (V)
float32 current           # Current draw (A)
int32 cpu_usage          # CPU usage percentage (0-100)
int32 memory_usage       # Memory usage percentage (0-100)
```

Create `msg/RobotStatus.msg` in `ce_robot_interfaces/msg/`:
```msg
# Robot Status with Nested Sensor Data
string name_robot          # Robot identifier
int64 number_robot         # Robot ID number
int64 temperature          # Temperature in Celsius
bool motor_ready           # Motor status
SensorData sensor_data     # Nested sensor data
string[] warnings          # Array of warning messages
uint8 status_code         # Status code (0=OK, 1=Warning, 2=Error)
```

**Step 2: Update CMakeLists.txt**

Update in `ce_robot_interfaces/CMakeLists.txt`:
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "msg/SensorData.msg"
  "msg/RobotStatus.msg"
)
```

**Step 3: Build message package**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
source install/setup.bash
ros2 interface show ce_robot_interfaces/msg/RobotStatus
```

**Step 4: Create nested message publisher**

Create `RobotStatus_publish.py` in `ce_robot/ce_robot/`:
```python
#!/usr/bin/env python3
"""
Robot Status Publisher with Nested Messages
Publishes complex nested message structures
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import RobotStatus, SensorData
import random


class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_publisher')
        self.publisher_ = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_callback)
        self.message_count = 0
        self.get_logger().info('Robot Status Publisher (Nested) started!')

    def publish_callback(self):
        """Publish nested robot status message"""
        msg = RobotStatus()
        
        # Fill top-level fields
        msg.name_robot = 'CE-ROBOT-NEO'
        msg.number_robot = 2001
        msg.temperature = random.randint(35, 65)
        msg.motor_ready = random.choice([True, False])
        
        # Fill nested SensorData
        sensor = SensorData()
        sensor.voltage = round(random.uniform(11.5, 12.6), 2)
        sensor.current = round(random.uniform(0.5, 3.5), 2)
        sensor.cpu_usage = random.randint(20, 90)
        sensor.memory_usage = random.randint(30, 85)
        msg.sensor_data = sensor
        
        # Fill array of warnings
        warnings = []
        if msg.temperature > 55:
            warnings.append('High temperature detected')
        if sensor.voltage < 11.8:
            warnings.append('Low battery voltage')
        if sensor.cpu_usage > 80:
            warnings.append('High CPU usage')
        if sensor.memory_usage > 75:
            warnings.append('High memory usage')
        msg.warnings = warnings
        
        # Set status code
        if len(warnings) == 0:
            msg.status_code = 0  # OK
        elif len(warnings) <= 2:
            msg.status_code = 1  # Warning
        else:
            msg.status_code = 2  # Error
        
        self.publisher_.publish(msg)
        self.message_count += 1
        
        status_emoji = {0: '✅', 1: '⚠️', 2: '❌'}
        self.get_logger().info(
            f'{status_emoji[msg.status_code]} Published #{self.message_count}: '
            f'{msg.name_robot} | Temp: {msg.temperature}°C | '
            f'Battery: {sensor.voltage}V | CPU: {sensor.cpu_usage}% | '
            f'Warnings: {len(warnings)}'
        )


def main(args=None):
    rclpy.init(args=args)
    publisher = RobotStatusPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 5: Create nested message subscriber**

Create `RobotStatus_subscribe.py` in `ce_robot/ce_robot/`:
```python
#!/usr/bin/env python3
"""
Robot Status Subscriber for Nested Messages
Processes complex nested message structures
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import RobotStatus


class RobotStatusSubscriber(Node):
    def __init__(self):
        super().__init__('robot_status_subscriber')
        self.subscriber = self.create_subscription(
            RobotStatus, 'robot_status', self.status_callback, 10
        )
        self.received_count = 0
        self.get_logger().info('Robot Status Subscriber (Nested) started!')

    def status_callback(self, msg):
        """Process nested robot status message"""
        self.received_count += 1
        
        # Extract nested sensor data
        sensor = msg.sensor_data
        
        # Format status display
        status_text = {0: 'OK', 1: 'WARNING', 2: 'ERROR'}
        status_emoji = {0: '✅', 1: '⚠️', 2: '❌'}
        
        self.get_logger().info(
            f'\n{"="*70}\n'
            f'Message #{self.received_count} | Status: {status_emoji[msg.status_code]} {status_text[msg.status_code]}\n'
            f'{"="*70}\n'
            f'Robot: {msg.name_robot} (ID: {msg.number_robot})\n'
            f'Temperature: {msg.temperature}°C | Motor: {"Ready" if msg.motor_ready else "Not Ready"}\n'
            f'\nSensor Data:\n'
            f'  🔋 Voltage: {sensor.voltage}V | Current: {sensor.current}A\n'
            f'  💻 CPU: {sensor.cpu_usage}% | Memory: {sensor.memory_usage}%\n'
            f'\nWarnings ({len(msg.warnings)}):'
        )
        
        if msg.warnings:
            for i, warning in enumerate(msg.warnings, 1):
                self.get_logger().warn(f'  {i}. {warning}')
        else:
            self.get_logger().info('  No warnings - All systems normal')
        
        self.get_logger().info('='*70)


def main(args=None):
    rclpy.init(args=args)
    subscriber = RobotStatusSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Make files executable:
```bash
chmod +x RobotStatus_publish.py
chmod +x RobotStatus_subscribe.py
```

**Step 6: Update ce_robot package.xml**

Ensure dependency exists:
```xml
<depend>ce_robot_interfaces</depend>
```

**Step 7: Update setup.py**

Add entry points:
```python
"03_robot_status_publisher = ce_robot.RobotStatus_publish:main",
"03_robot_status_subscriber = ce_robot.RobotStatus_subscribe:main",
```

**Step 8: Build and run**

Build both packages:
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces ce_robot --symlink-install
source install/setup.bash
```

Terminal 1 (Nested Publisher):
```bash
ros2 run ce_robot 03_robot_status_publisher
```

Terminal 2 (Nested Subscriber):
```bash
ros2 run ce_robot 03_robot_status_subscriber
```

### 🔍 Expected Output

**Publisher:**
```
[INFO] Robot Status Publisher (Nested) started!
[INFO] ✅ Published #1: CE-ROBOT-NEO | Temp: 45°C | Battery: 12.3V | CPU: 45% | Warnings: 0
[INFO] ⚠️ Published #2: CE-ROBOT-NEO | Temp: 58°C | Battery: 11.7V | CPU: 85% | Warnings: 3
```

**Subscriber:**
```
======================================================================
Message #2 | Status: ⚠️ WARNING
======================================================================
Robot: CE-ROBOT-NEO (ID: 2001)
Temperature: 58°C | Motor: Ready

Sensor Data:
  🔋 Voltage: 11.7V | Current: 2.3A
  💻 CPU: 85% | Memory: 72%

Warnings (3):
[WARN]   1. High temperature detected
[WARN]   2. Low battery voltage
[WARN]   3. High CPU usage
======================================================================
```

### 💡 Key Learning Points
- **Nested message design**: Organize related data in sub-messages
- **Message composition**: Combine simple messages into complex structures
- **Array fields**: Use arrays for variable-length data (warnings)
- **Status codes**: Implement enumeration-style status indicators
- **Conditional logic**: Generate dynamic warnings based on sensor values
- **Structured data access**: Navigate nested message hierarchies
- **Complex message definition**: Multi-level message organization

### ✅ Completion Criteria
- [ ] Created `SensorData.msg` with 4 fields
- [ ] Created `RobotStatus.msg` with nested SensorData
- [ ] RobotStatus includes array field for warnings
- [ ] Updated CMakeLists.txt to generate both new messages
- [ ] Built ce_robot_interfaces successfully
- [ ] Verified messages with `ros2 interface show`
- [ ] Created `RobotStatus_publish.py`
- [ ] Publisher populates nested sensor data correctly
- [ ] Publisher generates dynamic warnings array
- [ ] Publisher sets appropriate status code (0/1/2)
- [ ] Created `RobotStatus_subscribe.py`
- [ ] Subscriber accesses nested sensor data fields
- [ ] Subscriber displays all warnings from array
- [ ] Subscriber formats output readably
- [ ] Entry points configured in setup.py
- [ ] Both nodes run without errors
- [ ] Nested data transmitted correctly

---

## **📂 Final Directory Structure**

```
📁 ROS2_WS/
├── 📁 src/
│   ├── 📁 ce_robot_interfaces/
│   │   ├── 📁 msg/
│   │   │   ├── 📄 HardwareStatus.msg
│   │   │   ├── 📄 SensorData.msg            # Exercise 6
│   │   │   └── 📄 RobotStatus.msg           # Exercise 6
│   │   ├── 📄 package.xml
│   │   └── 📄 CMakeLists.txt
│   └── 📁 ce_robot/
│       ├── 📁 ce_robot/
│       │   ├── 📄 __init__.py
│       │   ├── 🐍 HardwareStatus_publish.py
│       │   ├── 🐍 HardwareStatus_subscribe.py
│       │   ├── 🐍 HardwareStatus_aggregate.py
│       │   ├── 🐍 HardwareStatus_validated.py       # Exercise 1
│       │   ├── 🐍 HardwareStatus_validated_sub.py   # Exercise 1
│       │   ├── 🐍 HardwareStatus_filter.py          # Exercise 2
│       │   ├── 🐍 HardwareStatus_recorder.py        # Exercise 3
│       │   ├── 🐍 HardwareStatus_playback.py        # Exercise 3
│       │   ├── 🐍 HardwareStatus_multi_publish.py   # Exercise 4
│       │   ├── 🐍 HardwareStatus_multi_aggregate.py # Exercise 4
│       │   ├── 🐍 HardwareStatus_adaptive_publish.py # Exercise 5
│       │   ├── 🐍 HardwareStatus_rate_monitor.py    # Exercise 5
│       │   ├── 🐍 RobotStatus_publish.py            # Exercise 6
│       │   └── 🐍 RobotStatus_subscribe.py          # Exercise 6
│       ├── 📄 package.xml
│       ├── 📄 setup.cfg
│       └── 📄 setup.py
├── 📁 recordings/                                   # Exercise 3 output
│   └── 📄 hw_status_YYYYMMDD_HHMMSS.csv
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

### **Beginner Track Exercises**

#### Exercise 1: Validation & Error Handling ✅
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

#### Exercise 2: Advanced Subscriber with Filtering 🔍
- [ ] Created `HardwareStatus_filter.py`
- [ ] Filtering logic implemented (>50°C)
- [ ] Deque used for rolling window (10 messages)
- [ ] Statistics calculated correctly (avg, max, percentage)
- [ ] Appropriate logging levels used (INFO/WARN)
- [ ] Entry point `03_hw_status_filter` works
- [ ] Only high-temperature messages stored
- [ ] Filter percentage displayed correctly

#### Exercise 3: Message Recording and Playback 📼
- [ ] Created `HardwareStatus_recorder.py`
- [ ] Recording directory created (`~/ros2_ws/recordings/`)
- [ ] CSV files saved with timestamps
- [ ] All 5 message fields recorded
- [ ] Created `HardwareStatus_playback.py`
- [ ] Playback loads CSV files correctly
- [ ] Variable playback speed works (1x, 2x, etc.)
- [ ] Progress tracking displays correctly
- [ ] Command-line arguments functional
- [ ] Entry points configured: `03_hw_status_recorder`, `03_hw_status_playback`

### **Intermediate Track Exercises**

#### Exercise 4: Multi-Robot Aggregator 🤖
- [ ] Created `HardwareStatus_multi_publish.py`
- [ ] Publisher rotates through 5 robots
- [ ] Each robot has unique name/ID
- [ ] Created `HardwareStatus_multi_aggregate.py`
- [ ] Dictionary-based per-robot tracking
- [ ] Per-robot statistics calculated (count, avg/max/min temp, motor rate)
- [ ] Last seen timestamp tracked
- [ ] Periodic report displays all robots
- [ ] Identifies hottest and coldest robots
- [ ] Entry points configured: `03_hw_multi_publisher`, `03_hw_multi_aggregator`

#### Exercise 5: Message Throttling and Rate Control ⏱️
- [ ] Created `HardwareStatus_adaptive_publish.py`
- [ ] Three operating modes implemented (NORMAL, BURST, THROTTLE)
- [ ] Dynamic rate adjustment working
- [ ] Mode cycles automatically (every 15 seconds)
- [ ] Rate clamped to min/max (0.5 Hz - 5 Hz)
- [ ] Created `HardwareStatus_rate_monitor.py`
- [ ] Rate measurement calculates correctly
- [ ] Deque tracks recent 50 timestamps
- [ ] Periodic rate reports displayed
- [ ] Entry points configured: `03_hw_adaptive_publisher`, `03_hw_rate_monitor`

#### Exercise 6: Nested Message Types 🏗️
- [ ] Created `SensorData.msg` with 4 fields
- [ ] Created `RobotStatus.msg` with nested structure
- [ ] Array field for warnings included
- [ ] CMakeLists.txt updated for both messages
- [ ] Message package built successfully
- [ ] Verified with `ros2 interface show`
- [ ] Created `RobotStatus_publish.py`
- [ ] Nested sensor data populated correctly
- [ ] Dynamic warnings array generated
- [ ] Status codes set appropriately (0/1/2)
- [ ] Created `RobotStatus_subscribe.py`
- [ ] Nested data extracted correctly
- [ ] Warnings displayed from array
- [ ] Entry points configured: `03_robot_status_publisher`, `03_robot_status_subscriber`

### **Overall Lab Completion**
- [ ] All beginner track exercises completed (1-3)
- [ ] All intermediate track exercises completed (4-6)
- [ ] All Python files executable
- [ ] All entry points configured in setup.py
- [ ] Packages build without errors
- [ ] All nodes run successfully
- [ ] Recording/playback tested with real data
- [ ] Multi-robot aggregation tested
- [ ] Rate control tested in all modes
- [ ] Nested messages transmitted correctly

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
