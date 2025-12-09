# **🧪 ROS 2 Custom Messages - Lab Exercises**

**Real-World Mobile Robot Control and Monitoring**

---

## **📌 Lab Information**

**Lab Title:** Mobile Robot Status System  
**Duration:** 90 minutes  
**Level:** Beginner to Intermediate  
**Prerequisites:** 
- Completed Readme.md (Custom Message Package Setup)
- Created `ce_robot_interfaces` package
- Understanding of basic ROS 2 publisher/subscriber patterns

**What You Already Know from Readme:**
✅ Creating message packages and `.msg` files  
✅ Building custom messages with colcon  
✅ Basic publisher and subscriber patterns  
✅ Using `HardwareStatus.msg` example  

**What You'll Build in This Lab:**
🤖 **Complete mobile robot monitoring system**  
📊 **Real-time robot telemetry tracking**  
⚠️ **Safety monitoring and alerts**  
🗺️ **Position and navigation status**  
🔋 **Battery and power management**  

---

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🎯 Lab Objectives**

By completing this lab, you will be able to:

1. **Design** a comprehensive message for mobile robot monitoring
2. **Create** custom messages for real-world robotics applications
3. **Implement** robot status publishers with realistic data
4. **Build** monitoring subscribers with safety checks
5. **Track** robot position, battery, velocity, and health
6. **Generate** alerts for critical robot conditions
7. **Apply** professional robotics development patterns

---

## **🤖 Real-World Scenario**

You're developing a monitoring system for an autonomous mobile robot used in warehouse operations. The robot needs to report:
- **Position** (X, Y coordinates in meters)
- **Velocity** (linear and angular speed)
- **Battery** (voltage, percentage, charging status)
- **Safety** (obstacle detection, emergency stop status)
- **Mission** (current task, status, progress)

This lab will guide you through creating a complete `RobotStatus` message and implementing a professional monitoring system.

---

## **📚 Lab Structure**

```
┌─────────────────────────────────────────────┐
│ Setup: Create RobotStatus Message          │
│ Design comprehensive robot monitoring msg   │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 1: Robot Status Publisher          │
│ Implement realistic robot telemetry         │
└─────────────────────────────────────────────┘
```

| Step | Title | Duration | Difficulty |
|------|-------|----------|------------|
| Setup | Design RobotStatus.msg | 15 min | ⭐⭐ |
| Ex 1 | Status Publisher | 25 min | ⭐⭐ |

---

## **Setup: Create RobotStatus Message 📝**

### 🎯 Objective
Design and build a comprehensive message for mobile robot monitoring.

### 📖 Message Design

Create `RobotStatus.msg` in `ce_robot_interfaces/msg/`:

```bash
cd ~/ros2_ws/src/ce_robot_interfaces/msg
touch RobotStatus.msg
```

Define the message structure:

```msg
# RobotStatus.msg
# Comprehensive status message for autonomous mobile robot

# Header
std_msgs/Header header          # Timestamp and frame_id

# Robot Identification
string robot_id                 # Unique robot identifier (e.g., "AMR-001")
string robot_name               # Human-readable name (e.g., "Warehouse Bot Alpha")

# Position and Pose (in meters and radians)
float64 position_x              # X coordinate in map frame
float64 position_y              # Y coordinate in map frame
float64 orientation_z           # Yaw angle (rotation around Z axis)

# Velocity (m/s and rad/s)
float32 linear_velocity         # Forward/backward speed
float32 angular_velocity        # Rotation speed

# Battery Status
float32 battery_voltage         # Battery voltage (V)
uint8 battery_percentage        # Battery level 0-100%
bool is_charging                # Charging status
uint16 battery_time_remaining   # Estimated minutes remaining

# Safety Status
bool emergency_stop             # Emergency stop engaged
bool obstacle_detected          # Obstacle in path
float32 obstacle_distance       # Distance to nearest obstacle (meters)

# Mission Status
string current_mission          # Current task (e.g., "PICKING", "DELIVERING", "IDLE")
uint8 mission_progress          # Mission completion 0-100%
string mission_status           # Status: "IN_PROGRESS", "COMPLETED", "FAILED", "PAUSED"

# System Health
float32 cpu_usage               # CPU usage percentage
float32 temperature             # System temperature (Celsius)
bool motors_enabled             # Motor system status
string health_status            # Overall: "HEALTHY", "WARNING", "ERROR", "CRITICAL"
```

### 📝 Update Build Configuration

Edit `ce_robot_interfaces/CMakeLists.txt`:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "msg/RobotStatus.msg"
  DEPENDENCIES std_msgs
)
```

### 🔨 Build the Message

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
source install/setup.bash
```

### ✅ Verify Message Structure

```bash
ros2 interface show ce_robot_interfaces/msg/RobotStatus
```

You should see all 22 fields listed.

---

## **Exercise 1: Robot Status Publisher 🤖**

### 🎯 Objective
Create a comprehensive robot status publisher that simulates a real autonomous mobile robot with:
- Realistic movement simulation
- Battery discharge/recharge cycles
- Mission state management
- Safety monitoring

### 📖 Background
In real warehouse robots, the robot continuously publishes its status including position, velocity, battery level, and mission state. This exercise simulates a robot moving through waypoints, consuming battery, and updating its mission status.

### 📝 Tasks

**Step 1: Create Robot Status Publisher**

Create `RobotStatus_publisher.py` in `ce_robot/ce_robot/`:

```python
#!/usr/bin/env python3
"""
Validated Hardware Status Publisher
Adds input validation and error handling
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
import random


class ValidatedHwPublisher(Node):
    def __init__(self):
        super().__init__('validated_hw_publisher')
        self.publisher_ = self.create_publisher(HardwareStatus, 'hardware_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)
        
        # Statistics tracking
        self.total_attempts = 0
        self.successful_publishes = 0
        self.failed_publishes = 0
        
        self.get_logger().info('Validated HW Publisher started!')

    def validate_message(self, msg):
        """
        Validate message fields
        Returns: (is_valid, error_message)
        """
        # Check name_robot
        if not msg.name_robot or len(msg.name_robot) == 0:
            return False, "name_robot cannot be empty"
        
        # Check number_robot range
        if msg.number_robot < 1 or msg.number_robot > 9999:
            return False, f"number_robot must be 1-9999, got {msg.number_robot}"
        
        # Check temperature range
        if msg.temperature < -40 or msg.temperature > 100:
            return False, f"temperature out of range (-40 to 100°C): {msg.temperature}°C"
        
        # Check debug_message length
        if len(msg.debug_message) > 200:
            return False, "debug_message exceeds 200 characters"
        
        return True, "OK"

    def publish_status(self):
        """Create, validate, and publish message"""
        self.total_attempts += 1
        
        # Create message with random data
        msg = HardwareStatus()
        msg.name_robot = 'CE-ROBOT'
        msg.number_robot = random.randint(1000, 1010)  # Valid range
        msg.temperature = random.randint(30, 70)  # Mostly valid, some edge cases
        msg.motor_ready = True
        msg.debug_message = f'Status update #{self.total_attempts}'
        
        # Occasionally generate invalid data for testing
        if self.total_attempts % 10 == 0:
            msg.temperature = 150  # Invalid temperature!
        
        # Validate before publishing
        is_valid, error_msg = self.validate_message(msg)
        
        if is_valid:
            self.publisher_.publish(msg)
            self.successful_publishes += 1
            self.get_logger().info(
                f'✓ Published: {msg.name_robot} | Temp: {msg.temperature}°C | '
                f'Success Rate: {self.get_success_rate():.1f}%'
            )
        else:
            self.failed_publishes += 1
            self.get_logger().error(
                f'❌ Validation FAILED: {error_msg} | '
                f'Failed: {self.failed_publishes}/{self.total_attempts}'
            )

    def get_success_rate(self):
        """Calculate success percentage"""
        if self.total_attempts == 0:
            return 0.0
        return (self.successful_publishes / self.total_attempts) * 100


def main(args=None):
    rclpy.init(args=args)
    node = ValidatedHwPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'\n📊 Final Statistics:\n'
            f'  Total Attempts: {node.total_attempts}\n'
            f'  Successful: {node.successful_publishes}\n'
            f'  Failed: {node.failed_publishes}\n'
            f'  Success Rate: {node.get_success_rate():.1f}%'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 2: Make Executable**

```bash
chmod +x HardwareStatus_validated.py
```

**Step 3: Update setup.py**

Add entry point in `ce_robot/setup.py`:
```python
"03_hw_validated_publisher = ce_robot.HardwareStatus_validated:main",
```

**Step 4: Build and Test**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
ros2 run ce_robot 03_hw_validated_publisher
```

### 🔍 Expected Output

```
[INFO] [validated_hw_publisher]: Validated HW Publisher started!
[INFO] [validated_hw_publisher]: ✓ Published: CE-ROBOT | Temp: 45°C | Success Rate: 100.0%
[INFO] [validated_hw_publisher]: ✓ Published: CE-ROBOT | Temp: 52°C | Success Rate: 100.0%
...
[ERROR] [validated_hw_publisher]: ❌ Validation FAILED: temperature out of range (-40 to 100°C): 150°C | Failed: 1/10
[INFO] [validated_hw_publisher]: ✓ Published: CE-ROBOT | Temp: 48°C | Success Rate: 90.9%
```

### 💡 Key Learning Points

- **Validation function** returns boolean and error message
- **Range checks** prevent invalid data from being published
- **Statistics tracking** monitors reliability
- **Error logging** uses `.error()` level for failed validation
- **Success rate** shows percentage of valid messages
- **Defensive programming** catches problems before they cause issues

### ✅ Completion Checklist

- [ ] Created `HardwareStatus_validated.py`
- [ ] Implemented `validate_message()` function
- [ ] Added validation for all 5 fields
- [ ] Track statistics (total, success, failed)
- [ ] Calculate and display success rate
- [ ] Test with intentionally invalid data
- [ ] Validation correctly rejects bad data
- [ ] Success rate displays correctly

---

## **Exercise 2: Threshold-Based Filtering 🔍**

### 🎯 Objective
Create an **alert subscriber** that filters messages and triggers warnings for critical conditions.

### 📖 Background
**Difference from Readme:** The basic subscriber (`HardwareStatus_subscribe.py`) displays ALL messages. In monitoring systems, you need:
- **Conditional processing** - only react to critical values
- **Alert thresholds** - trigger warnings above certain limits
- **Filtered statistics** - calculate metrics only on important data
- **Different log levels** - WARN for alerts, INFO for normal

### 📝 Tasks

**Step 1: Create Filtering Subscriber**

Create `HardwareStatus_filter.py` in `ce_robot/ce_robot/`:

```python
#!/usr/bin/env python3
"""
Filtering Hardware Status Subscriber
Filters and tracks high-temperature messages
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
from collections import deque


class FilteringSubscriber(Node):
    def __init__(self):
        super().__init__('filtering_subscriber')
        self.subscription = self.create_subscription(
            HardwareStatus,
            'hardware_status',
            self.status_callback,
            10
        )
        
        # Configuration
        self.temp_threshold = 55  # Alert if temperature > 55°C
        
        # Statistics
        self.total_messages = 0
        self.high_temp_count = 0
        self.high_temp_values = deque(maxlen=10)  # Keep last 10 high temps
        
        self.get_logger().info(
            f'Filtering Subscriber started! Temperature threshold: {self.temp_threshold}°C'
        )

    def status_callback(self, msg):
        """Process incoming messages with filtering"""
        self.total_messages += 1
        
        # Filter: Only process high temperature messages
        if msg.temperature > self.temp_threshold:
            self.high_temp_count += 1
            self.high_temp_values.append(msg.temperature)
            
            # Calculate statistics on filtered data
            avg_high = sum(self.high_temp_values) / len(self.high_temp_values)
            max_high = max(self.high_temp_values)
            percentage = (self.high_temp_count / self.total_messages) * 100
            
            self.get_logger().warn(
                f'🔥 HIGH TEMP ALERT: {msg.temperature}°C from {msg.name_robot} | '
                f'Count: {self.high_temp_count}/{self.total_messages} ({percentage:.1f}%) | '
                f'Avg: {avg_high:.1f}°C | Max: {max_high}°C'
            )
        else:
            # Log normal messages at INFO level
            self.get_logger().info(
                f'✓ Normal temp: {msg.temperature}°C from {msg.name_robot} | '
                f'Total: {self.total_messages}, High: {self.high_temp_count}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = FilteringSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'\n📊 Filter Statistics:\n'
            f'  Total Messages: {node.total_messages}\n'
            f'  High Temp Messages: {node.high_temp_count}\n'
            f'  Percentage: {(node.high_temp_count/node.total_messages*100):.1f}%'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 2: Make Executable**

```bash
chmod +x HardwareStatus_filter.py
```

**Step 3: Update setup.py**

```python
"03_hw_filter_subscriber = ce_robot.HardwareStatus_filter:main",
```

**Step 4: Build and Test**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

Run both nodes:
```bash
# Terminal 1
ros2 run ce_robot 03_hw_validated_publisher

# Terminal 2
ros2 run ce_robot 03_hw_filter_subscriber
```

### 🔍 Expected Output

```
[INFO] [filtering_subscriber]: Filtering Subscriber started! Temperature threshold: 55°C
[INFO] [filtering_subscriber]: ✓ Normal temp: 45°C from CE-ROBOT | Total: 1, High: 0
[INFO] [filtering_subscriber]: ✓ Normal temp: 52°C from CE-ROBOT | Total: 2, High: 0
[WARN] [filtering_subscriber]: 🔥 HIGH TEMP ALERT: 58°C from CE-ROBOT | Count: 1/3 (33.3%) | Avg: 58.0°C | Max: 58°C
[INFO] [filtering_subscriber]: ✓ Normal temp: 49°C from CE-ROBOT | Total: 4, High: 1
[WARN] [filtering_subscriber]: 🔥 HIGH TEMP ALERT: 67°C from CE-ROBOT | Count: 2/5 (40.0%) | Avg: 62.5°C | Max: 67°C
```

### 💡 Key Learning Points

- **Conditional filtering** processes only relevant messages
- **deque** efficiently stores last N values (rolling window)
- **Logging levels** (INFO vs WARN) show message importance
- **Percentage calculation** shows proportion of filtered messages
- **Statistical analysis** on filtered data (average, max)
- **Real-time monitoring** tracks patterns over time

### ✅ Completion Checklist

- [ ] Created `HardwareStatus_filter.py`
- [ ] Implemented temperature threshold filtering (>55°C)
- [ ] Used deque to store last 10 high temps
- [ ] Calculate percentage of high-temp messages
- [ ] Calculate average and max of high temps
- [ ] Use WARN level for alerts, INFO for normal
- [ ] Test with both publishers
- [ ] Filter correctly identifies high temperatures

---

## **Exercise 3: Advanced Statistics with Rolling Windows 📊**

### 🎯 Objective
Extend the basic aggregator with **rolling windows**, **periodic reporting**, and **comprehensive metrics**.

### 📖 Background
**Difference from Readme:** The basic aggregator (`HardwareStatus_aggregate.py`) only calculates:
- Simple count and average temperature

**This exercise adds (NEW):**
- **Rolling windows** - track last N values (not all history)
- **Min/Max tracking** - temperature extremes
- **Message rate calculation** - messages per second
- **Motor status statistics** - percentage ready vs not ready
- **Periodic reports** - automatic summary every 10 seconds
- **Formatted output** - professional report display

### 📝 Tasks

**Step 1: Create Statistics Node**

Create `HardwareStatus_stats.py` in `ce_robot/ce_robot/`:

```python
#!/usr/bin/env python3
"""
Statistics and Aggregation Node
Comprehensive data analysis and reporting
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
from collections import deque
from datetime import datetime


class StatisticsNode(Node):
    def __init__(self):
        super().__init__('statistics_node')
        self.subscription = self.create_subscription(
            HardwareStatus,
            'hardware_status',
            self.status_callback,
            10
        )
        
        # Data storage
        self.temperatures = deque(maxlen=50)  # Last 50 temperatures
        self.message_times = deque(maxlen=50)  # Last 50 message timestamps
        
        # Counters
        self.total_messages = 0
        self.motor_ready_count = 0
        self.motor_not_ready_count = 0
        
        # Min/Max tracking
        self.min_temp = float('inf')
        self.max_temp = float('-inf')
        
        # Create timer for periodic reports (every 10 seconds)
        self.report_timer = self.create_timer(10.0, self.print_report)
        
        self.get_logger().info('Statistics Node started! Collecting data...')

    def status_callback(self, msg):
        """Collect data from incoming messages"""
        self.total_messages += 1
        current_time = datetime.now()
        
        # Store temperature data
        self.temperatures.append(msg.temperature)
        self.message_times.append(current_time)
        
        # Track min/max
        if msg.temperature < self.min_temp:
            self.min_temp = msg.temperature
        if msg.temperature > self.max_temp:
            self.max_temp = msg.temperature
        
        # Track motor status
        if msg.motor_ready:
            self.motor_ready_count += 1
        else:
            self.motor_not_ready_count += 1
        
        # Log basic info
        self.get_logger().info(
            f'Received #{self.total_messages}: {msg.name_robot} | '
            f'Temp: {msg.temperature}°C | Motor: {msg.motor_ready}'
        )

    def print_report(self):
        """Generate and print comprehensive statistics report"""
        if self.total_messages == 0:
            self.get_logger().info('No data received yet...')
            return
        
        # Calculate statistics
        avg_temp = sum(self.temperatures) / len(self.temperatures)
        motor_ready_pct = (self.motor_ready_count / self.total_messages) * 100
        
        # Calculate message rate
        if len(self.message_times) >= 2:
            time_span = (self.message_times[-1] - self.message_times[0]).total_seconds()
            message_rate = len(self.message_times) / time_span if time_span > 0 else 0
        else:
            message_rate = 0
        
        # Print comprehensive report
        self.get_logger().info(
            f'\n'
            f'╔══════════════════════════════════════════════════════╗\n'
            f'║           STATISTICS REPORT                          ║\n'
            f'╠══════════════════════════════════════════════════════╣\n'
            f'║ Total Messages:     {self.total_messages:>6}                        ║\n'
            f'║ Message Rate:       {message_rate:>6.2f} msg/sec                 ║\n'
            f'║                                                      ║\n'
            f'║ Temperature Statistics:                              ║\n'
            f'║   Average:          {avg_temp:>6.1f}°C                      ║\n'
            f'║   Minimum:          {self.min_temp:>6}°C                      ║\n'
            f'║   Maximum:          {self.max_temp:>6}°C                      ║\n'
            f'║   Range:            {self.max_temp - self.min_temp:>6}°C                      ║\n'
            f'║                                                      ║\n'
            f'║ Motor Status:                                        ║\n'
            f'║   Ready:            {self.motor_ready_count:>6} ({motor_ready_pct:>5.1f}%)            ║\n'
            f'║   Not Ready:        {self.motor_not_ready_count:>6} ({100-motor_ready_pct:>5.1f}%)            ║\n'
            f'╚══════════════════════════════════════════════════════╝'
        )


def main(args=None):
    rclpy.init(args=args)
    node = StatisticsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_report()  # Final report on exit
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 2: Make Executable**

```bash
chmod +x HardwareStatus_stats.py
```

**Step 3: Update setup.py**

```python
"03_hw_statistics = ce_robot.HardwareStatus_stats:main",
```

**Step 4: Build and Test**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

Run the statistics node:
```bash
# Terminal 1
ros2 run ce_robot 03_hw_validated_publisher

# Terminal 2
ros2 run ce_robot 03_hw_statistics
```

### 🔍 Expected Output

```
[INFO] [statistics_node]: Statistics Node started! Collecting data...
[INFO] [statistics_node]: Received #1: CE-ROBOT | Temp: 45°C | Motor: True
[INFO] [statistics_node]: Received #2: CE-ROBOT | Temp: 52°C | Motor: True
...
[INFO] [statistics_node]: 
╔══════════════════════════════════════════════════════╗
║           STATISTICS REPORT                          ║
╠══════════════════════════════════════════════════════╣
║ Total Messages:         10                           ║
║ Message Rate:         1.00 msg/sec                   ║
║                                                      ║
║ Temperature Statistics:                              ║
║   Average:            52.3°C                         ║
║   Minimum:              45°C                         ║
║   Maximum:              67°C                         ║
║   Range:                22°C                         ║
║                                                      ║
║ Motor Status:                                        ║
║   Ready:                10 (100.0%)                  ║
║   Not Ready:             0 (  0.0%)                  ║
╚══════════════════════════════════════════════════════╝
```

### 💡 Key Learning Points

- **Periodic reporting** using timers
- **Rolling window** with maxlen deque
- **Min/max tracking** across all messages
- **Rate calculation** using timestamps
- **Percentage calculations** for categorical data
- **Formatted output** for readability
- **Comprehensive metrics** in single report

### ✅ Completion Checklist

- [ ] Created `HardwareStatus_stats.py`
- [ ] Store last 50 temperatures in deque (rolling window)
- [ ] Track message timestamps for rate calculation
- [ ] Calculate average temperature (on rolling window)
- [ ] Track min/max temperatures (across all messages)
- [ ] Count motor ready/not ready percentages
- [ ] Calculate message rate (msg/sec)
- [ ] Generate report every 10 seconds automatically
- [ ] Format report with box drawing characters
- [ ] Test with validated publisher running

---

## **Exercise 4: Multi-Robot Monitor 🤖🤖**

### 🎯 Objective
Create a monitoring system that tracks **multiple robots simultaneously** and provides per-robot statistics.

### 📖 Background
**New Concept:** All previous examples tracked a single robot. In real systems:
- Multiple robots publish to the same topic
- Each robot has a unique `name_robot` identifier
- Monitor needs to track statistics **per robot**
- Need to identify which robot has problems

### 📝 Tasks

**Step 1: Create Multi-Robot Monitor**

Create `HardwareStatus_multi_robot.py` in `ce_robot/ce_robot/`:

```python
#!/usr/bin/env python3
"""
Multi-Robot Hardware Monitor
Tracks multiple robots and provides per-robot statistics
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
from collections import defaultdict, deque


class MultiRobotMonitor(Node):
    def __init__(self):
        super().__init__('multi_robot_monitor')
        self.subscription = self.create_subscription(
            HardwareStatus,
            'hardware_status',
            self.status_callback,
            10
        )
        
        # Per-robot data storage (dictionary of robot_name -> data)
        self.robot_data = defaultdict(lambda: {
            'count': 0,
            'temperatures': deque(maxlen=10),
            'motor_ready_count': 0,
            'motor_not_ready_count': 0,
            'last_seen': None
        })
        
        self.total_messages = 0
        
        # Create timer for periodic reports (every 15 seconds)
        self.report_timer = self.create_timer(15.0, self.print_multi_robot_report)
        
        self.get_logger().info('Multi-Robot Monitor started! Tracking all robots...')

    def status_callback(self, msg):
        """Collect data per robot"""
        self.total_messages += 1
        robot_name = msg.name_robot
        
        # Update robot-specific data
        data = self.robot_data[robot_name]
        data['count'] += 1
        data['temperatures'].append(msg.temperature)
        data['last_seen'] = self.get_clock().now()
        
        if msg.motor_ready:
            data['motor_ready_count'] += 1
        else:
            data['motor_not_ready_count'] += 1
        
        # Log individual message
        self.get_logger().info(
            f'[{robot_name}] Temp: {msg.temperature}°C | Motor: {msg.motor_ready} | '
            f'Robot Count: {data["count"]} | Total: {self.total_messages}'
        )

    def print_multi_robot_report(self):
        """Generate comprehensive multi-robot report"""
        if self.total_messages == 0:
            self.get_logger().info('No data received yet...')
            return
        
        num_robots = len(self.robot_data)
        
        report = f'\n'
        report += f'╔═══════════════════════════════════════════════════════════╗\n'
        report += f'║        MULTI-ROBOT MONITORING REPORT                      ║\n'
        report += f'╠═══════════════════════════════════════════════════════════╣\n'
        report += f'║ Total Messages:    {self.total_messages:>6}                              ║\n'
        report += f'║ Active Robots:     {num_robots:>6}                              ║\n'
        report += f'╠═══════════════════════════════════════════════════════════╣\n'
        
        # Per-robot statistics
        for robot_name, data in sorted(self.robot_data.items()):
            count = data['count']
            temps = list(data['temperatures'])
            
            if len(temps) > 0:
                avg_temp = sum(temps) / len(temps)
                min_temp = min(temps)
                max_temp = max(temps)
            else:
                avg_temp = min_temp = max_temp = 0
            
            motor_ready_pct = (data['motor_ready_count'] / count * 100) if count > 0 else 0
            
            report += f'║ Robot: {robot_name:<20}                        ║\n'
            report += f'║   Messages:        {count:>6}                              ║\n'
            report += f'║   Avg Temp:        {avg_temp:>6.1f}°C                          ║\n'
            report += f'║   Min/Max:         {min_temp:>3}°C / {max_temp:>3}°C                        ║\n'
            report += f'║   Motor Ready:     {motor_ready_pct:>6.1f}%                           ║\n'
            report += f'╠═══════════════════════════════════════════════════════════╣\n'
        
        report += f'╚═══════════════════════════════════════════════════════════╝'
        
        self.get_logger().info(report)
        
        # Alert for robots with high average temperature
        for robot_name, data in self.robot_data.items():
            temps = list(data['temperatures'])
            if len(temps) > 0:
                avg_temp = sum(temps) / len(temps)
                if avg_temp > 60:
                    self.get_logger().warn(
                        f'⚠️  HIGH TEMP WARNING: {robot_name} average is {avg_temp:.1f}°C!'
                    )


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_multi_robot_report()  # Final report on exit
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 2: Create Multi-Robot Publisher**

Create `HardwareStatus_multi_pub.py` to simulate multiple robots:

```python
#!/usr/bin/env python3
"""
Multi-Robot Publisher
Simulates multiple robots publishing to the same topic
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
import random


class MultiRobotPublisher(Node):
    def __init__(self):
        super().__init__('multi_robot_publisher')
        self.publisher_ = self.create_publisher(HardwareStatus, 'hardware_status', 10)
        self.timer = self.create_timer(0.5, self.publish_status)  # Publish twice per second
        
        # Simulate 3 different robots
        self.robots = [
            {'name': 'CE-ROBOT-01', 'number': 1001, 'temp_base': 45},
            {'name': 'CE-ROBOT-02', 'number': 1002, 'temp_base': 55},
            {'name': 'CE-ROBOT-03', 'number': 1003, 'temp_base': 40},
        ]
        
        self.publish_count = 0
        self.get_logger().info('Multi-Robot Publisher started! Simulating 3 robots...')

    def publish_status(self):
        """Publish messages from different robots in rotation"""
        self.publish_count += 1
        
        # Round-robin through robots
        robot = self.robots[self.publish_count % len(self.robots)]
        
        msg = HardwareStatus()
        msg.name_robot = robot['name']
        msg.number_robot = robot['number']
        msg.temperature = robot['temp_base'] + random.randint(-5, 15)
        msg.motor_ready = random.choice([True, True, True, False])  # 75% ready
        msg.debug_message = f'Status update #{self.publish_count}'
        
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published: {msg.name_robot} | Temp: {msg.temperature}°C | Motor: {msg.motor_ready}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 3: Make Executable**

```bash
chmod +x HardwareStatus_multi_robot.py
chmod +x HardwareStatus_multi_pub.py
```

**Step 4: Update setup.py**

```python
"03_hw_multi_robot_monitor = ce_robot.HardwareStatus_multi_robot:main",
"03_hw_multi_robot_publisher = ce_robot.HardwareStatus_multi_pub:main",
```

**Step 5: Build and Test**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

Run both nodes:
```bash
# Terminal 1
ros2 run ce_robot 03_hw_multi_robot_publisher

# Terminal 2
ros2 run ce_robot 03_hw_multi_robot_monitor
```

### 🔍 Expected Output

**Publisher Terminal:**
```
[INFO] [multi_robot_publisher]: Published: CE-ROBOT-01 | Temp: 48°C | Motor: True
[INFO] [multi_robot_publisher]: Published: CE-ROBOT-02 | Temp: 62°C | Motor: True
[INFO] [multi_robot_publisher]: Published: CE-ROBOT-03 | Temp: 43°C | Motor: False
[INFO] [multi_robot_publisher]: Published: CE-ROBOT-01 | Temp: 51°C | Motor: True
```

**Monitor Terminal:**
```
[INFO] [multi_robot_monitor]: Multi-Robot Monitor started! Tracking all robots...
[INFO] [multi_robot_monitor]: [CE-ROBOT-01] Temp: 48°C | Motor: True | Robot Count: 1 | Total: 1
[INFO] [multi_robot_monitor]: [CE-ROBOT-02] Temp: 62°C | Motor: True | Robot Count: 1 | Total: 2
[INFO] [multi_robot_monitor]: [CE-ROBOT-03] Temp: 43°C | Motor: False | Robot Count: 1 | Total: 3
...
[INFO] [multi_robot_monitor]: 
╔═══════════════════════════════════════════════════════════╗
║        MULTI-ROBOT MONITORING REPORT                      ║
╠═══════════════════════════════════════════════════════════╣
║ Total Messages:        30                                 ║
║ Active Robots:          3                                 ║
╠═══════════════════════════════════════════════════════════╣
║ Robot: CE-ROBOT-01                                        ║
║   Messages:            10                                 ║
║   Avg Temp:          47.5°C                               ║
║   Min/Max:            42°C /  53°C                        ║
║   Motor Ready:        80.0%                               ║
╠═══════════════════════════════════════════════════════════╣
║ Robot: CE-ROBOT-02                                        ║
║   Messages:            10                                 ║
║   Avg Temp:          59.2°C                               ║
║   Min/Max:            54°C /  68°C                        ║
║   Motor Ready:        70.0%                               ║
╠═══════════════════════════════════════════════════════════╣
║ Robot: CE-ROBOT-03                                        ║
║   Messages:            10                                 ║
║   Avg Temp:          41.8°C                               ║
║   Min/Max:            37°C /  48°C                        ║
║   Motor Ready:        90.0%                               ║
╠═══════════════════════════════════════════════════════════╣
╚═══════════════════════════════════════════════════════════╝
[WARN] [multi_robot_monitor]: ⚠️  HIGH TEMP WARNING: CE-ROBOT-02 average is 59.2°C!
```

### 💡 Key Learning Points

- **defaultdict** for dynamic robot tracking
- **Per-entity statistics** using dictionaries
- **Shared topics** - multiple publishers on same topic
- **Entity identification** using message fields
- **Comparative analysis** across multiple sources
- **Automated alerts** for specific robots

### ✅ Completion Checklist

- [ ] Created `HardwareStatus_multi_robot.py`
- [ ] Created `HardwareStatus_multi_pub.py`
- [ ] Used defaultdict for dynamic robot tracking
- [ ] Track per-robot statistics separately
- [ ] Calculate per-robot averages and ranges
- [ ] Display all robots in single report
- [ ] Alert for robots with high temperature
- [ ] Test with 3 simulated robots
- [ ] Verify per-robot data is correctly separated

---

## **Exercise 5: Create Your Own Custom Message 📝**

### 🎯 Objective
Design and implement your **own custom message** for a battery monitoring system from scratch.

### 📖 Background
**New Skill:** So far, you've used the `HardwareStatus.msg` that was created in the Readme. Now you'll practice the complete workflow:
1. Design a message structure for a specific use case
2. Create the `.msg` file
3. Update build configuration
4. Build the message
5. Use it in publisher and subscriber

### 📝 Scenario
You're building a **battery monitoring system** for a fleet of robots. You need to track:
- Battery ID
- Voltage (in volts)
- Current (in amps)
- Charge percentage (0-100)
- Battery health status
- Time remaining (in minutes)

### Tasks

**Step 1: Design the Message**

Create `BatteryStatus.msg` in `ce_robot_interfaces/msg/`:

```bash
cd ~/ros2_ws/src/ce_robot_interfaces/msg
touch BatteryStatus.msg
```

Define the message structure:

```msg
# BatteryStatus.msg
# Message for monitoring robot battery status

string battery_id           # Unique battery identifier (e.g., "BAT-001")
float32 voltage             # Battery voltage in volts (e.g., 12.0-14.8V)
float32 current             # Current draw in amps (e.g., 0.0-10.0A)
uint8 charge_percentage     # Charge level 0-100%
string health_status        # Battery health: "Good", "Fair", "Poor", "Critical"
uint16 time_remaining       # Estimated time remaining in minutes
bool is_charging            # True if battery is currently charging
int64 timestamp             # Unix timestamp of reading
```

**Step 2: Update CMakeLists.txt**

Edit `ce_robot_interfaces/CMakeLists.txt` to include the new message:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "msg/BatteryStatus.msg"
)
```

**Step 3: Build the New Message**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
source install/setup.bash
```

**Step 4: Verify Message Structure**

```bash
ros2 interface show ce_robot_interfaces/msg/BatteryStatus
```

**Step 5: Create Battery Publisher**

Create `BatteryStatus_pub.py` in `ce_robot/ce_robot/`:

```python
#!/usr/bin/env python3
"""
Battery Status Publisher
Publishes battery monitoring data with realistic simulation
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import BatteryStatus
import random
import time


class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        self.publisher_ = self.create_publisher(BatteryStatus, 'battery_status', 10)
        self.timer = self.create_timer(2.0, self.publish_battery_status)
        
        # Simulate battery discharge
        self.charge_level = 100.0
        self.is_charging = False
        self.cycle_count = 0
        
        self.get_logger().info('Battery Publisher started!')

    def get_health_status(self, charge):
        """Determine health status based on charge level"""
        if charge > 75:
            return "Good"
        elif charge > 50:
            return "Fair"
        elif charge > 20:
            return "Poor"
        else:
            return "Critical"

    def publish_battery_status(self):
        """Publish battery status with realistic simulation"""
        self.cycle_count += 1
        
        msg = BatteryStatus()
        msg.battery_id = "BAT-001"
        
        # Simulate charging/discharging cycle
        if self.charge_level <= 20:
            self.is_charging = True
        elif self.charge_level >= 95:
            self.is_charging = False
        
        # Update charge level
        if self.is_charging:
            self.charge_level = min(100.0, self.charge_level + random.uniform(2.0, 5.0))
        else:
            self.charge_level = max(0.0, self.charge_level - random.uniform(0.5, 2.0))
        
        # Calculate voltage (proportional to charge)
        msg.voltage = 10.0 + (self.charge_level / 100.0) * 4.8  # 10V-14.8V range
        
        # Calculate current
        if self.is_charging:
            msg.current = random.uniform(3.0, 5.0)  # Charging current
        else:
            msg.current = random.uniform(1.0, 3.0)  # Discharge current
        
        msg.charge_percentage = int(self.charge_level)
        msg.health_status = self.get_health_status(self.charge_level)
        msg.time_remaining = int(self.charge_level * 1.5)  # Rough estimate in minutes
        msg.is_charging = self.is_charging
        msg.timestamp = int(time.time())
        
        self.publisher_.publish(msg)
        
        status_icon = "🔌" if self.is_charging else "🔋"
        self.get_logger().info(
            f'{status_icon} {msg.battery_id} | '
            f'{msg.voltage:.1f}V | {msg.current:.1f}A | '
            f'{msg.charge_percentage}% | {msg.health_status} | '
            f'~{msg.time_remaining}min'
        )


def main(args=None):
    rclpy.init(args=args)
    node = BatteryPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 6: Create Battery Subscriber**

Create `BatteryStatus_sub.py`:

```python
#!/usr/bin/env python3
"""
Battery Status Subscriber
Monitors battery data and generates alerts
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import BatteryStatus
from collections import deque


class BatterySubscriber(Node):
    def __init__(self):
        super().__init__('battery_subscriber')
        self.subscription = self.create_subscription(
            BatteryStatus,
            'battery_status',
            self.battery_callback,
            10
        )
        
        self.voltage_history = deque(maxlen=10)
        self.message_count = 0
        
        self.get_logger().info('Battery Subscriber started! Monitoring battery health...')

    def battery_callback(self, msg):
        """Process battery status and generate alerts"""
        self.message_count += 1
        self.voltage_history.append(msg.voltage)
        
        # Alert for critical battery
        if msg.health_status == "Critical":
            self.get_logger().error(
                f'⚠️  CRITICAL: {msg.battery_id} at {msg.charge_percentage}%! '
                f'Immediate action required!'
            )
        elif msg.health_status == "Poor":
            self.get_logger().warn(
                f'⚡ LOW BATTERY: {msg.battery_id} at {msg.charge_percentage}%'
            )
        
        # Alert for voltage anomalies
        if len(self.voltage_history) >= 5:
            avg_voltage = sum(self.voltage_history) / len(self.voltage_history)
            if abs(msg.voltage - avg_voltage) > 1.5:
                self.get_logger().warn(
                    f'⚠️  VOLTAGE SPIKE: {msg.voltage:.1f}V '
                    f'(avg: {avg_voltage:.1f}V)'
                )
        
        # Normal status logging
        charge_icon = "🔌" if msg.is_charging else "🔋"
        self.get_logger().info(
            f'[{self.message_count}] {charge_icon} {msg.battery_id} | '
            f'{msg.voltage:.1f}V | {msg.charge_percentage}% | '
            f'{msg.health_status} | Est: {msg.time_remaining}min'
        )


def main(args=None):
    rclpy.init(args=args)
    node = BatterySubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 7: Make Executable**

```bash
chmod +x BatteryStatus_pub.py
chmod +x BatteryStatus_sub.py
```

**Step 8: Update setup.py**

Add to `ce_robot/setup.py`:

```python
'03_battery_publisher = ce_robot.BatteryStatus_pub:main',
'03_battery_subscriber = ce_robot.BatteryStatus_sub:main',
```

**Step 9: Build and Test**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

Run the nodes:
```bash
# Terminal 1
ros2 run ce_robot 03_battery_publisher

# Terminal 2
ros2 run ce_robot 03_battery_subscriber
```

### 🔍 Expected Output

**Publisher:**
```
[INFO] [battery_publisher]: Battery Publisher started!
[INFO] [battery_publisher]: 🔋 BAT-001 | 14.3V | 1.8A | 95% | Good | ~142min
[INFO] [battery_publisher]: 🔋 BAT-001 | 14.2V | 2.1A | 93% | Good | ~139min
...
[INFO] [battery_publisher]: 🔌 BAT-001 | 10.8V | 4.2A | 18% | Critical | ~27min
[INFO] [battery_publisher]: 🔌 BAT-001 | 11.2V | 3.8A | 23% | Poor | ~34min
```

**Subscriber:**
```
[INFO] [battery_subscriber]: Battery Subscriber started! Monitoring battery health...
[INFO] [battery_subscriber]: [1] 🔋 BAT-001 | 14.3V | 95% | Good | Est: 142min
[WARN] [battery_subscriber]: ⚡ LOW BATTERY: BAT-001 at 24%
[ERROR] [battery_subscriber]: ⚠️  CRITICAL: BAT-001 at 18%! Immediate action required!
[INFO] [battery_subscriber]: [15] 🔌 BAT-001 | 11.2V | 23% | Poor | Est: 34min
```

### 💡 Key Learning Points

- **Message design** - choosing appropriate data types
- **Field naming** - descriptive and clear names
- **Data ranges** - realistic simulation values
- **Build process** - updating CMakeLists.txt for new messages
- **Message verification** - using `ros2 interface show`
- **Complete workflow** - from design to implementation

### ✅ Completion Checklist

- [ ] Created `BatteryStatus.msg` with 8 fields
- [ ] Updated CMakeLists.txt to include new message
- [ ] Built message package successfully
- [ ] Verified message with `ros2 interface show`
- [ ] Created battery publisher with charging simulation
- [ ] Created battery subscriber with alerts
- [ ] Made files executable
- [ ] Updated setup.py with new entry points
- [ ] Built ce_robot package successfully
- [ ] Tested publisher and subscriber together
- [ ] Observed charging/discharging cycle
- [ ] Saw critical battery alerts

### 🎓 What You Learned (NEW)

- **Complete message design workflow** from concept to implementation
- **Choosing data types** - int vs uint, float32 vs float64
- **Message documentation** - using comments in .msg files
- **Realistic simulations** - charging/discharging cycles
- **Alert systems** - different severity levels
- **Building multiple messages** in one package

---

## **📂 Final Directory Structure**

```
📁 ROS2_WS/
├── 📁 src/
│   ├── 📁 ce_robot_interfaces/
│   │   ├── 📁 msg/
│   │   │   ├── 📄 HardwareStatus.msg              # From Readme
│   │   │   └── 📄 BatteryStatus.msg               # Lab Ex5
│   │   ├── 📄 package.xml
│   │   └── 📄 CMakeLists.txt
│   └── 📁 ce_robot/
│       ├── 📁 ce_robot/
│       │   ├── 📄 __init__.py
│       │   ├── 🐍 HardwareStatus_publish.py          # From Readme - Basic Publisher
│       │   ├── 🐍 HardwareStatus_subscribe.py        # From Readme - Basic Subscriber
│       │   ├── 🐍 HardwareStatus_aggregate.py        # From Readme - Simple Aggregator
│       │   ├── 🐍 HardwareStatus_validated.py        # Lab Ex1 - Validation
│       │   ├── 🐍 HardwareStatus_filter.py           # Lab Ex2 - Filtering
│       │   ├── 🐍 HardwareStatus_stats.py            # Lab Ex3 - Advanced Stats
│       │   ├── 🐍 HardwareStatus_multi_robot.py      # Lab Ex4 - Multi-Robot Monitor
│       │   ├── 🐍 HardwareStatus_multi_pub.py        # Lab Ex4 - Multi-Robot Publisher
│       │   ├── 🐍 BatteryStatus_pub.py               # Lab Ex5 - Battery Publisher
│       │   └── 🐍 BatteryStatus_sub.py               # Lab Ex5 - Battery Subscriber
│       ├── 📄 package.xml
│       ├── 📄 setup.cfg
│       └── 📄 setup.py
└── 📁 install/
```

**Entry Points in setup.py:**
```python
entry_points={
    'console_scripts': [
        # From Readme (Basic patterns)
        '03_hw_status_publisher = ce_robot.HardwareStatus_publish:main',
        '03_hw_status_subscriber = ce_robot.HardwareStatus_subscribe:main',
        '03_hw_status_aggregator = ce_robot.HardwareStatus_aggregate:main',
        
        # Lab Exercises (Advanced patterns)
        '03_hw_validated_publisher = ce_robot.HardwareStatus_validated:main',
        '03_hw_filter_subscriber = ce_robot.HardwareStatus_filter:main',
        '03_hw_statistics = ce_robot.HardwareStatus_stats:main',
        '03_hw_multi_robot_monitor = ce_robot.HardwareStatus_multi_robot:main',
        '03_hw_multi_robot_publisher = ce_robot.HardwareStatus_multi_pub:main',
        '03_battery_publisher = ce_robot.BatteryStatus_pub:main',
        '03_battery_subscriber = ce_robot.BatteryStatus_sub:main',
    ],
},
```

---

## **🔍 Useful ROS 2 Commands for Testing**

### Message Inspection
```bash
# Check message structure
ros2 interface show ce_robot_interfaces/msg/HardwareStatus

# List active topics
ros2 topic list

# Echo messages
ros2 topic echo /hardware_status

# Check topic info
ros2 topic info /hardware_status
```

### Node Management
```bash
# List running nodes
ros2 node list

# Get node information
ros2 node info /validated_hw_publisher

# Visualize node graph
rqt_graph
```

### Debugging
```bash
# Check message frequency
ros2 topic hz /hardware_status

# Check message bandwidth
ros2 topic bw /hardware_status

# Monitor with rqt
rqt
```

---

## **✅ Lab Completion Checklist**

### Exercise 1: Input Validation
- [ ] Created `HardwareStatus_validated.py`
- [ ] Implemented `validate_message()` function
- [ ] Added range checks for all 5 fields
- [ ] Track success/failure statistics
- [ ] Calculate and display success rate
- [ ] Test with intentionally invalid data
- [ ] **Understand difference from Readme:** validation prevents bad data from entering system

### Exercise 2: Threshold Filtering
- [ ] Created `HardwareStatus_filter.py`
- [ ] Implemented temperature threshold (>55°C)
- [ ] Used deque for rolling window (last 10 high temps)
- [ ] Calculate percentage of filtered messages
- [ ] Calculate average and max of high temps only
- [ ] Used WARN for alerts, INFO for normal
- [ ] **Understand difference from Readme:** conditional processing vs displaying all

### Exercise 3: Advanced Statistics
- [ ] Created `HardwareStatus_stats.py`
- [ ] Store last 50 temperatures in rolling window
- [ ] Track timestamps for rate calculation
- [ ] Calculate average, min, max temperature
- [ ] Track motor status percentages
- [ ] Generate periodic reports (every 10 seconds)
- [ ] Format report with box characters
- [ ] **Understand difference from Readme:** rolling windows, rate calc, periodic reports vs simple avg

### Exercise 4: Multi-Robot Monitor
- [ ] Created `HardwareStatus_multi_robot.py`
- [ ] Created `HardwareStatus_multi_pub.py`
- [ ] Used defaultdict for dynamic robot tracking
- [ ] Track per-robot statistics separately
- [ ] Calculate per-robot averages and ranges
- [ ] Display all robots in single report
- [ ] Alert for specific robots with high temperature
- [ ] Test with 3 simulated robots
- [ ] **Understand new concept:** tracking multiple entities on shared topic

### Exercise 5: Create Custom Message
- [ ] Designed `BatteryStatus.msg` with 8 fields
- [ ] Updated CMakeLists.txt to include new message
- [ ] Built message package successfully
- [ ] Verified with `ros2 interface show`
- [ ] Created `BatteryStatus_pub.py` with charging simulation
- [ ] Created `BatteryStatus_sub.py` with alerts
- [ ] Made files executable
- [ ] Updated setup.py with battery entry points
- [ ] Built ce_robot package successfully
- [ ] Tested complete charging/discharging cycle
- [ ] **Understand new concept:** complete message design workflow

### Overall Lab
- [ ] All Python files are executable (`chmod +x`)
- [ ] All 7 new entry points added to setup.py
- [ ] Both packages build without errors
- [ ] All nodes run successfully
- [ ] Can run multiple nodes simultaneously
- [ ] Tested with `rqt_graph`
- [ ] **Understand advanced patterns:** validation, filtering, rolling windows, multi-entity tracking, custom message design

---

## **⚠️ Troubleshooting**

### Issue: "Permission denied" when running node
**Solution:**
```bash
chmod +x HardwareStatus_validated.py
chmod +x HardwareStatus_filter.py
chmod +x HardwareStatus_stats.py
```

### Issue: Entry point not found
**Solution:** Verify setup.py and rebuild
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

### Issue: No messages received
**Solution:** Check both nodes are running and topic names match
```bash
ros2 topic list
ros2 topic info /hardware_status
```
## **🎓 What You've Learned (Beyond the Readme)**

### **From Readme (Review):**
✅ Creating custom message packages  
✅ Defining `.msg` files  
✅ Basic publisher pattern  
✅ Basic subscriber pattern  
✅ Simple aggregation (count and average)  

### **NEW from Lab Exercise 1:**
✅ Input validation before publishing  
✅ Range checking for all data types  
✅ Error prevention (not just detection)  
✅ Success/failure rate tracking  

### **NEW from Lab Exercise 2:**
✅ Conditional filtering (alerts only)  
✅ Threshold-based monitoring  
✅ Statistical calculations on filtered data  
✅ Appropriate logging levels (WARN vs INFO)  

### **NEW from Lab Exercise 3:**
✅ Rolling window analysis (last N values)  
✅ Min/max tracking across time  
✅ Message rate calculation  
✅ Periodic automated reports  
✅ Professional formatted output  

### **NEW from Lab Exercise 4:**
✅ Multi-entity tracking (multiple robots)  
✅ Per-entity statistics using dictionaries  
✅ Shared topic patterns  
✅ Comparative analysis  
✅ Entity-specific alerts  

### **NEW from Lab Exercise 5:**
✅ Complete message design workflow  
✅ Choosing appropriate data types  
✅ Message field documentation  
✅ Building multiple messages in one package  
✅ Realistic simulation patterns  
✅ Alert severity levels  

### **Advanced Best Practices:**
✅ Defensive programming with validation  
✅ Data quality assurance  
✅ Scalable monitoring patterns  
✅ Production-ready error handling  
✅ Message design for specific use casesltering  
✅ Threshold-based monitoring  
✅ Statistical calculations  
✅ Rolling window analysis  
✅ Trend tracking  

### Best Practices
✅ Defensive programming  
✅ Appropriate logging levels  
✅ Statistics tracking  
✅ Formatted output  
✅ Error handling  

---

## **📚 Additional Resources**

- [ROS 2 Custom Interfaces](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [ROS 2 Python Node Guide](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
## **💡 Challenge Projects**

### Challenge 1: Persistent Alert System
Extend Exercise 2 to:
- Send alert if temperature stays above 60°C for 5 **consecutive** messages
- Track total alert count since start
- Add cooldown period (no repeated alerts for 30 seconds)

### Challenge 2: CSV Data Logger
Create a new node that:
- Subscribes to `hardware_status`
- Records all messages to CSV file with timestamp
- Allows playback from file using a separate publisher node
- Implements data compression for long-running logs

### Challenge 3: Robot Fleet Dashboard
Extend Exercise 4 to:
- Identify which robot has highest/lowest average temperature
- Track which robot sends most messages
- Calculate fleet-wide statistics (total avg across all robots)
- Generate comparison graphs between robots

### Challenge 4: Anomaly Detection
Create a node that:
- Tracks normal temperature range per robot
- Detects sudden temperature spikes (>10°C change)
- Identifies robots with unusual motor ready/not ready patterns
- Sends alerts for anomalous behavior

### Challenge 1: Temperature Alert System
Create a node that:
- Monitors temperature continuously
- Sends alert if temperature stays above 60°C for 5 consecutive messages
- Tracks alert frequency

### Challenge 2: Data Logger
Create a node that:
- Records all messages to CSV file
- Includes timestamp for each message
- Allows playback from file

### Challenge 3: Multi-Robot Monitor
Create a node that:
- Tracks messages from multiple robots (different names)
- Calculates statistics per robot
- Identifies which robot has highest average temperature

---

**🎉 Congratulations! You've completed the Custom Messages Lab!** 🚀✨

You now have practical experience with:
- Message validation
- Data filtering and processing
- Statistics and aggregation
- Real-world ROS 2 application patterns

**Keep practicing and building!** 💪
