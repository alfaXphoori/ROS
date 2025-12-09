# **🧪 ROS 2 Custom Messages - Lab Exercises**

**Advanced Patterns: Validation, Filtering, and Multi-Robot Systems**

---

## **📌 Lab Information**

**Lab Title:** Custom Messages - Advanced Applications  
**Duration:** 75 minutes  
**Level:** Beginner to Intermediate  
**Prerequisites:** 
- Completed Readme.md (Custom Message Package Setup)
- Created `ce_robot_interfaces` package with `HardwareStatus.msg`
- Built and tested basic publisher (`HardwareStatus_publish.py`)
- Built and tested basic subscriber (`HardwareStatus_subscribe.py`)
- Built and tested aggregator (`HardwareStatus_aggregate.py`)

**What You Already Know from Readme:**
✅ Creating message packages and `.msg` files  
✅ Building custom messages with colcon  
✅ Basic publisher pattern with random temperature  
✅ Basic subscriber pattern with message display  
✅ Simple aggregation (count and average temperature)  

**What You'll Learn in This Lab (NEW):**
🎯 **Input validation** before publishing (preventing bad data)  
🎯 **Conditional filtering** based on thresholds (alert systems)  
🎯 **Advanced statistics** with rolling windows and reporting  
🎯 **Multi-robot monitoring** (tracking multiple robots simultaneously)  

---

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🎯 Lab Objectives**

By completing this lab, you will be able to:

1. **Validate** custom message data before publishing
2. **Handle errors** gracefully in publisher and subscriber nodes
3. **Filter** messages based on conditions
4. **Calculate statistics** from message streams
5. **Track metrics** like message counts and success rates
6. **Implement alerts** for critical conditions
7. **Apply defensive programming** techniques

---

## **📚 Lab Structure**

```
┌─────────────────────────────────────────────┐
│ Exercise 1: Input Validation                │
│ Validate data BEFORE publishing (NEW)       │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 2: Threshold Filtering             │
│ Alert-based subscriber (NEW)                │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 3: Advanced Statistics             │
│ Rolling windows + periodic reports (NEW)    │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 4: Multi-Robot Monitor             │
│ Track multiple robots simultaneously (NEW)  │
└─────────────────────────────────────────────┘
```

| Exercise | Title | Duration | Difficulty |
|----------|-------|----------|------------|
## **Exercise 1: Input Validation ✅**

### 🎯 Objective
Extend the basic publisher from Readme with **input validation** to prevent publishing invalid data.

### 📖 Background
**Difference from Readme:** The basic publisher (`HardwareStatus_publish.py`) generates random data and publishes it directly. In production systems, you need to:
- Validate data **before** publishing (not after)
- Prevent invalid values from entering the system
- Track validation success/failure rates
- Log validation errors appropriatelyto your publisher to ensure all data is correct before publishing.

### 📖 Background
In real applications, you need to validate data to prevent:
- Invalid values (temperature too high/low)
- Missing or empty fields
- Out-of-range numbers
- Data type mismatches

### 📝 Tasks

**Step 1: Create Validated Publisher**

Create `HardwareStatus_validated.py` in `ce_robot/ce_robot/`:

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
│       │   ├── 🐍 HardwareStatus_publish.py          # From Readme - Basic Publisher
│       │   ├── 🐍 HardwareStatus_subscribe.py        # From Readme - Basic Subscriber
│       │   ├── 🐍 HardwareStatus_aggregate.py        # From Readme - Simple Aggregator
│       │   ├── 🐍 HardwareStatus_validated.py        # Lab Ex1 - Validation
│       │   ├── 🐍 HardwareStatus_filter.py           # Lab Ex2 - Filtering
│       │   ├── 🐍 HardwareStatus_stats.py            # Lab Ex3 - Advanced Stats
│       │   ├── 🐍 HardwareStatus_multi_robot.py      # Lab Ex4 - Multi-Robot Monitor
│       │   └── 🐍 HardwareStatus_multi_pub.py        # Lab Ex4 - Multi-Robot Publisher
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

### Overall Lab
- [ ] All Python files are executable (`chmod +x`)
- [ ] All 5 new entry points added to setup.py
- [ ] Package builds without errors
- [ ] All nodes run successfully
- [ ] Can run multiple nodes simultaneously
- [ ] Tested with `rqt_graph`
- [ ] **Understand advanced patterns** not in Readme: validation, filtering, rolling windows, multi-entity tracking

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

### **Advanced Best Practices:**
✅ Defensive programming with validation  
✅ Data quality assurance  
✅ Scalable monitoring patterns  
✅ Production-ready error handlingltering  
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
