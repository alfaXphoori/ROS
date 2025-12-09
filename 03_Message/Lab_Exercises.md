# **🧪 ROS 2 Custom Messages - Lab Exercises**

**Hands-On Practice: Building Real-World Applications with Custom Messages**

---

## **📌 Lab Information**

**Lab Title:** Custom Messages - Practical Applications  
**Duration:** 60 minutes  
**Level:** Beginner (University Students)  
**Prerequisites:** 
- Completed Readme.md (Custom Message Package Setup)
- Created `ce_robot_interfaces` package
- Built `HardwareStatus.msg`
- Created basic publisher and subscriber

**What You Already Know:**
✅ How to create message packages  
✅ How to define `.msg` files  
✅ How to build custom messages  
✅ Basic publisher and subscriber patterns  

**What You'll Learn in This Lab:**
🎯 Validation and error handling  
🎯 Data filtering and processing  
🎯 Statistics and aggregation  

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
│ Exercise 1: Message Validation              │
│ Add input validation to your publisher      │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 2: Data Filtering                  │
│ Create subscriber that filters messages     │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 3: Statistics & Aggregation        │
│ Track and analyze message patterns          │
└─────────────────────────────────────────────┘
```

| Exercise | Title | Duration | Difficulty |
|----------|-------|----------|------------|
| 1 | Message Validation | 20 min | ⭐⭐ |
| 2 | Data Filtering | 20 min | ⭐⭐ |
| 3 | Statistics & Aggregation | 20 min | ⭐⭐⭐ |

---

## **Exercise 1: Message Validation ✅**

### 🎯 Objective
Add robust validation to your publisher to ensure all data is correct before publishing.

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

## **Exercise 2: Data Filtering 🔍**

### 🎯 Objective
Create a subscriber that filters and processes only relevant messages.

### 📖 Background
In real systems, you often need to:
- Filter out normal data, focus on alerts
- Process only messages meeting certain criteria
- Calculate statistics on filtered data
- Implement threshold-based monitoring

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

## **Exercise 3: Statistics & Aggregation 📊**

### 🎯 Objective
Create an advanced subscriber that tracks comprehensive statistics.

### 📖 Background
Aggregation nodes collect data and calculate:
- Running averages
- Min/max values
- Message rates
- Trend analysis
- Periodic reports

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
- [ ] Store last 50 temperatures in deque
- [ ] Track message timestamps
- [ ] Calculate average temperature
- [ ] Track min/max temperatures
- [ ] Count motor ready/not ready
- [ ] Calculate message rate
- [ ] Generate report every 10 seconds
- [ ] Format report with box drawing
- [ ] Test with publisher running

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
│       │   ├── 🐍 HardwareStatus_publish.py          # From Readme
│       │   ├── 🐍 HardwareStatus_subscribe.py        # From Readme
│       │   ├── 🐍 HardwareStatus_aggregate.py        # From Readme
│       │   ├── 🐍 HardwareStatus_validated.py        # Exercise 1
│       │   ├── 🐍 HardwareStatus_filter.py           # Exercise 2
│       │   └── 🐍 HardwareStatus_stats.py            # Exercise 3
│       ├── 📄 package.xml
│       ├── 📄 setup.cfg
│       └── 📄 setup.py
└── 📁 install/
```

**Entry Points in setup.py:**
```python
entry_points={
    'console_scripts': [
        # From Readme
        '03_hw_status_publisher = ce_robot.HardwareStatus_publish:main',
        '03_hw_status_subscriber = ce_robot.HardwareStatus_subscribe:main',
        '03_hw_status_aggregator = ce_robot.HardwareStatus_aggregate:main',
        
        # Lab Exercises
        '03_hw_validated_publisher = ce_robot.HardwareStatus_validated:main',
        '03_hw_filter_subscriber = ce_robot.HardwareStatus_filter:main',
        '03_hw_statistics = ce_robot.HardwareStatus_stats:main',
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

### Exercise 1: Validation
- [ ] Created `HardwareStatus_validated.py`
- [ ] Implemented validation function
- [ ] Added range checks for all fields
- [ ] Track success/failure statistics
- [ ] Calculate and display success rate
- [ ] Test with invalid data
- [ ] Validation catches errors correctly

### Exercise 2: Filtering
- [ ] Created `HardwareStatus_filter.py`
- [ ] Implemented temperature threshold (>55°C)
- [ ] Used deque for rolling window
- [ ] Calculate percentage of filtered messages
- [ ] Calculate average and max values
- [ ] Used appropriate logging levels
- [ ] Filter works correctly

### Exercise 3: Statistics
- [ ] Created `HardwareStatus_stats.py`
- [ ] Store last 50 temperatures
- [ ] Track timestamps for rate calculation
- [ ] Calculate average, min, max temperature
- [ ] Track motor status counts
- [ ] Generate periodic reports (10 seconds)
- [ ] Format report with box characters
- [ ] All statistics calculate correctly

### Overall
- [ ] All Python files are executable
- [ ] All entry points added to setup.py
- [ ] Package builds without errors
- [ ] All nodes run successfully
- [ ] Can run multiple nodes simultaneously
- [ ] Tested with rqt_graph
- [ ] Understand validation, filtering, and aggregation patterns

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

### Issue: Import error for ce_robot_interfaces
**Solution:** Build interface package first
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
source install/setup.bash
```

---

## **🎓 What You've Learned**

### Validation Techniques
✅ Input validation before publishing  
✅ Range checking for numeric values  
✅ String length validation  
✅ Error message generation  
✅ Success rate tracking  

### Data Processing
✅ Conditional filtering  
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
- [ROS 2 Logging](https://docs.ros.org/en/jazzy/Tutorials/Demos/Logging-and-logger-configuration.html)
- [Python Collections (deque)](https://docs.python.org/3/library/collections.html#collections.deque)

---

## **🚀 Next Steps**

After completing this lab, you can:

1. **Extend validation** - Add more complex validation rules
2. **Multiple filters** - Create different filtering criteria
3. **Data visualization** - Plot statistics over time
4. **Alert system** - Send notifications for critical conditions
5. **Database storage** - Save messages to a database
6. **Web dashboard** - Create real-time monitoring dashboard

---

## **💡 Challenge Projects**

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
