# **ROS 2 Publisher & Subscriber Lab Exercises**

## **📚 Lab Overview**

This lab provides hands-on exercises to master Publisher & Subscriber patterns in ROS 2. Each exercise builds upon the previous one, progressing from basic to advanced concepts.

**Duration:** 2-3 hours  
**Level:** Beginner to Intermediate  
**Prerequisites:** ROS 2 Jazzy installed and first_node completed

---

## **🎯 Learning Objectives**

By completing this lab, you will be able to:
- ✅ Create basic Publisher nodes
- ✅ Create basic Subscriber nodes
- ✅ Understand topic-based communication
- ✅ Work with different message types
- ✅ Debug and monitor topics
- ✅ Implement advanced pub/sub patterns
- ✅ Handle multiple publishers and subscribers

---

## **📋 Lab Exercises**

### **Exercise 1: Basic Publisher (Beginner)**

**Objective:** Create a simple publisher that sends string messages

**Tasks:**
1. Navigate to your ce_robot package
2. Create `simple_publisher.py`
3. Implement a publisher that sends "Hello from Publisher" every 1 second
4. Run and verify output

**Expected Output:**
```
[INFO] [simple_publisher]: Publishing: "Hello from Publisher"
```

**Hints:**
```python
# Create publisher with String message type
# Use create_timer() for periodic publishing
# Use create_publisher() with topic name and queue size
```

**Solution:**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.pub = self.create_publisher(String, 'hello_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "Hello from Publisher"
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### **Exercise 2: Basic Subscriber (Beginner)**

**Objective:** Create a simple subscriber that listens to messages

**Tasks:**
1. Create `simple_subscriber.py`
2. Subscribe to the topic from Exercise 1
3. Print received messages with a timestamp

**Expected Output:**
```
[INFO] [simple_subscriber]: Received: "Hello from Publisher"
```

**Hints:**
```python
# Use create_subscription() to listen to a topic
# Implement a callback function for processing messages
```

**Solution:**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.sub = self.create_subscription(String, 'hello_topic', self.callback, 10)

    def callback(self, msg):
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.get_logger().info(f'[{timestamp}] Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### **Exercise 3: Counter Publisher (Beginner)**

**Objective:** Create a publisher that sends incrementing counter values

**Tasks:**
1. Create `counter_publisher.py`
2. Publish integer counter that increments every 500ms
3. Stop after reaching 100

**Expected Output:**
```
[INFO] [counter_publisher]: Counter: 0
[INFO] [counter_publisher]: Counter: 1
...
[INFO] [counter_publisher]: Counter: 100 - Finished!
```

**Solution:**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class CounterPublisher(Node):
    def __init__(self):
        super().__init__('counter_publisher')
        self.pub = self.create_publisher(Int32, 'counter_topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = Int32()
        msg.data = self.counter
        self.pub.publish(msg)
        self.get_logger().info(f'Counter: {self.counter}')
        self.counter += 1
        
        if self.counter > 100:
            self.get_logger().info('Finished!')
            exit()

def main(args=None):
    rclpy.init(args=args)
    node = CounterPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### **Exercise 4: Multiple Subscribers (Intermediate)**

**Objective:** Create multiple subscribers listening to the same topic

**Tasks:**
1. Create `counter_processor.py` - filters even numbers
2. Create `counter_logger.py` - logs odd numbers
3. Run both with the counter_publisher from Exercise 3
4. Verify both receive data correctly

**Expected Output (counter_processor):**
```
[INFO] [counter_processor]: Even: 0
[INFO] [counter_processor]: Even: 2
[INFO] [counter_processor]: Even: 4
```

**Expected Output (counter_logger):**
```
[INFO] [counter_logger]: Odd: 1
[INFO] [counter_logger]: Odd: 3
[INFO] [counter_logger]: Odd: 5
```

---

### **Exercise 5: Temperature Sensor Simulation (Intermediate)**

**Objective:** Simulate a temperature sensor with realistic data

**Tasks:**
1. Create `temperature_publisher.py`
2. Simulate temperature data with slight random variation
3. Publish every 2 seconds
4. Range: 20-30°C

**Expected Output:**
```
[INFO] [temperature_publisher]: Temperature: 24.53°C
[INFO] [temperature_publisher]: Temperature: 24.67°C
```

**Solution:**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.pub = self.create_publisher(Float32, 'temperature', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.base_temp = 25.0

    def timer_callback(self):
        temp = self.base_temp + random.uniform(-1, 1)
        msg = Float32()
        msg.data = temp
        self.pub.publish(msg)
        self.get_logger().info(f'Temperature: {temp:.2f}°C')

def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### **Exercise 6: Data Aggregation (Intermediate)**

**Objective:** Subscribe to multiple topics and aggregate data

**Tasks:**
1. Create `sensor_monitor.py`
2. Subscribe to temperature, humidity, and pressure topics
3. Display average of all three readings every 5 seconds
4. Log statistics (min, max, avg)

**Expected Output:**
```
[INFO] [sensor_monitor]: Average: 24.67
[INFO] [sensor_monitor]: Min: 23.45, Max: 25.89
```

---

### **Exercise 7: Data Filtering (Advanced)**

**Objective:** Filter and transform published data

**Tasks:**
1. Create `temperature_filter.py`
2. Subscribe to temperature topic
3. Only publish alerts when temperature > 28°C or < 18°C
4. Publish to `temperature_alert` topic

**Expected Output (when alert triggered):**
```
[WARN] [temperature_filter]: ALERT! Temperature too high: 29.5°C
```

---

### **Exercise 8: Rate-Limited Publisher (Advanced)**

**Objective:** Implement dynamic rate control

**Tasks:**
1. Create `variable_rate_publisher.py`
2. Publish data at variable rate (1Hz, 2Hz, 5Hz)
3. Change rate every 10 seconds
4. Measure and display actual publish rate

**Expected Output:**
```
[INFO] [variable_rate_publisher]: Publishing at 1 Hz
[INFO] [variable_rate_publisher]: Publishing at 2 Hz
[INFO] [variable_rate_publisher]: Publishing at 5 Hz
```

---

### **Exercise 9: Custom Message Type (Advanced)**

**Objective:** Create and use custom message types

**Tasks:**
1. Create `RobotStatus.msg` with:
   - battery_level (float)
   - temperature (float)
   - is_moving (bool)

2. Create `robot_status_publisher.py` that publishes this message
3. Create `robot_status_subscriber.py` that processes it
4. Display all fields in the subscriber

**RobotStatus.msg:**
```
float32 battery_level
float32 temperature
bool is_moving
```

---

### **Exercise 10: Debugging with ROS 2 Tools (Advanced)**

**Objective:** Master debugging and monitoring commands

**Tasks:**
1. Run publisher and subscriber from Exercise 1
2. Use `ros2 topic list` to list all topics
3. Use `ros2 topic echo` to view messages in real-time
4. Use `rqt_graph` to visualize the graph
5. Use `ros2 topic info -v` to get detailed topic info
6. Use `ros2 topic bw` to measure bandwidth
7. Document all commands and outputs

**Commands to Practice:**
```bash
# List all active topics
ros2 topic list

# View topic type
ros2 topic type /hello_topic

# Echo messages in real-time
ros2 topic echo /hello_topic

# Get detailed topic info
ros2 topic info /hello_topic --verbose

# Measure bandwidth usage
ros2 topic bw /hello_topic

# Visualize nodes and connections
rqt_graph

# Monitor system nodes
ros2 node list
```

---

## **📊 Challenge Exercises**

### **Challenge 1: System Monitor Dashboard**

**Objective:** Create a real-time system monitoring application

**Requirements:**
- Subscribe to CPU, memory, and disk topics
- Display data in terminal with formatting
- Update every 1 second
- Show warnings when thresholds exceeded

---

### **Challenge 2: Data Logger with Playback**

**Objective:** Log published data and replay it

**Requirements:**
- Subscribe to all topics and log to file
- Implement playback functionality
- Use ROS 2 bag files (if available)
- Compare live vs playback data

---

### **Challenge 3: Multi-Topic Synchronizer**

**Objective:** Synchronize data from multiple topics

**Requirements:**
- Subscribe to 3 sensor topics
- Ensure data is from same time window
- Combine data and publish synchronized output
- Handle timing misalignment

---

## **✅ Completion Checklist**

- [ ] Exercise 1: Basic Publisher completed
- [ ] Exercise 2: Basic Subscriber completed
- [ ] Exercise 3: Counter Publisher completed
- [ ] Exercise 4: Multiple Subscribers completed
- [ ] Exercise 5: Temperature Sensor completed
- [ ] Exercise 6: Data Aggregation completed
- [ ] Exercise 7: Data Filtering completed
- [ ] Exercise 8: Rate-Limited Publisher completed
- [ ] Exercise 9: Custom Message Type completed
- [ ] Exercise 10: Debugging Tools mastered
- [ ] Challenge 1 attempted
- [ ] Challenge 2 attempted
- [ ] Challenge 3 attempted

---

## **💡 Tips & Tricks**

1. **Always source setup.bash before running:**
   ```bash
   source ~/.bashrc
   ```

2. **Use print statements for debugging:**
   ```python
   print(f"Debug: {variable}")
   ```

3. **Monitor topics in real-time:**
   ```bash
   ros2 topic echo /topic_name
   ```

4. **Check node connectivity:**
   ```bash
   rqt_graph
   ```

5. **View all active topics:**
   ```bash
   ros2 topic list
   ```

6. **Get message type info:**
   ```bash
   ros2 interface show std_msgs/msg/String
   ```

---

## **🔗 Related Resources**

- [ROS 2 Official Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)
- [ROS 2 Pub/Sub Documentation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS 2 Message Types](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Interfaces.html)
- [ROS 2 Topic Naming Convention](https://design.ros2.org/articles/topic_and_service_names.html)

**🎓 Congratulations! You've completed the ROS 2 Publisher & Subscriber Lab!** 🚀✨
