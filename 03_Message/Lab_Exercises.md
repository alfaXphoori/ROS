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
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 2: Safety Monitor Subscriber       │
│ Monitor robot and generate safety alerts    │
└─────────────────────────────────────────────┘
```

| Step | Title | Duration | Difficulty |
|------|-------|----------|------------|
| Setup | Design RobotStatus.msg | 15 min | ⭐⭐ |
| Ex 1 | Status Publisher | 25 min | ⭐⭐⭐ |
| Ex 2 | Safety Monitor | 20 min | ⭐⭐ |

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
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "msg/RobotStatus.msg"
  DEPENDENCIES std_msgs
)
```

Add dependency in `ce_robot_interfaces/package.xml`:

```xml
<depend>std_msgs</depend>
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

You should see all 22 fields listed with the Header included.

---

## **Exercise 1: Robot Status Publisher 🤖**

### 🎯 Objective
Create a comprehensive robot status publisher that simulates a real autonomous mobile robot with:
- Realistic movement simulation (waypoint navigation)
- Battery discharge/recharge cycles
- Mission state management
- Safety monitoring (obstacle detection)
- System health tracking

### 📖 Background
In real warehouse robots, the robot continuously publishes its status including position, velocity, battery level, and mission state. This exercise simulates a robot moving through waypoints, consuming battery, detecting obstacles, and updating its mission status.

### 📝 Tasks

**Step 1: Create Robot Status Publisher**

Create `RobotStatus_publisher.py` in `ce_robot/ce_robot/`:

```python
#!/usr/bin/env python3
"""
Robot Status Publisher
Simulates realistic autonomous mobile robot with complete telemetry
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import RobotStatus
from std_msgs.msg import Header
import random
import math
import time


class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_publisher')
        self.publisher_ = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)
        
        # Robot state
        self.robot_id = "AMR-001"
        self.robot_name = "Warehouse Bot Alpha"
        
        # Position and movement
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_z = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        # Waypoints for movement simulation
        self.waypoints = [
            (0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0)
        ]
        self.current_waypoint_idx = 0
        
        # Battery state
        self.battery_voltage = 24.0
        self.battery_percentage = 100
        self.is_charging = False
        
        # Mission state
        self.missions = ["PICKING", "DELIVERING", "RETURNING", "IDLE"]
        self.current_mission = "IDLE"
        self.mission_progress = 0
        self.mission_status = "IN_PROGRESS"
        
        # Safety
        self.emergency_stop = False
        self.obstacle_detected = False
        self.obstacle_distance = 10.0
        
        # System health
        self.cpu_usage = 25.0
        self.temperature = 35.0
        self.motors_enabled = True
        self.health_status = "HEALTHY"
        
        self.get_logger().info(f'🤖 {self.robot_name} ({self.robot_id}) started!')

    def move_towards_waypoint(self):
        """Simulate robot movement towards current waypoint"""
        if self.emergency_stop or not self.motors_enabled:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            return
        
        target_x, target_y = self.waypoints[self.current_waypoint_idx]
        
        # Calculate distance and angle to target
        dx = target_x - self.position_x
        dy = target_y - self.position_y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        # If reached waypoint, move to next
        if distance < 0.5:
            self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)
            self.mission_progress = int((self.current_waypoint_idx / len(self.waypoints)) * 100)
            
            if self.current_waypoint_idx == 0:
                self.current_mission = random.choice(self.missions)
                self.mission_progress = 0
            return
        
        # Move towards waypoint
        self.linear_velocity = min(1.0, distance * 0.5)  # Max 1 m/s
        
        angle_diff = target_angle - self.orientation_z
        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        self.angular_velocity = angle_diff * 0.5  # Proportional control
        
        # Update position
        dt = 1.0  # 1 second update
        self.position_x += self.linear_velocity * math.cos(self.orientation_z) * dt
        self.position_y += self.linear_velocity * math.sin(self.orientation_z) * dt
        self.orientation_z += self.angular_velocity * dt

    def update_battery(self):
        """Simulate battery discharge and charging"""
        if self.is_charging:
            # Charging
            self.battery_percentage = min(100, self.battery_percentage + 2)
            self.battery_voltage = 22.0 + (self.battery_percentage / 100.0) * 3.0
            if self.battery_percentage >= 95:
                self.is_charging = False
                self.motors_enabled = True
        else:
            # Discharging (faster when moving)
            discharge_rate = 0.5 if self.linear_velocity > 0 else 0.1
            self.battery_percentage = max(0, self.battery_percentage - discharge_rate)
            self.battery_voltage = 22.0 + (self.battery_percentage / 100.0) * 3.0
            
            # Auto-charge when low
            if self.battery_percentage <= 20:
                self.is_charging = True
                self.motors_enabled = False
                self.current_mission = "CHARGING"
        
        # Calculate time remaining
        discharge_rate = 0.5 if not self.is_charging else -2.0
        if discharge_rate > 0:
            self.battery_time_remaining = int(self.battery_percentage / discharge_rate)
        else:
            self.battery_time_remaining = int((100 - self.battery_percentage) / abs(discharge_rate))

    def update_safety(self):
        """Simulate obstacle detection and safety monitoring"""
        # Random obstacle detection
        if random.random() < 0.1:
            self.obstacle_detected = True
            self.obstacle_distance = random.uniform(0.5, 3.0)
            if self.obstacle_distance < 1.0:
                self.emergency_stop = True
        else:
            self.obstacle_detected = False
            self.obstacle_distance = 10.0
            if self.emergency_stop and random.random() < 0.3:
                self.emergency_stop = False

    def update_health(self):
        """Update system health status"""
        # Simulate CPU usage based on activity
        base_cpu = 25.0
        movement_cpu = abs(self.linear_velocity) * 15.0
        self.cpu_usage = base_cpu + movement_cpu + random.uniform(-5, 5)
        
        # Simulate temperature
        self.temperature = 35.0 + (self.cpu_usage / 100.0) * 15.0 + random.uniform(-2, 2)
        
        # Determine overall health
        if self.emergency_stop or self.battery_percentage < 10 or self.temperature > 70:
            self.health_status = "CRITICAL"
        elif self.battery_percentage < 20 or self.temperature > 60:
            self.health_status = "ERROR"
        elif self.battery_percentage < 40 or self.obstacle_detected:
            self.health_status = "WARNING"
        else:
            self.health_status = "HEALTHY"

    def publish_status(self):
        """Publish complete robot status"""
        # Update all states
        self.move_towards_waypoint()
        self.update_battery()
        self.update_safety()
        self.update_health()
        
        # Create message
        msg = RobotStatus()
        
        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Robot ID
        msg.robot_id = self.robot_id
        msg.robot_name = self.robot_name
        
        # Position
        msg.position_x = self.position_x
        msg.position_y = self.position_y
        msg.orientation_z = self.orientation_z
        
        # Velocity
        msg.linear_velocity = self.linear_velocity
        msg.angular_velocity = self.angular_velocity
        
        # Battery
        msg.battery_voltage = self.battery_voltage
        msg.battery_percentage = self.battery_percentage
        msg.is_charging = self.is_charging
        msg.battery_time_remaining = self.battery_time_remaining
        
        # Safety
        msg.emergency_stop = self.emergency_stop
        msg.obstacle_detected = self.obstacle_detected
        msg.obstacle_distance = self.obstacle_distance
        
        # Mission
        msg.current_mission = self.current_mission
        msg.mission_progress = self.mission_progress
        msg.mission_status = self.mission_status
        
        # Health
        msg.cpu_usage = self.cpu_usage
        msg.temperature = self.temperature
        msg.motors_enabled = self.motors_enabled
        msg.health_status = self.health_status
        
        self.publisher_.publish(msg)
        
        # Log status with icons
        battery_icon = "🔌" if self.is_charging else "🔋"
        motion_icon = "🚀" if self.linear_velocity > 0 else "🛑"
        health_icon = "✅" if self.health_status == "HEALTHY" else "⚠️" if self.health_status == "WARNING" else "❌"
        
        self.get_logger().info(
            f'{health_icon} {self.robot_name} | '
            f'Pos: ({self.position_x:.1f}, {self.position_y:.1f}) | '
            f'{motion_icon} {self.linear_velocity:.2f} m/s | '
            f'{battery_icon} {self.battery_percentage}% | '
            f'Mission: {self.current_mission} ({self.mission_progress}%) | '
            f'Temp: {self.temperature:.1f}°C'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'\n🤖 {node.robot_name} shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 2: Make Executable**

```bash
chmod +x RobotStatus_publisher.py
```

**Step 3: Update package.xml**

Add dependency in `ce_robot/package.xml`:

```xml
<depend>std_msgs</depend>
```

**Step 4: Update setup.py**

Add entry point in `ce_robot/setup.py`:

```python
entry_points={
    'console_scripts': [
        # Previous entries...
        '03_robot_status_publisher = ce_robot.RobotStatus_publisher:main',
    ],
},
```

**Step 5: Build and Test**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

Run the publisher:
```bash
ros2 run ce_robot 03_robot_status_publisher
```

### 🔍 Expected Output

```
[INFO] [robot_status_publisher]: 🤖 Warehouse Bot Alpha (AMR-001) started!
[INFO] [robot_status_publisher]: ✅ Warehouse Bot Alpha | Pos: (0.0, 0.0) | 🛑 0.00 m/s | 🔋 100% | Mission: IDLE (0%) | Temp: 36.2°C
[INFO] [robot_status_publisher]: ✅ Warehouse Bot Alpha | Pos: (0.5, 0.0) | 🚀 0.50 m/s | 🔋 99% | Mission: PICKING (0%) | Temp: 42.5°C
[INFO] [robot_status_publisher]: ✅ Warehouse Bot Alpha | Pos: (2.3, 0.0) | 🚀 0.85 m/s | 🔋 98% | Mission: PICKING (0%) | Temp: 44.1°C
...
[INFO] [robot_status_publisher]: ⚠️ Warehouse Bot Alpha | Pos: (4.8, 4.2) | 🚀 0.42 m/s | 🔋 35% | Mission: DELIVERING (45%) | Temp: 51.3°C
...
[INFO] [robot_status_publisher]: ❌ Warehouse Bot Alpha | Pos: (3.2, 2.1) | 🛑 0.00 m/s | 🔌 18% | Mission: CHARGING (0%) | Temp: 39.8°C
```

### 💡 Key Learning Points

- **Realistic simulation** - waypoint navigation, battery cycles
- **State management** - position, velocity, mission tracking
- **Header usage** - timestamp and frame_id for ROS tf integration
- **Complex data types** - combining primitives into meaningful structure
- **System health monitoring** - CPU, temperature, overall status
- **Safety features** - emergency stop, obstacle detection
- **Professional logging** - icons and formatted output

### ✅ Completion Checklist

- [ ] Created `RobotStatus.msg` with 22 fields including Header
- [ ] Updated CMakeLists.txt with std_msgs dependency
- [ ] Built ce_robot_interfaces successfully
- [ ] Verified message with `ros2 interface show`
- [ ] Created `RobotStatus_publisher.py`
- [ ] Implemented waypoint navigation
- [ ] Implemented battery discharge/charge cycle
- [ ] Implemented safety monitoring
- [ ] Made file executable
- [ ] Updated setup.py with entry point
- [ ] Built ce_robot package successfully
- [ ] Tested publisher - robot moves through waypoints
- [ ] Observed battery discharge and auto-charging
- [ ] Saw health status changes based on conditions

---

## **Exercise 2: Safety Monitor Subscriber ⚠️**

### 🎯 Objective
Create a safety monitoring subscriber that:
- Monitors robot status in real-time
- Generates alerts for critical conditions
- Tracks battery levels and warnings
- Detects emergency stops and obstacles
- Logs safety events with timestamps
- Provides color-coded console output

### 📖 Background
In warehouse operations, safety monitoring is critical. The safety monitor subscribes to the robot status and generates alerts for:
- **🔴 CRITICAL**: Emergency stop, battery <10%, temperature >70°C
- **🟠 WARNING**: Battery <20%, obstacle detected, temperature >60°C
- **🟡 INFO**: Battery <40%, mission changes, system events
- **🟢 OK**: Normal operation

### 📝 Tasks

**Step 1: Create Safety Monitor Subscriber**

Create `RobotStatus_safety_monitor.py` in `ce_robot/ce_robot/`:

```python
#!/usr/bin/env python3
"""
Robot Safety Monitor
Monitors robot status and generates safety alerts
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import RobotStatus
from datetime import datetime


class RobotSafetyMonitor(Node):
    def __init__(self):
        super().__init__('robot_safety_monitor')
        
        # Subscribe to robot status
        self.subscription = self.create_subscription(
            RobotStatus,
            'robot_status',
            self.status_callback,
            10
        )
        
        # Alert tracking
        self.alert_history = []
        self.last_battery_alert = 100
        self.last_health_status = "HEALTHY"
        self.consecutive_warnings = 0
        self.emergency_stop_active = False
        
        # Statistics
        self.message_count = 0
        self.critical_count = 0
        self.warning_count = 0
        
        self.get_logger().info('🛡️ Safety Monitor started - watching for alerts...')

    def get_timestamp(self):
        """Get formatted timestamp"""
        return datetime.now().strftime("%H:%M:%S")

    def log_alert(self, level, message, robot_name="Robot"):
        """Log alert with color coding"""
        timestamp = self.get_timestamp()
        
        # Color codes for terminal
        icons = {
            'CRITICAL': '🔴',
            'WARNING': '🟠',
            'INFO': '🟡',
            'OK': '🟢'
        }
        
        icon = icons.get(level, '⚪')
        alert_msg = f'{icon} [{timestamp}] {level}: {robot_name} - {message}'
        
        # Store in history
        self.alert_history.append({
            'timestamp': timestamp,
            'level': level,
            'message': message,
            'robot': robot_name
        })
        
        # Keep only last 50 alerts
        if len(self.alert_history) > 50:
            self.alert_history.pop(0)
        
        # Count alerts
        if level == 'CRITICAL':
            self.critical_count += 1
        elif level == 'WARNING':
            self.warning_count += 1
        
        # Log with appropriate level
        if level == 'CRITICAL':
            self.get_logger().error(alert_msg)
        elif level == 'WARNING':
            self.get_logger().warn(alert_msg)
        elif level == 'INFO':
            self.get_logger().info(alert_msg)
        else:
            self.get_logger().info(alert_msg)

    def check_battery(self, msg):
        """Monitor battery levels"""
        battery = msg.battery_percentage
        robot_name = msg.robot_name
        
        # Critical battery level
        if battery < 10 and self.last_battery_alert > 10:
            self.log_alert('CRITICAL', 
                f'Battery critically low: {battery}% - Immediate charging required!',
                robot_name)
            self.last_battery_alert = battery
        
        # Low battery warning
        elif battery < 20 and self.last_battery_alert >= 20:
            self.log_alert('WARNING',
                f'Battery low: {battery}% - Return to charging station',
                robot_name)
            self.last_battery_alert = battery
        
        # Battery getting low
        elif battery < 40 and self.last_battery_alert >= 40:
            self.log_alert('INFO',
                f'Battery at {battery}% - Consider charging soon',
                robot_name)
            self.last_battery_alert = battery
        
        # Battery charged
        elif battery > 90 and self.last_battery_alert < 90:
            self.log_alert('OK',
                f'Battery charged: {battery}%',
                robot_name)
            self.last_battery_alert = battery
        
        # Charging status change
        if msg.is_charging and battery < 95:
            if battery % 20 == 0:  # Log every 20%
                self.log_alert('INFO',
                    f'Charging in progress: {battery}% (ETA: {msg.battery_time_remaining} min)',
                    robot_name)

    def check_safety(self, msg):
        """Monitor safety status"""
        robot_name = msg.robot_name
        
        # Emergency stop
        if msg.emergency_stop and not self.emergency_stop_active:
            self.log_alert('CRITICAL',
                f'EMERGENCY STOP ACTIVATED! Obstacle at {msg.obstacle_distance:.2f}m',
                robot_name)
            self.emergency_stop_active = True
        elif not msg.emergency_stop and self.emergency_stop_active:
            self.log_alert('OK',
                'Emergency stop cleared - resuming operations',
                robot_name)
            self.emergency_stop_active = False
        
        # Obstacle detection (without emergency stop)
        if msg.obstacle_detected and not msg.emergency_stop:
            self.log_alert('WARNING',
                f'Obstacle detected at {msg.obstacle_distance:.2f}m - reducing speed',
                robot_name)

    def check_health(self, msg):
        """Monitor system health"""
        robot_name = msg.robot_name
        
        # Health status change
        if msg.health_status != self.last_health_status:
            if msg.health_status == 'CRITICAL':
                self.log_alert('CRITICAL',
                    f'System health CRITICAL! (CPU: {msg.cpu_usage:.1f}%, Temp: {msg.temperature:.1f}°C)',
                    robot_name)
            elif msg.health_status == 'ERROR':
                self.log_alert('WARNING',
                    f'System health ERROR (CPU: {msg.cpu_usage:.1f}%, Temp: {msg.temperature:.1f}°C)',
                    robot_name)
            elif msg.health_status == 'WARNING':
                self.log_alert('WARNING',
                    f'System health warning (CPU: {msg.cpu_usage:.1f}%, Temp: {msg.temperature:.1f}°C)',
                    robot_name)
            elif msg.health_status == 'HEALTHY' and self.last_health_status != 'HEALTHY':
                self.log_alert('OK',
                    'System health restored to normal',
                    robot_name)
            
            self.last_health_status = msg.health_status
        
        # Critical temperature
        if msg.temperature > 70:
            self.log_alert('CRITICAL',
                f'Temperature critical: {msg.temperature:.1f}°C - System overheating!',
                robot_name)
        elif msg.temperature > 60:
            self.log_alert('WARNING',
                f'Temperature high: {msg.temperature:.1f}°C',
                robot_name)
        
        # High CPU usage
        if msg.cpu_usage > 90:
            self.log_alert('WARNING',
                f'CPU usage very high: {msg.cpu_usage:.1f}%',
                robot_name)
        
        # Motors disabled
        if not msg.motors_enabled and msg.health_status != 'CRITICAL':
            self.log_alert('INFO',
                'Motors disabled - robot stationary',
                robot_name)

    def check_mission(self, msg):
        """Monitor mission status"""
        robot_name = msg.robot_name
        
        # Mission completion
        if msg.mission_progress == 100 and msg.mission_status == "COMPLETED":
            self.log_alert('OK',
                f'Mission "{msg.current_mission}" completed successfully',
                robot_name)
        
        # Mission failed
        elif msg.mission_status == "FAILED":
            self.log_alert('WARNING',
                f'Mission "{msg.current_mission}" failed at {msg.mission_progress}%',
                robot_name)
        
        # Mission paused
        elif msg.mission_status == "PAUSED":
            self.log_alert('INFO',
                f'Mission "{msg.current_mission}" paused at {msg.mission_progress}%',
                robot_name)

    def status_callback(self, msg):
        """Process incoming robot status messages"""
        self.message_count += 1
        
        # Run all safety checks
        self.check_battery(msg)
        self.check_safety(msg)
        self.check_health(msg)
        self.check_mission(msg)
        
        # Print summary every 30 messages
        if self.message_count % 30 == 0:
            self.print_summary()

    def print_summary(self):
        """Print monitoring summary"""
        self.get_logger().info(
            f'\n📊 Safety Monitor Summary:\n'
            f'   Messages: {self.message_count}\n'
            f'   🔴 Critical Alerts: {self.critical_count}\n'
            f'   🟠 Warnings: {self.warning_count}\n'
            f'   📋 Recent Alerts: {len(self.alert_history)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotSafetyMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n🛡️ Safety Monitor shutting down...')
        node.print_summary()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 2: Make Executable**

```bash
chmod +x RobotStatus_safety_monitor.py
```

**Step 3: Update setup.py**

Add entry point in `ce_robot/setup.py`:

```python
entry_points={
    'console_scripts': [
        # Previous entries...
        '03_robot_status_publisher = ce_robot.RobotStatus_publisher:main',
        '03_robot_safety_monitor = ce_robot.RobotStatus_safety_monitor:main',
    ],
},
```

**Step 4: Build and Test**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

**Step 5: Run Both Nodes**

Terminal 1 - Start publisher:
```bash
ros2 run ce_robot 03_robot_status_publisher
```

Terminal 2 - Start safety monitor:
```bash
ros2 run ce_robot 03_robot_safety_monitor
```

### 🔍 Expected Output

**Safety Monitor Output:**
```
[INFO] [robot_safety_monitor]: 🛡️ Safety Monitor started - watching for alerts...
[INFO] [robot_safety_monitor]: 🟢 [10:30:15] OK: Warehouse Bot Alpha - Battery charged: 100%
[INFO] [robot_safety_monitor]: 🟡 [10:30:45] INFO: Warehouse Bot Alpha - Battery at 38% - Consider charging soon
[WARN] [robot_safety_monitor]: 🟠 [10:31:20] WARNING: Warehouse Bot Alpha - Battery low: 19% - Return to charging station
[WARN] [robot_safety_monitor]: 🟠 [10:31:25] WARNING: Warehouse Bot Alpha - Obstacle detected at 2.35m - reducing speed
[ERROR] [robot_safety_monitor]: 🔴 [10:31:30] CRITICAL: Warehouse Bot Alpha - EMERGENCY STOP ACTIVATED! Obstacle at 0.75m
[INFO] [robot_safety_monitor]: 🟢 [10:31:35] OK: Warehouse Bot Alpha - Emergency stop cleared - resuming operations
[ERROR] [robot_safety_monitor]: 🔴 [10:32:00] CRITICAL: Warehouse Bot Alpha - Battery critically low: 8% - Immediate charging required!
[INFO] [robot_safety_monitor]: 🟡 [10:32:05] INFO: Warehouse Bot Alpha - Motors disabled - robot stationary
[INFO] [robot_safety_monitor]: 🟡 [10:32:10] INFO: Warehouse Bot Alpha - Charging in progress: 20% (ETA: 40 min)

[INFO] [robot_safety_monitor]: 
📊 Safety Monitor Summary:
   Messages: 30
   🔴 Critical Alerts: 2
   🟠 Warnings: 4
   📋 Recent Alerts: 15
```

### 💡 Key Learning Points

- **Real-time monitoring** - subscribing and processing status updates
- **Alert generation** - triggering alerts based on conditions
- **State tracking** - remembering previous states to detect changes
- **Alert levels** - CRITICAL, WARNING, INFO, OK
- **Color-coded output** - visual feedback with icons
- **Statistics tracking** - counting alerts and events
- **Alert history** - maintaining recent alert log
- **Smart filtering** - avoiding duplicate alerts

### 🧪 Testing Scenarios

1. **Battery Monitoring**
   - Watch battery discharge from 100% → alerts at 40%, 20%, 10%
   - Observe charging alerts during recharge cycle
   - Verify critical alert at <10%

2. **Emergency Stop**
   - Wait for obstacle detection
   - Verify emergency stop when distance <1.0m
   - Confirm clear alert when obstacle removed

3. **Health Status**
   - Monitor temperature increases during movement
   - Verify health status changes (HEALTHY → WARNING → ERROR → CRITICAL)
   - Check CPU usage alerts

4. **Mission Tracking**
   - Observe mission changes (IDLE → PICKING → DELIVERING)
   - Watch mission progress updates
   - Verify completion/failure alerts

### ✅ Completion Checklist

- [ ] Created `RobotStatus_safety_monitor.py`
- [ ] Implemented battery monitoring with 3 alert levels
- [ ] Implemented emergency stop detection
- [ ] Implemented obstacle detection alerts
- [ ] Implemented health status monitoring
- [ ] Implemented mission tracking
- [ ] Added color-coded console output
- [ ] Added alert history tracking
- [ ] Added statistics summary
- [ ] Made file executable
- [ ] Updated setup.py with entry point
- [ ] Built ce_robot package successfully
- [ ] Tested with publisher running
- [ ] Verified battery alerts trigger correctly
- [ ] Verified emergency stop alerts
- [ ] Verified health status changes
- [ ] Confirmed summary prints every 30 messages

---

## **📂 Final Directory Structure**

```
📁 ROS2_WS/
├── 📁 src/
│   ├── 📁 ce_robot_interfaces/
│   │   ├── 📁 msg/
│   │   │   ├── 📄 HardwareStatus.msg        # From Readme
│   │   │   └── 📄 RobotStatus.msg            # Lab Setup
│   │   ├── 📄 package.xml
│   │   └── 📄 CMakeLists.txt
│   └── 📁 ce_robot/
│       ├── 📁 ce_robot/
│       │   ├── 📄 __init__.py
│       │   ├── 🐍 HardwareStatus_publish.py     # From Readme
│       │   ├── 🐍 HardwareStatus_subscribe.py   # From Readme
│       │   ├── 🐍 HardwareStatus_aggregate.py   # From Readme
│       │   ├── 🐍 RobotStatus_publisher.py      # Exercise 1
│       │   └── 🐍 RobotStatus_safety_monitor.py # Exercise 2
│       ├── 📄 package.xml
│       ├── 📄 setup.cfg
│       └── 📄 setup.py
└── 📁 install/
```

---

## **🔍 Useful ROS 2 Commands**

### Message Inspection
```bash
# Show RobotStatus structure
ros2 interface show ce_robot_interfaces/msg/RobotStatus

# List all custom messages
ros2 interface list | grep ce_robot

# Echo robot status topic
ros2 topic echo /robot_status

# Check topic info
ros2 topic info /robot_status

# Monitor message rate
ros2 topic hz /robot_status
```

### Debugging
```bash
# List all topics
ros2 topic list

# Check if publisher is running
ros2 node list

# Get node info
ros2 node info /robot_status_publisher

# Visualize nodes and topics
rqt_graph
```

---

## **⚠️ Troubleshooting**

### Issue: std_msgs/Header not found
**Solution:** Add std_msgs dependency
```bash
# In ce_robot_interfaces/package.xml
<depend>std_msgs</depend>

# In CMakeLists.txt
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  DEPENDENCIES std_msgs
)
```

### Issue: Module 'ce_robot_interfaces.msg' has no attribute 'RobotStatus'
**Solution:** Rebuild and source
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
source install/setup.bash
```

### Issue: Permission denied
**Solution:**
```bash
chmod +x RobotStatus_publisher.py
```

---

## **🎓 What You've Learned**

### Message Design
✅ Designing comprehensive messages for real applications  
✅ Using Header for timestamps and frames  
✅ Choosing appropriate data types for different sensors  
✅ Organizing message fields logically  
✅ Adding documentation comments to messages  

### Robot Simulation
✅ Waypoint navigation simulation  
✅ Battery discharge and charging cycles  
✅ Mission state management  
✅ Safety monitoring (emergency stop, obstacles)  
✅ System health tracking (CPU, temperature)  

### ROS 2 Best Practices
✅ Using standard message types (std_msgs/Header)  
✅ Professional logging with icons and formatting  
✅ Realistic simulation patterns  
✅ State machine implementation  
✅ Publishing complete robot telemetry  

---

## **📚 Additional Resources**

- [ROS 2 std_msgs Documentation](https://docs.ros.org/en/jazzy/p/std_msgs/)
- [ROS 2 Header Message](https://docs.ros.org/en/jazzy/p/std_msgs/interfaces/msg/Header.html)
- [ROS 2 Custom Interfaces](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [Mobile Robot Kinematics](https://robotics.stackexchange.com/questions/18014/differential-drive-mobile-robot-kinematics)

---

## **🚀 Next Steps**

Continue building your robot monitoring system:

1. ✅ **Exercise 2** - Create safety monitor subscriber *(Completed)*
2. **Exercise 3** - Implement mission control node
3. **Exercise 4** - Build dashboard with statistics
4. **Exercise 5** - Add multi-robot fleet management

---

**🎉 Congratulations! You've created a professional robot status system!** 🚀✨

You now have:
- ✅ Comprehensive `RobotStatus` message
- ✅ Realistic mobile robot simulation
- ✅ Complete telemetry publisher
- ✅ Foundation for advanced monitoring

**Keep building!** 💪🤖
