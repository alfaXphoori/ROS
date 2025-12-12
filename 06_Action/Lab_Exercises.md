# **🚀 ROS 2 Actions Lab Exercises**

## **📌 Project Title**

Hands-On Lab: Real-Life Robotics Applications with ROS 2 Actions

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

## **🛠 Overview**

This comprehensive lab demonstrates **real-world robotics applications** using ROS 2 Actions:
- **Battery Charging** - Simulate robot autonomous charging with progress monitoring
- **Navigation to Goal** - Move robot to target position with distance feedback
- **Gripper Control** - Execute pick-and-place operations with cancellation support
- **Action Definitions** - Create custom action interfaces for robot behaviors
- **Error Management** - Handle real-world failures (obstacles, low battery, sensor errors)

**Duration:** ~2.5 hours  
**Level:** Intermediate to Advanced  
**Prerequisites:** ROS 2 Jazzy installed, Parameters lab completed, understanding of robot systems

---

## **📊 Architecture**

```
┌─────────────────────────────────────────────┐
│ Exercise 1: Battery Charging Action 🔋      │
│ (Autonomous charging with safety checks)    │
│ • Monitor battery level during charging     │
│ • Validate battery health & temperature     │
│ • Report charging progress feedback         │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 2: Navigate to Goal Action 🗺️      │
│ (Robot movement with obstacle detection)    │
│ • Move to target (x, y) position           │
│ • Report distance remaining & ETA           │
│ • Detect obstacles and request cancellation │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 3: Gripper Pick & Place Action 🦾  │
│ (Manipulation with multi-stage feedback)    │
│ • Approach → Grasp → Lift → Move → Release │
│ • Force sensing and slip detection          │
│ • Cancel on object drop or grasp failure    │
└─────────────────────────────────────────────┘
```

---

## **Understanding Actions 📖**

ROS 2 actions provide a communication pattern for long-running, asynchronous tasks that require periodic feedback and can be cancelled. Unlike services (which block) or topics (which stream without acknowledgment), actions offer goal-oriented execution with progress monitoring.

### 📝 Task
1. List all action commands:
```bash
ros2 action --help
```

2. Examine action operations:
```bash
ros2 action list
ros2 action info /action_name
ros2 action send_goal /action_name action_type "goal_data" --feedback
```

3. Understand action lifecycle: PENDING → ACTIVE → SUCCEEDED/CANCELED/ABORTED

### 💡 Key Concepts
- **Goal**: What the client requests the server to do (sent once at start)
- **Result**: Final outcome returned when complete (sent once at end)
- **Feedback**: Periodic progress updates during execution (sent multiple times)
- **Cancellation**: Client can request goal termination mid-execution
- **Goal States**: PENDING, ACTIVE, SUCCEEDED, CANCELED, ABORTED
- **Non-Blocking**: Client continues running while server executes goal

### 🔍 Expected Output
```
Comparison of Action vs Service vs Topic:
- Actions: Long-running tasks (navigation, manipulation) - async with feedback
- Services: Quick calculations (add numbers, query data) - sync blocking
- Topics: Continuous streaming (sensor data, status) - one-way publish
```

---

## **⚙️ Package Configuration**

Before starting the exercises, ensure your package is properly configured:

```bash
cd ~/ros2_ws/src/ce_robot
```

### 📌 Updating `package.xml`

Verify dependencies exist in `package.xml`:

```xml
<depend>rclpy</depend>
<depend>ce_robot_interfaces</depend>
```

### 📌 Updating `setup.py`

Add entry points for all six nodes (3 servers + 3 clients) under `console_scripts`:

```python
entry_points={
    'console_scripts': [
        # Exercise 1: Battery Charging
        '06_battery_charging_server_ex1 = ce_robot.battery_charging_server_ex1:main',
        '06_battery_charging_client_ex1 = ce_robot.battery_charging_client_ex1:main',
        
        # Exercise 2: Navigate to Goal
        '06_navigate_server_ex2 = ce_robot.navigate_server_ex2:main',
        '06_navigate_client_ex2 = ce_robot.navigate_client_ex2:main',
        
        # Exercise 3: Gripper Pick & Place
        '06_gripper_server_ex3 = ce_robot.gripper_server_ex3:main',
        '06_gripper_client_ex3 = ce_robot.gripper_client_ex3:main',
    ],
},
```

---

## **📚 Learning Path Overview**

| Exercise | Title | Robot Application | Duration |
|----------|-------|-------------------|----------|
| 1 | Battery Charging Action 🔋 | Autonomous charging station | 50 min |
| 2 | Navigate to Goal Action 🗺️ | Mobile robot navigation | 45 min |
| 3 | Gripper Pick & Place Action 🦾 | Robotic manipulation | 55 min |

---

## **Exercise 1: Battery Charging Action 🔋**

### **📋 Task**

Simulate a robot autonomously charging its battery at a charging station. The action monitors battery level, validates battery health, reports charging progress with feedback, and handles charging failures (overheating, connection issues).

**Real-World Applications:**
- Warehouse robots returning to charging stations
- Vacuum cleaning robots auto-charging
- Delivery robots managing power autonomously

### **Create Custom Action Definition**

First, create the `BatteryCharging.action` file:

```bash
cd ~/ros2_ws/src/ce_robot_interfaces/action
touch BatteryCharging.action
```

#### **File: BatteryCharging.action**

```
# Goal: Target battery level and charging rate
int32 target_level      # Target battery percentage (0-100)
float32 charging_rate   # Charging rate in %/second

---

# Result: Final battery status
int32 final_level       # Final battery percentage achieved
float32 charging_time   # Total time spent charging (seconds)
bool success           # Whether charging completed successfully
string message         # Status message or error description

---

# Feedback: Charging progress
int32 current_level     # Current battery percentage
float32 temperature     # Battery temperature in Celsius
string status           # Charging status: "Initializing", "Charging", "Balancing", "Complete"
```

**Update `CMakeLists.txt`** in `ce_robot_interfaces`:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "msg/RobotTag.msg"
  "srv/CalRectangle.srv"
  "action/CountUntil.action"
  "action/DistanceCalc.action"
  "action/BatteryCharging.action"    # Add this line
  DEPENDENCIES builtin_interfaces
)
```

**Rebuild interfaces:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
source install/setup.bash
```

---

### **📁 File Location**

```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
touch battery_charging_server_ex1.py
chmod +x battery_charging_server_ex1.py
```

**Directory Structure:**
```
📁 ros2_ws/
└── 📁 src/
    └── 📁 ce_robot/
        └── 📁 ce_robot/
            ├── 📄 __init__.py
            └── 🐍 battery_charging_server_ex1.py    ← Create this file
```

---

### **Create Server File**

#### **File: battery_charging_server_ex1.py**

```python
#!/usr/bin/env python3
"""
Exercise 1: Battery Charging Server
Simulates robot autonomous charging with safety monitoring
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ce_robot_interfaces.action import BatteryCharging
import time
import random


class BatteryChargingActionServer(Node):
    def __init__(self):
        super().__init__('battery_charging_server_ex1')
        
        self._action_server = ActionServer(
            self,
            BatteryCharging,
            'battery_charging_ex1',
            self.execute_callback
        )
        
        # Simulate initial battery state
        self.current_battery = 20  # Start at 20%
        
        self.get_logger().info('🔋 Battery Charging Server started')
        self.get_logger().info(f'Initial battery level: {self.current_battery}%')

    def validate_goal(self, goal):
        """Validate charging parameters"""
        errors = []
        
        if goal.target_level < 0 or goal.target_level > 100:
            errors.append('Target level must be 0-100%')
        
        if goal.target_level <= self.current_battery:
            errors.append(f'Target ({goal.target_level}%) must be > current ({self.current_battery}%)')
        
        if goal.charging_rate <= 0 or goal.charging_rate > 10:
            errors.append('Charging rate must be 0.1-10 %/second')
        
        return errors

    def execute_callback(self, goal_handle):
        """Execute battery charging with safety monitoring"""
        goal = goal_handle.request
        start_time = time.time()
        
        self.get_logger().info(
            f'[EX1] 🔌 Starting charge: {self.current_battery}% → {goal.target_level}% at {goal.charging_rate}%/s'
        )
        
        # Validate goal
        errors = self.validate_goal(goal)
        if errors:
            self.get_logger().error('[EX1] ❌ Charging failed - validation errors:')
            for error in errors:
                self.get_logger().error(f'  - {error}')
            goal_handle.abort()
            return BatteryCharging.Result(
                final_level=self.current_battery,
                charging_time=0.0,
                success=False,
                message='; '.join(errors)
            )
        
        feedback_msg = BatteryCharging.Feedback()
        
        try:
            # Phase 1: Initialize charging
            feedback_msg.status = "Initializing"
            feedback_msg.current_level = self.current_battery
            feedback_msg.temperature = 25.0
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info('[EX1] 🔧 Initializing charging connection...')
            time.sleep(1)
            
            # Phase 2: Charging loop
            feedback_msg.status = "Charging"
            while self.current_battery < goal.target_level:
                # Simulate charging increment
                self.current_battery = min(
                    self.current_battery + goal.charging_rate,
                    goal.target_level
                )
                
                # Simulate battery temperature (increases during charging)
                feedback_msg.temperature = 25.0 + (self.current_battery / 100.0) * 15.0 + random.uniform(-2, 2)
                
                # Check for overheating
                if feedback_msg.temperature > 45.0:
                    self.get_logger().error(f'[EX1] 🔥 Overheating detected: {feedback_msg.temperature:.1f}°C')
                    goal_handle.abort()
                    return BatteryCharging.Result(
                        final_level=self.current_battery,
                        charging_time=time.time() - start_time,
                        success=False,
                        message=f'Overheating at {feedback_msg.temperature:.1f}°C'
                    )
                
                # Publish feedback
                feedback_msg.current_level = int(self.current_battery)
                goal_handle.publish_feedback(feedback_msg)
                
                self.get_logger().info(
                    f'[EX1] ⚡ Charging: {feedback_msg.current_level}% | Temp: {feedback_msg.temperature:.1f}°C'
                )
                time.sleep(1)
            
            # Phase 3: Balancing (final 5% takes longer)
            if goal.target_level >= 95:
                feedback_msg.status = "Balancing"
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info('[EX1] ⚖️  Balancing cells...')
                time.sleep(2)
            
            # Complete
            charging_time = time.time() - start_time
            feedback_msg.status = "Complete"
            goal_handle.publish_feedback(feedback_msg)
            
            goal_handle.succeed()
            result = BatteryCharging.Result()
            result.final_level = int(self.current_battery)
            result.charging_time = charging_time
            result.success = True
            result.message = f'Charged to {result.final_level}% in {charging_time:.1f}s'
            
            self.get_logger().info(f'[EX1] ✅ Charging complete: {result.message}')
            return result
            
        except Exception as e:
            self.get_logger().error(f'[EX1] ❌ Charging error: {str(e)}')
            goal_handle.abort()
            return BatteryCharging.Result(
                final_level=self.current_battery,
                charging_time=time.time() - start_time,
                success=False,
                message=f'Error: {str(e)}'
            )


def main(args=None):
    rclpy.init(args=args)
    server = BatteryChargingActionServer()
    rclpy.spin(server)


if __name__ == '__main__':
    main()
```

---

### **Create Client File**

#### **File: battery_charging_client_ex1.py**

```python
#!/usr/bin/env python3
"""
Exercise 1: Battery Charging Client
Sends charging goal and monitors progress
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ce_robot_interfaces.action import BatteryCharging


class BatteryChargingActionClient(Node):
    def __init__(self):
        super().__init__('battery_charging_client_ex1')
        self._action_client = ActionClient(
            self,
            BatteryCharging,
            'battery_charging_ex1'
        )
        self.get_logger().info('🔋 Battery Charging Client initialized')

    def send_goal(self, target_level, charging_rate):
        """Send charging goal"""
        self.get_logger().info('[EX1] 📡 Waiting for charging server...')
        self._action_client.wait_for_server()
        
        goal_msg = BatteryCharging.Goal()
        goal_msg.target_level = target_level
        goal_msg.charging_rate = charging_rate
        
        self.get_logger().info(
            f'[EX1] 🎯 Requesting charge to {target_level}% at {charging_rate}%/s'
        )
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('[EX1] ❌ Charging goal rejected!')
            return
        
        self.get_logger().info('[EX1] ✅ Charging goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Receive charging feedback"""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[EX1] 📊 {fb.status}: {fb.current_level}% | {fb.temperature:.1f}°C'
        )

    def result_callback(self, future):
        """Receive final result"""
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f'[EX1] ✅ {result.message}'
            )
        else:
            self.get_logger().error(
                f'[EX1] ❌ Charging failed: {result.message}'
            )


def main(args=None):
    rclpy.init(args=args)
    client = BatteryChargingActionClient()
    
    # Request: Charge to 80% at 5%/second
    client.send_goal(target_level=80, charging_rate=5.0)
    rclpy.spin(client)


if __name__ == '__main__':
    main()
```

---

### **Build and Test**

#### **Step 1: Build the Packages**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

#### **Step 2: Run the Nodes**

**Terminal 1 - Start Charging Server:**
```bash
ros2 run ce_robot 06_battery_charging_server_ex1
```

**Terminal 2 - Run Charging Client:**
```bash
ros2 run ce_robot 06_battery_charging_client_ex1
```

---

### **Expected Output**

**Server Terminal:**
```
[INFO] [battery_charging_server_ex1]: 🔋 Battery Charging Server started
[INFO] [battery_charging_server_ex1]: Initial battery level: 20%
[INFO] [battery_charging_server_ex1]: [EX1] 🔌 Starting charge: 20% → 80% at 5.0%/s
[INFO] [battery_charging_server_ex1]: [EX1] 🔧 Initializing charging connection...
[INFO] [battery_charging_server_ex1]: [EX1] ⚡ Charging: 25% | Temp: 28.3°C
[INFO] [battery_charging_server_ex1]: [EX1] ⚡ Charging: 30% | Temp: 29.7°C
[INFO] [battery_charging_server_ex1]: [EX1] ⚡ Charging: 35% | Temp: 31.2°C
...
[INFO] [battery_charging_server_ex1]: [EX1] ⚡ Charging: 80% | Temp: 37.4°C
[INFO] [battery_charging_server_ex1]: [EX1] ✅ Charging complete: Charged to 80% in 13.2s
```

**Client Terminal:**
```
[INFO] [battery_charging_client_ex1]: 🔋 Battery Charging Client initialized
[INFO] [battery_charging_client_ex1]: [EX1] 📡 Waiting for charging server...
[INFO] [battery_charging_client_ex1]: [EX1] 🎯 Requesting charge to 80% at 5.0%/s
[INFO] [battery_charging_client_ex1]: [EX1] ✅ Charging goal accepted!
[INFO] [battery_charging_client_ex1]: [EX1] 📊 Initializing: 20% | 25.0°C
[INFO] [battery_charging_client_ex1]: [EX1] 📊 Charging: 25% | 28.3°C
[INFO] [battery_charging_client_ex1]: [EX1] 📊 Charging: 30% | 29.7°C
...
[INFO] [battery_charging_client_ex1]: [EX1] 📊 Complete: 80% | 37.4°C
[INFO] [battery_charging_client_ex1]: [EX1] ✅ Charged to 80% in 13.2s
```

---

### **💡 Key Concepts**

| Concept | Description | Implementation |
|---------|-------------|----------------|
| **Real Robot Task** | Autonomous charging simulation | Battery monitoring + safety checks |
| **Goal Validation** | Verify parameters before execution | `validate_goal()` function |
| **Multi-Phase Execution** | Initialize → Charge → Balance | Status: "Initializing", "Charging", "Balancing" |
| **Safety Monitoring** | Temperature and overheating detection | Abort if temp > 45°C |
| **Progress Feedback** | Real-time battery level + temperature | `feedback_msg.current_level`, `temperature` |
| **Failure Handling** | Graceful error recovery | `goal_handle.abort()` with error message |

---

### **🔍 Testing Variations**

Test different charging scenarios by modifying the client's `main()`:

```python
# Test 1: Normal charging (20% → 80%)
client.send_goal(target_level=80, charging_rate=5.0)

# Test 2: Fast charging (higher rate)
client.send_goal(target_level=60, charging_rate=8.0)

# Test 3: Full charge with balancing (95%+)
client.send_goal(target_level=100, charging_rate=3.0)

# Test 4: Invalid goal (target < current)
client.send_goal(target_level=10, charging_rate=5.0)  # Should reject

# Test 5: Invalid rate
client.send_goal(target_level=80, charging_rate=15.0)  # Should reject
```

---

### **✅ Exercise 1 Completion Checklist**

- [ ] Created `BatteryCharging.action` definition
- [ ] Updated `CMakeLists.txt` and rebuilt interfaces
- [ ] Created `battery_charging_server_ex1.py` with safety monitoring
- [ ] Created `battery_charging_client_ex1.py`
- [ ] Added entry points to `setup.py`
- [ ] Built packages successfully with `colcon build`
- [ ] Server validates charging parameters
- [ ] Server monitors battery temperature
- [ ] Feedback shows charging progress (level + temperature)
- [ ] Normal charging (20% → 80%) completes successfully
- [ ] Invalid goals (target < current, invalid rate) rejected
- [ ] Overheating detection aborts charging
- [ ] Understood real robot autonomous charging pattern

---

## **Exercise 2: Navigate to Goal Action 🗺️**

### **📋 Task**

Simulate a mobile robot navigating to a target (x, y) position. The action calculates distance remaining, estimates time of arrival (ETA), reports progress feedback, and handles obstacles that require cancellation.

**Real-World Applications:**
- Warehouse robots navigating to shelves
- Delivery robots moving to target locations
- Service robots approaching customers
- AGVs (Automated Guided Vehicles) in factories

### **Create Custom Action Definition**

First, create the `NavigateToGoal.action` file:

```bash
cd ~/ros2_ws/src/ce_robot_interfaces/action
touch NavigateToGoal.action
```

#### **File: NavigateToGoal.action**

```
# Goal: Target position for robot navigation
float32 target_x        # Target x coordinate (meters)
float32 target_y        # Target y coordinate (meters)
float32 speed           # Movement speed (m/s)

---

# Result: Navigation outcome
bool success            # Whether robot reached goal
float32 final_x         # Final x position achieved
float32 final_y         # Final y position achieved
float32 travel_time     # Total navigation time (seconds)
string message          # Status message or error description

---

# Feedback: Navigation progress
float32 current_x       # Current x position
float32 current_y       # Current y position
float32 distance_remaining  # Distance to goal (meters)
float32 eta_seconds     # Estimated time of arrival (seconds)
string status           # Status: "Moving", "Avoiding Obstacle", "Near Goal"
```

**Update `CMakeLists.txt`** in `ce_robot_interfaces`:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "msg/RobotTag.msg"
  "srv/CalRectangle.srv"
  "action/CountUntil.action"
  "action/BatteryCharging.action"
  "action/NavigateToGoal.action"    # Add this line
  DEPENDENCIES builtin_interfaces
)
```

**Rebuild interfaces:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
source install/setup.bash
```

---

### **📁 File Location**

```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
touch navigate_server_ex2.py
chmod +x navigate_server_ex2.py
```

---

### **Create Server File**

#### **File: navigate_server_ex2.py**

```python
#!/usr/bin/env python3
"""
Exercise 2: Navigate to Goal Server
Simulates mobile robot navigation with obstacle detection
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ce_robot_interfaces.action import NavigateToGoal
import math
import time
import random


class NavigateToGoalActionServer(Node):
    def __init__(self):
        super().__init__('navigate_server_ex2')
        
        self._action_server = ActionServer(
            self,
            NavigateToGoal,
            'navigate_to_goal_ex2',
            self.execute_callback
        )
        
        # Robot starting position
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        self.get_logger().info('🗺️  Navigate to Goal Server started')
        self.get_logger().info(f'Robot initial position: ({self.robot_x:.2f}, {self.robot_y:.2f})')

    def validate_goal(self, goal):
        """Validate navigation goal"""
        errors = []
        
        if math.isnan(goal.target_x) or math.isnan(goal.target_y):
            errors.append('Target coordinates contain NaN')
        
        if abs(goal.target_x) > 100 or abs(goal.target_y) > 100:
            errors.append('Target too far from workspace (max ±100m)')
        
        if goal.speed <= 0 or goal.speed > 2.0:
            errors.append('Speed must be 0.1-2.0 m/s')
        
        # Check if already at goal
        dist = math.sqrt((goal.target_x - self.robot_x)**2 + (goal.target_y - self.robot_y)**2)
        if dist < 0.1:
            errors.append(f'Already at goal (distance: {dist:.3f}m)')
        
        return errors

    def execute_callback(self, goal_handle):
        """Execute navigation to goal"""
        goal = goal_handle.request
        start_time = time.time()
        
        self.get_logger().info(
            f'[EX2] 🚀 Navigating from ({self.robot_x:.2f}, {self.robot_y:.2f}) '
            f'to ({goal.target_x:.2f}, {goal.target_y:.2f}) at {goal.speed}m/s'
        )
        
        # Validate goal
        errors = self.validate_goal(goal)
        if errors:
            self.get_logger().error('[EX2] ❌ Navigation failed - validation errors:')
            for error in errors:
                self.get_logger().error(f'  - {error}')
            goal_handle.abort()
            return NavigateToGoal.Result(
                success=False,
                final_x=self.robot_x,
                final_y=self.robot_y,
                travel_time=0.0,
                message='; '.join(errors)
            )
        
        feedback_msg = NavigateToGoal.Feedback()
        
        try:
            # Calculate initial distance
            total_distance = math.sqrt(
                (goal.target_x - self.robot_x)**2 + (goal.target_y - self.robot_y)**2
            )
            
            self.get_logger().info(f'[EX2] 📏 Total distance: {total_distance:.2f}m')
            
            # Navigation loop
            while True:
                # Calculate remaining distance
                distance_remaining = math.sqrt(
                    (goal.target_x - self.robot_x)**2 + (goal.target_y - self.robot_y)**2
                )
                
                # Check if goal reached
                if distance_remaining < 0.1:
                    feedback_msg.status = "Goal Reached"
                    break
                
                # Simulate obstacle detection (5% chance per iteration)
                if random.random() < 0.05:
                    self.get_logger().warn('[EX2] ⚠️  Obstacle detected! Stopping navigation.')
                    goal_handle.abort()
                    return NavigateToGoal.Result(
                        success=False,
                        final_x=self.robot_x,
                        final_y=self.robot_y,
                        travel_time=time.time() - start_time,
                        message=f'Obstacle detected at ({self.robot_x:.2f}, {self.robot_y:.2f})'
                    )
                
                # Move robot toward goal
                dx = goal.target_x - self.robot_x
                dy = goal.target_y - self.robot_y
                angle = math.atan2(dy, dx)
                
                # Move distance per iteration (speed * time_step)
                step_distance = min(goal.speed * 0.5, distance_remaining)
                self.robot_x += step_distance * math.cos(angle)
                self.robot_y += step_distance * math.sin(angle)
                
                # Calculate ETA
                eta = distance_remaining / goal.speed if goal.speed > 0 else 0
                
                # Determine status
                if distance_remaining < 1.0:
                    feedback_msg.status = "Near Goal"
                else:
                    feedback_msg.status = "Moving"
                
                # Publish feedback
                feedback_msg.current_x = self.robot_x
                feedback_msg.current_y = self.robot_y
                feedback_msg.distance_remaining = distance_remaining
                feedback_msg.eta_seconds = eta
                goal_handle.publish_feedback(feedback_msg)
                
                self.get_logger().info(
                    f'[EX2] 📍 Position: ({self.robot_x:.2f}, {self.robot_y:.2f}) | '
                    f'Remaining: {distance_remaining:.2f}m | ETA: {eta:.1f}s | {feedback_msg.status}'
                )
                
                time.sleep(0.5)
            
            # Success
            travel_time = time.time() - start_time
            goal_handle.succeed()
            
            self.get_logger().info(
                f'[EX2] ✅ Navigation complete! '
                f'Reached ({self.robot_x:.2f}, {self.robot_y:.2f}) in {travel_time:.1f}s'
            )
            
            return NavigateToGoal.Result(
                success=True,
                final_x=self.robot_x,
                final_y=self.robot_y,
                travel_time=travel_time,
                message=f'Reached goal in {travel_time:.1f}s'
            )
            
        except Exception as e:
            self.get_logger().error(f'[EX2] ❌ Navigation error: {str(e)}')
            goal_handle.abort()
            return NavigateToGoal.Result(
                success=False,
                final_x=self.robot_x,
                final_y=self.robot_y,
                travel_time=time.time() - start_time,
                message=f'Error: {str(e)}'
            )


def main(args=None):
    rclpy.init(args=args)
    server = NavigateToGoalActionServer()
    rclpy.spin(server)


if __name__ == '__main__':
    main()
```

---

### **Create Client File**

#### **File: navigate_client_ex2.py**

```python
#!/usr/bin/env python3
"""
Exercise 2: Navigate to Goal Client
Sends navigation goal and monitors progress
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ce_robot_interfaces.action import NavigateToGoal


class NavigateToGoalActionClient(Node):
    def __init__(self):
        super().__init__('navigate_client_ex2')
        self._action_client = ActionClient(
            self,
            NavigateToGoal,
            'navigate_to_goal_ex2'
        )
        self.get_logger().info('🗺️  Navigate to Goal Client initialized')

    def send_goal(self, target_x, target_y, speed):
        """Send navigation goal"""
        self.get_logger().info('[EX2] 📡 Waiting for navigation server...')
        self._action_client.wait_for_server()
        
        goal_msg = NavigateToGoal.Goal()
        goal_msg.target_x = target_x
        goal_msg.target_y = target_y
        goal_msg.speed = speed
        
        self.get_logger().info(
            f'[EX2] 🎯 Requesting navigation to ({target_x:.2f}, {target_y:.2f}) at {speed}m/s'
        )
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('[EX2] ❌ Navigation goal rejected!')
            return
        
        self.get_logger().info('[EX2] ✅ Navigation goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Receive navigation feedback"""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[EX2] 📊 {fb.status}: ({fb.current_x:.2f}, {fb.current_y:.2f}) | '
            f'{fb.distance_remaining:.2f}m remaining | ETA: {fb.eta_seconds:.1f}s'
        )

    def result_callback(self, future):
        """Receive final result"""
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f'[EX2] ✅ {result.message}'
            )
        else:
            self.get_logger().error(
                f'[EX2] ❌ Navigation failed: {result.message}'
            )


def main(args=None):
    rclpy.init(args=args)
    client = NavigateToGoalActionClient()
    
    # Navigate to position (5.0, 5.0) at 1.0 m/s
    client.send_goal(target_x=5.0, target_y=5.0, speed=1.0)
    rclpy.spin(client)


if __name__ == '__main__':
    main()
```

---

### **Build and Test**

#### **Step 1: Build the Packages**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

#### **Step 2: Run the Nodes**

**Terminal 1 - Start Navigation Server:**
```bash
ros2 run ce_robot 06_navigate_server_ex2
```

**Terminal 2 - Run Navigation Client:**
```bash
ros2 run ce_robot 06_navigate_client_ex2
```

---

### **Expected Output**

**Server Terminal:**
```
[INFO] [navigate_server_ex2]: 🗺️  Navigate to Goal Server started
[INFO] [navigate_server_ex2]: Robot initial position: (0.00, 0.00)
[INFO] [navigate_server_ex2]: [EX2] 🚀 Navigating from (0.00, 0.00) to (5.00, 5.00) at 1.0m/s
[INFO] [navigate_server_ex2]: [EX2] 📏 Total distance: 7.07m
[INFO] [navigate_server_ex2]: [EX2] 📍 Position: (0.35, 0.35) | Remaining: 6.57m | ETA: 6.6s | Moving
[INFO] [navigate_server_ex2]: [EX2] 📍 Position: (0.71, 0.71) | Remaining: 6.07m | ETA: 6.1s | Moving
[INFO] [navigate_server_ex2]: [EX2] 📍 Position: (1.06, 1.06) | Remaining: 5.57m | ETA: 5.6s | Moving
...
[INFO] [navigate_server_ex2]: [EX2] 📍 Position: (4.82, 4.82) | Remaining: 0.25m | ETA: 0.3s | Near Goal
[INFO] [navigate_server_ex2]: [EX2] ✅ Navigation complete! Reached (5.00, 5.00) in 7.2s
```

**Client Terminal:**
```
[INFO] [navigate_client_ex2]: 🗺️  Navigate to Goal Client initialized
[INFO] [navigate_client_ex2]: [EX2] 📡 Waiting for navigation server...
[INFO] [navigate_client_ex2]: [EX2] 🎯 Requesting navigation to (5.00, 5.00) at 1.0m/s
[INFO] [navigate_client_ex2]: [EX2] ✅ Navigation goal accepted!
[INFO] [navigate_client_ex2]: [EX2] 📊 Moving: (0.35, 0.35) | 6.57m remaining | ETA: 6.6s
[INFO] [navigate_client_ex2]: [EX2] 📊 Moving: (0.71, 0.71) | 6.07m remaining | ETA: 6.1s
...
[INFO] [navigate_client_ex2]: [EX2] 📊 Near Goal: (4.82, 4.82) | 0.25m remaining | ETA: 0.3s
[INFO] [navigate_client_ex2]: [EX2] ✅ Reached goal in 7.2s
```

---

### **💡 Key Concepts**

| Concept | Description | Implementation |
|---------|-------------|----------------|
| **Real Robot Navigation** | Mobile robot path planning | Position tracking + distance calculation |
| **Goal Validation** | Verify reachable targets | Check coordinates within workspace bounds |
| **Progress Monitoring** | Real-time position feedback | `current_x`, `current_y`, `distance_remaining`, `ETA` |
| **Obstacle Detection** | Environment awareness | Random obstacle simulation (5% chance) |
| **Dynamic Status** | Contextual feedback | "Moving", "Near Goal", "Avoiding Obstacle" |
| **Failure Handling** | Abort on obstacles | `goal_handle.abort()` with obstacle message |

---

### **🔍 Testing Variations**

Test different navigation scenarios by modifying the client's `main()`:

```python
# Test 1: Short distance navigation
client.send_goal(target_x=2.0, target_y=2.0, speed=1.0)

# Test 2: Fast navigation (higher speed)
client.send_goal(target_x=10.0, target_y=0.0, speed=2.0)

# Test 3: Diagonal navigation
client.send_goal(target_x=-5.0, target_y=5.0, speed=0.5)

# Test 4: Invalid goal (already at position)
client.send_goal(target_x=0.0, target_y=0.0, speed=1.0)  # Should reject

# Test 5: Out of workspace bounds
client.send_goal(target_x=150.0, target_y=150.0, speed=1.0)  # Should reject
```

---

### **✅ Exercise 2 Completion Checklist**

- [ ] Created `NavigateToGoal.action` definition
- [ ] Updated `CMakeLists.txt` and rebuilt interfaces
- [ ] Created `navigate_server_ex2.py` with obstacle detection
- [ ] Created `navigate_client_ex2.py`
- [ ] Added entry points to `setup.py`
- [ ] Built packages successfully with `colcon build`
- [ ] Server validates navigation parameters
- [ ] Feedback shows position, distance remaining, and ETA
- [ ] Robot successfully navigates to (5.0, 5.0)
- [ ] Obstacle detection triggers abort (test multiple times)
- [ ] Invalid goals rejected (out of bounds, already at goal)
- [ ] Understood real robot navigation patterns

---

## **Exercise 3: Gripper Pick & Place Action 🤖**

### **📋 Task**

Simulate a robotic gripper performing a pick-and-place operation with multiple stages: Approach → Grasp → Lift → Move → Release. The action demonstrates cancellation handling (object drop detection), force sensing, and multi-stage manipulation feedback.

**Real-World Applications:**
- Manufacturing pick-and-place robots
- Warehouse sorting and packing systems
- Collaborative robots (cobots) in assembly lines
- Robotic arms in research labs

### **Create Custom Action Definition**

First, create the `GripperPickPlace.action` file:

```bash
cd ~/ros2_ws/src/ce_robot_interfaces/action
touch GripperPickPlace.action
```

#### **File: GripperPickPlace.action**

```
# Goal: Object to pick and target location
string object_id        # ID of object to pick
float32 target_x        # Drop-off x coordinate
float32 target_y        # Drop-off y coordinate
float32 grip_force      # Gripping force (Newtons)

---

# Result: Operation outcome
bool success            # Whether pick-and-place succeeded
string final_stage      # Last completed stage
float32 execution_time  # Total execution time (seconds)
string message          # Status message or error description

---

# Feedback: Multi-stage progress
string stage            # Current stage: "Approaching", "Grasping", "Lifting", "Moving", "Releasing"
float32 progress_percent    # Stage progress (0-100%)
float32 gripper_force   # Current gripper force (Newtons)
bool object_detected    # Whether object is in gripper
string status_message   # Detailed status
```

**Update `CMakeLists.txt`** in `ce_robot_interfaces`:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "msg/RobotTag.msg"
  "srv/CalRectangle.srv"
  "action/CountUntil.action"
  "action/BatteryCharging.action"
  "action/NavigateToGoal.action"
  "action/GripperPickPlace.action"    # Add this line
  DEPENDENCIES builtin_interfaces
)
```

**Rebuild interfaces:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
source install/setup.bash
```

---

### **📁 File Location**

```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
touch gripper_server_ex3.py
chmod +x gripper_server_ex3.py
```

---

### **Create Server File**

#### **File: gripper_server_ex3.py**

```python
#!/usr/bin/env python3
"""
Exercise 3: Gripper Pick & Place Server
Simulates robotic gripper manipulation with cancellation
"""

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from ce_robot_interfaces.action import GripperPickPlace
import time
import random


class GripperPickPlaceActionServer(Node):
    def __init__(self):
        super().__init__('gripper_server_ex3')
        
        self._action_server = ActionServer(
            self,
            GripperPickPlace,
            'gripper_pick_place_ex3',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.goal_counter = 0
        self.get_logger().info('🤖 Gripper Pick & Place Server started')

    def goal_callback(self, goal_request):
        """Accept or reject goal based on validation"""
        self.get_logger().info(f'[EX3] 📋 Received goal for object: {goal_request.object_id}')
        
        # Validate grip force
        if goal_request.grip_force < 1.0 or goal_request.grip_force > 50.0:
            self.get_logger().error('[EX3] ❌ Invalid grip force (must be 1-50N)')
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle cancellation request"""
        self.get_logger().info('[EX3] 🛑 Cancellation requested')
        return CancelResponse.ACCEPT

    def validate_goal(self, goal):
        """Validate goal parameters"""
        errors = []
        
        if not goal.object_id or len(goal.object_id) == 0:
            errors.append('Object ID cannot be empty')
        
        if abs(goal.target_x) > 10 or abs(goal.target_y) > 10:
            errors.append('Target position out of reach (max ±10m)')
        
        return errors

    def execute_stage(self, goal_handle, stage_name, duration, feedback_msg):
        """Execute a single manipulation stage with cancellation check"""
        steps = 10
        for step in range(steps + 1):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().warn(f'[EX3] 🛑 Cancelled during {stage_name}')
                goal_handle.canceled()
                return False
            
            # Update progress
            progress = (step / steps) * 100
            feedback_msg.stage = stage_name
            feedback_msg.progress_percent = progress
            feedback_msg.status_message = f'{stage_name} {progress:.0f}%'
            
            # Simulate force sensing
            if stage_name == "Grasping":
                feedback_msg.gripper_force = (step / steps) * feedback_msg.gripper_force
                feedback_msg.object_detected = (step > 5)
            elif stage_name in ["Lifting", "Moving"]:
                feedback_msg.object_detected = True
                # Simulate object drop (2% chance per step)
                if random.random() < 0.02:
                    self.get_logger().error('[EX3] ⚠️  Object slipped! Aborting...')
                    goal_handle.abort()
                    return False
            
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'[EX3] {feedback_msg.status_message}')
            time.sleep(duration / steps)
        
        return True

    def execute_callback(self, goal_handle):
        """Execute pick-and-place operation"""
        self.goal_counter += 1
        goal_id = self.goal_counter
        goal = goal_handle.request
        start_time = time.time()
        
        self.get_logger().info(
            f'[EX3:#{goal_id}] 🚀 Starting pick-and-place: {goal.object_id} → '
            f'({goal.target_x:.2f}, {goal.target_y:.2f}) with {goal.grip_force}N force'
        )
        
        # Validate goal
        errors = self.validate_goal(goal)
        if errors:
            self.get_logger().error('[EX3] ❌ Validation failed:')
            for error in errors:
                self.get_logger().error(f'  - {error}')
            goal_handle.abort()
            return GripperPickPlace.Result(
                success=False,
                final_stage="Validation",
                execution_time=0.0,
                message='; '.join(errors)
            )
        
        feedback_msg = GripperPickPlace.Feedback()
        feedback_msg.gripper_force = goal.grip_force
        feedback_msg.object_detected = False
        
        try:
            # Stage 1: Approach object
            if not self.execute_stage(goal_handle, "Approaching", 2.0, feedback_msg):
                return self._create_result(goal_handle, False, "Approaching", start_time, "Cancelled")
            
            # Stage 2: Grasp object
            if not self.execute_stage(goal_handle, "Grasping", 1.5, feedback_msg):
                return self._create_result(goal_handle, False, "Grasping", start_time, "Cancelled")
            
            # Check if object grasped
            if not feedback_msg.object_detected:
                self.get_logger().error('[EX3] ❌ Failed to grasp object')
                goal_handle.abort()
                return GripperPickPlace.Result(
                    success=False,
                    final_stage="Grasping",
                    execution_time=time.time() - start_time,
                    message="Object grasp failed"
                )
            
            # Stage 3: Lift object
            if not self.execute_stage(goal_handle, "Lifting", 1.0, feedback_msg):
                return self._create_result(goal_handle, False, "Lifting", start_time, "Object dropped")
            
            # Stage 4: Move to target
            if not self.execute_stage(goal_handle, "Moving", 3.0, feedback_msg):
                return self._create_result(goal_handle, False, "Moving", start_time, "Object dropped")
            
            # Stage 5: Release object
            feedback_msg.gripper_force = 0.0
            if not self.execute_stage(goal_handle, "Releasing", 1.0, feedback_msg):
                return self._create_result(goal_handle, False, "Releasing", start_time, "Cancelled")
            
            # Success
            execution_time = time.time() - start_time
            goal_handle.succeed()
            
            self.get_logger().info(
                f'[EX3:#{goal_id}] ✅ Pick-and-place complete! '
                f'{goal.object_id} moved to ({goal.target_x:.2f}, {goal.target_y:.2f}) in {execution_time:.1f}s'
            )
            
            return GripperPickPlace.Result(
                success=True,
                final_stage="Releasing",
                execution_time=execution_time,
                message=f'Completed in {execution_time:.1f}s'
            )
            
        except Exception as e:
            self.get_logger().error(f'[EX3:#{goal_id}] ❌ Execution error: {str(e)}')
            goal_handle.abort()
            return GripperPickPlace.Result(
                success=False,
                final_stage="Error",
                execution_time=time.time() - start_time,
                message=f'Error: {str(e)}'
            )

    def _create_result(self, goal_handle, success, stage, start_time, message):
        """Helper to create result with current state"""
        return GripperPickPlace.Result(
            success=success,
            final_stage=stage,
            execution_time=time.time() - start_time,
            message=message
        )


def main(args=None):
    rclpy.init(args=args)
    server = GripperPickPlaceActionServer()
    rclpy.spin(server)


if __name__ == '__main__':
    main()
```

---

### **Create Client File**

#### **File: gripper_client_ex3.py**

```python
#!/usr/bin/env python3
"""
Exercise 3: Gripper Pick & Place Client
Tests manipulation with cancellation support
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ce_robot_interfaces.action import GripperPickPlace
import threading
import time


class GripperPickPlaceActionClient(Node):
    def __init__(self):
        super().__init__('gripper_client_ex3')
        self._action_client = ActionClient(
            self,
            GripperPickPlace,
            'gripper_pick_place_ex3'
        )
        self.goal_handle = None
        self.get_logger().info('🤖 Gripper Pick & Place Client initialized')

    def send_goal(self, object_id, target_x, target_y, grip_force, cancel_after=None):
        """Send pick-and-place goal"""
        self.cancel_after = cancel_after
        self.get_logger().info('[EX3] 📡 Waiting for gripper server...')
        self._action_client.wait_for_server()
        
        goal_msg = GripperPickPlace.Goal()
        goal_msg.object_id = object_id
        goal_msg.target_x = target_x
        goal_msg.target_y = target_y
        goal_msg.grip_force = grip_force
        
        self.get_logger().info(
            f'[EX3] 🎯 Requesting pick-and-place: {object_id} → ({target_x:.2f}, {target_y:.2f}) at {grip_force}N'
        )
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error('[EX3] ❌ Goal rejected!')
            return
        
        self.get_logger().info('[EX3] ✅ Goal accepted!')
        
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)
        
        # Schedule cancellation if requested
        if self.cancel_after:
            threading.Thread(target=self._cancel_after_delay, daemon=True).start()

    def _cancel_after_delay(self):
        """Cancel goal after delay for testing"""
        time.sleep(self.cancel_after)
        if self.goal_handle:
            self.get_logger().info('[EX3] 🛑 Requesting cancellation...')
            self._cancel_future = self.goal_handle.cancel_goal_async()
            self._cancel_future.add_done_callback(self.cancel_callback)

    def cancel_callback(self, future):
        """Handle cancellation response"""
        cancel_response = future.result()
        if cancel_response.return_code == 0:
            self.get_logger().info('[EX3] ✅ Cancellation accepted')
        else:
            self.get_logger().warn('[EX3] ❌ Cancellation rejected')

    def feedback_callback(self, feedback_msg):
        """Receive manipulation feedback"""
        fb = feedback_msg.feedback
        obj_status = "✅ Held" if fb.object_detected else "❌ No object"
        self.get_logger().info(
            f'[EX3] 📊 {fb.stage} ({fb.progress_percent:.0f}%) | '
            f'Force: {fb.gripper_force:.1f}N | {obj_status}'
        )

    def result_callback(self, future):
        """Receive final result"""
        result = future.result()
        res = result.result
        
        if result.status == 4:  # CANCELED
            self.get_logger().info(f'[EX3] 🛑 Operation CANCELLED at {res.final_stage}')
        elif result.status == 5:  # ABORTED
            self.get_logger().error(f'[EX3] ❌ Operation ABORTED: {res.message}')
        elif res.success:
            self.get_logger().info(f'[EX3] ✅ SUCCESS: {res.message}')
        else:
            self.get_logger().error(f'[EX3] ❌ FAILED: {res.message}')


def main(args=None):
    rclpy.init(args=args)
    client = GripperPickPlaceActionClient()
    
    # Test 1: Normal operation (no cancellation)
    client.send_goal(
        object_id="Box_A",
        target_x=3.0,
        target_y=2.0,
        grip_force=15.0,
        cancel_after=None  # Change to 4.0 to test cancellation
    )
    rclpy.spin(client)


if __name__ == '__main__':
    main()
```

---

### **Build and Test**

#### **Step 1: Build the Packages**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

#### **Step 2: Run the Nodes**

**Terminal 1 - Start Gripper Server:**
```bash
ros2 run ce_robot 06_gripper_server_ex3
```

**Terminal 2 - Run Gripper Client:**
```bash
ros2 run ce_robot 06_gripper_client_ex3
```

---

### **Expected Output**

**Server Terminal (Normal Operation):**
```
[INFO] [gripper_server_ex3]: 🤖 Gripper Pick & Place Server started
[INFO] [gripper_server_ex3]: [EX3] 📋 Received goal for object: Box_A
[INFO] [gripper_server_ex3]: [EX3:#1] 🚀 Starting pick-and-place: Box_A → (3.00, 2.00) with 15.0N force
[INFO] [gripper_server_ex3]: [EX3] Approaching 0%
[INFO] [gripper_server_ex3]: [EX3] Approaching 10%
...
[INFO] [gripper_server_ex3]: [EX3] Grasping 50%
[INFO] [gripper_server_ex3]: [EX3] Grasping 100%
[INFO] [gripper_server_ex3]: [EX3] Lifting 0%
...
[INFO] [gripper_server_ex3]: [EX3] Moving 100%
[INFO] [gripper_server_ex3]: [EX3] Releasing 100%
[INFO] [gripper_server_ex3]: [EX3:#1] ✅ Pick-and-place complete! Box_A moved to (3.00, 2.00) in 8.6s
```

**Client Terminal:**
```
[INFO] [gripper_client_ex3]: 🤖 Gripper Pick & Place Client initialized
[INFO] [gripper_client_ex3]: [EX3] 📡 Waiting for gripper server...
[INFO] [gripper_client_ex3]: [EX3] 🎯 Requesting pick-and-place: Box_A → (3.00, 2.00) at 15.0N
[INFO] [gripper_client_ex3]: [EX3] ✅ Goal accepted!
[INFO] [gripper_client_ex3]: [EX3] 📊 Approaching (10%) | Force: 15.0N | ❌ No object
[INFO] [gripper_client_ex3]: [EX3] 📊 Grasping (50%) | Force: 7.5N | ❌ No object
[INFO] [gripper_client_ex3]: [EX3] 📊 Grasping (100%) | Force: 15.0N | ✅ Held
[INFO] [gripper_client_ex3]: [EX3] 📊 Lifting (50%) | Force: 15.0N | ✅ Held
[INFO] [gripper_client_ex3]: [EX3] 📊 Moving (80%) | Force: 15.0N | ✅ Held
[INFO] [gripper_client_ex3]: [EX3] 📊 Releasing (100%) | Force: 0.0N | ✅ Held
[INFO] [gripper_client_ex3]: [EX3] ✅ SUCCESS: Completed in 8.6s
```

**With Cancellation (change `cancel_after=4.0` in client):**
```
Server: [EX3] 🛑 Cancelled during Lifting
Client: [EX3] 🛑 Requesting cancellation...
Client: [EX3] ✅ Cancellation accepted
Client: [EX3] 🛑 Operation CANCELLED at Lifting
```

---

### **💡 Key Concepts**

| Concept | Description | Implementation |
|---------|-------------|----------------|
| **Multi-Stage Execution** | Sequential manipulation phases | Approach → Grasp → Lift → Move → Release |
| **Goal/Cancel Callbacks** | Pre-execution validation | `goal_callback()`, `cancel_callback()` |
| **Cancellation Handling** | Mid-execution termination | `is_cancel_requested`, `goal_handle.canceled()` |
| **Force Sensing** | Gripper force feedback | `feedback_msg.gripper_force` |
| **Object Detection** | Grasp success verification | `feedback_msg.object_detected` |
| **Failure Simulation** | Object drop detection (2% chance) | Random abort during lift/move |
| **Status Codes** | Operation outcomes | SUCCEEDED (4), CANCELED (5), ABORTED (6) |

---

### **🔍 Testing Variations**

Test different manipulation scenarios by modifying the client's `main()`:

```python
# Test 1: Normal operation (full success)
client.send_goal("Box_A", 3.0, 2.0, 15.0, cancel_after=None)

# Test 2: Cancel during lifting
client.send_goal("Box_B", 5.0, 1.0, 20.0, cancel_after=4.0)

# Test 3: Cancel during moving
client.send_goal("Box_C", 8.0, 8.0, 12.0, cancel_after=6.0)

# Test 4: Invalid grip force (should reject goal)
client.send_goal("Box_D", 3.0, 2.0, 60.0, cancel_after=None)  # Force > 50N

# Test 5: Out of reach target (should reject)
client.send_goal("Box_E", 15.0, 15.0, 15.0, cancel_after=None)  # Beyond ±10m

# Test 6: Run multiple times to trigger object drop (2% chance)
for i in range(10):
    client.send_goal(f"Box_{i}", 3.0, 2.0, 15.0, cancel_after=None)
```

---

### **✅ Exercise 3 Completion Checklist**

- [ ] Created `GripperPickPlace.action` definition
- [ ] Updated `CMakeLists.txt` and rebuilt interfaces
- [ ] Created `gripper_server_ex3.py` with multi-stage execution
- [ ] Created `gripper_client_ex3.py` with cancellation support
- [ ] Added entry points to `setup.py`
- [ ] Built packages successfully with `colcon build`
- [ ] Server implements goal_callback and cancel_callback
- [ ] Multi-stage feedback: Approaching, Grasping, Lifting, Moving, Releasing
- [ ] Object detection toggles during grasping
- [ ] Normal operation completes all 5 stages successfully
- [ ] Cancellation works during any stage
- [ ] Object drop simulation triggers abort (run multiple times)
- [ ] Invalid goals rejected (force, target position)
- [ ] Understood advanced action patterns (goal/cancel callbacks, multi-stage)

---

## **📚 Commands Reference**

### **Action Introspection**

```bash
# List all available action servers
ros2 action list

# Get detailed information about an action
ros2 action info /battery_charging_ex1
ros2 action info /navigate_to_goal_ex2
ros2 action info /gripper_pick_place_ex3

# Show action type definition
ros2 interface show ce_robot_interfaces/action/BatteryCharging
ros2 interface show ce_robot_interfaces/action/NavigateToGoal
ros2 interface show ce_robot_interfaces/action/GripperPickPlace
```

### **Sending Goals from Command Line**

```bash
# Exercise 1: Battery Charging
ros2 action send_goal /battery_charging_ex1 \
  ce_robot_interfaces/action/BatteryCharging \
  "{target_level: 80, charging_rate: 5.0}" \
  --feedback

# Exercise 2: Navigate to Goal
ros2 action send_goal /navigate_to_goal_ex2 \
  ce_robot_interfaces/action/NavigateToGoal \
  "{target_x: 5.0, target_y: 5.0, speed: 1.0}" \
  --feedback

# Exercise 3: Gripper Pick & Place
ros2 action send_goal /gripper_pick_place_ex3 \
  ce_robot_interfaces/action/GripperPickPlace \
  "{object_id: 'TestBox', target_x: 3.0, target_y: 2.0, grip_force: 15.0}" \
  --feedback
# Press Ctrl+C to cancel during execution
```

### **Debugging Commands**

```bash
# Check if action server is running
ros2 node list | grep _ex

# View node info
ros2 node info /battery_charging_server_ex1

# Monitor action topics
ros2 topic list | grep battery_charging_ex1
ros2 topic echo /_action/battery_charging_ex1/feedback

# Check action interface definition
ros2 interface proto ce_robot_interfaces/action/BatteryCharging
```

---

## **✅ Complete Lab Completion Checklist**

### **Exercise 1: Battery Charging Action** 🔋
- [ ] Created `BatteryCharging.action` definition
- [ ] Implemented autonomous charging with safety monitoring
- [ ] Multi-phase execution (Initialize, Charge, Balance)
- [ ] Temperature tracking and overheating detection
- [ ] Successfully tested normal charging (20% → 80%)
- [ ] Validated error handling (invalid parameters, overheating)
- [ ] Understood real robot power management patterns

### **Exercise 2: Navigate to Goal Action** 🗺️
- [ ] Created `NavigateToGoal.action` definition
- [ ] Implemented mobile robot navigation simulation
- [ ] Real-time ETA calculation and distance tracking
- [ ] Obstacle detection triggering abort
- [ ] Successfully navigated to target position
- [ ] Tested various speeds and target locations
- [ ] Understood autonomous navigation patterns

### **Exercise 3: Gripper Pick & Place Action** 🤖
- [ ] Created `GripperPickPlace.action` definition
- [ ] Implemented multi-stage manipulation (5 stages)
- [ ] Goal and cancel callbacks for validation
- [ ] Force sensing and object detection feedback
- [ ] Object drop simulation (failure handling)
- [ ] Cancellation works at any stage
- [ ] Understood advanced manipulation patterns

### **Overall Understanding** 🎓
- [ ] Can explain real-world robotics applications of actions
- [ ] Understand when to use Actions for robot tasks
- [ ] Can implement action servers with multi-stage execution
- [ ] Know how to handle robot-specific errors (obstacles, drops, overheating)
- [ ] Understand cancellation for safety-critical operations
- [ ] Can debug robot actions using ros2 command line tools
- [ ] Ready to implement actions in production robot systems

---

## **💡 Tips & Best Practices**

### **Design Guidelines for Robot Actions**
1. **Use Actions for physical operations** (charging, navigation, manipulation)
2. **Validate robot state** before execution (battery, position, sensors)
3. **Check cancellation for safety** (user abort, emergency stop)
4. **Provide rich feedback** (position, progress, sensor data)
5. **Handle failures gracefully** (obstacles, drops, timeouts)

### **Common Robot Action Patterns**
```python
# Pattern 1: Multi-stage robot operation
stages = ["Initialize", "Execute", "Verify", "Complete"]
for stage in stages:
    if not execute_stage(stage):
        return abort_with_state()

# Pattern 2: With sensor monitoring
while not goal_reached():
    if goal_handle.is_cancel_requested:
        safe_stop()
        return canceled()
    if sensor_detects_problem():
        return abort_with_error()
    move_towards_goal()

# Pattern 3: With safety validation
def goal_callback(goal):
    if not is_safe(goal):
        return GoalResponse.REJECT
    return GoalResponse.ACCEPT
```

### **Debugging Robot Actions**
- Use `ros2 action list` to verify servers are running
- Use `ros2 topic echo` to monitor feedback in real-time
- Add detailed logging at each stage
- Test failure scenarios (obstacles, drops, errors)
- Use visualization tools (rviz2) for spatial tasks

---

## **🎯 Next Steps**

After completing these robotics exercises:

1. **Integrate with Real Hardware:**
   - Connect to actual robot controllers
   - Use real sensors (force, proximity, encoders)
   - Interface with motor drivers
   - Add safety interlocks

2. **Advanced Robot Behaviors:**
   - Multi-robot coordination (fleet management)
   - Adaptive behaviors (learning from failures)
   - Complex manipulation (assembly tasks)
   - Autonomous mission planning

3. **Production Robotics:**
   - Add comprehensive safety checks
   - Implement recovery behaviors
   - Create robot-specific action libraries
   - Add performance monitoring and logging

4. **Integration with ROS 2 Ecosystem:**
   - Combine with nav2 for real navigation
   - Use MoveIt2 for real manipulation
   - Integrate with tf2 for coordinate transforms
   - Add rviz2 visualization for debugging

---

## **📖 Summary**

Congratulations! You've completed all three real-world robotics action exercises:

- ✅ **Exercise 1:** Battery charging with safety monitoring 🔋
- ✅ **Exercise 2:** Robot navigation with obstacle detection 🗺️
- ✅ **Exercise 3:** Gripper manipulation with multi-stage execution 🤖

**Key Takeaways:**
- Actions enable **real robotics applications** (charging, navigation, manipulation)
- **Multi-stage execution** mirrors actual robot operations
- **Safety monitoring** (temperature, obstacles, force) is critical
- **Cancellation support** enables emergency stops and user control
- **Rich feedback** provides visibility into robot state
- Actions are the foundation for **professional robot systems**

You now have practical skills to implement action-based behaviors in real robots! 🎉🤖

---
        time.sleep(3)
        if self.goal_handle:
            self.get_logger().info('[EX3] Requesting cancellation...')
            self._cancel_future = self.goal_handle.cancel_goal_async()
            self._cancel_future.add_done_callback(self.cancel_callback)

    def cancel_callback(self, future):
        """Handle cancellation response"""
        cancel_response = future.result()
        if cancel_response.return_code == 0:
            self.get_logger().info('[EX3] Cancellation accepted')
        else:
            self.get_logger().warn('[EX3] Cancellation rejected')

    def feedback_callback(self, feedback_msg):
        """Receive feedback"""
        count = feedback_msg.feedback.current_count
        self.get_logger().info(f'[EX3] Feedback: count={count}')

    def result_callback(self, future):
        """Receive result or cancellation status"""
        result = future.result()
        
        if result.status == 4:  # CANCELED
            self.get_logger().info(f'[EX3] Goal was CANCELLED')
        elif result.status == 3:  # ABORTED
            self.get_logger().info(f'[EX3] Goal was ABORTED')
        else:  # SUCCEEDED
            total = result.result.total_count
            self.get_logger().info(f'[EX3] Goal SUCCEEDED: total_count={total}')


def main(args=None):
    rclpy.init(args=args)
    client = MonitoredActionClient()
    client.send_goal(target=10, period=1)
    rclpy.spin(client)


if __name__ == '__main__':
    main()
```

### **Testing Exercise 3**

**Build and run:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces ce_robot --symlink-install
---

### **Build and Test**

#### **Step 1: Build the Package**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces ce_robot --symlink-install
source install/setup.bash
```

#### **Step 2: Run the Nodes**

**Terminal 1 - Start Action Server:**
```bash
ros2 run ce_robot 06_monitored_action_server_ex3
```

**Terminal 2 - Run Action Client:**
```bash
ros2 run ce_robot 06_monitored_action_client_ex3
```

---

### **Expected Output**

**Server Terminal (demonstrates cancellation after 3 counts):**
```
[INFO] [monitored_action_server_ex3]: Monitored Action Server (Exercise 3) started
[INFO] [monitored_action_server_ex3]: [EX3:#1] START: count to 10 with 1s period
[INFO] [monitored_action_server_ex3]: [EX3:#1] Count: 1/10
[INFO] [monitored_action_server_ex3]: [EX3:#1] Count: 2/10
[INFO] [monitored_action_server_ex3]: [EX3:#1] Count: 3/10
[INFO] [monitored_action_server_ex3]: [EX3:#1] CANCELLED at count 4
```

**Client Terminal:**
```
[INFO] [monitored_action_client_ex3]: Monitored Action Client (Exercise 3) initialized
[INFO] [monitored_action_client_ex3]: [EX3] Waiting for server...
[INFO] [monitored_action_client_ex3]: [EX3] Sending goal: target=10, period=1s
[INFO] [monitored_action_client_ex3]: [EX3] Goal accepted!
[INFO] [monitored_action_client_ex3]: [EX3] Feedback: count=1
[INFO] [monitored_action_client_ex3]: [EX3] Feedback: count=2
[INFO] [monitored_action_client_ex3]: [EX3] Feedback: count=3
[INFO] [monitored_action_client_ex3]: [EX3] Requesting cancellation...
[INFO] [monitored_action_client_ex3]: [EX3] Cancellation accepted
[INFO] [monitored_action_client_ex3]: [EX3] Goal was CANCELLED
```

---

### **💡 Key Concepts**

| Concept | Description | Implementation |
|---------|-------------|----------------|
| **Cancellation Check** | Monitor for cancel requests | `goal_handle.is_cancel_requested` |
| **Cancel Handling** | Mark goal as canceled | `goal_handle.canceled()` |
| **Goal Status Codes** | Track goal states | SUCCEEDED(3), CANCELED(4), ABORTED(5) |
| **Partial Results** | Return progress on cancel | `total_count = current_count` |
| **Async Cancellation** | Client-side cancel request | `goal_handle.cancel_goal_async()` |
| **Threading** | Background cancellation testing | `threading.Thread(target=...)` |

---

### **🔍 Testing Variations**

Modify the client to test different scenarios:

```python
# Test 1: No cancellation (let it complete)
# Comment out the cancellation threading code
client.send_goal(target=5, period=1)

---
    if goal_handle.is_cancel_requested:
        goal_handle.canceled()
        return partial_result
    # ... execution code

# Pattern 3: With error handling
try:
    # ... execution code
    goal_handle.succeed()
except Exception as e:
    self.get_logger().error(f'Error: {e}')
    goal_handle.abort()
```

### **Debugging Tips**
- Use `ros2 action list` to verify server is running
- Use `ros2 action info` to check server details
- Use `--feedback` flag to monitor progress
- Check logs for error messages
- Test cancellation with Ctrl+C on command line goals

---

## **🎯 Next Steps**

After completing these exercises:

1. **Explore Advanced Features:**
   - Multi-threaded executors
   - Goal queuing and prioritization
   - Multiple concurrent goals
   - Goal preemption

2. **Real Robotics Applications:**
   - Navigation (move to goal)
   - Manipulation (pick and place)
   - Charging sequences
   - Sensor calibration

3. **Integration:**
   - Combine with parameters for configuration
   - Use launch files for complex systems
   - Integrate with tf2 for coordinate transforms
   - Add visualization with rviz2

4. **Production Readiness:**
   - Add comprehensive error handling
   - Implement state recovery
   - Add timeout mechanisms
   - Create unit tests

---

## **📖 Summary**

Congratulations! You've completed all three action exercises:

- ✅ **Exercise 1:** Built basic action server/client with feedback
- ✅ **Exercise 2:** Implemented validation and error handling
- ✅ **Exercise 3:** Added cancellation and state management

**Key Takeaways:**
- Actions enable **non-blocking asynchronous execution**
- **Goal-Feedback-Result** pattern provides progress monitoring
- **Cancellation support** allows graceful interruption
- **Validation** ensures robust error handling
- Actions are essential for **long-running robotics tasks**

You now have the skills to implement professional action-based systems in ROS 2! 🎉

---
