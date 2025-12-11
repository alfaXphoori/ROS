# **🚀 ROS 2 Actions Lab Exercises**

## **📌 Project Title**

Hands-On Lab: Master ROS 2 Actions for Long-Running Tasks with Feedback

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

## **🛠 Overview**

This comprehensive lab demonstrates **action-based asynchronous task execution** in ROS 2:
- **Action Definitions** - Create custom action interfaces with goal, result, and feedback
- **Action Servers** - Implement long-running tasks with periodic feedback publishing
- **Action Clients** - Send goals and monitor execution progress in real-time
- **Cancellation Handling** - Support mid-execution goal cancellation
- **Error Management** - Validate inputs and handle execution failures gracefully

**Duration:** ~3 hours  
**Level:** Intermediate to Advanced  
**Prerequisites:** ROS 2 Jazzy installed, Parameters lab completed, understanding of callbacks

---

## **📊 Architecture**

```
┌─────────────────────────────────────────────┐
│ Exercise 1: Basic Count Until Action        │
│ (Simple goal, feedback, result)             │
│ • Count from 1 to target with progress      │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 2: Distance Calculator with Validation │
│ (Parameter validation, goal rejection)      │
│ • Input validation & error handling         │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 3: Monitored Actions with Cancellation │
│ (Goal status tracking, cancellation)        │
│ • Real-time progress monitoring & cancel    │
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
        # Exercise 1: Basic Count Until
        '06_count_until_server_ex1 = ce_robot.count_until_server_ex1:main',
        '06_count_until_client_ex1 = ce_robot.count_until_client_ex1:main',
        
        # Exercise 2: Distance Calculator
        '06_distance_calc_server_ex2 = ce_robot.distance_calc_server_ex2:main',
        '06_distance_calc_client_ex2 = ce_robot.distance_calc_client_ex2:main',
        
        # Exercise 3: Monitored Actions
        '06_monitored_action_server_ex3 = ce_robot.monitored_action_server_ex3:main',
        '06_monitored_action_client_ex3 = ce_robot.monitored_action_client_ex3:main',
    ],
},
```

---

## **📚 Learning Path Overview**

| Exercise | Title | Level | Duration |
|----------|-------|-------|----------|
| 1 | Basic Count Until Action | Beginner-Intermediate | 45 min |
| 2 | Distance Calculator with Validation | Intermediate | 50 min |
| 3 | Monitored Actions with Cancellation | Advanced | 55 min |

---

## **Exercise 1: Basic Count Until Action (Beginner-Intermediate) 🔢**

### **📋 Task**

Create a basic action that counts from 1 to a target number, publishing feedback at each step and returning the final count. This exercise introduces action fundamentals: goal sending, feedback receiving, and result handling.

### **📁 File Location**

Navigate to your ROS 2 workspace and create the Python file:

```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
touch count_until_server_ex1.py
chmod +x count_until_server_ex1.py
```

**Directory Structure:**
```
📁 ros2_ws/
└── 📁 src/
    └── 📁 ce_robot/
        └── 📁 ce_robot/
            ├── 📄 __init__.py
            └── 🐍 count_until_server_ex1.py    ← Create this file
```

### **Create Server File**

#### **File: count_until_server_ex1.py**

```python
#!/usr/bin/env python3
"""
Exercise 1: Basic Count Until Server
Counts from 1 to target with simple feedback
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ce_robot_interfaces.action import CountUntil
import time


class CountUntilActionServer(Node):
    def __init__(self):
        super().__init__('count_until_server_ex1')
        
        self._action_server = ActionServer(
            self,
            CountUntil,
            'count_until_ex1',
            self.execute_callback
        )
        
        self.get_logger().info('Count Until Action Server (Exercise 1) started')

    def execute_callback(self, goal_handle):
        """Execute the counting task"""
        target = goal_handle.request.target
        period = goal_handle.request.period
        
        self.get_logger().info(
            f'[EX1] Executing: count to {target} with {period}s period'
        )
        
        feedback_msg = CountUntil.Feedback()
        
        # Count from 1 to target
        for count in range(1, target + 1):
            # Publish feedback
            feedback_msg.current_count = count
            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().info(f'[EX1] Count: {count}/{target}')
            time.sleep(period)
        
        # Succeed and return result
        goal_handle.succeed()
        result = CountUntil.Result()
        result.total_count = target
        
        self.get_logger().info(f'[EX1] Goal succeeded! Total: {target}')
        return result


def main(args=None):
    rclpy.init(args=args)
    server = CountUntilActionServer()
    rclpy.spin(server)


if __name__ == '__main__':
    main()
```

### **Create Client File**

#### **File: count_until_client_ex1.py**

```python
#!/usr/bin/env python3
"""
Exercise 1: Basic Count Until Client
Sends goal and receives feedback and result
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ce_robot_interfaces.action import CountUntil


class CountUntilActionClient(Node):
    def __init__(self):
        super().__init__('count_until_client_ex1')
        self._action_client = ActionClient(
            self,
            CountUntil,
            'count_until_ex1'
        )
        self.get_logger().info('Count Until Action Client (Exercise 1) initialized')

    def send_goal(self, target, period):
        """Send goal to server"""
        self.get_logger().info(f'[EX1] Waiting for server...')
        self._action_client.wait_for_server()
        
        goal_msg = CountUntil.Goal()
        goal_msg.target = target
        goal_msg.period = period
        
        self.get_logger().info(f'[EX1] Sending goal: target={target}, period={period}s')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('[EX1] Goal rejected!')
            return
        
        self.get_logger().info('[EX1] Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Receive feedback"""
        count = feedback_msg.feedback.current_count
        self.get_logger().info(f'[EX1] Feedback: count={count}')

    def result_callback(self, future):
        """Receive result"""
        result = future.result().result
        self.get_logger().info(f'[EX1] Result: total_count={result.total_count}')


def main(args=None):
    rclpy.init(args=args)
    client = CountUntilActionClient()
    client.send_goal(target=5, period=1)
    rclpy.spin(client)


if __name__ == '__main__':
    main()
```

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
ros2 run ce_robot 06_count_until_server_ex1
```

**Terminal 2 - Run Action Client:**
```bash
ros2 run ce_robot 06_count_until_client_ex1
```

---

### **Expected Output**

**Server Terminal:**
```
[INFO] [count_until_server_ex1]: Count Until Action Server (Exercise 1) started
[INFO] [count_until_server_ex1]: [EX1] Executing: count to 5 with 1s period
[INFO] [count_until_server_ex1]: [EX1] Count: 1/5
[INFO] [count_until_server_ex1]: [EX1] Count: 2/5
[INFO] [count_until_server_ex1]: [EX1] Count: 3/5
[INFO] [count_until_server_ex1]: [EX1] Count: 4/5
[INFO] [count_until_server_ex1]: [EX1] Count: 5/5
[INFO] [count_until_server_ex1]: [EX1] Goal succeeded! Total: 5
```

**Client Terminal:**
```
[INFO] [count_until_client_ex1]: Count Until Action Client (Exercise 1) initialized
[INFO] [count_until_client_ex1]: [EX1] Waiting for server...
[INFO] [count_until_client_ex1]: [EX1] Sending goal: target=5, period=1s
[INFO] [count_until_client_ex1]: [EX1] Goal accepted!
[INFO] [count_until_client_ex1]: [EX1] Feedback: count=1
[INFO] [count_until_client_ex1]: [EX1] Feedback: count=2
[INFO] [count_until_client_ex1]: [EX1] Feedback: count=3
[INFO] [count_until_client_ex1]: [EX1] Feedback: count=4
[INFO] [count_until_client_ex1]: [EX1] Feedback: count=5
[INFO] [count_until_client_ex1]: [EX1] Result: total_count=5
```

---

### **💡 Key Concepts**

| Concept | Description | Implementation |
|---------|-------------|----------------|
| **Action Server** | Executes long-running tasks | `ActionServer(self, CountUntil, 'count_until_ex1', callback)` |
| **Execute Callback** | Main task execution function | `execute_callback(self, goal_handle)` |
| **Feedback Publishing** | Send progress updates | `goal_handle.publish_feedback(feedback_msg)` |
| **Goal Completion** | Mark successful completion | `goal_handle.succeed()` |
| **Result Return** | Send final result | `return CountUntil.Result(total_count=target)` |

---

### **🔍 Testing Variations**

Try modifying the client to test different scenarios:

```python
# Test 1: Fast counting
client.send_goal(target=10, period=0.5)

# Test 2: Slow counting
client.send_goal(target=3, period=2)

# Test 3: Large target
client.send_goal(target=20, period=0.2)
```

---

### **✅ Exercise 1 Completion Checklist**

- [ ] Created `count_until_server_ex1.py` with ActionServer
- [ ] Created `count_until_client_ex1.py` with ActionClient
- [ ] Added entry points to `setup.py`
- [ ] Built package successfully with `colcon build`
- [ ] Server starts and logs initialization message
- [ ] Client connects and sends goal
- [ ] Feedback messages received at each count
- [ ] Final result received: `total_count=5`
- [ ] Tested with different target and period values
- [ ] Understood goal → feedback → result lifecycle

---

## **Exercise 2: Distance Calculator with Validation (Intermediate) ⚠️**

### **📋 Task**

Create an action that calculates the distance between two points with input validation, demonstrating goal rejection, error handling, and progressive feedback with percentage completion.

### **Create Custom Action Definition**

First, create the `DistanceCalc.action` file:

```bash
cd ~/ros2_ws/src/ce_robot_interfaces/action
touch DistanceCalc.action
```

#### **File: DistanceCalc.action**

```
# Calculate distance between two points
float32 x1
float32 y1
float32 x2
float32 y2

---

# Result: calculated distance
float32 distance

---

# Feedback: calculation status
string status
float32 progress_percent
```

**Update `CMakeLists.txt`** in `ce_robot_interfaces`:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "msg/RobotTag.msg"
  "srv/CalRectangle.srv"
  "action/CountUntil.action"
  "action/DistanceCalc.action"    # Add this line
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

### **Create Server File**

#### **File: distance_calc_server_ex2.py**

```python
#!/usr/bin/env python3
"""
Exercise 2: Distance Calculator Server with Validation
Calculates distance with input validation
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ce_robot_interfaces.action import DistanceCalc
import math
import time


class DistanceCalcActionServer(Node):
    def __init__(self):
        super().__init__('distance_calc_server_ex2')
        
        self._action_server = ActionServer(
            self,
            DistanceCalc,
            'distance_calc_ex2',
            self.execute_callback
        )
        
        self.get_logger().info('Distance Calculator Server (Exercise 2) started')

    def validate_goal(self, goal):
        """Validate goal parameters"""
        errors = []
        
        # Check for valid numbers (not NaN or infinity)
        for val, name in [
            (goal.x1, 'x1'), (goal.y1, 'y1'),
            (goal.x2, 'x2'), (goal.y2, 'y2')
        ]:
            if math.isnan(val) or math.isinf(val):
                errors.append(f'{name} is invalid (NaN or infinity)')
        
        # Check if points are too far (> 1000)
        if abs(goal.x1) > 1000 or abs(goal.y1) > 1000:
            errors.append('Point 1 coordinates too large (|x| or |y| > 1000)')
        if abs(goal.x2) > 1000 or abs(goal.y2) > 1000:
            errors.append('Point 2 coordinates too large (|x| or |y| > 1000)')
        
        return errors

    def execute_callback(self, goal_handle):
        """Execute distance calculation with validation"""
        goal = goal_handle.request
        
        self.get_logger().info(
            f'[EX2] Calculating distance from ({goal.x1}, {goal.y1}) to ({goal.x2}, {goal.y2})'
        )
        
        # Validate goal
        errors = self.validate_goal(goal)
        if errors:
            self.get_logger().error('[EX2] Goal validation failed:')
            for error in errors:
                self.get_logger().error(f'  - {error}')
            goal_handle.abort()
            return DistanceCalc.Result(distance=-1.0)
        
        feedback_msg = DistanceCalc.Feedback()
        
        try:
            # Simulate calculation steps
            for step in range(1, 4):
                feedback_msg.status = f'Calculating... (step {step}/3)'
                feedback_msg.progress_percent = (step / 3) * 100
                goal_handle.publish_feedback(feedback_msg)
                
                self.get_logger().info(f'[EX2] {feedback_msg.status} ({feedback_msg.progress_percent:.0f}%)')
                time.sleep(0.5)
            
            # Calculate distance
            dx = goal.x2 - goal.x1
            dy = goal.y2 - goal.y1
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Final feedback
            feedback_msg.status = 'Complete'
            feedback_msg.progress_percent = 100.0
            goal_handle.publish_feedback(feedback_msg)
            
            # Return result
            goal_handle.succeed()
            result = DistanceCalc.Result()
            result.distance = distance
            
            self.get_logger().info(f'[EX2] Goal succeeded! Distance: {distance:.2f}')
            return result
            
        except Exception as e:
            self.get_logger().error(f'[EX2] Execution failed: {str(e)}')
            goal_handle.abort()
            return DistanceCalc.Result(distance=-1.0)


def main(args=None):
    rclpy.init(args=args)
    server = DistanceCalcActionServer()
    rclpy.spin(server)


if __name__ == '__main__':
    main()
```

### **Client File: distance_calc_client_ex2.py**

```python
#!/usr/bin/env python3
"""
Exercise 2: Distance Calculator Client
Tests validation and error handling
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ce_robot_interfaces.action import DistanceCalc


class DistanceCalcActionClient(Node):
    def __init__(self):
        super().__init__('distance_calc_client_ex2')
        self._action_client = ActionClient(
            self,
            DistanceCalc,
            'distance_calc_ex2'
        )
        self.get_logger().info('Distance Calculator Client (Exercise 2) initialized')

    def send_goal(self, x1, y1, x2, y2):
        """Send goal with validation test"""
        self.get_logger().info(f'[EX2] Waiting for server...')
        self._action_client.wait_for_server()
        
        goal_msg = DistanceCalc.Goal()
        goal_msg.x1 = x1
        goal_msg.y1 = y1
        goal_msg.x2 = x2
        goal_msg.y2 = y2
        
        self.get_logger().info(
            f'[EX2] Sending goal: ({x1}, {y1}) to ({x2}, {y2})'
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
            self.get_logger().error('[EX2] Goal rejected by server!')
            return
        
        self.get_logger().info('[EX2] Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Receive feedback"""
        fb = feedback_msg.feedback
        self.get_logger().info(f'[EX2] {fb.status} ({fb.progress_percent:.0f}%)')

    def result_callback(self, future):
        """Receive result or error"""
        result = future.result().result
        if result.distance < 0:
            self.get_logger().error('[EX2] Goal failed or aborted')
        else:
            self.get_logger().info(f'[EX2] Distance: {result.distance:.2f} units')


def main(args=None):
    rclpy.init(args=args)
    client = DistanceCalcActionClient()
    
    # Test 1: Valid goal
    client.send_goal(0.0, 0.0, 3.0, 4.0)
    rclpy.spin(client)


if __name__ == '__main__':
    main()
```

### **Testing Exercise 2**

**Build and run:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces ce_robot --symlink-install
source install/setup.bash

# Terminal 1
ros2 run ce_robot distance_calc_server_ex2

# Terminal 2
ros2 run ce_robot distance_calc_client_ex2
```

**Expected Output:**
```
Server: [EX2] Calculating distance from (0.0, 0.0) to (3.0, 4.0)
Server: [EX2] Calculating... (step 1/3) (33%)
Client: [EX2] Calculating... (step 1/3) (33%)
Server: [EX2] Calculating... (step 2/3) (67%)
Client: [EX2] Calculating... (step 2/3) (67%)
Server: [EX2] Calculating... (step 3/3) (100%)
Client: [EX2] Calculating... (step 3/3) (100%)
Server: [EX2] Goal succeeded! Distance: 5.00
Client: [EX2] Distance: 5.00 units
```

---

### **💡 Key Concepts**

| Concept | Description | Implementation |
|---------|-------------|----------------|
| **Input Validation** | Check parameters before execution | `validate_goal()` function |
| **Goal Rejection** | Abort invalid requests | `goal_handle.abort()` |
| **Error Handling** | Try-except for exceptions | `try: ... except Exception as e:` |
| **Progress Feedback** | Multi-step progress updates | `feedback_msg.progress_percent` |
| **Result with Error** | Negative value indicates error | `DistanceCalc.Result(distance=-1.0)` |
| **Action States** | State transitions | PENDING → ACTIVE → SUCCEEDED/ABORTED |

---

### **🔍 Testing Variations**

Test different scenarios by modifying the client's `main()`:

```python
# Test 1: Valid distance (3-4-5 triangle)
client.send_goal(0.0, 0.0, 3.0, 4.0)  # Expected: 5.0

# Test 2: Invalid coordinates (too large)
client.send_goal(0.0, 0.0, 2000.0, 0.0)  # Expected: Goal rejected

# Test 3: Same point (distance = 0)
client.send_goal(10.0, 10.0, 10.0, 10.0)  # Expected: 0.0

# Test 4: Diagonal distance
client.send_goal(-5.0, -5.0, 5.0, 5.0)  # Expected: 14.14
```

---

### **✅ Exercise 2 Completion Checklist**

- [ ] Created `DistanceCalc.action` definition
- [ ] Updated `CMakeLists.txt` to include new action
- [ ] Rebuilt `ce_robot_interfaces` package
- [ ] Created `distance_calc_server_ex2.py` with validation
- [ ] Created `distance_calc_client_ex2.py`
- [ ] Added entry points to `setup.py`
- [ ] Built package successfully with `colcon build`
- [ ] Server validates input and logs validation results
- [ ] Client receives progressive feedback (33%, 67%, 100%)
- [ ] Valid goal (3-4-5 triangle) returns distance: 5.00
- [ ] Invalid goal (large coordinates) triggers abort
- [ ] Tested error handling with invalid inputs
- [ ] Understood `goal_handle.abort()` for errors

---

## **Exercise 3: Battery Charging Simulator with Cancellation (Advanced) 🚀**

### **📋 Task**

Implement a battery charging simulator that demonstrates cancellation handling, state management, multi-threaded execution, and graceful shutdown support. This exercise covers advanced action patterns used in real robotics applications.

---

### **Create Server File**

#### **File: monitored_action_server_ex3.py**

```python
#!/usr/bin/env python3
"""
Exercise 3: Monitored Action Server with Cancellation
Supports goal cancellation and state tracking
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ce_robot_interfaces.action import CountUntil
import time


class MonitoredActionServer(Node):
    def __init__(self):
        super().__init__('monitored_action_server_ex3')
        
        self._action_server = ActionServer(
            self,
            CountUntil,
            'monitored_action_ex3',
            self.execute_callback
        )
        
        self.goal_counter = 0
        self.get_logger().info('Monitored Action Server (Exercise 3) started')

    def execute_callback(self, goal_handle):
        """Execute with cancellation support"""
        self.goal_counter += 1
        goal_id = self.goal_counter
        
        target = goal_handle.request.target
        period = goal_handle.request.period
        
        self.get_logger().info(
            f'[EX3:#{goal_id}] START: count to {target} with {period}s period'
        )
        
        feedback_msg = CountUntil.Feedback()
        cancelled = False
        
        try:
            # Count from 1 to target
            for count in range(1, target + 1):
                # Check for cancellation request
                if goal_handle.is_cancel_requested:
                    self.get_logger().info(f'[EX3:#{goal_id}] CANCELLED at count {count}')
                    goal_handle.canceled()
                    cancelled = True
                    break
                
                # Publish feedback
                feedback_msg.current_count = count
                goal_handle.publish_feedback(feedback_msg)
                
                self.get_logger().info(f'[EX3:#{goal_id}] Count: {count}/{target}')
                time.sleep(period)
            
            if not cancelled:
                # Normal completion
                goal_handle.succeed()
                result = CountUntil.Result()
                result.total_count = target
                self.get_logger().info(f'[EX3:#{goal_id}] SUCCEEDED: completed {target} counts')
                return result
            else:
                # Return partial result for cancelled goal
                result = CountUntil.Result()
                result.total_count = feedback_msg.current_count
                return result
                
        except Exception as e:
            self.get_logger().error(f'[EX3:#{goal_id}] ABORTED: {str(e)}')
            goal_handle.abort()
            return CountUntil.Result(total_count=-1)


def main(args=None):
    rclpy.init(args=args)
    server = MonitoredActionServer()
    rclpy.spin(server)


if __name__ == '__main__':
    main()
```

---

### **Create Client File**

#### **File: monitored_action_client_ex3.py**

```python
#!/usr/bin/env python3
"""
Exercise 3: Monitored Action Client with Cancellation
Tests cancellation handling and goal tracking
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ce_robot_interfaces.action import CountUntil
import threading
import time


class MonitoredActionClient(Node):
    def __init__(self):
        super().__init__('monitored_action_client_ex3')
        self._action_client = ActionClient(
            self,
            CountUntil,
            'monitored_action_ex3'
        )
        self.goal_handle = None
        self.get_logger().info('Monitored Action Client (Exercise 3) initialized')

    def send_goal(self, target, period):
        """Send goal to server"""
        self.get_logger().info(f'[EX3] Waiting for server...')
        self._action_client.wait_for_server()
        
        goal_msg = CountUntil.Goal()
        goal_msg.target = target
        goal_msg.period = period
        
        self.get_logger().info(f'[EX3] Sending goal: target={target}, period={period}s')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error('[EX3] Goal rejected!')
            return
        
        self.get_logger().info('[EX3] Goal accepted!')
        
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)
        
        # Schedule cancellation after 3 seconds (for testing)
        threading.Thread(target=self._cancel_after_delay, daemon=True).start()

    def _cancel_after_delay(self):
        """Cancel goal after delay for testing"""
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

# Test 2: Immediate cancellation
def _cancel_after_delay(self):
    time.sleep(0.5)  # Cancel after 0.5 seconds
    ...

# Test 3: Late cancellation (after completion)
def _cancel_after_delay(self):
    time.sleep(15)  # Try to cancel after completion
    ...
```

**Testing Goal Status with ros2 command line:**
```bash
# Send goal with feedback display
ros2 action send_goal /monitored_action_ex3 \
  ce_robot_interfaces/action/CountUntil \
  "{target: 5, period: 1}" \
  --feedback

# Cancel during execution (Ctrl+C in the terminal)
```

---

### **✅ Exercise 3 Completion Checklist**

- [ ] Created `monitored_action_server_ex3.py` with cancellation support
- [ ] Created `monitored_action_client_ex3.py` with cancel request
- [ ] Added entry points to `setup.py`
- [ ] Built package successfully with `colcon build`
- [ ] Server checks `is_cancel_requested` during execution
- [ ] Client sends cancellation request after 3 seconds
- [ ] Server logs "CANCELLED at count X" message
- [ ] Client receives cancellation confirmation
- [ ] Partial result returned with current count
- [ ] Tested goal status codes (SUCCEEDED, CANCELED)
- [ ] Understood cleanup on cancellation
- [ ] Tested variations (no cancel, late cancel)

---

## **📚 Commands Reference**

### **Action Introspection**

```bash
# List all available action servers
ros2 action list

# Get detailed information about an action
ros2 action info /count_until_ex1
ros2 action info /distance_calc_ex2
ros2 action info /monitored_action_ex3

# Show action type definition
ros2 interface show ce_robot_interfaces/action/CountUntil
ros2 interface show ce_robot_interfaces/action/DistanceCalc
```

### **Sending Goals from Command Line**

```bash
# Exercise 1: Count Until (basic)
ros2 action send_goal /count_until_ex1 \
  ce_robot_interfaces/action/CountUntil \
  "{target: 5, period: 1}" \
  --feedback

# Exercise 2: Distance Calculator
ros2 action send_goal /distance_calc_ex2 \
  ce_robot_interfaces/action/DistanceCalc \
  "{x1: 0.0, y1: 0.0, x2: 3.0, y2: 4.0}" \
  --feedback

# Exercise 3: Monitored Action
ros2 action send_goal /monitored_action_ex3 \
  ce_robot_interfaces/action/CountUntil \
  "{target: 10, period: 1}" \
  --feedback
# Press Ctrl+C to cancel during execution
```

### **Debugging Commands**

```bash
# Check if action server is running
ros2 node list | grep server_ex

# View node info
ros2 node info /count_until_server_ex1

# Monitor action topics
ros2 topic list | grep count_until_ex1
ros2 topic echo /_action/count_until_ex1/feedback

# Check action interface definition
ros2 interface proto ce_robot_interfaces/action/CountUntil
```

---

## **✅ Complete Lab Completion Checklist**

### **Exercise 1: Basic Actions** ✅
- [ ] Created `count_until_server_ex1.py` and `count_until_client_ex1.py`
- [ ] Server implements `execute_callback()` with feedback publishing
- [ ] Client implements feedback and result callbacks
- [ ] Successfully tested goal → feedback → result flow
- [ ] Understood action lifecycle fundamentals

### **Exercise 2: Validation & Error Handling** ⚠️
- [ ] Created `DistanceCalc.action` custom action definition
- [ ] Updated `CMakeLists.txt` and rebuilt interfaces
- [ ] Created `distance_calc_server_ex2.py` with `validate_goal()`
- [ ] Created `distance_calc_client_ex2.py`
- [ ] Tested valid goal execution (3-4-5 triangle → 5.0)
- [ ] Tested invalid goal rejection (large coordinates)
- [ ] Understood `goal_handle.abort()` for error handling
- [ ] Implemented progressive feedback with percentage

### **Exercise 3: Cancellation & Advanced Patterns** 🚀
- [ ] Created `monitored_action_server_ex3.py` with cancellation check
- [ ] Created `monitored_action_client_ex3.py` with cancel request
- [ ] Server checks `is_cancel_requested` in execution loop
- [ ] Client sends cancellation after 3 seconds
- [ ] Server responds with `goal_handle.canceled()`
- [ ] Partial results returned on cancellation
- [ ] Tested goal status codes (SUCCEEDED, CANCELED, ABORTED)
- [ ] Understood threading for async operations

### **Overall Understanding** 🎓
- [ ] Can explain Goal-Feedback-Result pattern
- [ ] Understand when to use Actions vs Services
- [ ] Can implement action servers and clients
- [ ] Know how to handle errors and validation
- [ ] Understand cancellation and cleanup
- [ ] Can debug actions using ros2 command line tools
- [ ] Ready to implement actions in real robotics applications

---

## **💡 Tips & Best Practices**

### **Design Guidelines**
1. **Use Actions for long-running tasks** (> 1 second)
2. **Validate goals early** before starting execution
3. **Check cancellation frequently** in execution loops
4. **Provide meaningful feedback** with progress information
5. **Handle errors gracefully** with proper cleanup

### **Common Patterns**
```python
# Pattern 1: Basic execution with feedback
for i in range(target):
    feedback.value = i
    goal_handle.publish_feedback(feedback)
    time.sleep(period)
goal_handle.succeed()

# Pattern 2: With cancellation check
for i in range(target):
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
