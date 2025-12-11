# **🚀 ROS 2 Actions Lab Exercises**

Master long-running asynchronous tasks with feedback using ROS 2 Actions through progressive hands-on exercises.

---

## **📌 Project Title**

Create and Use ROS 2 Actions for Long-Running Tasks with Feedback

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🛠 Lab Overview**

This lab provides hands-on exercises to master action implementation, feedback handling, and cancellation patterns in ROS 2. Each exercise builds upon the previous one, progressing from basic action patterns through advanced goal management and state handling.

**Duration:** ~3 hours
**Level:** Intermediate to Advanced
**Prerequisites:** ROS 2 Jazzy installed, Parameters lab completed, basic knowledge of callbacks

---

## **🎯 Learning Objectives**

By completing this lab, you will be able to:

- ✅ Create custom action definition files (.action)
- ✅ Implement action servers with execute callbacks
- ✅ Create action clients with goal sending
- ✅ Handle feedback during execution
- ✅ Implement cancellation handling
- ✅ Return results and manage goal states
- ✅ Monitor action status and progress
- ✅ Debug actions with ROS 2 tools
- ✅ Handle multiple concurrent goals
- ✅ Implement best practices for action design

---

## **📊 Lab Architecture**

```
┌────────────────────────────────────────────────────────────┐
│ Exercise 1: Basic Action (Count Until)                     │
│ (Simple goal, feedback, result)                            │
└──────────────────────┬─────────────────────────────────────┘
                       │
                       ▼
┌────────────────────────────────────────────────────────────┐
│ Exercise 2: Action with Validation & Error Handling        │
│ (Parameter validation, goal rejection, error states)       │
└──────────────────────┬─────────────────────────────────────┘
                       │
                       ▼
┌────────────────────────────────────────────────────────────┐
│ Exercise 3: Monitored Actions with Cancellation            │
│ (Goal status tracking, cancellation, progress monitoring)  │
└────────────────────────────────────────────────────────────┘
```

---

## **📚 Learning Path Overview**

| Exercise | Title | Level | Duration | Focus |
|----------|-------|-------|----------|-------|
| 1 | Basic Count Until | Beginner-Intermediate | 45 min | Fundamentals |
| 2 | Distance Calculator with Validation | Intermediate | 50 min | Error handling |
| 3 | Monitored Actions | Advanced | 55 min | Cancellation & state |

---

## **Exercise 1: Basic Count Until Action (Beginner-Intermediate) 🔢**

### **📋 Task**

Create a basic action that counts from 1 to a target number, publishing feedback at each step and returning the final count.

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

### **Files to Create**

**Server File: count_until_server_ex1.py**

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

**Client File: count_until_client_ex1.py**

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

### **Testing Exercise 1**

**Build:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces ce_robot --symlink-install
source install/setup.bash
```

**Terminal 1 - Run Server:**
```bash
ros2 run ce_robot count_until_server_ex1
```

**Terminal 2 - Run Client:**
```bash
ros2 run ce_robot count_until_client_ex1
```

**Expected Output (Server):**
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

**Expected Output (Client):**
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

### **Key Concepts**

- Basic action server with `execute_callback()`
- Goal sending with `send_goal_async()`
- Feedback publishing with `publish_feedback()`
- Result handling with callbacks
- Action lifecycle: goal → feedback → result

---

## **Exercise 2: Action with Validation & Error Handling (Intermediate) ⚠️**

### **📋 Task**

Create an action that validates input parameters and demonstrates goal rejection, error handling, and state management.

### **Custom Action: DistanceCalc.action**

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

### **Server File: distance_calc_server_ex2.py**

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

### **Key Concepts**

- Parameter validation before execution
- Goal rejection with `goal_handle.abort()`
- Progress feedback with percentage
- Error handling in callbacks
- Action states: PENDING → ACTIVE → SUCCEEDED/ABORTED

---

## **Exercise 3: Monitored Actions with Cancellation (Advanced) 🛑**

### **📋 Task**

Create an action that can be cancelled during execution, with goal status monitoring and proper cleanup.

### **Server File: monitored_action_server_ex3.py**

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

### **Client File: monitored_action_client_ex3.py**

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
source install/setup.bash

# Terminal 1
ros2 run ce_robot monitored_action_server_ex3

# Terminal 2
ros2 run ce_robot monitored_action_client_ex3
```

**Expected Output (demonstrates cancellation):**
```
Server: [EX3:#1] START: count to 10 with 1s period
Server: [EX3:#1] Count: 1/10
Client: [EX3] Feedback: count=1
Server: [EX3:#1] Count: 2/10
Client: [EX3] Feedback: count=2
Server: [EX3:#1] Count: 3/10
Client: [EX3] Feedback: count=3
Client: [EX3] Requesting cancellation...
Server: [EX3:#1] CANCELLED at count 4
Client: [EX3] Cancellation accepted
Client: [EX3] Goal was CANCELLED
```

### **Key Concepts**

- Checking for cancellation: `goal_handle.is_cancel_requested`
- Handling cancellation: `goal_handle.canceled()`
- Goal status codes (SUCCEEDED, ABORTED, CANCELED)
- Proper cleanup on cancellation
- Threading for asynchronous cancellation testing

---

## **Commands Reference**

```bash
# List available action servers
ros2 action list

# Get action information
ros2 action info /action_name

# Send goal from command line
ros2 action send_goal /action_name \
  ce_robot_interfaces/action/CountUntil \
  "{target: 5, period: 1}" \
  --feedback

# Send goal and wait for result
ros2 action send_goal /action_name \
  ce_robot_interfaces/action/CountUntil \
  "{target: 10, period: 1}"
```

---

## **✅ Completion Checklist**

- [ ] Exercise 1: Basic Count Until completed
  - [ ] Server and client created
  - [ ] Feedback successfully transmitted
  - [ ] Result received correctly
  - [ ] Multiple goals can be sent

- [ ] Exercise 2: Distance Calculator with Validation completed
  - [ ] Input validation working
  - [ ] Valid goals executed
  - [ ] Invalid goals rejected/aborted
  - [ ] Progress feedback displayed
  - [ ] Error handling functional

- [ ] Exercise 3: Monitored Actions completed
  - [ ] Server accepts and executes goals
  - [ ] Client can request cancellation
  - [ ] Server responds to cancellation request
  - [ ] Goal status properly tracked
  - [ ] Partial results returned for cancelled goals

- [ ] All packages build successfully
- [ ] All servers run without errors
- [ ] All clients connect and send goals
- [ ] Feedback properly transmitted
- [ ] Results received correctly
- [ ] Cancellation works properly

---

## **💡 Tips & Tricks**

1. **Always check for cancellation in loops:**
   ```python
   if goal_handle.is_cancel_requested:
       goal_handle.canceled()
       return result
   ```

2. **Publish feedback regularly:**
   ```python
   feedback_msg = ActionName.Feedback()
   feedback_msg.progress = 50
   goal_handle.publish_feedback(feedback_msg)
   ```

3. **Handle three outcome states:**
   ```python
   goal_handle.succeed()      # Normal completion
   goal_handle.abort()        # Error occurred
   goal_handle.canceled()     # Cancelled by client
   ```

4. **Use try-except for robustness:**
   ```python
   try:
       # Execute action
   except Exception as e:
       goal_handle.abort()
       return error_result
   ```

5. **Debug with logging:**
   ```bash
   ros2 run package_name node_name --ros-args --log-level DEBUG
   ```

---

**🎓 Congratulations! You've completed the ROS 2 Actions Lab!** 🚀✨
