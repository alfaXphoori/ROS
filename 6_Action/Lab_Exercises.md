# **🚀 ROS 2 Actions Lab Exercises**

Master asynchronous task execution with feedback, goal tracking, and result handling through progressive hands-on exercises.

---

## **📌 Project Title**

Create and Use ROS 2 Actions for Asynchronous Task Execution

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🛠 Lab Overview**

This lab provides hands-on exercises to master action servers and clients in ROS 2. Each exercise progresses from basic goal submission through advanced feedback mechanisms and task management patterns.

**Duration:** ~2.5 hours
**Level:** Intermediate to Advanced
**Prerequisites:** ROS 2 Jazzy installed, Parameters lab completed

---

## **🎯 Learning Objectives**

By completing this lab, you will be able to:

- ✅ Define action interfaces (.action files)
- ✅ Implement action servers with goal handling
- ✅ Create action clients with feedback subscription
- ✅ Send goals and process feedback asynchronously
- ✅ Handle goal cancellation and preemption
- ✅ Implement proper error handling
- ✅ Monitor action status and progress
- ✅ Retrieve results after goal completion
- ✅ Debug actions using ROS 2 tools
- ✅ Design scalable action-based architectures

---

## **📊 Lab Architecture**

```
┌──────────────────────────────────────────────────┐
│ Exercise 1: Basic Counter Action                │
│ (Simple goal, feedback, result)                  │
└────────────────┬─────────────────────────────────┘
                 │
                 ▼
┌──────────────────────────────────────────────────┐
│ Exercise 2: Timer Action with Status            │
│ (Complex feedback, time tracking)                │
└────────────────┬─────────────────────────────────┘
                 │
                 ▼
┌──────────────────────────────────────────────────┐
│ Exercise 3: Task Manager Action                 │
│ (Multiple goals, queue management, cancellation)│
└──────────────────────────────────────────────────┘
```

---

## **📚 Learning Path Overview**

| Exercise | Title | Level | Duration |
|----------|-------|-------|----------|
| 1 | Basic Counter Action | Intermediate | 40 min |
| 2 | Timer Action | Intermediate | 45 min |
| 3 | Task Manager | Advanced | 50 min |

---

## **Exercise 1: Basic Counter Action (Intermediate) 📊**

### **📋 Task**

Create a simple action that counts from 1 to a target value, sending feedback for each count and returning the final result.

### **Action Definition: CountUntil.action**

```
# Goal: Target value to count until
int32 target

---

# Feedback: Current count
int32 current

---

# Result: Final result
int32 total_count
bool success
```

### **File: count_action_server.py**

```python
#!/usr/bin/env python3
"""
Exercise 1: Basic Counter Action Server
Counts from 1 to target with feedback
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ce_robot_interfaces.action import CountUntil
import time


class CountUntilServer(Node):
    def __init__(self):
        super().__init__('count_action_server')
        self._action_server = ActionServer(
            self,
            CountUntil,
            'count_until',
            self.execute_callback
        )
        self.get_logger().info('Count Until Action Server initialized')

    def execute_callback(self, goal_handle):
        """Execute counting goal"""
        self.get_logger().info(f'Received goal: target={goal_handle.request.target}')
        
        # Validate goal
        if goal_handle.request.target <= 0:
            self.get_logger().warn('Goal target must be positive')
            goal_handle.abort()
            result_msg = CountUntil.Result()
            result_msg.total_count = 0
            result_msg.success = False
            return result_msg
        
        # Initialize feedback and result
        feedback_msg = CountUntil.Feedback()
        result_msg = CountUntil.Result()
        
        # Count from 1 to target
        for count in range(1, goal_handle.request.target + 1):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().info(f'Goal cancelled at count={count}')
                goal_handle.canceled()
                result_msg.total_count = count - 1
                result_msg.success = False
                return result_msg
            
            # Send feedback
            feedback_msg.current = count
            self.get_logger().info(f'Count: {count}/{goal_handle.request.target}')
            goal_handle.publish_feedback(feedback_msg)
            
            # Simulate work with delay
            time.sleep(0.5)
        
        # Goal succeeded
        goal_handle.succeed()
        result_msg.total_count = goal_handle.request.target
        result_msg.success = True
        
        self.get_logger().info(
            f'Goal succeeded! Counted to {result_msg.total_count}'
        )
        return result_msg


def main(args=None):
    rclpy.init(args=args)
    server = CountUntilServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **File: count_action_client.py**

```python
#!/usr/bin/env python3
"""
Exercise 1: Basic Counter Action Client
Submits counting goals and monitors progress
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ce_robot_interfaces.action import CountUntil


class CountUntilClient(Node):
    def __init__(self):
        super().__init__('count_action_client')
        self._action_client = ActionClient(
            self,
            CountUntil,
            'count_until'
        )

    def send_goal(self, target):
        """Send a counting goal"""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return
        
        goal_msg = CountUntil.Goal()
        goal_msg.target = target
        
        self.get_logger().info(f'Sending goal: target={target}')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            self.get_result_callback
        )

    def feedback_callback(self, feedback_msg):
        """Handle periodic feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback received: current count = {feedback.current}'
        )

    def get_result_callback(self, future):
        """Handle final result"""
        result = future.result().result
        self.get_logger().info(
            f'Result: total_count={result.total_count}, '
            f'success={result.success}'
        )


def main(args=None):
    rclpy.init(args=args)
    client = CountUntilClient()
    
    # Send goal to count until 5
    client.send_goal(5)
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **Testing Exercise 1**

**Build:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

**Terminal 1 - Run server:**
```bash
ros2 run ce_robot count_server
```

**Terminal 2 - Run client:**
```bash
ros2 run ce_robot count_client
```

**Expected Output (Client):**
```
[INFO] [count_action_client]: Sending goal: target=5
[INFO] [count_action_client]: Goal accepted
[INFO] [count_action_client]: Feedback received: current count = 1
[INFO] [count_action_client]: Feedback received: current count = 2
[INFO] [count_action_client]: Feedback received: current count = 3
[INFO] [count_action_client]: Feedback received: current count = 4
[INFO] [count_action_client]: Feedback received: current count = 5
[INFO] [count_action_client]: Result: total_count=5, success=True
```

### **Key Concepts**

- Basic action definition structure
- ActionServer initialization and goal handling
- Feedback publishing during execution
- Result message construction
- Asynchronous callback handling
- Goal validation and error handling

---

## **Exercise 2: Timer Action with Status (Intermediate) ⏱️**

### **📋 Task**

Create an action that manages a timer, sending periodic feedback with elapsed time and remaining time, supporting pause/resume functionality.

### **Action Definition: TimerControl.action**

```
# Goal: Timer duration in seconds
float64 duration
bool auto_start

---

# Feedback: Timer status
float64 elapsed_time
float64 remaining_time
string state

---

# Result: Timer execution summary
float64 total_elapsed
bool completed
string reason
```

### **File: timer_action_server.py**

```python
#!/usr/bin/env python3
"""
Exercise 2: Timer Action Server
Manages countdown timer with status feedback
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ce_robot_interfaces.action import TimerControl
import time
from threading import Event


class TimerServer(Node):
    def __init__(self):
        super().__init__('timer_action_server')
        self._action_server = ActionServer(
            self,
            TimerControl,
            'timer_control',
            self.execute_callback
        )
        self.get_logger().info('Timer Action Server initialized')

    def execute_callback(self, goal_handle):
        """Execute timer goal"""
        duration = goal_handle.request.duration
        auto_start = goal_handle.request.auto_start
        
        self.get_logger().info(
            f'Timer goal: duration={duration}s, auto_start={auto_start}'
        )
        
        # Validate duration
        if duration <= 0:
            self.get_logger().warn('Duration must be positive')
            goal_handle.abort()
            result_msg = TimerControl.Result()
            result_msg.total_elapsed = 0
            result_msg.completed = False
            result_msg.reason = 'Invalid duration'
            return result_msg
        
        feedback_msg = TimerControl.Feedback()
        result_msg = TimerControl.Result()
        
        start_time = time.time()
        elapsed = 0
        
        # Timer loop
        while elapsed < duration:
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Timer cancelled')
                goal_handle.canceled()
                result_msg.total_elapsed = elapsed
                result_msg.completed = False
                result_msg.reason = 'Cancelled'
                return result_msg
            
            # Send feedback
            elapsed = time.time() - start_time
            remaining = max(0, duration - elapsed)
            
            feedback_msg.elapsed_time = elapsed
            feedback_msg.remaining_time = remaining
            feedback_msg.state = 'RUNNING'
            
            self.get_logger().info(
                f'Timer: {elapsed:.1f}s / {duration}s '
                f'(remaining: {remaining:.1f}s)'
            )
            
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)
        
        # Timer completed
        goal_handle.succeed()
        result_msg.total_elapsed = duration
        result_msg.completed = True
        result_msg.reason = 'Completed'
        
        self.get_logger().info('Timer completed')
        return result_msg


def main(args=None):
    rclpy.init(args=args)
    server = TimerServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **File: timer_action_client.py**

```python
#!/usr/bin/env python3
"""
Exercise 2: Timer Action Client
Submits timer goals and monitors status
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ce_robot_interfaces.action import TimerControl


class TimerClient(Node):
    def __init__(self):
        super().__init__('timer_action_client')
        self._action_client = ActionClient(
            self,
            TimerControl,
            'timer_control'
        )

    def send_goal(self, duration, auto_start=True):
        """Send a timer goal"""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return
        
        goal_msg = TimerControl.Goal()
        goal_msg.duration = duration
        goal_msg.auto_start = auto_start
        
        self.get_logger().info(
            f'Starting timer: {duration}s (auto_start={auto_start})'
        )
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        """Handle goal acceptance"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        
        self.get_logger().info('Timer goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            self.get_result_callback
        )

    def feedback_callback(self, feedback_msg):
        """Handle feedback"""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Timer: {fb.elapsed_time:.1f}s elapsed, '
            f'{fb.remaining_time:.1f}s remaining '
            f'[{fb.state}]'
        )

    def get_result_callback(self, future):
        """Handle result"""
        result = future.result().result
        self.get_logger().info(
            f'Timer result: total_elapsed={result.total_elapsed:.1f}s, '
            f'completed={result.completed}, reason={result.reason}'
        )


def main(args=None):
    rclpy.init(args=args)
    client = TimerClient()
    
    # Send 5-second timer
    client.send_goal(5.0)
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **Testing Exercise 2**

**Terminal 1 - Server:**
```bash
ros2 run ce_robot timer_server
```

**Terminal 2 - Client:**
```bash
ros2 run ce_robot timer_client
```

**Expected Output (Client):**
```
[INFO] [timer_action_client]: Starting timer: 5.0s
[INFO] [timer_action_client]: Timer goal accepted
[INFO] [timer_action_client]: Timer: 0.1s elapsed, 4.9s remaining [RUNNING]
[INFO] [timer_action_client]: Timer: 0.2s elapsed, 4.8s remaining [RUNNING]
...
[INFO] [timer_action_client]: Timer: 4.9s elapsed, 0.1s remaining [RUNNING]
[INFO] [timer_action_client]: Timer result: total_elapsed=5.0s, completed=True, reason=Completed
```

### **Key Concepts**

- Complex action definitions with multiple feedback fields
- Real-time status tracking during execution
- State management in feedback messages
- Time tracking and calculation
- Graceful completion handling

---

## **Exercise 3: Task Manager Action (Advanced) 🎯**

### **📋 Task**

Create an advanced action that manages task queues with priorities, supports goal cancellation, and provides detailed progress metrics.

### **Action Definition: TaskManagement.action**

```
# Goal
string task_name
int32 priority
float64 estimated_duration

---

# Feedback
int32 progress_percentage
int32 queue_position
string current_status
int32 tasks_completed

---

# Result
bool success
float64 actual_duration
string completion_message
```

### **File: task_manager_server.py**

```python
#!/usr/bin/env python3
"""
Exercise 3: Task Manager Action Server
Manages task queue with priorities and tracking
"""

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from ce_robot_interfaces.action import TaskManagement
import time
from collections import deque


class TaskManagerServer(Node):
    def __init__(self):
        super().__init__('task_manager_server')
        self._action_server = ActionServer(
            self,
            TaskManagement,
            'task_management',
            self.execute_callback
        )
        self.task_queue = deque()
        self.tasks_completed = 0
        self.get_logger().info('Task Manager Server initialized')

    def execute_callback(self, goal_handle):
        """Execute task management goal"""
        task_name = goal_handle.request.task_name
        priority = goal_handle.request.priority
        duration = goal_handle.request.estimated_duration
        
        self.get_logger().info(
            f'New task: {task_name} (priority={priority}, '
            f'duration={duration}s)'
        )
        
        # Validate task
        if not task_name or duration <= 0:
            self.get_logger().warn('Invalid task parameters')
            goal_handle.abort()
            result_msg = TaskManagement.Result()
            result_msg.success = False
            result_msg.actual_duration = 0
            result_msg.completion_message = 'Invalid parameters'
            return result_msg
        
        # Add to queue
        self.task_queue.append((priority, task_name))
        queue_pos = len(self.task_queue)
        
        feedback_msg = TaskManagement.Feedback()
        result_msg = TaskManagement.Result()
        
        # Simulate task execution
        start_time = time.time()
        steps = int(duration * 10)  # 10 steps per second
        
        for step in range(steps):
            # Check cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().info(f'Task {task_name} cancelled')
                goal_handle.canceled()
                result_msg.success = False
                result_msg.actual_duration = time.time() - start_time
                result_msg.completion_message = 'Cancelled'
                return result_msg
            
            # Calculate progress
            progress = int((step / steps) * 100)
            remaining_queue = max(0, len(self.task_queue) - 1)
            
            feedback_msg.progress_percentage = progress
            feedback_msg.queue_position = queue_pos
            feedback_msg.current_status = f'Processing: {task_name}'
            feedback_msg.tasks_completed = self.tasks_completed
            
            self.get_logger().info(
                f'Task {task_name}: {progress}% '
                f'(Queue: {remaining_queue} tasks remaining)'
            )
            
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)
        
        # Task completed
        goal_handle.succeed()
        self.tasks_completed += 1
        actual_duration = time.time() - start_time
        
        result_msg.success = True
        result_msg.actual_duration = actual_duration
        result_msg.completion_message = (
            f'Task {task_name} completed in {actual_duration:.2f}s'
        )
        
        self.get_logger().info(
            f'Task completed: {task_name} '
            f'(actual: {actual_duration:.2f}s)'
        )
        
        # Remove from queue
        if (priority, task_name) in self.task_queue:
            self.task_queue.remove((priority, task_name))
        
        return result_msg


def main(args=None):
    rclpy.init(args=args)
    server = TaskManagerServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **File: task_manager_client.py**

```python
#!/usr/bin/env python3
"""
Exercise 3: Task Manager Action Client
Submits tasks and monitors execution
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ce_robot_interfaces.action import TaskManagement
import sys


class TaskManagerClient(Node):
    def __init__(self):
        super().__init__('task_manager_client')
        self._action_client = ActionClient(
            self,
            TaskManagement,
            'task_management'
        )
        self.task_count = 0

    def send_task(self, task_name, priority=1, duration=5.0):
        """Submit a task"""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Task manager not available')
            return
        
        goal_msg = TaskManagement.Goal()
        goal_msg.task_name = task_name
        goal_msg.priority = priority
        goal_msg.estimated_duration = duration
        
        self.get_logger().info(
            f'Submitting task: {task_name} '
            f'(priority={priority}, duration={duration}s)'
        )
        
        self.task_count += 1
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        """Handle goal acceptance"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Task rejected')
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            self.get_result_callback
        )

    def feedback_callback(self, feedback_msg):
        """Handle feedback"""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Progress: {fb.progress_percentage}% | '
            f'Queue: {fb.queue_position} | '
            f'Status: {fb.current_status} | '
            f'Completed: {fb.tasks_completed}'
        )

    def get_result_callback(self, future):
        """Handle task completion"""
        result = future.result().result
        self.get_logger().info(
            f'Task result: success={result.success}, '
            f'duration={result.actual_duration:.2f}s, '
            f'message={result.completion_message}'
        )


def main(args=None):
    rclpy.init(args=args)
    client = TaskManagerClient()
    
    # Submit multiple tasks
    client.send_task('data_processing', priority=1, duration=3.0)
    client.send_task('file_backup', priority=2, duration=2.0)
    client.send_task('system_check', priority=3, duration=2.5)
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **Testing Exercise 3**

**Terminal 1 - Server:**
```bash
ros2 run ce_robot task_manager_server
```

**Terminal 2 - Client:**
```bash
ros2 run ce_robot task_manager_client
```

**Expected Output (Client):**
```
[INFO] [task_manager_client]: Submitting task: data_processing (priority=1, duration=3.0s)
[INFO] [task_manager_client]: Submitting task: file_backup (priority=2, duration=2.0s)
[INFO] [task_manager_client]: Submitting task: system_check (priority=3, duration=2.5s)
[INFO] [task_manager_client]: Progress: 5% | Queue: 1 | Status: Processing: data_processing | Completed: 0
[INFO] [task_manager_client]: Progress: 10% | Queue: 1 | Status: Processing: data_processing | Completed: 0
...
[INFO] [task_manager_client]: Task result: success=True, duration=3.01s, message=Task data_processing completed in 3.01s
```

### **Key Concepts**

- Task queue management
- Priority handling
- Multiple concurrent tasks
- Advanced progress tracking
- Queue position feedback
- Complex result structures

---

## **ROS 2 Action CLI Commands**

```bash
# List all actions
ros2 action list

# Get action info
ros2 action info /action_name

# Send goal via CLI
ros2 action send_goal /count_until ce_robot_interfaces/action/CountUntil "{target: 10}"

# Send goal with feedback
ros2 action send_goal /count_until ce_robot_interfaces/action/CountUntil "{target: 10}" --feedback

# Show action definition
ros2 interface show ce_robot_interfaces/action/CountUntil
```

---

## **✅ Completion Checklist**

- [ ] Exercise 1: Basic Counter Action completed
  - [ ] CountUntil.action defined
  - [ ] Action server accepts goals
  - [ ] Feedback sent for each count
  - [ ] Result returned correctly

- [ ] Exercise 2: Timer Action completed
  - [ ] TimerControl.action defined
  - [ ] Real-time feedback with elapsed/remaining
  - [ ] State tracking implemented
  - [ ] Cancellation handled

- [ ] Exercise 3: Task Manager completed
  - [ ] TaskManagement.action defined
  - [ ] Task queue managed
  - [ ] Priority tracking working
  - [ ] Progress percentage calculated
  - [ ] Multiple tasks can be submitted

- [ ] All servers and clients build successfully
- [ ] Actions visible with `ros2 action list`
- [ ] CLI goal submission works
- [ ] All feedback messages received
- [ ] All results returned correctly

---

## **💡 Tips & Tricks**

1. **Always check for cancellation in loops:**
   ```python
   if goal_handle.is_cancel_requested:
       goal_handle.canceled()
       return result
   ```

2. **Register feedback callback before sending goal:**
   ```python
   self._action_client.send_goal_async(
       goal_msg,
       feedback_callback=self.feedback_callback
   )
   ```

3. **Validate goal parameters early:**
   ```python
   if not self.is_valid_goal(goal_handle.request):
       goal_handle.abort()
   ```

4. **Use meaningful status strings:**
   ```python
   feedback_msg.state = 'RUNNING|PAUSED|WAITING'
   ```

5. **Track action state transitions:**
   - UNKNOWN → ACCEPTED → EXECUTING → SUCCEEDED/CANCELED/FAILED

6. **Test with CLI before complex clients:**
   ```bash
   ros2 action send_goal /action_name Package/action/Type "{field: value}"
   ```

---

**🎓 Congratulations! You've mastered ROS 2 Actions!** 🚀✨
