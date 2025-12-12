# **🚀 ROS 2 Actions Fundamentals**

Learn how to implement long-running asynchronous tasks with feedback using ROS 2 Actions.

---

## **📌 Project Title**

Long-Running Tasks with Feedback and Cancellation using ROS 2 Actions

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🛠 Overview**

Actions in ROS 2 provide a communication pattern designed for long-running, asynchronous tasks that require periodic feedback and the ability to be cancelled. Unlike services (which block until complete) or topics (which stream data without acknowledgment), actions offer goal-oriented execution with progress monitoring.

**Key Capabilities:**
- ✅ Send goals asynchronously (non-blocking)
- ✅ Receive periodic feedback during execution
- ✅ Cancel goals mid-execution
- ✅ Track goal status (pending, active, succeeded, cancelled, aborted)
- ✅ Handle multiple concurrent goals
- ✅ Return final results when complete

**What You'll Learn:**
- Creating custom action definitions (.action files)
- Implementing action servers with execute callbacks
- Building action clients with feedback handling
- Managing goal cancellation and status tracking
- Handling multiple concurrent goals
- Best practices for action design patterns

**Actions vs Other Communication Methods:**

| Feature | Topics | Services | Actions | Parameters |
|---------|--------|----------|---------|------------|
| **Pattern** | Publish-Subscribe | Request-Response | Goal-Feedback-Result | Get-Set |
| **Duration** | Continuous | Instant (ms) | Long-running (sec-min) | Persistent |
| **Feedback** | Streaming | None | Periodic updates | None |
| **Cancellation** | N/A | No | Yes | N/A |
| **Blocking** | No | Yes | No | No |
| **Use Case** | Sensor data | Calculations | Navigation, Tasks | Configuration |

---

## **📊 Architecture Diagram**

```
┌────────────────────────────────────────────────────────────────┐
│  Step 1: Action Definition (ce_robot_interfaces)               │
│  ┌────────────────────────────────────────────────────────┐    │
│  │  CountUntil.action                                     │    │
│  │  • Goal: target (count to), period (delay)             │    │
│  │  • Result: total_count (final value)                   │    │
│  │  • Feedback: current_count (progress)                  │    │
│  └────────────────────────────────────────────────────────┘    │
└────────────────────────────────────────────────────────────────┘
                            ⬇️
┌────────────────────────────────────────────────────────────────┐
│  Step 2: Action Server (count_until_server)                    │
│                                                                │
│  ┌────────────────────────────────────────────────────────┐    │
│  │ 🎯 Goal Reception:                                     │    │
│  │                                                        │    │
│  │ • Client sends goal (target=10, period=1)              │    │
│  │ • Server accepts/rejects goal                          │    │
│  │ • Goal handle created for tracking                     │    │
│  └────────────────────────────────────────────────────────┘    │
│                          ⬇️                                    │
│                                                                │
│  ┌────────────────────────────────────────────────────────┐    │
│  │ 🔄 Execute Callback (Long-Running Task):               │    │
│  │                                                        │    │
│  │ • Loop from 1 to target                                │    │
│  │ • Check for cancellation request                       │    │
│  │ • Publish feedback (current_count) periodically        │    │
│  │ • Sleep for period duration                            │    │
│  │ • Continue until complete or cancelled                 │    │
│  └────────────────────────────────────────────────────────┘    │
│                          ⬇️                                    │
│                                                                │
│  ┌────────────────────────────────────────────────────────┐    │
│  │ ✅ Goal Completion:                                    │    │
│  │                                                        │    │
│  │ • goal_handle.succeed() - Task completed               │    │
│  │ • goal_handle.canceled() - Client cancelled            │    │
│  │ • goal_handle.abort() - Error occurred                 │    │
│  │ • Return Result (total_count)                          │    │
│  └────────────────────────────────────────────────────────┘    │
└────────────────────────────────────────────────────────────────┘
                            ⬇️
┌────────────────────────────────────────────────────────────────┐
│  Step 3: Action Client (count_until_client)                    │
│                                                                │
│  ┌────────────────────────────────────────────────────────┐    │
│  │ 📤 Send Goal:                                          │    │
│  │                                                        │    │
│  │ • Create goal message (target, period)                 │    │
│  │ • send_goal_async() - Non-blocking call                │    │
│  │ • Register feedback callback                           │    │
│  │ • Wait for goal acceptance                             │    │
│  └────────────────────────────────────────────────────────┘    │
│                          ⬇️                                    │
│                                                                │
│  ┌────────────────────────────────────────────────────────┐    │
│  │ 📊 Feedback Callback:                                  │    │
│  │                                                        │    │
│  │ • Receives current_count updates                       │    │
│  │ • Display progress: "Feedback: count=3"                │    │
│  │ • Monitor execution in real-time                       │    │
│  │ • Optional: Cancel if conditions met                   │    │
│  └────────────────────────────────────────────────────────┘    │
│                          ⬇️                                    │
│                                                                │
│  ┌────────────────────────────────────────────────────────┐    │
│  │ 🎉 Result Callback:                                    │    │
│  │                                                        │    │
│  │ • Receive final result (total_count)                   │    │
│  │ • Check goal status (succeeded/cancelled/aborted)      │    │
│  │ • Process result data                                  │    │
│  │ • Log completion message                               │    │
│  └────────────────────────────────────────────────────────┘    │
└────────────────────────────────────────────────────────────────┘
                            ⬇️
     Action Lifecycle:  PENDING → ACTIVE → (SUCCEEDED | CANCELED | ABORTED)

┌──────────────────────────────────────────────────────────┐
│  Action Communication Flow                               │
│                                                          │
│  Client                          Server                  │
│    │                               │                     │
│    ├─ 1. send_goal() ──────────────>                     │
│    │                         2. accept/reject            │
│    │                         3. execute_callback()       │
│    │                               │                     │
│    │   4. Feedback (periodic)      │                     │
│    │ <──── publish_feedback() ─────│                     │
│    │                               │                     │
│    │   5. Feedback (periodic)      │                     │
│    │ <──── publish_feedback() ─────│                     │
│    │                               │                     │
│    │ 6. cancel_goal() (optional)──>│                     │
│    │                         7. handle cancellation      │
│    │                               │                     │
│    │   8. Result (final)           │                     │
│    │ <──── goal_handle.succeed() ──│                     │
│    ▼                               ▼                     │
│  Process                        Complete                 │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

---

## **Step 1: Create Custom Action Definition**

Before creating action servers and clients, you need to define the action interface that specifies what data will be sent (Goal), what progress updates will be provided (Feedback), and what final data will be returned (Result).

### **Create Action Interface Package**

Navigate to your interfaces package and create the action directory:

```bash
cd ~/ros2_ws/src/ce_robot_interfaces
mkdir -p action
cd action
```

### **Define Custom Action**

Create the action definition file:

```bash
touch CountUntil.action
```

#### **File: CountUntil.action**

Actions are defined in `.action` files with three components separated by `---`:

```
# Goal: What the client wants the server to do
int32 target
int32 period

---

# Result: What the server returns when done
int32 total_count

---

# Feedback: Periodic updates during execution
int32 current_count
```

**Action Components Explained:**

**Goal (Client → Server):**
- `target`: The number to count up to (e.g., 10)
- `period`: Delay between each count in seconds (e.g., 1)
- Sent once when client initiates the action
- Server decides to accept or reject

**Result (Server → Client):**
- `total_count`: Final count reached when complete
- Sent once when goal succeeds, is cancelled, or aborted
- Contains final state of the task

**Feedback (Server → Client):**
- `current_count`: Current progress (1, 2, 3... up to target)
- Sent periodically during execution
- Allows client to monitor real-time progress

### **📁 Directory Structure**

```
📁 ros2_ws/
└── 📁 src/
    └── 📁 ce_robot_interfaces/
        ├── 📁 action/
        │   └── 📄 CountUntil.action    ← Create this file
        ├── 📁 msg/
        │   ├── 📄 HardwareStatus.msg
        │   └── 📄 RobotTag.msg
        ├── 📁 srv/
        │   └── 📄 CalRectangle.srv
        ├── 📄 CMakeLists.txt
        └── 📄 package.xml
```

### **Configure Action Package**

**Update CMakeLists.txt:**

Add action generation to your interfaces package:

```cmake
cmake_minimum_required(VERSION 3.8)
project(ce_robot_interfaces)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate interfaces (messages, services, actions)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "msg/RobotTag.msg"
  "srv/CalRectangle.srv"
  "action/CountUntil.action"
  DEPENDENCIES builtin_interfaces
)

ament_package()
```

**Update package.xml:**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>builtin_interfaces</depend>
  <depend>action_msgs</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### **Build Action Package**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
source install/setup.bash
```

### **Verify Action Creation**

Check that the action was generated successfully:

```bash
# Verify CountUntil action
ros2 interface show ce_robot_interfaces/action/CountUntil
```

**Expected Output:**
```
# Goal
int32 target
int32 period
---
# Result
int32 total_count
---
# Feedback
int32 current_count
```

```bash
# List available action types
ros2 interface list | grep action
```

You should see:
```
ce_robot_interfaces/action/CountUntil
```

---

## **Example Use Case: CountUntil Action**

The `CountUntil` action is ideal for demonstrating action fundamentals. Real-world applications include:

- **Task Progress Monitoring**: Track completion percentage of long-running operations
- **Sequence Execution**: Execute a series of steps with progress feedback
- **Timeout-Based Operations**: Perform actions for a specific duration
- **Calibration Routines**: Step through calibration procedures with status updates

**Example Goal:**
```python
goal = CountUntil.Goal()
goal.target = 10      # Count to 10
goal.period = 1       # Wait 1 second between counts
```

**Feedback Updates:**
```
Feedback: current_count = 1
Feedback: current_count = 2
Feedback: current_count = 3
...
Feedback: current_count = 10
```

**Final Result:**
```python
result.total_count = 10  # Successfully counted to 10
```

---

## **Step 2: Create Action Server**

The action server receives goals from clients, executes long-running tasks, publishes periodic feedback, and returns final results.

### **📁 File Location**

Navigate to your ROS 2 workspace and create the Python file:

```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
touch count_until_server.py
chmod +x count_until_server.py
```

**Directory Structure:**
```
📁 ros2_ws/
└── 📁 src/
    └── 📁 ce_robot/
        └── 📁 ce_robot/
            ├── 📄 __init__.py
            └── 🐍 count_until_server.py    ← Create this file
```

### **Implementation: count_until_server.py**

#### **File: count_until_server.py**

```python
#!/usr/bin/env python3
"""
Action Server: Count Until
Counts from 1 to target with feedback
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ce_robot_interfaces.action import CountUntil
import time


class CountUntilActionServer(Node):
    def __init__(self):
        super().__init__('count_until_server')
        
        self._action_server = ActionServer(
            self,
            CountUntil,
            'count_until',
            self.execute_callback
        )
        
        self.get_logger().info('Count Until Action Server started')

    def execute_callback(self, goal_handle):
        """Execute the counting task"""
        self.get_logger().info(
            f'Executing goal: count to {goal_handle.request.target} '
            f'with {goal_handle.request.period}s period'
        )
        
        # Initialize feedback
        feedback_msg = CountUntil.Feedback()
        
        # Count from 1 to target
        for count in range(1, goal_handle.request.target + 1):
            # Check if goal was cancelled
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled')
                goal_handle.canceled()
                return CountUntil.Result(total_count=count - 1)
            
            # Update feedback
            feedback_msg.current_count = count
            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().info(f'Counting: {count}/{goal_handle.request.target}')
            
            # Wait for period
            time.sleep(goal_handle.request.period)
        
        # Goal succeeded
        goal_handle.succeed()
        
        result = CountUntil.Result()
        result.total_count = goal_handle.request.target
        
        self.get_logger().info('Goal completed successfully')
        
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = CountUntilActionServer()
    
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Code Explanation:**

**1. ActionServer Creation:**
```python
self._action_server = ActionServer(
    self,                    # Node instance
    CountUntil,             # Action type
    'count_until',          # Action name
    self.execute_callback   # Execution function
)
```

**2. Execute Callback:**
- Receives `goal_handle` containing the goal request
- Accesses goal data: `goal_handle.request.target`, `goal_handle.request.period`
- Main execution loop processes the long-running task

**3. Feedback Publishing:**
```python
feedback_msg = CountUntil.Feedback()
feedback_msg.current_count = count
goal_handle.publish_feedback(feedback_msg)
```

**4. Cancellation Handling:**
```python
if goal_handle.is_cancel_requested:
    goal_handle.canceled()
    return CountUntil.Result(total_count=count - 1)
```

**5. Goal Completion:**
```python
goal_handle.succeed()
result = CountUntil.Result()
result.total_count = goal_handle.request.target
return result
```

**Goal States:**
- **PENDING**: Goal received, waiting to execute
- **ACTIVE**: Currently executing in `execute_callback()`
- **SUCCEEDED**: Called `goal_handle.succeed()`
- **CANCELED**: Called `goal_handle.canceled()`
- **ABORTED**: Called `goal_handle.abort()` on error

---

## **Step 3: Create Action Client**

The action client sends goals to the server, receives periodic feedback, and handles final results.

### **Implementation: count_until_client.py**

#### **File: count_until_client.py**

```python
#!/usr/bin/env python3
"""
Action Client: Count Until
Sends goal to server and monitors progress
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ce_robot_interfaces.action import CountUntil


class CountUntilActionClient(Node):
    def __init__(self):
        super().__init__('count_until_client')
        
        self._action_client = ActionClient(
            self,
            CountUntil,
            'count_until'
        )
        
        self.get_logger().info('Count Until Action Client initialized')

    def send_goal(self, target, period):
        """Send goal to action server"""
        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()
        
        goal_msg = CountUntil.Goal()
        goal_msg.target = target
        goal_msg.period = period
        
        self.get_logger().info(f'Sending goal: target={target}, period={period}s')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle server's response to goal"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return
        
        self.get_logger().info('Goal accepted by server')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Receive feedback during execution"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: current_count = {feedback.current_count}')

    def get_result_callback(self, future):
        """Handle final result"""
        result = future.result().result
        self.get_logger().info(f'Result: total_count = {result.total_count}')


def main(args=None):
    rclpy.init(args=args)
    action_client = CountUntilActionClient()
    
    # Send goal with target=5 and period=1 second
    action_client.send_goal(target=5, period=1)
    
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Code Explanation:**

**1. ActionClient Creation:**
```python
self._action_client = ActionClient(
    self,              # Node instance
    CountUntil,       # Action type
    'count_until'     # Action name
)
```

**2. Sending Goal (Asynchronous):**
```python
self._send_goal_future = self._action_client.send_goal_async(
    goal_msg,
    feedback_callback=self.feedback_callback  # Register feedback handler
)
self._send_goal_future.add_done_callback(self.goal_response_callback)
```

**3. Goal Response Callback:**
- Checks if goal was accepted or rejected
- If accepted, requests result with `goal_handle.get_result_async()`
- Registers result callback

**4. Feedback Callback:**
- Called periodically during execution
- Receives `feedback_msg.feedback.current_count`
- Allows real-time monitoring of progress

**5. Result Callback:**
- Called once when goal completes (succeeded/cancelled/aborted)
- Accesses final result: `future.result().result.total_count`
- Check status: `future.result().status`

**Callback Flow:**
```
send_goal_async()
    ↓
goal_response_callback() → goal accepted/rejected
    ↓ (if accepted)
feedback_callback() → called multiple times during execution
    ↓
get_result_callback() → final result received
```

---

## **Step 4: Package Configuration**

Configure your Python package to install and run the action server and client.

### **Update setup.py**

Add entry points for the action nodes:

```python
from setuptools import setup

package_name = 'ce_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@ksu.ac.th',
    description='CE Robot ROS 2 examples',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            '06_count_until_server = ce_robot.count_until_server:main',
            '06_count_until_client = ce_robot.count_until_client:main',
        ],
    },
)
```

### **Update package.xml**

Ensure action dependencies are included:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ce_robot</name>
  <version>0.0.0</version>
  <description>CE Robot ROS 2 implementation package</description>
  <maintainer email="student@ksu.ac.th">Student</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>ce_robot_interfaces</depend>
  <depend>rclpy</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## **Step 5: Build and Run**

### **Build Packages**

Build both the interfaces and implementation packages:

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces 
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

**Build Output:**
```
Starting >>> ce_robot_interfaces
Finished <<< ce_robot_interfaces [2.5s]
Starting >>> ce_robot
Finished <<< ce_robot [1.2s]

Summary: 2 packages finished [3.8s]
```

### **Verify Installation**

Check that the action and nodes are available:

```bash
# Verify action interface
ros2 interface show ce_robot_interfaces/action/CountUntil

# List executable nodes
ros2 pkg executables ce_robot
```

Expected output:
```
ce_robot 06_count_until_server
ce_robot 06_count_until_client
```

---

## **Method 1: Running with Basic Configuration**

### **Terminal 1 - Start Action Server**

```bash
ros2 run ce_robot 06_count_until_server
```

**Expected Output:**
```
[INFO] [1733875234.123] [count_until_server]: Count Until Action Server started
```

The server is now running and waiting for goals from clients.

### **Terminal 2 - Send Goal via Client**

```bash
ros2 run ce_robot 06_count_until_client
```

**Expected Output (Client):**
```
[INFO] [1733875240.456] [count_until_client]: Count Until Action Client initialized
[INFO] [1733875240.457] [count_until_client]: Waiting for action server...
[INFO] [1733875240.458] [count_until_client]: Sending goal: target=5, period=1s
[INFO] [1733875240.459] [count_until_client]: Goal accepted by server
[INFO] [1733875241.460] [count_until_client]: Feedback: current_count = 1
[INFO] [1733875242.461] [count_until_client]: Feedback: current_count = 2
[INFO] [1733875243.462] [count_until_client]: Feedback: current_count = 3
[INFO] [1733875244.463] [count_until_client]: Feedback: current_count = 4
[INFO] [1733875245.464] [count_until_client]: Feedback: current_count = 5
[INFO] [1733875245.465] [count_until_client]: Result: total_count = 5
```

**Expected Output (Server):**
```
[INFO] [1733875240.459] [count_until_server]: Executing goal: count to 5 with 1s period
[INFO] [1733875241.460] [count_until_server]: Counting: 1/5
[INFO] [1733875242.461] [count_until_server]: Counting: 2/5
[INFO] [1733875243.462] [count_until_server]: Counting: 3/5
[INFO] [1733875244.463] [count_until_server]: Counting: 4/5
[INFO] [1733875245.464] [count_until_server]: Counting: 5/5
[INFO] [1733875245.465] [count_until_server]: Goal completed successfully
```

**Screenshot: Count to 5 with 1s period**

![Count Until 5 - Server and Client](imgs/01_Count5_1.png)

**Screenshot: Count to 10 with 2s period (using command line)**

![Count Until 10 - Command Line](imgs/02_Count10_1.png)

**What's Happening:**
1. Client sends goal (target=5, period=1)
2. Server accepts goal and starts execution
3. Server counts from 1 to 5, sleeping 1 second between counts
4. Server publishes feedback after each count
5. Client receives and logs each feedback message
6. Server completes and sends final result
7. Client receives result (total_count=5)

---

## **Method 2: Command Line Action Interaction**

You can send goals directly from the command line without writing a client.

### **List Available Action Servers**

```bash
ros2 action list
```

**Output:**
```
/count_until
```

### **Get Action Information**

```bash
ros2 action info /count_until
```

**Output:**
```
Action: /count_until
Action clients: 0
Action servers: 1
    /count_until_server
```

### **Show Action Type**

```bash
ros2 interface show ce_robot_interfaces/action/CountUntil
```

**Output:**
```
# Goal
int32 target
int32 period
---
# Result
int32 total_count
---
# Feedback
int32 current_count
```

### **Send Goal from Command Line**

```bash
ros2 action send_goal /count_until ce_robot_interfaces/action/CountUntil \
  "{target: 10, period: 1}" \
  --feedback
```

**Output:**
```
Waiting for an action server to become available...
Sending goal:
   target: 10
   period: 1

Goal accepted with ID: [unique_id_here]

Feedback:
  current_count: 1

Feedback:
  current_count: 2

Feedback:
  current_count: 3

Feedback:
  current_count: 4

Feedback:
  current_count: 5

Feedback:
  current_count: 6

Feedback:
  current_count: 7

Feedback:
  current_count: 8

Feedback:
  current_count: 9

Feedback:
  current_count: 10

Result:
  total_count: 10

Goal finished with status: SUCCEEDED
```

### **Send Goal Without Feedback**

```bash
ros2 action send_goal /count_until ce_robot_interfaces/action/CountUntil \
  "{target: 5, period: 2}"
```

**Output:**
```
Waiting for an action server to become available...
Sending goal:
   target: 5
   period: 2

Goal accepted with ID: [unique_id_here]

Result:
  total_count: 5

Goal finished with status: SUCCEEDED
```

**Note:** Without `--feedback` flag, you only see the final result.

---

## **Method 3: Testing Goal Cancellation**

Actions can be cancelled mid-execution. Let's test this functionality.

### **Terminal 1 - Start Server**

```bash
ros2 run ce_robot 06_count_until_server
```

### **Terminal 2 - Send Long-Running Goal**

```bash
ros2 action send_goal /count_until ce_robot_interfaces/action/CountUntil \
  "{target: 20, period: 1}" \
  --feedback
```

### **Terminal 3 - Cancel Goal (while running)**

While the goal is executing, cancel it:

```bash
# List active goals
ros2 action list

# Get goal ID from Terminal 2 output, then cancel
# The goal will be automatically cancelled when you press Ctrl+C in Terminal 2
```

Or press **Ctrl+C** in Terminal 2 to cancel the goal.

**Server Output (on cancellation):**
```
[INFO] [count_until_server]: Executing goal: count to 20 with 1s period
[INFO] [count_until_server]: Counting: 1/20
[INFO] [count_until_server]: Counting: 2/20
[INFO] [count_until_server]: Counting: 3/20
[INFO] [count_until_server]: Goal cancelled
```

**Client Output (on cancellation):**
```
Feedback:
  current_count: 1

Feedback:
  current_count: 2

Feedback:
  current_count: 3

Goal finished with status: CANCELED
```

---

## **Debugging and Monitoring Tools**

### **Monitor Action Activity**

```bash
# Echo action goal messages
ros2 action echo /count_until/goal

# Echo action feedback messages
ros2 action echo /count_until/feedback

# Echo action result messages
ros2 action echo /count_until/result
```

### **View Action Status**

```bash
# Show active action servers
ros2 action list

# Get detailed information
ros2 action info /count_until

# Show action interface
ros2 interface show ce_robot_interfaces/action/CountUntil
```

### **Check Node Status**

```bash
# List running nodes
ros2 node list

# Get node info
ros2 node info /count_until_server
ros2 node info /count_until_client
```

---

## **Common Action Patterns**

### **Pattern 1: Progress Tracking**

Use feedback to track percentage completion:

```python
# In execute_callback
total_steps = goal_handle.request.target
for step in range(1, total_steps + 1):
    feedback_msg.current_count = step
    progress_percent = (step / total_steps) * 100
    self.get_logger().info(f'Progress: {progress_percent:.1f}%')
    goal_handle.publish_feedback(feedback_msg)
```

### **Pattern 2: Goal Validation**

Validate goals before execution:

```python
def execute_callback(self, goal_handle):
    # Validate goal
    if goal_handle.request.target <= 0:
        self.get_logger().error('Invalid target: must be positive')
        goal_handle.abort()
        return CountUntil.Result(total_count=0)
    
    if goal_handle.request.period < 0.1:
        self.get_logger().error('Invalid period: too short')
        goal_handle.abort()
        return CountUntil.Result(total_count=0)
    
    # Execute goal...
```

### **Pattern 3: Multiple Concurrent Goals**

Handle multiple goals with goal callbacks:

```python
from rclpy.action import ActionServer, GoalResponse

self._action_server = ActionServer(
    self,
    CountUntil,
    'count_until',
    self.execute_callback,
    goal_callback=self.goal_callback  # Add goal callback
)

def goal_callback(self, goal_request):
    """Accept or reject new goals"""
    self.get_logger().info(f'Received goal request: target={goal_request.target}')
    
    # Can reject based on server state
    if self.is_busy:
        return GoalResponse.REJECT
    
    return GoalResponse.ACCEPT
```

### **Pattern 4: Error Handling**

Handle errors gracefully:

```python
def execute_callback(self, goal_handle):
    try:
        # Execute task
        for count in range(1, goal_handle.request.target + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return CountUntil.Result(total_count=count - 1)
            
            feedback_msg.current_count = count
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(goal_handle.request.period)
        
        goal_handle.succeed()
        return CountUntil.Result(total_count=goal_handle.request.target)
        
    except Exception as e:
        self.get_logger().error(f'Execution failed: {str(e)}')
        goal_handle.abort()
        return CountUntil.Result(total_count=0)
```

---

## **📝 Key Concepts Summary**

### **Action Components**

| Component | Purpose | Direction | Frequency | Example |
|-----------|---------|-----------|-----------|---------|
| **Goal** | Task request | Client → Server | Once (start) | `{target: 10, period: 1}` |
| **Result** | Final outcome | Server → Client | Once (end) | `{total_count: 10}` |
| **Feedback** | Progress updates | Server → Client | Periodic | `{current_count: 1...10}` |

### **Action Lifecycle**

```
Client                          Server
  │                               │
  ├─ send_goal() ──────────────────>
  │                          execute_callback()
  │                          publish_feedback()
  │ <────── feedback_callback ─────│
  │                          publish_feedback()
  │ <────── feedback_callback ─────│
  │                          goal_handle.succeed()
  │ <────── result_callback ──────│
  ▼                               ▼
```

### **Goal States**

| State | Meaning | Server Action | Status Code |
|-------|---------|---------------|-------------|
| `PENDING` | Received, waiting | - | 0 |
| `ACTIVE` | Currently executing | `publish_feedback()` | 1 |
| `SUCCEEDED` | Completed successfully | `goal_handle.succeed()` | 4 |
| `CANCELED` | Client cancelled | `goal_handle.canceled()` | 5 |
| `ABORTED` | Failed/Error | `goal_handle.abort()` | 6 |

### **Action vs Service vs Topic**

| Aspect | Action | Service | Topic |
|--------|--------|---------|-------|
| **Duration** | Long (sec-min) | Short (ms) | Continuous |
| **Feedback** | Yes (periodic) | No | Streaming |
| **Blocking** | No (async) | Yes (sync) | No |
| **Cancellation** | Yes | No | N/A |
| **Result** | Yes | Yes | No |
| **Use Case** | Tasks, Navigation | Calculations | Sensor data |

---

## **⚠️ Troubleshooting**

### **Common Issues and Solutions**

| Issue | Cause | Solution |
|-------|-------|----------|
| **"Action server not found"** | Server not running or name mismatch | Check server is running: `ros2 action list` |
| **"Cannot find action type"** | Action not generated/built | Rebuild: `colcon build --packages-select ce_robot_interfaces` |
| **"Goal rejected"** | Goal validation failed | Check server logs for validation errors |
| **"No feedback received"** | Callback not registered | Ensure `feedback_callback` is passed to `send_goal_async()` |
| **"Import error"** | Package not sourced | Run: `source ~/ros2_ws/install/setup.bash` |
| **"Module not found"** | Wrong package dependencies | Check `package.xml` has `ce_robot_interfaces` dependency |

### **Debugging Commands**

```bash
# Check if action server is running
ros2 action list

# Get detailed action information
ros2 action info /count_until

# Verify action interface exists
ros2 interface show ce_robot_interfaces/action/CountUntil

# Check node is running
ros2 node list

# View node details
ros2 node info /count_until_server

# Monitor action messages
ros2 action echo /count_until/feedback
ros2 action echo /count_until/result

# Check package installation
ros2 pkg executables ce_robot

# View build logs
colcon build --packages-select ce_robot --event-handlers console_direct+
```

---

## **📚 Additional Resources**

### **ROS 2 Documentation**
- [Actions Concepts](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Actions.html)
- [Writing Action Servers & Clients (Python)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Py-Action-Server-Client.html)
- [Action Design Guidelines](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-ROS-2-Actions.html)
- [Command Line Tools](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions.html)

### **Related Concepts**
- Services vs Actions: When to use which
- Lifecycle management for action servers
- Multi-threaded executors for concurrent goals
- Action namespaces and remapping

---

## **✅ Verification Checklist**

Complete this checklist to verify your action implementation:

### **Setup**
- [ ] Created `CountUntil.action` file in `ce_robot_interfaces/action/`
- [ ] Updated `CMakeLists.txt` to generate action interfaces
- [ ] Updated `package.xml` with action dependencies
- [ ] Built interfaces package successfully: `colcon build --packages-select ce_robot_interfaces`
- [ ] Verified action interface: `ros2 interface show ce_robot_interfaces/action/CountUntil`

### **Server Implementation**
- [ ] Created `count_until_server.py` in `ce_robot/ce_robot/`
- [ ] Implemented `execute_callback()` method
- [ ] Added feedback publishing in loop
- [ ] Implemented cancellation handling
- [ ] Set goal completion status (succeed/cancel/abort)
- [ ] Added server entry point in `setup.py`
- [ ] Server builds without errors
- [ ] Server runs and logs startup message

### **Client Implementation**
- [ ] Created `count_until_client.py` in `ce_robot/ce_robot/`
- [ ] Implemented `send_goal()` method
- [ ] Registered `feedback_callback()`
- [ ] Registered `goal_response_callback()`
- [ ] Registered `get_result_callback()`
- [ ] Added client entry point in `setup.py`
- [ ] Client builds without errors
- [ ] Client connects to server successfully

### **Testing**
- [ ] Server accepts goals from client
- [ ] Client receives goal acceptance confirmation
- [ ] Feedback messages are received by client
- [ ] Final result is received by client
- [ ] Goal can be cancelled (press Ctrl+C)
- [ ] Command-line goal sending works: `ros2 action send_goal`
- [ ] Action appears in list: `ros2 action list`
- [ ] Action info shows correct details: `ros2 action info /count_until`

### **Best Practices**
- [ ] Code includes docstrings and comments
- [ ] Error handling implemented (try-except blocks)
- [ ] Logging messages are informative
- [ ] Files are executable (`chmod +x`)
- [ ] Code follows PEP 8 style guidelines

---

## **🚀 Next Steps**

Congratulations on completing ROS 2 Actions! Here's what to learn next:

### **Recommended Path: Launch Files (07_Launch)**

**Why Launch Files?**
- Automate starting multiple nodes simultaneously
- Configure parameters for multiple nodes at once
- Create repeatable, shareable configurations
- Essential for complex ROS 2 systems

**What You'll Learn:**
- Writing Python launch files
- Launching nodes with parameters
- Conditional node launching
- Launch file composition
- Event handlers and lifecycle management

**Duration:** ~1.5 hours | **Level:** Beginner to Intermediate

---

### **Alternative Path: Simulation (08_Simulation)**

**Why Simulation?**
- Test robots without physical hardware
- Validate algorithms in realistic environments
- Rapid prototyping and iteration
- Safe testing of dangerous scenarios

**What You'll Learn:**
- Webots robot simulator integration
- Creating robot models (URDF)
- Sensor simulation (cameras, lidars)
- Physics engine configuration
- ROS 2-Webots bridge

**Duration:** ~3 hours | **Level:** Advanced

---

### **Learning Path Overview:**

```
01_Publisher_Subscriber ──> 02_Server_Client ──> 03_Message ──> 04_Service
         ⬇️
05_Parameters ──> 06_Actions ──> 07_Launch ──> 08_Simulation
              (Current)     (Recommended)   (Advanced)
```

---

## **📝 Summary**

In this tutorial, you learned:

✅ **Action Fundamentals**
- Actions provide goal-oriented, asynchronous communication
- Three components: Goal, Result, Feedback
- Non-blocking execution with cancellation support

✅ **Implementation Skills**
- Created custom action definitions (.action files)
- Built action servers with execute callbacks
- Developed action clients with feedback handling
- Configured packages for action support

✅ **Advanced Techniques**
- Goal validation and rejection
- Cancellation handling
- Error management (abort)
- Multiple concurrent goals
- Status tracking

✅ **Tools & Debugging**
- Command-line action interaction
- Monitoring with `ros2 action` commands
- Debugging techniques and logging

**You're now ready to build complex, long-running tasks with real-time feedback in ROS 2!** 🚀

---

**🎓 Congratulations on completing ROS 2 Actions Fundamentals!** 🎉✨

Ready for the next challenge? Head to **07_Launch** to learn how to orchestrate multiple nodes! 🚀
