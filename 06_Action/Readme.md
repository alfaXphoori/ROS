# **🚀 ROS 2 Actions Fundamentals**

Learn how to implement long-running asynchronous tasks with feedback using ROS 2 Actions.

---

## **📌 Project Title**

Create and Use ROS 2 Actions for Long-Running Tasks with Feedback

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🛠 Overview**

Actions in ROS 2 are for tasks that:
- Take time to complete (seconds to minutes)
- Need periodic feedback during execution
- Can be cancelled mid-execution
- Require sending results when completed

**Actions vs Topics vs Services vs Parameters:**

```
Topics:      Continuous data stream (one-way)
Services:    Synchronous request-response (blocking)
Actions:     Asynchronous long-running with feedback (non-blocking)
Parameters:  Configuration values (persistent)
```

**Key Features of Actions:**
- **Goal:** Client sends what to do (async, non-blocking)
- **Feedback:** Server sends periodic updates during execution
- **Result:** Server sends final result when complete
- **Cancellation:** Client can cancel task before completion
- **Goal Status:** Server tracks execution state

---

## **📊 Architecture Diagram**

```
┌──────────────────────────────────────────────────────────┐
│                    Action Architecture                   │
├──────────────────────────────────────────────────────────┤
│                                                          │
│  ┌─────────────────┐              ┌─────────────────┐    │
│  │  Action Client  │              │ Action Server   │    │
│  │   (Behavior)    │              │  (Executor)     │    │
│  └────────┬────────┘              └────────┬────────┘    │
│           │                                │             │
│      1. Send Goal                          │             │
│      (goal_handle)     ────────────────────>             │
│           │                          2. Start execution  │
│           │            Feedback                          │
│      3. Receive <───────────────────────                 │
│      (progress%)       3. Receive Feedback               │
│           │                          4. Execute task     │
│           │                          5. Send Result      │
│      4. Get Result  <─────────────────────               │
│           │            (result_data)                     │
│           ▼                                              │
│       Process                                            │
│       Result                                             │
│                                                          │
│  ┌─────────────────┐              ┌─────────────────┐    │
│  │  .action File   │              │   Topics Used   │    │
│  ├─────────────────┤              ├─────────────────┤    │
│  │ - Goal type     │              │ - /goal         │    │
│  │ - Result type   │              │ - /feedback     │    │
│  │ - Feedback type │              │ - /status       │    │
│  └─────────────────┘              │ - /result       │    │
│                                   └─────────────────┘   │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

---

## **Step 1: Create Custom Action Definition**

### **File: CountUntil.action**

Actions are defined in `.action` files with three components: Goal, Result, and Feedback.

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

### **Directory Structure**

```bash
ce_robot_interfaces/
├── action/
│   └── CountUntil.action          # Action definition
├── msg/
│   └── HardwareStatus.msg
├── srv/
│   └── CalRectangle.srv
├── CMakeLists.txt
└── package.xml
```

### **Update CMakeLists.txt**

Add action generation to your interfaces package:

```cmake
cmake_minimum_required(VERSION 3.8)
project(ce_robot_interfaces)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "srv/CalRectangle.srv"
  "action/CountUntil.action"
)

ament_package()
```

### **Build and Verify**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
source install/setup.bash

# List available actions
ros2 action list

# Get action details
ros2 action info /count_until_server
```

---

## **Step 2: Create Action Server**

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

### **File: count_until_server.py**

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

---

## **Step 3: Create Action Client**

### **File: count_until_client.py**

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

---

## **Step 4: Package Configuration**

### **Update setup.py**

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
            'count_until_server = ce_robot.count_until_server:main',
            'count_until_client = ce_robot.count_until_client:main',
        ],
    },
)
```

### **Update package.xml**

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

## **Step 5: Running Actions**

### **Build Packages**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces ce_robot --symlink-install
source install/setup.bash
```

### **Terminal 1 - Start Action Server**

```bash
ros2 run ce_robot count_until_server
```

**Expected Output:**
```
[INFO] [count_until_server]: Count Until Action Server started
[INFO] [count_until_server]: Executing goal: count to 5 with 1s period
[INFO] [count_until_server]: Counting: 1/5
[INFO] [count_until_server]: Counting: 2/5
[INFO] [count_until_server]: Counting: 3/5
[INFO] [count_until_server]: Counting: 4/5
[INFO] [count_until_server]: Counting: 5/5
[INFO] [count_until_server]: Goal completed successfully
```

### **Terminal 2 - Send Action Goal**

```bash
ros2 run ce_robot count_until_client
```

**Expected Output:**
```
[INFO] [count_until_client]: Count Until Action Client initialized
[INFO] [count_until_client]: Waiting for action server...
[INFO] [count_until_client]: Sending goal: target=5, period=1s
[INFO] [count_until_client]: Goal accepted by server
[INFO] [count_until_client]: Feedback: current_count = 1
[INFO] [count_until_client]: Feedback: current_count = 2
[INFO] [count_until_client]: Feedback: current_count = 3
[INFO] [count_until_client]: Feedback: current_count = 4
[INFO] [count_until_client]: Feedback: current_count = 5
[INFO] [count_until_client]: Result: total_count = 5
```

### **Command Line Action Interaction**

```bash
# List action servers
ros2 action list

# Get action info
ros2 action info /count_until

# Send goal from command line
ros2 action send_goal /count_until ce_robot_interfaces/action/CountUntil \
  "{target: 10, period: 1}" \
  --feedback

# Send goal with feedback display
ros2 action send_goal /count_until ce_robot_interfaces/action/CountUntil \
  "{target: 5, period: 2}" \
  --feedback
```

---

## **📝 Key Concepts**

### **Action File Components**

| Component | Purpose | Example |
|-----------|---------|---------|
| **Goal** | What client requests | `int32 target` |
| **Result** | Final output when done | `int32 total_count` |
| **Feedback** | Periodic progress updates | `int32 current_count` |

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

| State | Meaning | Server Action |
|-------|---------|---------------|
| `PENDING` | Received, waiting to execute | - |
| `ACTIVE` | Currently executing | `publish_feedback()` |
| `SUCCEEDED` | Completed successfully | `goal_handle.succeed()` |
| `CANCELED` | Client requested cancellation | `goal_handle.canceled()` |
| `ABORTED` | Failed during execution | `goal_handle.abort()` |

### **Action vs Service Comparison**

| Aspect | Action | Service |
|--------|--------|---------|
| **Duration** | Long-running (sec-min) | Quick (ms) |
| **Feedback** | Yes (periodic) | No (only result) |
| **Blocking** | No (async) | Yes (sync) |
| **Cancellation** | Yes | No |
| **Use Case** | Navigation, manipulation | Queries, calculations |

---

## **⚠️ Troubleshooting**

### **Issue: "Action server not found"**
- **Cause:** Server not running or action name mismatch
- **Solution:** Ensure server is running, check action name with `ros2 action list`

### **Issue: "Cannot find action type"**
- **Cause:** Action interface not generated/built
- **Solution:** Rebuild interfaces: `colcon build --packages-select ce_robot_interfaces`

### **Issue: "Goal rejected by server"**
- **Cause:** Server's `execute_callback()` returned False
- **Solution:** Check server logs for validation errors

### **Issue: "Feedback not received"**
- **Cause:** Callback not registered or server not publishing feedback
- **Solution:** Add feedback callback before sending goal

---

## **📚 Resources**

- [ROS 2 Actions Documentation](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Actions.html)
- [ROS 2 Actions Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Py-Action-Server-Client.html)
- [Action Definition (IDL)](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-ROS-2-Actions.html)
- [ROS 2 Action Command Line Tools](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions.html)

---

## **✅ Verification Checklist**

- [ ] Action definition file (.action) created
- [ ] Action generated in build (colcon build successful)
- [ ] Action server code written
- [ ] Action client code written
- [ ] Server and client packages configured
- [ ] Server runs without errors
- [ ] Client connects to server
- [ ] Goal sent successfully
- [ ] Feedback received during execution
- [ ] Result received when complete
- [ ] Goal can be cancelled
- [ ] Command line actions work with `ros2 action send_goal`

---

## **🚀 Next Steps**

After mastering actions, continue with:

### **Option 1: Launch Files (7_Launch) - Recommended**
**Automate launching multiple nodes with parameters and configurations**
- 🚀 Launch complex systems with single command
- ⚙️ Configure multiple nodes at once
- 📝 Create reusable launch scripts
- **Duration:** ~1.5 hours | **Level:** Beginner to Intermediate

### **Option 2: Simulation (8_Simulation)**
**Test your nodes in realistic 3D environments**
- 🤖 Webots robot simulator integration
- 🎮 Physics simulation and visualization
- 🔄 ROS 2 middleware connection
- **Duration:** ~3 hours | **Level:** Advanced

### **Recommended Learning Path:**
```
Parameters ➜ Actions ➜ Launch Files ➜ Simulation
           (Done)  (Next)   (Then)      (Advanced)
```

---

**🎓 Congratulations! You've learned ROS 2 Actions!** 🚀✨
