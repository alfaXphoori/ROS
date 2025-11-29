# **🚀 ROS 2 Actions Fundamentals**

Master asynchronous long-running tasks with feedback, goal cancellation, and result handling in ROS 2.

---

## **📌 Project Title**

Create and Use ROS 2 Actions for Asynchronous Task Execution

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🛠 Overview**

Actions in ROS 2 provide a way to execute long-running asynchronous tasks with feedback and goal cancellation capability. Unlike services (synchronous request-response) and topics (one-way continuous data), actions enable:

- Long-duration operations with periodic feedback
- Goal submission with unique tracking
- Feedback during task execution
- Goal cancellation capability
- Result retrieval after completion
- Status monitoring throughout execution

**What You'll Learn:**
- Action definition (.action files)
- Server implementation for handling goals
- Client implementation for submitting goals
- Feedback mechanisms for progress updates
- Goal cancellation and preemption
- Result handling and completion
- Best practices for action-based systems

---

## **📊 Architecture Diagram**

```
┌─────────────────────────────────────────────────────────┐
│   Action Client                                         │
│                                                         │
│  1. Send Goal Request                                   │
│  2. Receive Feedback (periodic)                         │
│  3. Receive Result (on completion)                      │
│  4. Cancel Goal (optional)                              │
└──────────────┬──────────────────────────────────────────┘
               │
        ROS 2 Action Server
               │
┌──────────────▼──────────────────────────────────────────┐
│   Action Server                                         │
│                                                         │
│  📋 Goal Handler:                                       │
│     - Accept/Reject goals                               │
│     - Handle goal execution                             │
│     - Track status                                      │
│                                                         │
│  📊 Feedback Publisher:                                 │
│     - Send periodic updates                             │
│     - Report progress                                   │
│     - Include intermediate results                      │
│                                                         │
│  ✅ Result Publisher:                                   │
│     - Send final result                                 │
│     - Set success/failure                               │
│     - Provide completion data                           │
│                                                         │
│  ⏹️ Cancellation Handler:                               │
│     - Gracefully stop execution                         │
│     - Clean up resources                                │
│     - Return partial results if needed                  │
└─────────────────────────────────────────────────────────┘

Timeline of Action Execution:
═════════════════════════════

Client                         Server
  │                              │
  │──────── Send Goal ─────────→ │ Accept/Reject
  │                              │
  │←─── Feedback (progress) ─────│ (repeats)
  │                              │
  │←─── Feedback (progress) ─────│
  │                              │
  │         [Execution...]       │
  │                              │
  │←──── Result (completion) ────│
  │                              │
```

---

## **Action Definition: CountUntil.action**

```
# Goal: Count until this value
int32 target

---

# Feedback: Current count
int32 current

---

# Result: Final count achieved
int32 total_count
bool success
```

---

## **Step 1: Create Action Package**

### **Directory Structure**

```bash
ce_robot/
├── action/
│   └── CountUntil.action
├── ce_robot/
│   ├── count_action_server.py
│   ├── count_action_client.py
│   ├── __init__.py
├── setup.py
├── setup.cfg
├── package.xml
├── CMakeLists.txt (if using C++)
```

### **Create Action Definition: action/CountUntil.action**

```
# Goal: The target value to count until
int32 target

---

# Feedback: Current count value
int32 current

---

# Result: Final result information
int32 total_count
bool success
```

### **Update CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(ce_robot)

if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "9" OR
   CMAKE_C_COMPILER_VERSION VERSION_LESS "9")
  message(WARNING "C compiler too old, disabling some packages")
  return()
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CountUntil.action"
)

ament_package()
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
  <depend>action_msgs</depend>
  <depend>rclcpp_action</depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

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
            'count_server = ce_robot.count_action_server:main',
            'count_client = ce_robot.count_action_client:main',
        ],
    },
)
```

---

## **Step 2: Implement Action Server**

### **File: ce_robot/count_action_server.py**

```python
#!/usr/bin/env python3
"""
Action Server: Count Until
Counts from 1 to target value with periodic feedback
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
        self.get_logger().info('Action Server started')

    def execute_callback(self, goal_handle):
        """Execute the action goal"""
        self.get_logger().info(f'Received goal: target={goal_handle.request.target}')
        
        # Initialize feedback and result
        feedback_msg = CountUntil.Feedback()
        result_msg = CountUntil.Result()
        
        # Count from 1 to target
        for count in range(1, goal_handle.request.target + 1):
            # Check if goal was cancelled
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled')
                goal_handle.canceled()
                result_msg.total_count = count - 1
                result_msg.success = False
                return result_msg
            
            # Send feedback
            feedback_msg.current = count
            self.get_logger().info(f'Counting: {count}')
            goal_handle.publish_feedback(feedback_msg)
            
            # Sleep to simulate work
            time.sleep(1.0)
        
        # Goal succeeded
        goal_handle.succeed()
        result_msg.total_count = goal_handle.request.target
        result_msg.success = True
        
        self.get_logger().info(f'Goal succeeded! Total count: {result_msg.total_count}')
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

---

## **Step 3: Implement Action Client**

### **File: ce_robot/count_action_client.py**

```python
#!/usr/bin/env python3
"""
Action Client: Count Until
Submits counting goals and monitors progress
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ce_robot_interfaces.action import CountUntil


class CountUntilClient(Node):
    def __init__(self):
        super().__init__('count_action_client')
        self._action_client = ActionClient(self, CountUntil, 'count_until')

    def send_goal(self, target):
        """Send a counting goal"""
        goal_msg = CountUntil.Goal()
        goal_msg.target = target
        
        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending goal: target={target}')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: current count = {feedback.current}')

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
    
    # Send a goal to count until 5
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

---

## **Step 4: Build and Test**

### **Build Package**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

### **Generate Action Interfaces**

```bash
# After build, interfaces should be generated in install/
# They can be imported as:
# from ce_robot_interfaces.action import CountUntil
```

### **Terminal 1: Run Action Server**

```bash
ros2 run ce_robot count_server
```

**Expected Output:**
```
[INFO] [count_action_server]: Action Server started
[INFO] [count_action_server]: Received goal: target=5
[INFO] [count_action_server]: Counting: 1
[INFO] [count_action_server]: Counting: 2
[INFO] [count_action_server]: Counting: 3
[INFO] [count_action_server]: Counting: 4
[INFO] [count_action_server]: Counting: 5
[INFO] [count_action_server]: Goal succeeded! Total count: 5
```

### **Terminal 2: Run Action Client**

```bash
ros2 run ce_robot count_client
```

**Expected Output:**
```
[INFO] [count_action_client]: Sending goal: target=5
[INFO] [count_action_client]: Goal accepted
[INFO] [count_action_client]: Feedback: current count = 1
[INFO] [count_action_client]: Feedback: current count = 2
[INFO] [count_action_client]: Feedback: current count = 3
[INFO] [count_action_client]: Feedback: current count = 4
[INFO] [count_action_client]: Feedback: current count = 5
[INFO] [count_action_client]: Result: total_count=5, success=True
```

---

## **Step 5: Monitor Actions**

### **List Active Actions**

```bash
ros2 action list
```

**Output:**
```
/count_until
```

### **Send Goal via CLI**

```bash
ros2 action send_goal /count_until ce_robot_interfaces/action/CountUntil "{target: 10}"
```

### **Monitor Goal Status**

```bash
ros2 action send_goal /count_until ce_robot_interfaces/action/CountUntil "{target: 10}" \
  --feedback
```

### **Get Action Info**

```bash
ros2 interface show ce_robot_interfaces/action/CountUntil
```

---

## **📝 Key Concepts**

### **Action vs Service vs Topic**

| Aspect | Action | Service | Topic |
|--------|--------|---------|-------|
| **Use Case** | Long-running tasks | Simple queries | Continuous data |
| **Duration** | Minutes/Hours | Milliseconds | Continuous |
| **Feedback** | Periodic updates | None | One-way |
| **Cancellation** | Supported | Not supported | N/A |
| **Result** | Final result | Single response | Streaming |
| **Example** | Robot navigation | Get temperature | Sensor data |

### **Action States**

```
UNKNOWN
  │
  ├─→ ACCEPTED → EXECUTING → SUCCEEDED ✓
  │      │           │
  │      └─────────→ CANCELED ✗
  │
  └─→ REJECTED ✗
```

### **Action Messages**

- **Goal** - What the client wants the server to do
- **Feedback** - Progress updates from server to client
- **Result** - Final outcome after completion
- **Status** - Current state of goal execution

### **Callback Functions**

```python
# Server callbacks
execute_callback()           # Main task execution
goal_callback()             # Accept/reject goals
cancel_callback()           # Handle cancellation

# Client callbacks
goal_response_callback()    # Goal accepted/rejected
feedback_callback()         # Periodic feedback
get_result_callback()       # Final result
```

---

## **⚠️ Troubleshooting**

### **Issue: "Action server not available"**
- **Cause:** Server not running or not built
- **Solution:** Build package and start server first

### **Issue: "Cannot import action interfaces"**
- **Cause:** Package not built or CMakeLists.txt missing
- **Solution:** Ensure CMakeLists.txt has rosidl_generate_interfaces()

### **Issue: "Goal rejected"**
- **Cause:** Server rejected the goal in goal_callback()
- **Solution:** Check server-side goal validation logic

### **Issue: "Feedback not received"**
- **Cause:** Feedback callback not registered
- **Solution:** Pass feedback_callback to send_goal_async()

---

## **📚 Resources**

- [ROS 2 Actions Documentation](https://docs.ros.org/en/jazzy/Concepts/Intermediate/Actions.html)
- [ROS 2 Action CLI](https://docs.ros.org/en/jazzy/Concepts/Intermediate/Actions.html#ros2-action-command)
- [ROS 2 Action Composition](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Composition.html)
- [ROS 2 Action Lifecycle](https://docs.ros.org/en/jazzy/Concepts/Intermediate/Actions.html#goal-state-machine)

---

## **✅ Verification Checklist**

- [ ] Action package created
- [ ] Action definition (.action file) created
- [ ] CMakeLists.txt configured
- [ ] Action interfaces generated after build
- [ ] Action server implemented
- [ ] Action client implemented
- [ ] Server accepts and executes goals
- [ ] Client sends goals and receives feedback
- [ ] Result properly returned
- [ ] `ros2 action list` shows actions
- [ ] Goal cancellation works (optional)

---

## **🚀 Next Steps**

After mastering actions, you're ready for the next advanced topics:

### **Option 1: Launch Files (7_Launch) - Recommended Next**
**Automate launching multiple nodes with parameters**
- 🚀 Launch complex systems with one command
- ⚙️ Configure multiple nodes at startup
- 📝 Create reusable system configurations
- **Duration:** ~1.5 hours | **Level:** Beginner to Intermediate

### **Option 2: Simulation (8_Simulation)**
**Test your actions in Webots robot simulator**
- 🤖 Virtual robot with 3D environment
- 🎮 Physics-based simulation
- 🔄 Integration with ROS 2 actions
- **Duration:** ~3 hours | **Level:** Advanced

### **Option 3: Advanced Patterns (Extensions)**
**Combine actions with parameters and launch files**
- 🔧 Dynamic action configuration
- 🎯 Multi-robot action coordination
- 📊 Action result aggregation
- **Duration:** Variable | **Level:** Advanced

### **Recommended Learning Path:**
```
Parameters → Actions → Launch Files → Simulation → Advanced Patterns
             (You are here!)  (Next)    (Then)     (Advanced)
```

---

**🎓 Congratulations! You've learned ROS 2 actions!** 🚀✨
