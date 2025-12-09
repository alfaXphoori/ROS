# **🔧 ROS 2 Custom Services - Lab Exercises**

**Real-World Warehouse Robot Command and Control**

---

## **📌 Lab Information**

**Lab Title:** Warehouse Robot Service Control System  
**Duration:** 90 minutes  
**Level:** Beginner to Intermediate  
**Prerequisites:** 
- Completed Readme.md (Basic Service Setup)
- Completed 03_Message lab (Custom Messages)
- Created `ce_robot_interfaces` package
- Understanding of request-response patterns

**What You Already Know from Readme:**
✅ Creating service packages and `.srv` files  
✅ Building custom services with colcon  
✅ Basic server/client patterns  
✅ Using `CalRectangle.srv` example  

**What You'll Build in This Lab:**
🤖 **Complete robot command control system**  
📍 **Navigation and movement services**  
🔧 **Robot manipulation services**  
⚙️ **Configuration and calibration services**  
🎯 **Multi-step mission execution**  

---

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🎯 Lab Objectives**

By completing this lab, you will be able to:

1. **Design** services for robot command and control
2. **Create** custom services for real-world robotics operations
3. **Implement** navigation command services
4. **Build** robot manipulation control services
5. **Handle** service requests with validation and error handling
6. **Coordinate** multiple service calls for complex tasks
7. **Apply** professional robotics service patterns

---

## **🤖 Real-World Scenario**

You're developing a command and control system for an autonomous warehouse robot. The robot needs services to:
- **Navigate** to specific locations (waypoints, docking stations)
- **Manipulate** objects (pick, place, grip control)
- **Configure** its behavior (speed limits, safety zones)
- **Execute** complex missions (multi-step operations)

This lab will guide you through creating professional robot control services for warehouse automation.

---

## **📚 Lab Structure**

```
┌─────────────────────────────────────────────┐
│ Setup: Create Robot Control Services       │
│ Design services for robot commands          │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 1: Robot Navigation Service        │
│ Move robot to target positions              │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 2: Gripper Control Service         │
│ Pick and place object manipulation          │
└─────────────────────────────────────────────┘
```

| Step | Title | Duration | Difficulty |
|------|-------|----------|------------|
| Setup | Design Robot Services | 15 min | ⭐⭐ |
| Ex 1 | Navigation Service | 30 min | ⭐⭐⭐ |
| Ex 2 | Gripper Control | 25 min | ⭐⭐ |

---

## **Setup: Create Robot Control Services 📝**

### 🎯 Objective
Design and build services for warehouse robot control operations.

### 📖 Service Definitions

Create service files in `ce_robot_interfaces/srv/`:

**1. NavigateToPosition.srv** - Move robot to target location
```srv
# Request
float64 target_x           # Target X coordinate (meters)
float64 target_y           # Target Y coordinate (meters)
float64 target_yaw         # Target orientation (radians)
float32 max_speed          # Maximum speed (m/s)
bool precise_positioning   # Use high-precision mode

---

# Response
bool success               # Navigation success
float64 final_x            # Final X position reached
float64 final_y            # Final Y position reached
float64 final_yaw          # Final orientation reached
float64 distance_traveled  # Total distance traveled (meters)
float32 time_elapsed       # Time taken (seconds)
string message             # Status message
```

**2. GripperCommand.srv** - Control robot gripper
```srv
# Request
int32 command              # 1=Open, 2=Close, 3=SetPosition
float32 position           # Gripper position (0.0-1.0, 0=closed)
float32 force              # Grip force (0.0-1.0)
bool check_object          # Verify object presence

---

# Response
bool success               # Command success
bool object_detected       # Object detected in gripper
float32 actual_position    # Actual gripper position
float32 actual_force       # Actual grip force applied
string message             # Status message
```

### 📝 Update Build Configuration

Edit `ce_robot_interfaces/CMakeLists.txt`:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "msg/RobotStatus.msg"
  "srv/CalRectangle.srv"
  "srv/NavigateToPosition.srv"
  "srv/GripperCommand.srv"
)
```

### 🔨 Build the Services

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
source install/setup.bash
```

### ✅ Verify Service Structures

```bash
ros2 interface show ce_robot_interfaces/srv/NavigateToPosition
ros2 interface show ce_robot_interfaces/srv/GripperCommand
```

---

## **Exercise 1: Robot Navigation Service 🗺️**

### **📝 Task**

Create a navigation service that moves the robot to target positions with validation and safety checks.

### 🎯 Objective
Implement a realistic navigation service that:
- Validates target positions are within bounds
- Calculates optimal path and distance
- Simulates movement with realistic timing
- Reports actual final position and statistics
- Handles edge cases and errors

### **📖 Background**

In warehouse robots, navigation services are critical for:
- Moving to pickup locations
- Traveling to delivery stations
- Returning to docking/charging stations
- Positioning for precise operations

This exercise simulates realistic navigation with path planning, speed control, and position tracking.

### **Step 1: Create Navigation Server**

Create `navigate_to_position_server.py` in `ce_robot/ce_robot/`:

```python
#!/usr/bin/env python3
"""
Exercise 1: Robot Navigation Service Server
Moves robot to target positions with validation
"""

import rclpy
import math
import time
from rclpy.node import Node
from ce_robot_interfaces.srv import NavigateToPosition


class NavigationServer(Node):
    def __init__(self):
        super().__init__('navigation_server')
        
        self.srv = self.create_service(
            NavigateToPosition,
            'navigate_to_position',
            self.navigate_callback
        )
        
        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Workspace bounds (meters)
        self.workspace_min_x = -10.0
        self.workspace_max_x = 10.0
        self.workspace_min_y = -10.0
        self.workspace_max_y = 10.0
        
        # Robot specifications
        self.default_max_speed = 1.0  # m/s
        self.max_speed_limit = 2.0    # m/s
        
        self.navigation_count = 0
        
        self.get_logger().info('🗺️  Navigation Server started')
        self.get_logger().info(f'   Current position: ({self.current_x}, {self.current_y})')
        self.get_logger().info(f'   Workspace: X[{self.workspace_min_x}, {self.workspace_max_x}], '
                             f'Y[{self.workspace_min_y}, {self.workspace_max_y}]')

    def validate_position(self, x, y):
        """Check if position is within workspace bounds"""
        return (self.workspace_min_x <= x <= self.workspace_max_x and
                self.workspace_min_y <= y <= self.workspace_max_y)

    def calculate_distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def navigate_callback(self, request, response):
        """Handle navigation request"""
        self.navigation_count += 1
        
        target_x = request.target_x
        target_y = request.target_y
        target_yaw = request.target_yaw
        max_speed = request.max_speed if request.max_speed > 0 else self.default_max_speed
        precise = request.precise_positioning
        
        self.get_logger().info(
            f'\n📍 Navigation Request #{self.navigation_count}:\n'
            f'   From: ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_yaw:.2f})\n'
            f'   To: ({target_x:.2f}, {target_y:.2f}, {target_yaw:.2f})\n'
            f'   Max Speed: {max_speed:.2f} m/s | Precise: {precise}'
        )
        
        # Validate target position
        if not self.validate_position(target_x, target_y):
            response.success = False
            response.final_x = self.current_x
            response.final_y = self.current_y
            response.final_yaw = self.current_yaw
            response.distance_traveled = 0.0
            response.time_elapsed = 0.0
            response.message = f'❌ Target position out of bounds!'
            
            self.get_logger().error(response.message)
            return response
        
        # Validate speed
        if max_speed > self.max_speed_limit:
            max_speed = self.max_speed_limit
            self.get_logger().warn(f'⚠️  Speed limited to {self.max_speed_limit} m/s')
        
        # Calculate distance and estimated time
        distance = self.calculate_distance(
            self.current_x, self.current_y,
            target_x, target_y
        )
        
        # Simulate navigation time (distance / speed + rotation time)
        rotation_time = abs(target_yaw - self.current_yaw) * 0.5  # 0.5 sec per radian
        travel_time = distance / max_speed
        total_time = travel_time + rotation_time
        
        # Add precision time if precise positioning enabled
        if precise:
            total_time += 1.0  # Extra second for precise positioning
        
        self.get_logger().info(
            f'🚀 Navigating {distance:.2f}m | ETA: {total_time:.1f}s'
        )
        
        # Simulate navigation (in real robot, this would be actual movement)
        time.sleep(min(total_time / 10, 2.0))  # Scaled-down simulation time
        
        # Update robot position (simulate successful navigation)
        self.current_x = target_x
        self.current_y = target_y
        self.current_yaw = target_yaw
        
        # Prepare response
        response.success = True
        response.final_x = self.current_x
        response.final_y = self.current_y
        response.final_yaw = self.current_yaw
        response.distance_traveled = distance
        response.time_elapsed = float(total_time)
        response.message = f'✅ Navigation complete! Traveled {distance:.2f}m in {total_time:.1f}s'
        
        self.get_logger().info(
            f'✅ Arrived at ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_yaw:.2f})'
        )
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NavigationServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n🗺️  Navigation Server shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **Step 2: Create Navigation Client**

Create `navigate_to_position_client.py` in `ce_robot/ce_robot/`:

```python
#!/usr/bin/env python3
"""
Exercise 1: Robot Navigation Service Client
Sends navigation commands to robot
Usage: ros2 run ce_robot navigate_client <x> <y> [yaw] [speed] [precise]
"""

import sys
import rclpy
from rclpy.node import Node
from ce_robot_interfaces.srv import NavigateToPosition


class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')

    def navigate_to(self, x, y, yaw=0.0, max_speed=1.0, precise=False):
        """Send navigation request"""
        
        client = self.create_client(NavigateToPosition, 'navigate_to_position')
        
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Waiting for navigation service...')
        
        request = NavigateToPosition.Request()
        request.target_x = x
        request.target_y = y
        request.target_yaw = yaw
        request.max_speed = max_speed
        request.precise_positioning = precise
        
        self.get_logger().info(
            f'🎯 Requesting navigation to ({x}, {y}, {yaw:.2f}) '
            f'@ {max_speed} m/s'
        )
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 3:
        print('Usage: navigate_client <x> <y> [yaw] [speed] [precise]')
        print('Example: navigate_client 5.0 3.0')
        print('Example: navigate_client 5.0 3.0 1.57 1.5 true')
        sys.exit(1)
    
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
        speed = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
        precise = sys.argv[5].lower() == 'true' if len(sys.argv) > 5 else False
    except ValueError:
        print('Error: Invalid input values')
        sys.exit(1)
    
    node = NavigationClient()
    response = node.navigate_to(x, y, yaw, speed, precise)
    
    if response.success:
        node.get_logger().info(
            f'\n✅ Navigation Success!\n'
            f'   Final Position: ({response.final_x:.2f}, {response.final_y:.2f}, {response.final_yaw:.2f})\n'
            f'   Distance: {response.distance_traveled:.2f}m\n'
            f'   Time: {response.time_elapsed:.1f}s\n'
            f'   Message: {response.message}'
        )
    else:
        node.get_logger().error(
            f'\n❌ Navigation Failed!\n'
            f'   Message: {response.message}'
        )
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **Step 3: Setup and Build**

**Make files executable:**
```bash
chmod +x navigate_to_position_server.py
chmod +x navigate_to_position_client.py
```

**Update setup.py** in `ce_robot/setup.py`:
```python
entry_points={
    'console_scripts': [
        # Previous entries...
        '04_navigate_server = ce_robot.navigate_to_position_server:main',
        '04_navigate_client = ce_robot.navigate_to_position_client:main',
    ],
},
```

**Build:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

### **Testing Exercise 1**

**Terminal 1 - Server:**
```bash
ros2 run ce_robot 04_navigate_server
```

**Terminal 2 - Client (Basic navigation):**
```bash
ros2 run ce_robot 04_navigate_client 5.0 3.0
```

**Expected Output (Server):**
```
[INFO] [navigation_server]: 🗺️  Navigation Server started
[INFO] [navigation_server]:    Current position: (0.0, 0.0)
[INFO] [navigation_server]:    Workspace: X[-10.0, 10.0], Y[-10.0, 10.0]

[INFO] [navigation_server]: 
📍 Navigation Request #1:
   From: (0.00, 0.00, 0.00)
   To: (5.00, 3.00, 0.00)
   Max Speed: 1.00 m/s | Precise: False
[INFO] [navigation_server]: 🚀 Navigating 5.83m | ETA: 5.8s
[INFO] [navigation_server]: ✅ Arrived at (5.00, 3.00, 0.00)
```

**Expected Output (Client):**
```
[INFO] [navigation_client]: 🎯 Requesting navigation to (5.0, 3.0, 0.00) @ 1.0 m/s
[INFO] [navigation_client]: 
✅ Navigation Success!
   Final Position: (5.00, 3.00, 0.00)
   Distance: 5.83m
   Time: 5.8s
   Message: ✅ Navigation complete! Traveled 5.83m in 5.8s
```

**Test with precise positioning:**
```bash
ros2 run ce_robot 04_navigate_client -2.5 4.0 1.57 1.5 true
```

**Test out-of-bounds (should fail):**
```bash
ros2 run ce_robot 04_navigate_client 15.0 20.0
```

### **💡 Key Learning Points**

- **Position validation** - Check workspace bounds before navigation
- **Distance calculation** - Euclidean distance for path planning
- **Speed limiting** - Safety constraints on maximum velocity
- **Time estimation** - Calculate realistic navigation duration
- **State tracking** - Maintain current robot position
- **Error handling** - Graceful failure for invalid requests
- **Professional logging** - Clear status messages with icons

### ✅ Completion Checklist - Exercise 1

- [ ] Created NavigateToPosition.srv
- [ ] Built ce_robot_interfaces successfully
- [ ] Created navigate_to_position_server.py
- [ ] Created navigate_to_position_client.py
- [ ] Updated setup.py with entry points
- [ ] Built ce_robot package successfully
- [ ] Server starts and listens for requests
- [ ] Client successfully connects to service
- [ ] Position validation works (rejects out-of-bounds)
- [ ] Distance calculation is correct
- [ ] Speed limiting enforced
- [ ] Precise positioning mode works
- [ ] State tracking maintains robot position

---

## **Exercise 2: Gripper Control Service 🤖**

### **📝 Task**

Create a gripper control service for pick-and-place operations with object detection.

### 🎯 Objective
Implement a realistic gripper service that:
- Opens and closes the gripper
- Sets precise gripper positions
- Applies controlled grip force
- Detects object presence
- Validates commands and parameters
- Reports actual gripper state

### **📖 Background**

In warehouse robots, gripper control is essential for:
- Picking items from shelves
- Placing items in bins/containers
- Handling objects of different sizes
- Detecting successful grasps

This exercise simulates realistic gripper control with position control, force management, and object detection.

### **Step 1: Create Gripper Server**

Create `gripper_control_server.py` in `ce_robot/ce_robot/`:

```python
#!/usr/bin/env python3
"""
Exercise 2: Gripper Control Service Server
Controls robot gripper for pick and place operations
"""

import rclpy
import time
import random
from rclpy.node import Node
from ce_robot_interfaces.srv import GripperCommand


class GripperControlServer(Node):
    def __init__(self):
        super().__init__('gripper_control_server')
        
        self.srv = self.create_service(
            GripperCommand,
            'gripper_command',
            self.gripper_callback
        )
        
        # Gripper state
        self.current_position = 1.0  # 1.0 = fully open
        self.current_force = 0.0
        self.object_in_gripper = False
        
        # Gripper specifications
        self.min_position = 0.0   # Fully closed
        self.max_position = 1.0   # Fully open
        self.max_force = 1.0      # Maximum grip force
        
        self.command_count = 0
        
        self.get_logger().info('🤖 Gripper Control Server started')
        self.get_logger().info(f'   Current state: Position={self.current_position:.2f}, Force={self.current_force:.2f}')

    def simulate_object_detection(self, position):
        """Simulate object detection based on gripper position"""
        # Object detected if gripper is somewhat closed (0.2-0.6 range)
        if 0.2 <= position <= 0.6:
            return random.choice([True, True, False])  # 66% detection rate
        return False

    def gripper_callback(self, request, response):
        """Handle gripper command"""
        self.command_count += 1
        
        command = request.command
        position = request.position
        force = request.force
        check_object = request.check_object
        
        self.get_logger().info(
            f'\n🤖 Gripper Command #{self.command_count}:\n'
            f'   Command: {command} (1=Open, 2=Close, 3=SetPosition)\n'
            f'   Position: {position:.2f} | Force: {force:.2f}\n'
            f'   Check Object: {check_object}'
        )
        
        # Validate command
        if command not in [1, 2, 3]:
            response.success = False
            response.object_detected = False
            response.actual_position = self.current_position
            response.actual_force = self.current_force
            response.message = '❌ Invalid command! Use 1=Open, 2=Close, 3=SetPosition'
            self.get_logger().error(response.message)
            return response
        
        # Process command
        if command == 1:  # Open
            target_position = 1.0
            self.get_logger().info('📂 Opening gripper...')
            
        elif command == 2:  # Close
            target_position = 0.0
            self.get_logger().info('📁 Closing gripper...')
            
        elif command == 3:  # Set Position
            # Validate position
            if not (0.0 <= position <= 1.0):
                response.success = False
                response.object_detected = False
                response.actual_position = self.current_position
                response.actual_force = self.current_force
                response.message = f'❌ Invalid position: {position}. Must be 0.0-1.0'
                self.get_logger().error(response.message)
                return response
            
            target_position = position
            self.get_logger().info(f'🎯 Setting gripper to position {target_position:.2f}...')
        
        # Validate force
        if not (0.0 <= force <= 1.0):
            force = min(max(force, 0.0), 1.0)
            self.get_logger().warn(f'⚠️  Force clamped to {force:.2f}')
        
        # Simulate movement time
        movement_distance = abs(target_position - self.current_position)
        movement_time = movement_distance * 2.0  # 2 seconds per full range
        time.sleep(min(movement_time / 5, 1.0))  # Scaled simulation
        
        # Update gripper state
        self.current_position = target_position
        self.current_force = force if target_position < 0.5 else 0.0  # Force only when closing
        
        # Check for object if requested
        if check_object:
            self.object_in_gripper = self.simulate_object_detection(self.current_position)
        else:
            self.object_in_gripper = False
        
        # Prepare response
        response.success = True
        response.object_detected = self.object_in_gripper
        response.actual_position = float(self.current_position)
        response.actual_force = float(self.current_force)
        
        if self.object_in_gripper:
            response.message = f'✅ Gripper command complete! Object detected at position {self.current_position:.2f}'
        else:
            response.message = f'✅ Gripper at position {self.current_position:.2f}'
        
        self.get_logger().info(
            f'✅ Complete: Position={self.current_position:.2f}, '
            f'Force={self.current_force:.2f}, Object={self.object_in_gripper}'
        )
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GripperControlServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n🤖 Gripper Control Server shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **Step 2: Create Gripper Client**

Create `gripper_control_client.py` in `ce_robot/ce_robot/`:

```python
#!/usr/bin/env python3
"""
Exercise 2: Gripper Control Service Client
Sends gripper commands
Usage: ros2 run ce_robot gripper_client <command> [position] [force] [check_object]
Commands: 1=Open, 2=Close, 3=SetPosition
"""

import sys
import rclpy
from rclpy.node import Node
from ce_robot_interfaces.srv import GripperCommand


class GripperClient(Node):
    def __init__(self):
        super().__init__('gripper_client')

    def send_command(self, command, position=0.5, force=0.5, check_object=True):
        """Send gripper command"""
        
        client = self.create_client(GripperCommand, 'gripper_command')
        
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Waiting for gripper service...')
        
        request = GripperCommand.Request()
        request.command = command
        request.position = position
        request.force = force
        request.check_object = check_object
        
        cmd_names = {1: "OPEN", 2: "CLOSE", 3: "SET_POSITION"}
        self.get_logger().info(
            f'🤖 Sending gripper command: {cmd_names.get(command, "UNKNOWN")}'
        )
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print('Usage: gripper_client <command> [position] [force] [check_object]')
        print('Commands:')
        print('  1 = Open gripper')
        print('  2 = Close gripper')
        print('  3 = Set position (requires position 0.0-1.0)')
        print('Examples:')
        print('  gripper_client 1              # Open')
        print('  gripper_client 2              # Close')
        print('  gripper_client 3 0.5 0.8 true # Set to 50%, force 80%, check object')
        sys.exit(1)
    
    try:
        command = int(sys.argv[1])
        position = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
        force = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
        check_object = sys.argv[4].lower() == 'true' if len(sys.argv) > 4 else True
    except ValueError:
        print('Error: Invalid input values')
        sys.exit(1)
    
    node = GripperClient()
    response = node.send_command(command, position, force, check_object)
    
    if response.success:
        node.get_logger().info(
            f'\n✅ Gripper Command Success!\n'
            f'   Position: {response.actual_position:.2f}\n'
            f'   Force: {response.actual_force:.2f}\n'
            f'   Object Detected: {response.object_detected}\n'
            f'   Message: {response.message}'
        )
    else:
        node.get_logger().error(
            f'\n❌ Gripper Command Failed!\n'
            f'   Message: {response.message}'
        )
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **Step 3: Setup and Build**

**Make files executable:**
```bash
chmod +x gripper_control_server.py
chmod +x gripper_control_client.py
```

**Update setup.py** in `ce_robot/setup.py`:
```python
entry_points={
    'console_scripts': [
        # Previous entries...
        '04_navigate_server = ce_robot.navigate_to_position_server:main',
        '04_navigate_client = ce_robot.navigate_to_position_client:main',
        '04_gripper_server = ce_robot.gripper_control_server:main',
        '04_gripper_client = ce_robot.gripper_control_client:main',
    ],
},
```

**Build:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

### **Testing Exercise 2**

**Terminal 1 - Server:**
```bash
ros2 run ce_robot 04_gripper_server
```

**Terminal 2 - Client (Open gripper):**
```bash
ros2 run ce_robot 04_gripper_client 1
```

**Expected Output (Server):**
```
[INFO] [gripper_control_server]: 🤖 Gripper Control Server started
[INFO] [gripper_control_server]:    Current state: Position=1.00, Force=0.00

[INFO] [gripper_control_server]: 
🤖 Gripper Command #1:
   Command: 1 (1=Open, 2=Close, 3=SetPosition)
   Position: 0.50 | Force: 0.50
   Check Object: True
[INFO] [gripper_control_server]: 📂 Opening gripper...
[INFO] [gripper_control_server]: ✅ Complete: Position=1.00, Force=0.00, Object=False
```

**Expected Output (Client):**
```
[INFO] [gripper_client]: 🤖 Sending gripper command: OPEN
[INFO] [gripper_client]: 
✅ Gripper Command Success!
   Position: 1.00
   Force: 0.00
   Object Detected: False
   Message: ✅ Gripper at position 1.00
```

**Test closing with object detection:**
```bash
ros2 run ce_robot 04_gripper_client 2
```

**Test custom position:**
```bash
ros2 run ce_robot 04_gripper_client 3 0.4 0.7 true
```

**Expected Output (with object detected):**
```
[INFO] [gripper_control_server]: 🤖 Gripper Command #3:
   Command: 3 (1=Open, 2=Close, 3=SetPosition)
   Position: 0.40 | Force: 0.70
   Check Object: True
[INFO] [gripper_control_server]: 🎯 Setting gripper to position 0.40...
[INFO] [gripper_control_server]: ✅ Complete: Position=0.40, Force=0.70, Object=True

[gripper_client]: ✅ Gripper Command Success!
   Position: 0.40
   Force: 0.70
   Object Detected: True
   Message: ✅ Gripper command complete! Object detected at position 0.40
```

### **💡 Key Learning Points**

- **Command validation** - Check command types before execution
- **Position control** - Precise gripper positioning (0.0-1.0 range)
- **Force management** - Apply appropriate grip force
- **Object detection** - Simulate object presence sensing
- **State management** - Track gripper position and force
- **Error handling** - Validate ranges and return error messages
- **Professional logging** - Clear command and status reporting

### ✅ Completion Checklist - Exercise 2

- [ ] Created GripperCommand.srv
- [ ] Built ce_robot_interfaces successfully
- [ ] Created gripper_control_server.py
- [ ] Created gripper_control_client.py
- [ ] Updated setup.py with entry points
- [ ] Built ce_robot package successfully
- [ ] Server starts and listens for requests
- [ ] Open command works
- [ ] Close command works
- [ ] Set position command works
- [ ] Position validation works (rejects <0 or >1)
- [ ] Force validation and clamping works
- [ ] Object detection simulation works
- [ ] State tracking maintains gripper state

---

## **📂 Final Directory Structure**

```
📁 ROS2_WS/
├── 📁 src/
│   ├── 📁 ce_robot_interfaces/
│   │   ├── 📁 srv/
│   │   │   ├── 📄 CalRectangle.srv           # From Readme
│   │   │   ├── 📄 NavigateToPosition.srv     # Lab Exercise 1
│   │   │   └── 📄 GripperCommand.srv          # Lab Exercise 2
│   │   ├── 📄 package.xml
│   │   └── 📄 CMakeLists.txt
│   └── 📁 ce_robot/
│       ├── 📁 ce_robot/
│       │   ├── 📄 __init__.py
│       │   ├── 🐍 CalRect_server.py                    # From Readme
│       │   ├── 🐍 CalRect_client.py                    # From Readme
│       │   ├── 🐍 navigate_to_position_server.py       # Exercise 1
│       │   ├── 🐍 navigate_to_position_client.py       # Exercise 1
│       │   ├── 🐍 gripper_control_server.py            # Exercise 2
│       │   └── 🐍 gripper_control_client.py            # Exercise 2
│       ├── 📄 package.xml
│       ├── 📄 setup.cfg
│       └── 📄 setup.py
└── 📁 install/
```

---

## **🔍 Useful ROS 2 Commands**

### Service Inspection
```bash
# List all active services
ros2 service list

# View service type
ros2 service type /navigate_to_position

# View service definition
ros2 interface show ce_robot_interfaces/srv/NavigateToPosition
ros2 interface show ce_robot_interfaces/srv/GripperCommand

# Call service from command line
ros2 service call /navigate_to_position ce_robot_interfaces/srv/NavigateToPosition "{target_x: 5.0, target_y: 3.0, target_yaw: 0.0, max_speed: 1.0, precise_positioning: false}"

ros2 service call /gripper_command ce_robot_interfaces/srv/GripperCommand "{command: 1, position: 0.5, force: 0.5, check_object: true}"

# View service information
ros2 service info /navigate_to_position

# Visualize service connections
rqt_graph

# List all nodes
ros2 node list

# Get node information
ros2 node info /navigation_server
ros2 node info /gripper_control_server
```

---

## **✅ Complete Lab Checklist**

**Setup:**
- [ ] ce_robot_interfaces package exists
- [ ] All .srv files created
- [ ] CMakeLists.txt updated with all services
- [ ] Services built successfully
- [ ] All service interfaces verified

**Exercise 1: Navigation Service**
- [ ] NavigateToPosition.srv defined
- [ ] Server implemented with validation
- [ ] Client implemented with command-line args
- [ ] Position bounds checking works
- [ ] Distance calculation correct
- [ ] Speed limiting enforced
- [ ] State tracking maintains position
- [ ] All tests passed

**Exercise 2: Gripper Control**
- [ ] GripperCommand.srv defined
- [ ] Server implemented with state management
- [ ] Client implemented with commands
- [ ] Open/Close commands work
- [ ] Set position command works
- [ ] Position/force validation works
- [ ] Object detection simulation works
- [ ] All tests passed

**Integration:**
- [ ] All services built successfully
- [ ] All clients connect and receive responses
- [ ] Command-line service calls work
- [ ] rqt_graph shows proper connections
- [ ] All error cases handled gracefully

---

## **💡 Tips & Tricks**

1. **Always source setup.bash before running:**
   ```bash
   source ~/.bashrc
   ```

2. **Rebuild services when .srv files change:**
   ```bash
   colcon build --packages-select ce_robot_interfaces
   source install/setup.bash
   ```

3. **Test services from command line before running clients:**
   ```bash
   ros2 service call /service_name package/srv/ServiceType "{field: value}"
   ```

4. **Use rqt_graph to visualize connections:**
   ```bash
   rqt_graph
   ```

5. **Monitor service calls in real-time:**
   ```bash
   ros2 run rqt_service_caller rqt_service_caller
   ```

6. **Always wait for service availability before calling:**
   ```python
   while not client.wait_for_service(timeout_sec=1.0):
       self.get_logger().info('Waiting for service...')
   ```

7. **Use type conversion for service fields:**
   ```python
   response.actual_position = float(self.current_position)
   response.command = int(request.command)
   ```

---

## **🎓 What You've Learned**

### Service Design
✅ Designing services for robot command and control  
✅ Creating request/response structures for real operations  
✅ Choosing appropriate data types for robot parameters  
✅ Organizing service fields logically  
✅ Adding validation and error handling  

### Robot Operations
✅ Navigation service with position validation  
✅ Gripper control with state management  
✅ Object detection simulation  
✅ Command validation and safety checks  
✅ State tracking across service calls  

### ROS 2 Best Practices
✅ Professional service patterns  
✅ Error handling and validation  
✅ Clear logging with status messages  
✅ Command-line argument parsing  
✅ Service client patterns  

---

## **📚 Additional Resources**

- [ROS 2 Services Documentation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
- [ROS 2 Custom Interfaces](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [Robot Navigation Concepts](https://navigation.ros.org/)
- [Gripper Control Systems](https://robotics.stackexchange.com/questions/tagged/gripper)

---

## **🚀 Next Steps**

Continue building your robot control system:

1. **Integrate** navigation and gripper services for pick-and-place
2. **Add** mission planning service (multi-step operations)
3. **Create** charging dock service (autonomous charging)
4. **Implement** inventory management service
5. **Build** fleet coordination for multiple robots

---

**🎉 Congratulations! You've created a professional robot control service system!** 🚀✨

You now have:
- ✅ Navigation service for robot movement
- ✅ Gripper service for object manipulation
- ✅ Complete warehouse robot control foundation
- ✅ Professional service patterns and validation

**Keep building!** 💪🤖
    def __init__(self):
        super().__init__('shape_analyzer_server')
        
        self.srv = self.create_service(
            ShapeAnalyzer,
            'analyze_shape',
            self.analyze_shape_callback
        )
        
        self.shape_count = 0
        self.get_logger().info('Shape Analyzer Server started')
        self.get_logger().info('Service: /analyze_shape')

    def analyze_shape_callback(self, request, response):
        """Analyze geometric shape properties"""
        self.shape_count += 1
        
        shape_type = request.shape_type
        param1 = request.param1
        param2 = request.param2
        param3 = request.param3
        
        self.get_logger().info(
            f'Request #{self.shape_count}: Type={shape_type}, '
            f'Params=({param1}, {param2}, {param3})'
        )
        
        if shape_type == 1:  # Circle
            response.shape_name = 'Circle'
            response.area = math.pi * param1**2
            response.perimeter = 2 * math.pi * param1
            response.properties = f'Radius: {param1}'
        
        elif shape_type == 2:  # Triangle
            response.shape_name = 'Triangle'
            # Using Heron's formula
            s = (param1 + param2 + param3) / 2
            area = math.sqrt(s * (s - param1) * (s - param2) * (s - param3))
            response.area = area
            response.perimeter = param1 + param2 + param3
            response.properties = (
                f'Sides: {param1}, {param2}, {param3} | '
                f'Semi-perimeter: {s}'
            )
        
        elif shape_type == 3:  # Ellipse
            response.shape_name = 'Ellipse'
            response.area = math.pi * param1 * param2
            # Approximate perimeter using Ramanujan's formula
            h = ((param1 - param2)**2) / ((param1 + param2)**2)
            perimeter = math.pi * (param1 + param2) * (
                1 + (3*h) / (10 + math.sqrt(4 - 3*h))
            )
            response.perimeter = perimeter
            response.properties = (
                f'Semi-major axis: {param1}, '
                f'Semi-minor axis: {param2}'
            )
        
        else:
            self.get_logger().warn(f'Unknown shape type: {shape_type}')
            response.shape_name = 'Unknown'
            response.area = 0
            response.perimeter = 0
            response.properties = 'Invalid shape type'
        
        self.get_logger().info(
            f'{response.shape_name}: '
            f'Area={response.area:.4f}, '
            f'Perimeter={response.perimeter:.4f}'
        )
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ShapeAnalyzerServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **File: shape_analyzer_client.py**

```python
#!/usr/bin/env python3
"""
Exercise 3: Shape Analyzer Client
Usage: ros2 run ce_robot shape_analyzer_client <shape_type> <param1> [param2] [param3]
Types: 1=Circle (r), 2=Triangle (a,b,c), 3=Ellipse (a,b)
"""

import sys
import rclpy
from rclpy.node import Node
from ce_robot_interfaces.srv import ShapeAnalyzer


class ShapeAnalyzerClient(Node):
    def __init__(self):
        super().__init__('shape_analyzer_client')

    def analyze_shape(self, shape_type, param1, param2=0, param3=0):
        """Request shape analysis"""
        
        client = self.create_client(ShapeAnalyzer, 'analyze_shape')
        
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        
        request = ShapeAnalyzer.Request()
        request.shape_type = shape_type
        request.param1 = param1
        request.param2 = param2
        request.param3 = param3
        
        self.get_logger().info('Analyzing shape...')
        future = client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 3:
        print('Usage: shape_analyzer_client <shape_type> <param1> [param2] [param3]')
        print('Types:')
        print('  1 = Circle (requires: radius)')
        print('  2 = Triangle (requires: side1 side2 side3)')
        print('  3 = Ellipse (requires: semi_major semi_minor)')
        print('Examples:')
        print('  shape_analyzer_client 1 5')
        print('  shape_analyzer_client 2 3 4 5')
        print('  shape_analyzer_client 3 5 3')
        sys.exit(1)
    
    try:
        shape_type = int(sys.argv[1])
        param1 = float(sys.argv[2])
        param2 = float(sys.argv[3]) if len(sys.argv) > 3 else 0
        param3 = float(sys.argv[4]) if len(sys.argv) > 4 else 0
    except ValueError:
        print('Error: Invalid input values')
        sys.exit(1)
    
    node = ShapeAnalyzerClient()
    response = node.analyze_shape(shape_type, param1, param2, param3)
    
    node.get_logger().info(f'\n=== Shape Analysis Results ===')
    node.get_logger().info(f'Shape: {response.shape_name}')
    node.get_logger().info(f'Area: {response.area:.4f}')
    node.get_logger().info(f'Perimeter: {response.perimeter:.4f}')
    node.get_logger().info(f'Properties: {response.properties}')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **Testing Exercise 3**

**Terminal 1 - Server:**
```bash
ros2 run ce_robot shape_analyzer_server
```

**Terminal 2 - Client (Circle with radius 5):**
```bash
ros2 run ce_robot shape_analyzer_client 1 5
```

**Expected Output:**
```
[INFO] [shape_analyzer_server]: Request #1: Type=1, Params=(5, 0, 0)
[INFO] [shape_analyzer_server]: Circle: Area=78.5398, Perimeter=31.4159

[INFO] [shape_analyzer_client]: === Shape Analysis Results ===
[INFO] [shape_analyzer_client]: Shape: Circle
[INFO] [shape_analyzer_client]: Area: 78.5398
[INFO] [shape_analyzer_client]: Perimeter: 31.4159
[INFO] [shape_analyzer_client]: Properties: Radius: 5
```

**Terminal 2 - Client (Triangle with sides 3, 4, 5):**
```bash
ros2 run ce_robot shape_analyzer_client 2 3 4 5
```

**Expected Output:**
```
[INFO] [shape_analyzer_server]: Request #2: Type=2, Params=(3, 4, 5)
[INFO] [shape_analyzer_server]: Triangle: Area=6.0000, Perimeter=12.0000

[INFO] [shape_analyzer_client]: === Shape Analysis Results ===
[INFO] [shape_analyzer_client]: Shape: Triangle
[INFO] [shape_analyzer_client]: Area: 6.0000
[INFO] [shape_analyzer_client]: Perimeter: 12.0000
[INFO] [shape_analyzer_client]: Properties: Sides: 3, 4, 5 | Semi-perimeter: 6
```

### **Key Concepts**

- Complex service definitions with multiple parameters
- Conditional shape-specific calculations
- Mathematical formulas (Heron's, ellipse approximation)
- Rich response data structures
- Request/response logging and metrics

---

## **Commands to Practice**

```bash
# List all active services
ros2 service list

# View service type
ros2 service type /cal_rect

# View service definition
ros2 interface show ce_robot_interfaces/srv/CalRectangle

# Call service from command line
ros2 service call /cal_rect ce_robot_interfaces/srv/CalRectangle "{length: 5.5, width: 3.2}"

# View service information
ros2 service info /cal_rect

# Visualize service connections
rqt_graph

# List all nodes
ros2 node list

# Get node information
ros2 node info /calrect_server
```

---

## **✅ Completion Checklist**

- [ ] Exercise 1: Rectangle Calculator completed
  - [ ] CalRectangle.srv defined
  - [ ] Server runs successfully
  - [ ] Client connects and receives responses
  - [ ] Input validation working

- [ ] Exercise 2: Distance Calculator completed
  - [ ] DistanceCalculator.srv defined
  - [ ] Euclidean distance calculation correct
  - [ ] Manhattan distance calculation correct
  - [ ] Multiple request types handled

- [ ] Exercise 3: Shape Analyzer completed
  - [ ] ShapeAnalyzer.srv defined
  - [ ] Circle analysis working
  - [ ] Triangle analysis working
  - [ ] Ellipse analysis working
  - [ ] Complex response structures handled

- [ ] All services built successfully
- [ ] All clients connect and receive responses
- [ ] Command-line service calls work
- [ ] rqt_graph shows proper connections
- [ ] All error cases handled gracefully

---

## **💡 Tips & Tricks**

1. **Always source setup.bash before running:**
   ```bash
   source ~/.bashrc
   ```

2. **Rebuild services when .srv files change:**
   ```bash
   colcon build --packages-select ce_robot_interfaces
   source install/setup.bash
   ```

3. **Test services from command line before running clients:**
   ```bash
   ros2 service call /service_name package/srv/ServiceType "{field: value}"
   ```

4. **Use rqt_graph to visualize connections:**
   ```bash
   rqt_graph
   ```

5. **Monitor service calls in real-time:**
   ```bash
   ros2 service call /service_name --verbose
   ```

6. **Always wait for service availability before calling:**
   ```python
   while not client.wait_for_service(timeout_sec=1.0):
       self.get_logger().info('Waiting for service...')
   ```

---

**🎓 Congratulations! You've completed the ROS 2 Custom Service Lab!** 🚀✨
