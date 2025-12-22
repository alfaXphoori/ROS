# **🎯 ROS 2 Launch Files Lab Exercises**

Build advanced launch configurations with conditional logic, namespaces, remappings, and event handlers - taking your robot system automation to the next level.

---

## **📌 Project Title**

Advanced Launch File Techniques for Production Robot Systems

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🛠 Lab Overview**

This lab extends the concepts from **Readme.md** with advanced launch file techniques used in production robot systems. You'll learn conditional launching, namespace management, topic remapping, event handlers, and YAML-based configuration - skills essential for deploying complex multi-robot fleets.

**Prerequisites:** Complete `07_Launch/Readme.md` first! This lab builds on simple_launch.py and robot_system_launch.py

**Duration:** ~1.5 hours
**Level:** Intermediate to Advanced
**What You Already Know:** Basic launch files, arguments, multiple nodes

---

## **\ud83c\udfed Real-World Robot Scenarios**

These exercises simulate actual robot deployments:

**\ud83d\udce6 Amazon Fulfillment Center:**
- 50+ robots working simultaneously
- Different types: pickers, transporters, sorters
- Battery monitoring and auto-charging
- Zone-based task assignment

**\ud83c\udfed Manufacturing Floor:**
- Heavy-duty transport (1500kg pallets)
- Safety-critical navigation systems
- 24/7 operation with fault tolerance
- Automatic recovery from failures

**\ud83d\udecd\ufe0f Grocery Warehouse:**
- Compact robots for narrow aisles
- Temperature-sensitive cargo
- Real-time inventory tracking
- Multi-robot coordination

Each exercise teaches techniques used in these real deployments!

---

## **🎯 Learning Objectives**

By completing this lab, you will master:

- ✅ **Conditional node launching** based on arguments
- ✅ **Namespace management** for multi-robot systems
- ✅ **Topic/service remapping** to avoid conflicts
- ✅ **Event handlers** for node failure recovery
- ✅ **YAML configuration** for complex parameter sets
- ✅ **Launch file composition** (including other launch files)
- ✅ **Environment variable** configuration
- ✅ **Group actions** for organized node management
- ✅ **Launch testing** and validation strategies

---

## **📊 Lab Architecture**

### **System Overview**

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         WAREHOUSE ROBOT FLEET SYSTEM                    │
│                    (Multi-Robot Launch Management)                      │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                    ┌───────────────┼───────────────┐
                    ▼               ▼               ▼
        ┌──────────────────┐ ┌──────────────┐ ┌──────────────────┐
        │  Exercise 1      │ │ Exercise 2   │ │  Exercise 3      │
        │  Conditional     │ │ Multi-Robot  │ │  Event Handlers  │
        │  Launch Logic    │ │ Namespaces   │ │  & Monitoring    │
        └──────────────────┘ └──────────────┘ └──────────────────┘
                    │               │               │
                    └───────────────┼───────────────┘
                                    ▼
                        ┌──────────────────────┐
                        │    Exercise 4        │
                        │  YAML Config &       │
                        │  Composition         │
                        └──────────────────────┘
```

### **Exercise 1: Conditional Launch (30 min)**

```
Launch Arguments:
├── enable_publisher (true/false)
├── enable_service (true/false)
├── enable_action (true/false)
└── robot_mode (production/development/simulation)

Conditional Nodes:
┌────────────────────────────────────────────────────┐
│ IfCondition(enable_publisher)                      │
│   ├─> robot_tag_publisher                          │
│   │    ├─ Topic: /robot_tag (RobotTag)             │
│   │    └─ Rate: 2.0 Hz                             │
│   │    └─ Realistic warehouse parameters           │
│   │                                                │
│ IfCondition(enable_service)                        │
│   ├─> navigation_service                           │
│   │    └─ Service: /navigation_path                │
│   │    └─ NavigationPath (obstacle avoidance)      │
│   │                                                │
│ IfCondition(enable_action)                         │
│   ├─> task_queue_action                            │
│   │    └─ Action: /pick_items                      │
│   │    └─ PickItems (order fulfillment)            │
│   │                                                │
│ UnlessCondition(robot_mode == 'production')        │
│   └─> debug_monitor                                │
│        └─ System diagnostics with robot tracking   │
└────────────────────────────────────────────────────┘

Real-World Scenario: Amazon Fulfillment Center
├─ 20 robots with cameras (scanner mode)
├─ 30 robots without cameras (transport mode)
└─ 2 robots in development mode (testing)
```

### **Exercise 2: Multi-Robot Namespaces (35 min)**

```
Warehouse Floor Layout:
┌──────────────────────────────────────────────────────────────────────┐
│                        WAREHOUSE ZONES                               │
├──────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ZONE A: Loading Dock              ZONE B: Picking Area              │
│  ┌─────────────────────────────┐   ┌──────────────────────────────┐  │
│  │   robot1/                   │   │  robot2/                     │  │
│  │   ├─ robot_tag_publisher    │   |  ├─ robot_tag_publisher      │  │
│  │   │   └─ /robot1/robot_tag  │   |  │   └─ /robot2/robot_tag    │  │
│  │   ├─ navigation_service     │   |  └─ task_queue_action        │  │
│  │   │   └─ /robot1/navigation_path │ │    └─ /robot2/pick_items  │  │
│  │   └─ AMR-TRANSPORT-HEAVY-001│   |  └─ AMR-PICKER-LIGHT-002     │  │
│  │       Payload: 1000kg       │   |      Payload: 50kg           |  │
│  │       Battery: 78%          │   |      Battery: 18% (LOW!)     │  │
│  └─────────────────────────────┘   └──────────────────────────────┘  │
│                                                                      │
│  ZONE C: Sorting Station                                             │
│  ┌─────────────────────────────────────┐                             │
│  │   robot3/                           │                             │
│  │   ├─ robot_tag_publisher            │                             │
│  │   │   └─ /robot3/robot_tag          │                             │
│  │   ├─ navigation_service             │                             │
│  │   │   └─ /robot3/navigation_path    │                             │
│  │   ├─ task_queue_action              │                             │
│  │   │   └─ /robot3/pick_items         │                             │
│  │   └─ AMR-DELIVERY-MULTI-003         │                             │
│  │       Payload: 300kg                │                             │ 
│  │       Battery: 92%                  │                             │
│  └─────────────────────────────────────┘                             │
│                                                                      │
└──────────────────────────────────────────────────────────────────────┘

Namespace Isolation:
/robot1/robot_tag  ──┐
/robot2/robot_tag  ──┼──> No topic conflicts!
/robot3/robot_tag  ──┘

/robot1/navigation_path  ──┐
/robot3/navigation_path  ──┼──> No service conflicts!
```

### **Exercise 3: Event Handlers & Monitoring (35 min)**

```
Node Lifecycle & Fault Tolerance:
┌───────────────────────────────────────────────────────────┐
│                      NODE MONITORING SYSTEM               │
├───────────────────────────────────────────────────────────┤
│                                                           │
│  ┌─────────────────────────────────────────────────────┐  │
│  │  robot_tag_publisher (Battery Monitor)              │  │
│  │  ┌───────────────┐                                  │  │
│  │  │ OnProcessStart│──> ✅ "Publisher started"        │  │
│  │  └───────────────┘                                  │  │
│  │  ┌───────────────┐                                  │  │
│  │  │ OnProcessExit │──> ⚠️  "Publisher crashed!"      │  │
│  │  └───────────────┘        │                         │  │
│  │                           ├─> Wait 3 seconds        │  │
│  │                           └─> Auto-restart node     │  │
│  └─────────────────────────────────────────────────────┘  │
│                                                           │
│  ┌─────────────────────────────────────────────────────┐  │
│  │  navigation_service (CRITICAL - Path Planning)      │  │
│  │  ┌───────────────┐                                  │  │
│  │  │ OnProcessStart│──> ✅ "Navigation ready"         │  │
│  │  └───────────────┘                                  │  │
│  │  ┌───────────────┐                                  │  │
│  │  │ OnProcessExit │──> ❌ "CRITICAL FAILURE!"        │  │
│  │  └───────────────┘        │                         │  │
│  │                           └─> EmitEvent(Shutdown)   │  │
│  │                               [STOP ALL ROBOTS]     │  │
│  └─────────────────────────────────────────────────────┘  │
│                                                           │
│  ┌─────────────────────────────────────────────────────┐  │
│  │  task_queue_action (Non-critical Order Processing)  │  │
│  │  ┌──────────────┐                                   │  │
│  │  │ OnProcessExit│──> ⚠️  "Order processor exited"   │  │
│  │  └──────────────┘        │                          │  │
│  │                           └─> Log only (continue)   │  │
│  └─────────────────────────────────────────────────────┘  │
│                                                           │
└───────────────────────────────────────────────────────────┘

Failure Scenarios:
├─ Battery Monitor crashes  → Auto-restart (3 sec delay)
├─ Camera freezes           → Auto-restart (recalibrate)
├─ WiFi disconnects         → Auto-reconnect
└─ Navigation fails         → SHUTDOWN ALL (safety critical!)
```

### **Progressive Complexity**

```
Readme.md (Foundation)              This Lab (Advanced)
─────────────────────────           ────────────────────────
simple_launch.py                    Exercise 1
├─ 1 node                           ├─ Conditional logic
└─ Fixed parameters                 ├─ IfCondition/UnlessCondition
                                    └─ Runtime decisions
         │
         ├────────────────────────> Exercise 2
         │                          ├─ Multiple robot instances
robot_system_launch.py              ├─ Namespace isolation
├─ 3 nodes                          └─ Topic/service remapping
├─ Launch arguments                 
└─ Parameter passing                Exercise 3
                                    ├─ Process monitoring
                                    ├─ Automatic restart
                                    └─ Fault tolerance

                                    Exercise 4
                                    ├─ External YAML configs
                                    ├─ Launch composition
                                    └─ Multi-site deployment
```

---

## **📚 Learning Path Overview**

| Exercise | Title | Level | Duration | Focus |
|----------|-------|-------|----------|-------|
| 1 | Conditional Launch Logic | Intermediate | 30 min | IfCondition, UnlessCondition |
| 2 | Multi-Robot Namespaces | Intermediate | 35 min | Namespaces, remapping |
| 3 | Event Handlers & Monitoring | Advanced | 35 min | OnProcessExit, RegisterEventHandler |
| 4 | YAML Config & Composition | Advanced | 20 min | External configs, IncludeLaunchDescription |

---

## **Exercise 1: Conditional Launch Logic (Intermediate) 🔀**

### **📋 Objective**

Learn to launch nodes conditionally based on arguments - essential for flexible system configurations (e.g., simulation vs. real robot, different sensor configurations).

### **🎯 What You'll Learn**

- Use `IfCondition` and `UnlessCondition`
- Create flexible launch files with optional nodes
- Handle boolean arguments properly
- Build production-ready configuration systems

### **💡 Real-World Use Case**

**Scenario:** Amazon-style fulfillment center with 50+ robots. During peak hours, you need:
- **Camera-equipped robots** for barcode scanning (20 units)
- **Standard transport robots** without cameras (30 units)  
- **Development/test robots** with debug logging enabled (2 units)

Your launch file must adapt based on `robot_type` argument:
- `robot_type:=scanner` → Enables camera node
- `robot_type:=transport` → No camera, max speed
- `robot_mode:=development` → Debug logging, slower speeds, test sensors

This prevents deploying wrong configurations and ensures safety!

> **📦 Reference Files Available:** All complete files for this exercise (interfaces, nodes, and launch file) are available in `07_Launch/src/exercise_1/` for reference. You can use these as examples or copy them to your workspace.

### **📝 Step 1: Create Custom Interface Files**

First, create the custom service and action interfaces that our nodes will use:

**Create NavigationPath service:**
```bash
cd ~/ros2_ws/src/ce_robot_interfaces/srv
touch NavigationPath.srv
```

Add the following content to `NavigationPath.srv`:


**Create PickItems action:**
```bash
cd ~/ros2_ws/src/ce_robot_interfaces/action
touch PickItems.action
```

Add the following content to `PickItems.action`:


**Update CMakeLists.txt for interfaces:**
```bash
cd ~/ros2_ws/src/ce_robot_interfaces
```

Edit `CMakeLists.txt` and add these interfaces to the `rosidl_generate_interfaces` section:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  # ... existing interfaces ...
  "srv/NavigationPath.srv"
  "action/PickItems.action"
  DEPENDENCIES builtin_interfaces
)
```

**Build the interfaces:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
source install/setup.bash
```

### **📝 Step 2: Create Support Python Files**

Create the three Python node files that the launch file will use:

```bash
cd ~/ros2_ws/src/ce_robot_launch/launch
touch navigation_service.py task_queue_action.py debug_monitor.py
chmod +x navigation_service.py task_queue_action.py debug_monitor.py
```

Copy the node implementations from the `07_Launch/src/exercise_1/nodes/` directory:
- `navigation_service.py` - Navigation path planning service
- `task_queue_action.py` - Order picking action server  
- `debug_monitor.py` - System diagnostics monitor

Or create them manually with the code from the files in `exercise_1/nodes/`.

**Update setup.py (for Python packages):**

```python
    entry_points={
        'console_scripts': [
            "07_navigate_service = ce_robot.navigate_service:main",
            "07_task_queue_action = ce_robot.task_queue_action:main",
            "07_debug_monitor = ce_robot.debug_monitor:main",
        ],
    },
```
---

### **🧪 Test Individual Nodes**

**Test Navigation Service (standalone):**
```bash
# Terminal 1: Run the navigation service
ros2 run ce_robot 07_navigate_service --ros-args \
  -p robot_id:=AMR-TEST-001 \
  -p robot_type:=transport \
  -p zone_id:=TEST-ZONE \
  -p max_payload_kg:=500.0 \
  -p safety_margin_m:=0.5

# Terminal 2: Call the service
ros2 service call /navigation_path ce_robot_interfaces/srv/NavigationPath \
  "{obstacle_length: 2.0, obstacle_width: 1.5, robot_x: 5.0, robot_y: 3.0, \
    safety_margin: 0.5, zone_id: 'TEST-ZONE'}"

# Expected response: safe_area, clearances, can_navigate status, recommended_action
```

**Test Task Queue Action Server (standalone):**
```bash
# Terminal 1: Run the action server
ros2 run ce_robot 07_task_queue_action --ros-args \
  -p robot_id:=AMR-PICKER-001 \
  -p robot_type:=picker \
  -p zone_id:=PICKING-ZONE \
  -p max_items_per_trip:=20 \
  -p battery_level:=85.0 \
  -p picking_speed_items_per_min:=12.0

# Terminal 2: Send an action goal
ros2 action send_goal /pick_items ce_robot_interfaces/action/PickItems \
  "{target_items: 5, time_per_item: 5.0, order_id: 'ORD-12345', \
    zone_id: 'PICKING-ZONE', priority: 8, max_weight_kg: 25.0}" \
  --feedback

# Expected: Progress updates with items picked, battery consumed, elapsed time
# Note: priority is an integer (1=low, 10=urgent), not a string
```

**Test Debug Monitor (standalone):**
```bash
# Terminal 1: Run the debug monitor
ros2 run ce_robot 07_debug_monitor --ros-args \
  -p robot_id:=AMR-DEBUG-001 \
  -p robot_type:=test \
  -p zone_id:=DEBUG-LAB \
  -p diagnostic_rate_hz:=0.5 \
  -p enable_network_check:=true \
  -p enable_ros_diagnostics:=true

# Monitor will print diagnostic reports every 2 seconds
# Check CPU, memory, disk usage, network status, and issue counts
```

---

### **📝 Step 3: Create Conditional Launch File**

Create the conditional launch file in your launch package:

```bash
cd ~/ros2_ws/src/ce_robot_launch/launch
touch conditional_robot_launch.py
chmod +x conditional_robot_launch.py
```

Open the file in your editor and add the following code:

**Location:** `~/ros2_ws/src/ce_robot_launch/launch/conditional_robot_launch.py`

```python
#!/usr/bin/env python3
"""
Exercise 1: Conditional Launch
Launches nodes based on boolean conditions
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with conditional nodes"""
    
    # Declare arguments
    enable_publisher_arg = DeclareLaunchArgument(
        'enable_publisher',
        default_value='true',
        description='Enable robot tag publisher',
        choices=['true', 'false']
    )
    
    enable_service_arg = DeclareLaunchArgument(
        'enable_service',
        default_value='true',
        description='Enable rectangle calculation service',
        choices=['true', 'false']
    )
    
    enable_action_arg = DeclareLaunchArgument(
        'enable_action',
        default_value='false',
        description='Enable count action server',
        choices=['true', 'false']
    )
    
    robot_mode_arg = DeclareLaunchArgument(
        'robot_mode',
        default_value='production',
        description='Robot operation mode',
        choices=['production', 'development', 'simulation']
    )
    
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='transport',
        description='Robot type: transport (heavy duty), picker (light duty), sorter (medium duty)',
        choices=['transport', 'picker', 'sorter']
    )
    
    # Get configurations
    enable_publisher = LaunchConfiguration('enable_publisher')
    enable_service = LaunchConfiguration('enable_service')
    enable_action = LaunchConfiguration('enable_action')
    robot_mode = LaunchConfiguration('robot_mode')
    robot_type = LaunchConfiguration('robot_type')
    
    # Conditional publisher (launches only if enabled)
    # Realistic warehouse robot with battery monitoring and position tracking
    publisher_node = Node(
        package='ce_robot',
        executable='05_robot_tag_param',
        name='robot_tag_publisher',
        output='screen',
        parameters=[
            {'robot_id': 'AMR-WH-A-001'},  # Autonomous Mobile Robot, Warehouse A, Unit 001
            {'robot_type': robot_type},  # Dynamic robot type from argument
            {'zone_id': 'WAREHOUSE-A-DOCK-3'},
            {'fleet_number': 1},
            {'tag_publish_rate': 2.0},
            {'max_payload_kg': 500.0},
            {'current_location': 'DOCK-3-BAY-12'},
            {'assigned_task': 'TRANSPORT-TO-ZONE-B'},
            {'priority_level': 7},  # High priority for urgent deliveries
            {'battery_level': 85.0},  # Battery percentage
            {'operation_hours': 142.5},  # Hours in service
        ],
        condition=IfCondition(enable_publisher)
    )
    
    # Conditional service - Navigation path planning (launches only if enabled)
    # Real-world: Calculate safe paths around obstacles with safety margins
    service_node = Node(
        package='ce_robot',
        executable='07_navigate_service',
        name='navigation_service',
        output='screen',
        parameters=[
            {'robot_id': 'AMR-WH-A-001'},
            {'robot_type': robot_type},
            {'zone_id': 'WAREHOUSE-A-DOCK-3'},
            {'max_payload_kg': 500.0},
            {'safety_margin_m': 0.5},
        ],
        condition=IfCondition(enable_service)
    )
    
    # Conditional action - Order picking task queue (launches only if enabled)
    # Real-world: Process warehouse orders with battery and weight management
    action_node = Node(
        package='ce_robot',
        executable='07_task_queue_action',
        name='task_queue_action',
        output='screen',
        parameters=[
            {'robot_id': 'AMR-WH-A-001'},
            {'robot_type': 'picker'},  # Picker optimized for order fulfillment
            {'zone_id': 'WAREHOUSE-A-PICKING-ZONE'},
            {'max_items_per_trip': 50},
            {'battery_level': 100.0},
            {'picking_speed_items_per_min': 12.0},
        ],
        condition=IfCondition(enable_action)
    )
    
    # Debug monitor - Launches UNLESS in production mode
    # Real-world: System diagnostics with CPU, memory, disk monitoring during development
    debug_node = Node(
        package='ce_robot',
        executable='07_debug_monitor',
        name='debug_monitor',
        output='screen',
        parameters=[
            {'robot_id': 'AMR-DEV-001'},
            {'robot_type': 'development'},
            {'zone_id': 'TEST-LAB'},
            {'diagnostic_rate_hz': 0.2},  # Every 5 seconds
            {'enable_network_check': True},
            {'enable_ros_diagnostics': True},
        ],
        condition=UnlessCondition(
            PythonExpression(["'", robot_mode, "' == 'production'"])
        )
    )
    
    # Startup log with mode and type information
    startup_log = LogInfo(
        msg=['🚀 Starting robot system in [', robot_mode, '] mode with type [', robot_type, ']'],
    )
    
    return LaunchDescription([
        # Arguments
        enable_publisher_arg,
        enable_service_arg,
        enable_action_arg,
        robot_mode_arg,
        robot_type_arg,
        
        # Startup log
        startup_log,
        
        # Conditional nodes
        publisher_node,
        service_node,
        action_node,
        debug_node,
    ])
```

### **🧪 Step 4: Build and Test**

**Build both packages:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces ce_robot_launch --symlink-install
source install/setup.bash
```

**Verify interfaces are available:**
```bash
ros2 interface show ce_robot_interfaces/srv/NavigationPath
ros2 interface show ce_robot_interfaces/action/PickItems
```

### **🧪 Testing Exercise 1**

**Test 1 - All nodes enabled (default):**
```bash
ros2 launch ce_robot_launch conditional_robot_launch.py
```

**Expected:** Publisher ✅, Service ✅, Action ❌, Debug ❌

**Test 2 - Enable action server:**
```bash
ros2 launch ce_robot_launch conditional_robot_launch.py enable_action:=true
```

**Expected:** Publisher ✅, Service ✅, Action ✅, Debug ❌

**Test 3 - Disable publisher:**
```bash
ros2 launch ce_robot_launch conditional_robot_launch.py enable_publisher:=false
```

**Expected:** Publisher ❌, Service ✅, Action ❌, Debug ❌

**Test 4 - Development mode (includes debug):**
```bash
ros2 launch ce_robot_launch conditional_robot_launch.py robot_mode:=development
```

**Expected:** Publisher ✅, Service ✅, Action ❌, Debug ✅

**Verify running nodes:**
```bash
ros2 node list
```

---

### **💡 Key Concepts Learned**

1. **IfCondition** - Launch node only if condition is true
2. **UnlessCondition** - Launch node only if condition is false
3. **PythonExpression** - Evaluate Python expressions for complex conditions
4. **LogInfo** - Print messages during launch
5. **choices** parameter - Validate argument values

### **🎯 Challenge: Battery-Aware Robot Deployment**

**Task:** Modify the launch file to check battery level and adjust behavior:

```python
# If battery < 20%: Launch with reduced speed and enable charging node
# If battery 20-50%: Normal operation but skip heavy tasks
# If battery > 50%: Full performance, enable all features

battery_level_arg = DeclareLaunchArgument(
    'battery_level',
    default_value='100.0',
    description='Current battery level (0-100)'
)

# Use PythonExpression to enable charging node when battery < 20
charging_required = PythonExpression([
    "float('", LaunchConfiguration('battery_level'), "') < 20.0"
])

charging_node = Node(
    package='ce_robot',
    executable='battery_charging_client',
    condition=IfCondition(charging_required)
)
```

**Real benefit:** Prevents robot from accepting heavy transport tasks when battery is low, automatically routes to charging station.

---

## **Exercise 2: Multi-Robot with Namespaces (Intermediate) 🤖🤖**

### **📋 Objective**

Learn to launch multiple instances of the same node without conflicts using namespaces and remapping - critical for multi-robot fleets.

### **🎯 What You'll Learn**

- Use namespaces to isolate robot instances
- Remap topics to prevent collisions
- Configure multiple robots with different parameters
- Handle node name uniqueness

### **💡 Real-World Use Case**

**Scenario:** Warehouse has 3 zones (Loading, Picking, Sorting), each needs its own robot:
- **Robot 1 (Loading Dock):** Heavy transport, handles pallets up to 1000kg
- **Robot 2 (Picking Area):** Light picker, needs frequent charging cycles
- **Robot 3 (Sorting Station):** Multi-function, handles urgent deliveries

Each robot:
- Publishes to its own topic: `/robot1/robot_tag`, `/robot2/robot_tag`, `/robot3/robot_tag`
- Has isolated services: `/robot1/cal_rect`, `/robot2/count_until`
- Can be monitored independently without interference
- Reports battery level, location, and assigned tasks

Namespaces prevent collisions when 50+ robots operate simultaneously!

> **📦 Reference Files Available:** All complete files for this exercise (interfaces, nodes, and launch file) are available in `07_Launch/src/exercise_2/` for reference. You can use these as examples or copy them to your workspace.

### **📝 Step 1: Create Custom Interface Files**

First, create the custom interfaces for multi-robot fleet management:

**Create RobotStatusLaunch message:**
```bash
cd ~/ros2_ws/src/ce_robot_interfaces/msg
touch RobotStatusLaunch.msg
```

Add the following content to `RobotStatusLaunch.msg`:

**Create TaskAssignment service:**
```bash
cd ~/ros2_ws/src/ce_robot_interfaces/srv
touch TaskAssignment.srv
```

Add the following content to `TaskAssignment.srv`:


**Create FleetCommand message:**
```bash
cd ~/ros2_ws/src/ce_robot_interfaces/msg
touch FleetCommand.msg
```

Add the following content to `FleetCommand.msg`:

**Update CMakeLists.txt for interfaces:**
```bash
cd ~/ros2_ws/src/ce_robot_interfaces
```

Edit `CMakeLists.txt` and add these interfaces to the `rosidl_generate_interfaces` section:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  # ... existing interfaces ...
  "msg/RobotStatusLaunch.msg"
  "msg/FleetCommand.msg"
  "srv/TaskAssignment.srv"
  DEPENDENCIES builtin_interfaces
)
```

**Build the interfaces:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
source install/setup.bash
```

**Verify interfaces are available:**
```bash
ros2 interface show ce_robot_interfaces/msg/RobotStatusLaunch
ros2 interface show ce_robot_interfaces/srv/TaskAssignment
ros2 interface show ce_robot_interfaces/msg/FleetCommand
```

---

### **📝 Step 2: Create Multi-Robot Node Files**

Create the three Python node files for the multi-robot system:

```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
touch robot_status_publisher.py zone_coordinator_service.py fleet_monitor.py
chmod +x robot_status_publisher.py zone_coordinator_service.py fleet_monitor.py
```

Copy the node implementations from the `07_Launch/src/exercise_2/nodes/` directory:
- `robot_status_publisher.py` - Namespace-aware robot status publisher
- `zone_coordinator_service.py` - Zone-based task assignment service
- `fleet_monitor.py` - Central fleet monitoring with system metrics

Or create them manually with the code from the files in `exercise_2/nodes/`.

**Update setup.py:**

Edit `~/ros2_ws/src/ce_robot/setup.py` and add the new executables:

```python
entry_points={
    'console_scripts': [
        # ... existing entries ...
        "07_robot_status = ce_robot.robot_status_publisher:main",
        "07_zone_coordinator = ce_robot.zone_coordinator_service:main",
        "07_fleet_monitor = ce_robot.fleet_monitor:main",
    ],
},
```

---

### **🧪 Test Individual Nodes**

**Test Robot Status Publisher:**
```bash
# Terminal 1: Run the robot status publisher
ros2 run ce_robot 07_robot_status --ros-args \
  -p robot_id:=AMR-TEST-001 \
  -p robot_type:=transport \
  -p zone_id:=TEST-ZONE \
  -p fleet_number:=1 \
  -p max_payload_kg:=500.0 \
  -p battery_level:=85.0 \
  -p current_location:=TEST-LOCATION \
  -p assigned_task:=TEST-TASK \
  -p status_rate_hz:=1.0

# Terminal 2: Monitor the status
ros2 topic echo /robot_status

# Expected: RobotStatusLaunch messages with robot info, battery level, location
```

**Test Zone Coordinator Service:**
```bash
# Terminal 1: Run the zone coordinator service
ros2 run ce_robot 07_zone_coordinator --ros-args \
  -p robot_id:=AMR-TEST-002 \
  -p zone_id:=WAREHOUSE-A \
  -p robot_type:=picker \
  -p max_capacity:=50.0

# Terminal 2: Call the service to request a task
ros2 service call /request_task example_interfaces/srv/SetBool "{data: true}"

# Expected: Task assignment response with task type and zone information
```

**Test Fleet Monitor (Standalone - Limited Functionality):**
```bash
# Terminal 1: Run the fleet monitor
ros2 run ce_robot 07_fleet_monitor --ros-args \
  -p fleet_id:=TEST-FLEET \
  -p num_robots:=3 \
  -p monitor_rate_hz:=0.5

# Terminal 2: Monitor the fleet status output
ros2 topic echo /fleet_status

# Expected: Fleet monitoring reports every 2 seconds with:
# - Fleet ID and robot count
# - System metrics (CPU, Memory, Disk usage)
# - Fleet uptime
# Note: Robot-specific data will be empty since no robots are running
```

**Important:** The fleet monitor is designed to aggregate data from multiple robot namespaces. For full functionality testing, see Step 4 where it monitors the complete multi-robot system with all three robots running.

---

### **📝 Step 3: Create Multi-Robot Launch File

Create the multi-robot launch file in your launch package:

```bash
cd ~/ros2_ws/src/ce_robot_launch/launch
touch multi_robot_launch.py
chmod +x multi_robot_launch.py
```

Open the file in your editor and add the following code:

### **📁 File: multi_robot_launch.py**

**Location:** `~/ros2_ws/src/ce_robot_launch/launch/multi_robot_launch.py`

```python
#!/usr/bin/env python3
"""
Exercise 2: Multi-Robot Launch with Namespaces
Launches 3 robot instances with isolated namespaces using custom Exercise 2 nodes
Real-world scenario: Multi-zone warehouse with independent robot operations
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """Generate launch description for multi-robot system with namespace isolation"""
    
    # Declare arguments
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='3',
        description='Number of robots to launch (1-3)'
    )
    
    fleet_id_arg = DeclareLaunchArgument(
        'fleet_id',
        default_value='WAREHOUSE-FLEET-A',
        description='Fleet identifier'
    )
    
    # Fleet Monitor - Central monitoring node (no namespace, monitors all robots)
    fleet_monitor_node = Node(
        package='ce_robot',
        executable='07_fleet_monitor',
        name='fleet_monitor',
        output='screen',
        parameters=[
            {'fleet_id': LaunchConfiguration('fleet_id')},
            {'num_robots': LaunchConfiguration('num_robots')},
            {'monitor_rate_hz': 0.5},
        ],
    )
    
    # Robot 1: Heavy transport robot in loading dock
    # Real-world: Handles pallets and heavy cargo, high priority operations
    robot1_group = GroupAction([
        PushRosNamespace('robot1'),
        Node(
            package='ce_robot',
            executable='07_robot_status',
            name='robot_status_publisher',
            output='screen',
            parameters=[
                {'robot_id': 'AMR-TRANSPORT-HEAVY-001'},
                {'robot_type': 'transport'},
                {'zone_id': 'WAREHOUSE-A-LOADING-DOCK'},
                {'fleet_number': 1},
                {'max_payload_kg': 1000.0},  # Heavy-duty transport
                {'current_location': 'DOCK-A-STATION-5'},
                {'assigned_task': 'PALLET-TRANSPORT-TO-STORAGE'},
                {'battery_level': 78.0},
                {'status_rate_hz': 2.0},
            ],
        ),
        Node(
            package='ce_robot',
            executable='07_zone_coordinator',
            name='zone_coordinator',
            output='screen',
            parameters=[
                {'robot_id': 'AMR-TRANSPORT-HEAVY-001'},
                {'zone_id': 'WAREHOUSE-A-LOADING-DOCK'},
                {'robot_type': 'transport'},
                {'max_capacity': 1000.0},
            ],
        ),
    ])
    
    # Robot 2: Picker robot with low battery - returning to charging
    # Real-world: Light picker for order fulfillment, needs charging
    robot2_group = GroupAction([
        PushRosNamespace('robot2'),
        Node(
            package='ce_robot',
            executable='07_robot_status',
            name='robot_status_publisher',
            output='screen',
            parameters=[
                {'robot_id': 'AMR-PICKER-LIGHT-002'},
                {'robot_type': 'picker'},
                {'zone_id': 'WAREHOUSE-B-PICKING-AREA'},
                {'fleet_number': 2},
                {'max_payload_kg': 50.0},  # Light picker
                {'current_location': 'AISLE-B-12-SHELF-3'},
                {'assigned_task': 'RETURN-TO-CHARGING-STATION'},
                {'battery_level': 18.5},  # Low battery - needs charging!
                {'status_rate_hz': 1.5},
            ],
        ),
        Node(
            package='ce_robot',
            executable='07_zone_coordinator',
            name='zone_coordinator',
            output='screen',
            parameters=[
                {'robot_id': 'AMR-PICKER-LIGHT-002'},
                {'zone_id': 'WAREHOUSE-B-PICKING-AREA'},
                {'robot_type': 'picker'},
                {'max_capacity': 50.0},
            ],
        ),
    ])
    
    # Robot 3: Multi-function delivery robot at sorting station
    # Real-world: Medium capacity, active delivery operations
    robot3_group = GroupAction([
        PushRosNamespace('robot3'),
        Node(
            package='ce_robot',
            executable='07_robot_status',
            name='robot_status_publisher',
            output='screen',
            parameters=[
                {'robot_id': 'AMR-DELIVERY-MULTI-003'},
                {'robot_type': 'transport'},
                {'zone_id': 'WAREHOUSE-C-SORTING-STATION'},
                {'fleet_number': 3},
                {'max_payload_kg': 300.0},  # Medium capacity
                {'current_location': 'SORT-C-CONVEYOR-7'},
                {'assigned_task': 'PACKAGE-DELIVERY-ROUTE-12'},
                {'battery_level': 92.0},  # Fully charged, ready for operations
                {'status_rate_hz': 3.0},  # Fast updates for active operations
            ],
        ),
        Node(
            package='ce_robot',
            executable='07_zone_coordinator',
            name='zone_coordinator',
            output='screen',
            parameters=[
                {'robot_id': 'AMR-DELIVERY-MULTI-003'},
                {'zone_id': 'WAREHOUSE-C-SORTING-STATION'},
                {'robot_type': 'transport'},
                {'max_capacity': 300.0},
            ],
        ),
    ])
    
    return LaunchDescription([
        num_robots_arg,
        fleet_id_arg,
        fleet_monitor_node,
        robot1_group,
        robot2_group,
        robot3_group,
    ])
```

### **🧪 Step 4: Build and Test**

**Build both packages:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces ce_robot ce_robot_launch --symlink-install
source install/setup.bash
```

**Verify interfaces:**
```bash
ros2 interface show ce_robot_interfaces/msg/RobotStatusLaunch
ros2 interface show ce_robot_interfaces/srv/TaskAssignment
ros2 interface show ce_robot_interfaces/msg/FleetCommand
```

**Verify node executables:**
```bash
ros2 pkg executables ce_robot | grep 07_
```

---

### **🧪 Testing Exercise 2**

**Launch the multi-robot system:**
```bash
ros2 launch ce_robot_launch multi_robot_launch.py
```

**Test Individual Robot Namespaces:**

**Launch the multi-robot system:**
```bash
ros2 launch ce_robot_launch multi_robot_launch.py
```

**Test Robot 1 nodes:**
```bash
# Check robot1 nodes
ros2 node list | grep robot1

# Monitor robot1 tag publisher
ros2 topic echo /robot1/robot_tag

# Call robot1 rectangle service
ros2 service call /robot1/cal_rect ce_robot_interfaces/srv/CalRectangle \
  "{length: 2.5, width: 1.5}"
```

**Test Robot 2 nodes:**
```bash
# Check robot2 nodes
ros2 node list | grep robot2

# Monitor robot2 tag publisher
ros2 topic echo /robot2/robot_tag

# Send goal to robot2 count action
ros2 action send_goal /robot2/count_until ce_robot_interfaces/action/CountUntil \
  "{target_number: 10, period: 1.0}" --feedback
```

**Test Robot 3 nodes:**
```bash
# Check robot3 nodes
ros2 node list | grep robot3

# Monitor robot3 tag publisher
ros2 topic echo /robot3/robot_tag

# Call robot3 services
ros2 service call /robot3/cal_rect ce_robot_interfaces/srv/CalRectangle \
  "{length: 3.0, width: 2.0}"

ros2 action send_goal /robot3/count_until ce_robot_interfaces/action/CountUntil \
  "{target_number: 5, period: 0.5}" --feedback
```

**Verify namespaced nodes:**
```bash
ros2 node list
```

**Expected output:**
```
/robot1/robot_tag_publisher
/robot1/rect_server
/robot2/robot_tag_publisher
/robot2/count_server
/robot3/robot_tag_publisher
/robot3/rect_server
/robot3/count_server
```

**Check namespaced topics:**
```bash
ros2 topic list
```

**Expected:**
```
/robot1/robot_tag
/robot2/robot_tag
/robot3/robot_tag
```

**Verify no topic conflicts:**
```bash
# All three robots publish to their own topics
ros2 topic info /robot1/robot_tag
ros2 topic info /robot2/robot_tag
ros2 topic info /robot3/robot_tag

# Each should show 1 publisher (no conflicts)
```

**Test Fleet Monitor (monitors all robots):**
```bash
# The fleet monitor is already running from the launch file
# It monitors all robot namespaces and publishes fleet-wide metrics

# Monitor the fleet status output in real-time
ros2 topic echo /fleet_status

# Expected: Reports every 2 seconds with:
# - Fleet ID and active robot count
# - System metrics (CPU, Memory, Disk usage)
# - Fleet uptime and monitoring duration
# - Status from all robot namespaces (robot1, robot2, robot3)
```

**Check fleet monitor node details:**
```bash
# View fleet monitor node information
ros2 node info /fleet_monitor

# Expected subscriptions:
# - /robot1/robot_status
# - /robot2/robot_status  
# - /robot3/robot_status
# Expected publications:
# - /fleet_status (std_msgs/msg/String)
```

**Verify fleet monitor parameters:**
```bash
# Check the fleet monitor configuration
ros2 param list /fleet_monitor

# Get specific parameter values
ros2 param get /fleet_monitor fleet_id
ros2 param get /fleet_monitor num_robots
ros2 param get /fleet_monitor monitor_rate_hz

# Expected output:
# fleet_id: WAREHOUSE-FLEET-A
# num_robots: 3
# monitor_rate_hz: 0.5
```

**Test fleet monitoring with robot status:**
```bash
# In another terminal, monitor individual robot status
ros2 topic echo /robot1/robot_status

# The fleet monitor subscribes to all robot status topics
# and aggregates the information in /fleet_status
# Compare individual robot data with fleet-wide report
```

---

### **💡 Key Concepts Learned**

1. **GroupAction** - Group nodes together for organized management
2. **PushRosNamespace** - Add namespace prefix to all nodes in group
3. **Node isolation** - Same node names in different namespaces don't conflict
4. **Namespace paths** - Access topics/services via `/namespace/name`
5. **Multi-robot systems** - Deploy fleets without code duplication

### **🎯 Challenge: Dynamic Robot Fleet Deployment**

**Task:** Create a launch file that accepts `num_robots` and `robot_ids` arguments to spawn a dynamic fleet:

```python
num_robots_arg = DeclareLaunchArgument(
    'num_robots',
    default_value='5',
    description='Number of robots in fleet'
)

robot_prefix_arg = DeclareLaunchArgument(
    'robot_prefix',
    default_value='AMR-FLEET',
    description='Robot ID prefix (e.g., AMR-FLEET-001, AMR-FLEET-002)'
)

# Advanced: Use OpaqueFunction to generate robot nodes dynamically
def generate_robot_nodes(context):
    num_robots = int(context.launch_configurations['num_robots'])
    prefix = context.launch_configurations['robot_prefix']
    
    nodes = []
    for i in range(1, num_robots + 1):
        namespace = f'robot{i}'
        robot_id = f'{prefix}-{i:03d}'
        # Create node for each robot...
    return nodes
```

**Real benefit:** Deploy 10, 20, or 50 robots with one command. Used in scalable warehouse systems.

---

## **Exercise 3: Event Handlers & Monitoring (Advanced) 🔍**

### **📋 Objective**

Learn to handle node failures, restart crashed nodes, and log system events - essential for production reliability.

### **🎯 What You'll Learn**

- Use `RegisterEventHandler` for process monitoring
- Handle `OnProcessExit` events
- Implement automatic node restart on failure
- Log system events and node state changes
- Build fault-tolerant launch configurations

### **💡 Real-World Use Case**

**Scenario:** Industrial warehouse with 24/7 operations:
- **Battery monitoring system** occasionally crashes due to CAN bus timeouts
- **LiDAR navigation node** may fail during firmware updates
- **Camera driver** can freeze when processing corrupted frames

**Critical failures** (must shutdown entire system):
- Navigation system failure → Robot could collide
- Safety system failure → Emergency stop not working

**Non-critical failures** (auto-restart):
- Camera freeze → Restart driver, continue with other sensors
- WiFi disconnect → Reconnect automatically
- Battery monitor → Restart and recalibrate

Your launch file automatically:
1. Detects when battery monitor crashes
2. Waits 3 seconds (allows hardware reset)
3. Restarts the node with fresh initialization
4. Logs the incident for maintenance team
5. If critical navigation fails → Immediately stops robot for safety

> **📦 Reference Files Available:** All complete files for this exercise are available in `07_Launch/src/exercise_3/` for reference.

### **📝 Step 1: Create Production Monitoring Nodes**

Exercise 3 uses custom production-grade nodes that simulate real warehouse robot systems. These nodes are available in `07_Launch/src/exercise_3/nodes/`:

**Create the node files:**
```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
touch battery_monitor_node.py navigation_controller_node.py task_processor_node.py
chmod +x battery_monitor_node.py navigation_controller_node.py task_processor_node.py
```

Copy the node implementations from `07_Launch/src/exercise_3/nodes/`:
- `battery_monitor_node.py` - Real-time battery monitoring (HIGH criticality)
- `navigation_controller_node.py` - Safety-critical navigation (CRITICAL)
- `task_processor_node.py` - Warehouse task queue (NON-CRITICAL)

**Update setup.py:**

Edit `~/ros2_ws/src/ce_robot/setup.py` and add the new executables:

```python
entry_points={
    'console_scripts': [
        # ... existing entries ...
        '07_battery_monitor = ce_robot.battery_monitor_node:main',
        '07_navigation_controller = ce_robot.navigation_controller_node:main',
        '07_task_processor = ce_robot.task_processor_node:main',
    ],
},
```

**Build the package:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

No additional interface creation required - these nodes use String messages for JSON-formatted data.

---

### **📝 Step 2: Create Production Configuration Files**

For production robot systems, create configuration files for failure tracking, monitoring policies, and systemd integration.

**Create configuration directory:**
```bash
mkdir -p ~/ros2_ws/src/ce_robot_launch/config
cd ~/ros2_ws/src/ce_robot_launch/config
```

Copy configuration files from the `07_Launch/src/exercise_3/config/` directory:
- `failure_counter.py` - Persistent failure tracking with JSON storage
- `robot_monitoring.yaml` - Centralized monitoring configuration
- `robot_monitor.service` - Systemd service for 24/7 operation

Or create them manually with the code from the files in `exercise_3/config/`.

**Create monitoring configuration file:**
```bash
touch robot_monitoring.yaml
```

**Add the following content to `robot_monitoring.yaml`:**

```yaml
# Robot Monitoring Configuration
# Used in production deployments for failure tracking and recovery

monitoring:
  # Failure tracking settings
  failure_log_dir: "/tmp/robot_failures"
  max_restart_attempts: 3
  restart_delay_seconds: 3.0
  
  # Critical nodes (system shuts down on failure)
  critical_nodes:
    - navigation_service
    - safety_system
    - emergency_stop
  
  # Non-critical nodes (auto-restart on failure)
  non_critical_nodes:
    - battery_monitor
    - camera_driver
    - wifi_manager
  
  # Alert thresholds
  alerts:
    consecutive_failures_threshold: 3
    alert_email: "maintenance@warehouse.com"
    alert_sms: "+1-555-0123"
  
  # Health check intervals (seconds)
  health_check:
    battery_monitor: 5.0
    navigation: 1.0
    camera: 10.0
```

**Create failure counter script:**
```bash
touch failure_counter.py
chmod +x failure_counter.py
```

**Add the following content to `failure_counter.py`:**

```python
#!/usr/bin/env python3
"""
Failure Counter for Production Robot Systems
Tracks node failures and manages restart policies
"""

import os
import json
from datetime import datetime

class FailureCounter:
    """Track and manage node failure counts for production robots"""
    
    def __init__(self, log_dir="/tmp/robot_failures"):
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
    
    def get_failure_count(self, node_name):
        """Read failure count from file"""
        filepath = os.path.join(self.log_dir, f"{node_name}_failures.json")
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
                return data.get('count', 0)
        except FileNotFoundError:
            return 0
    
    def increment_failure_count(self, node_name):
        """Increment and save failure count with timestamp"""
        filepath = os.path.join(self.log_dir, f"{node_name}_failures.json")
        count = self.get_failure_count(node_name) + 1
        
        data = {
            'node_name': node_name,
            'count': count,
            'last_failure': datetime.now().isoformat(),
            'robot_id': os.getenv('ROBOT_ID', 'UNKNOWN')
        }
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        
        return count
    
    def reset_failure_count(self, node_name):
        """Reset failure count after successful recovery"""
        filepath = os.path.join(self.log_dir, f"{node_name}_failures.json")
        if os.path.exists(filepath):
            os.remove(filepath)
    
    def log_failure(self, node_name, error_message=""):
        """Log detailed failure information"""
        logfile = os.path.join(self.log_dir, "failure_log.txt")
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        with open(logfile, 'a') as f:
            f.write(f"[{timestamp}] {node_name} FAILED - {error_message}\\n")

# Example usage in launch file
if __name__ == "__main__":
    counter = FailureCounter()
    
    # Simulate battery monitor failure
    count = counter.increment_failure_count("battery_monitor")
    print(f"Battery monitor failed {count} time(s)")
    
    if count >= 3:
        print("⚠️ ALERT: Battery monitor failed 3 times! Manual intervention required.")
        counter.log_failure("battery_monitor", "Persistent CAN bus timeout")
    else:
        print(f"Attempting auto-restart ({count}/3)...")
```

**Create systemd service file for production deployment:**
```bash
touch robot_monitor.service
```

**Add the following content to `robot_monitor.service`:**

```ini
# Systemd service file for production robot monitoring
# Install to: /etc/systemd/system/robot_monitor.service
# Usage:
#   sudo cp robot_monitor.service /etc/systemd/system/
#   sudo systemctl daemon-reload
#   sudo systemctl enable robot_monitor.service
#   sudo systemctl start robot_monitor.service

[Unit]
Description=ROS 2 Robot Monitoring System
After=network.target

[Service]
Type=simple
User=robot
Group=robot
WorkingDirectory=/home/robot/ros2_ws

# Set robot identification
Environment="ROBOT_ID=AMR-PROD-001"
Environment="WAREHOUSE_ZONE=MAIN-FLOOR"
Environment="ROS_DOMAIN_ID=42"

# Source ROS 2 setup
ExecStartPre=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/robot/ros2_ws/install/setup.bash'

# Launch the monitored system
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/robot/ros2_ws/install/setup.bash && ros2 launch ce_robot_launch monitored_system_launch.py'

# Restart policy
Restart=on-failure
RestartSec=10
StartLimitInterval=5min
StartLimitBurst=3

# Logging
StandardOutput=journal
StandardError=journal
SyslogIdentifier=robot_monitor

[Install]
WantedBy=multi-user.target
```

**Verify new nodes are available:**
```bash
ros2 pkg executables ce_robot | grep -E '07_battery_monitor|07_navigation_controller|07_task_processor'
```

**Expected output:**
```
ce_robot 07_battery_monitor
ce_robot 07_navigation_controller
ce_robot 07_task_processor
```

---

### **🧪 Test Individual Nodes**

**Test Battery Monitor Node:**
```bash
# Terminal 1: Run the battery monitor
ros2 run ce_robot 07_battery_monitor --ros-args \
  -p robot_id:=AMR-BATTERY-001 \
  -p battery_capacity_ah:=100.0 \
  -p battery_voltage_nominal:=48.0 \
  -p monitor_rate_hz:=2.0 \
  -p simulate_failure:=false
```

# Terminal 2: Monitor battery status
```bash
ros2 topic echo /battery_status
```

# Expected: JSON-formatted battery data (voltage, current, temperature, SoC)

**Test Navigation Controller Node:**

# Terminal 1: Run the navigation controller
```bash
ros2 run ce_robot 07_navigation_controller --ros-args \
  -p robot_id:=AMR-NAV-001 \
  -p max_speed_ms:=2.5 \
  -p safety_radius_m:=0.75 \
  -p simulate_failure:=false
```

# Terminal 2: Monitor navigation status
```bash
ros2 topic echo /navigation_status
```

# Terminal 3: Start navigation
```bash
ros2 service call /navigation_command example_interfaces/srv/SetBool "{data: true}"
```

# Expected: Real-time position updates, obstacle detection, path planning

**Test Task Processor Node:**

# Terminal 1: Run the task processor
```bash
ros2 run ce_robot 07_task_processor --ros-args \
  -p robot_id:=AMR-TASK-001 \
  -p robot_type:=picker \
  -p max_tasks_per_hour:=50 \
  -p simulate_failure:=false
```

# Terminal 2: Monitor task status
```bash
ros2 topic echo /task_status
```

# Expected: Task queue updates (completed, pending, failed), performance metrics

---

### **📝 Step 3: Create Production Monitoring Launch File**

The launch file from `07_Launch/src/exercise_3/launch/monitored_system_launch.py` demonstrates production-grade event handling with intelligent failure recovery.

**Location:** `~/ros2_ws/src/ce_robot_launch/launch/monitored_system_launch.py`

```python
#!/usr/bin/env python3
"""
Exercise 3: Event Handlers and Production Monitoring Launch File
Implements node failure detection and automatic restart with production-grade failure tracking
Real-world scenario: 24/7 warehouse robot with intelligent failure recovery
"""

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    EmitEvent,
    TimerAction,
    OpaqueFunction
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Import the failure counter (add config directory to path)
config_dir = Path(__file__).parent.parent / 'config'
sys.path.insert(0, str(config_dir))

try:
    from failure_counter import FailureCounter
except ImportError:
    print("⚠️ Warning: failure_counter.py not found. Using basic monitoring.")
    FailureCounter = None


def generate_launch_description():
    """Generate launch description with event monitoring and failure tracking"""
    
    # Initialize failure counter for production monitoring
    failure_counter = FailureCounter() if FailureCounter else None
    
    # Arguments
    enable_auto_restart_arg = DeclareLaunchArgument(
        'enable_auto_restart',
        default_value='true',
        description='Enable automatic node restart on failure'
    )
    
    critical_node_arg = DeclareLaunchArgument(
        'critical_node',
        default_value='navigation',
        description='Critical node that triggers shutdown if it fails',
        choices=['battery', 'navigation', 'task', 'none']
    )
    
    max_restart_attempts_arg = DeclareLaunchArgument(
        'max_restart_attempts',
        default_value='3',
        description='Maximum restart attempts before escalation'
    )
    
    simulate_failures_arg = DeclareLaunchArgument(
        'simulate_failures',
        default_value='false',
        description='Enable simulated failures for testing event handlers',
        choices=['true', 'false']
    )
    
    # Get configurations
    enable_auto_restart = LaunchConfiguration('enable_auto_restart')
    critical_node = LaunchConfiguration('critical_node')
    max_restart_attempts = LaunchConfiguration('max_restart_attempts')
    simulate_failures = LaunchConfiguration('simulate_failures')
    
    # Battery Monitor Node - HIGH criticality (auto-restart)
    battery_monitor_node = Node(
        package='ce_robot',
        executable='07_battery_monitor',
        name='battery_monitor',
        output='screen',
        parameters=[
            {'robot_id': 'AMR-BATTERY-MONITOR-001'},
            {'robot_type': 'transport'},
            {'zone_id': 'WAREHOUSE-MAIN-FLOOR'},
            {'battery_capacity_ah': 100.0},
            {'battery_voltage_nominal': 48.0},
            {'low_battery_threshold': 20.0},
            {'critical_battery_threshold': 10.0},
            {'monitor_rate_hz': 2.0},
            {'simulate_failure': simulate_failures},
        ],
    )
    
    # Navigation Controller Node - CRITICAL (shutdown on failure)
    navigation_controller_node = Node(
        package='ce_robot',
        executable='07_navigation_controller',
        name='navigation_controller',
        output='screen',
        parameters=[
            {'robot_id': 'AMR-NAV-CONTROLLER-001'},
            {'robot_type': 'transport'},
            {'zone_id': 'WAREHOUSE-MAIN-FLOOR'},
            {'max_speed_ms': 2.5},
            {'safety_radius_m': 0.75},
            {'lidar_range_m': 20.0},
            {'status_rate_hz': 1.0},
            {'simulate_failure': simulate_failures},
        ],
    )
    
    # Task Processor Node - NON-CRITICAL (log only)
    task_processor_node = Node(
        package='ce_robot',
        executable='07_task_processor',
        name='task_processor',
        output='screen',
        parameters=[
            {'robot_id': 'AMR-TASK-PROCESSOR-001'},
            {'robot_type': 'picker'},
            {'zone_id': 'WAREHOUSE-PICKING-AREA'},
            {'max_tasks_per_hour': 50},
            {'process_rate_hz': 0.5},
            {'simulate_failure': simulate_failures},
        ],
    )
    
    # Event handler: Log when battery monitor starts
    battery_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=battery_monitor_node,
            on_start=[
                LogInfo(msg='✅ Battery Monitor started successfully'),
            ]
        )
    )
    
    # Event handler: Battery monitor exit with intelligent failure tracking
    def handle_battery_exit(event, context):
        """Handle battery monitor exit with failure counting and escalation"""
        if failure_counter:
            node_name = 'battery_monitor'
            count = failure_counter.increment_failure_count(node_name)
            failure_counter.log_failure(node_name, "Battery monitoring system crashed")
            
            if count >= 3:
                return [
                    LogInfo(msg=f'❌ CRITICAL: Battery monitor failed {count} times! Manual intervention required.'),
                    LogInfo(msg='⚠️ Alerting maintenance team...'),
                    LogInfo(msg='📧 Email sent to: maintenance@warehouse.com'),
                    LogInfo(msg='📱 SMS alert sent to on-call engineer'),
                    EmitEvent(event=Shutdown(reason=f'Persistent node failure: {node_name}'))
                ]
            else:
                return [
                    LogInfo(msg=f'⚠️ Battery Monitor exited! Attempt {count}/3. Restarting in 3 seconds...'),
                    LogInfo(msg=f'📊 Failure logged with robot tracking ID'),
                    TimerAction(
                        period=3.0,
                        actions=[
                            Node(
                                package='ce_robot',
                                executable='07_battery_monitor',
                                name='battery_monitor',
                                output='screen',
                                parameters=[
                                    {'robot_id': f'AMR-BATTERY-MONITOR-RESTART-{count}'},
                                    {'robot_type': 'transport'},
                                    {'zone_id': 'WAREHOUSE-RECOVERY'},
                                    {'battery_capacity_ah': 100.0},
                                    {'monitor_rate_hz': 2.0},
                                    {'simulate_failure': False},  # Don't simulate on restart
                                ],
                            ),
                        ]
                    ),
                ]
        else:
            # Fallback without failure counter
            return [
                LogInfo(msg='⚠️ Battery Monitor exited! Attempting restart in 3 seconds...'),
                TimerAction(
                    period=3.0,
                    actions=[
                        Node(
                            package='ce_robot',
                            executable='07_battery_monitor',
                            name='battery_monitor',
                            output='screen',
                            parameters=[
                                {'robot_id': 'AMR-BATTERY-MONITOR-RESTART'},
                                {'robot_type': 'transport'},
                                {'monitor_rate_hz': 2.0},
                            ],
                        ),
                    ]
                ),
            ]
    
    battery_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=battery_monitor_node,
            on_exit=[OpaqueFunction(function=handle_battery_exit)]
        )
    )
    
    # Event handler: Navigation controller start
    navigation_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=navigation_controller_node,
            on_start=[
                LogInfo(msg='✅ Navigation Controller started successfully'),
                LogInfo(msg='🗺️ Safety systems online - collision avoidance active'),
            ]
        )
    )
    
    # Event handler: CRITICAL navigation controller failure
    def handle_navigation_exit(event, context):
        """Handle critical navigation controller failure - immediate shutdown"""
        if failure_counter:
            node_name = 'navigation_controller'
            failure_counter.log_failure(
                node_name, 
                "CRITICAL: Navigation controller crashed - SAFETY RISK!"
            )
        
        return [
            LogInfo(msg='❌ CRITICAL: Rectangle Service failed! Shutting down system...'),
            LogInfo(msg='🚨 SAFETY ALERT: Navigation compromised - emergency stop initiated'),
            EmitEvent(event=Shutdown(reason='Critical service failure - safety system')),
        ]
    
    service_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=service_node,
            on_exit=[OpaqueFunction(function=handle_service_exit)]
        )
    )
    
    # Event handler: Log action server lifecycle
    action_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=action_node,
            on_start=[
                LogInfo(msg='✅ Count Action Server started successfully'),
            ]
        )
    )
    
    action_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=action_node,
            on_exit=[
                LogInfo(msg='⚠️  Count Action Server exited (non-critical)'),
            ]
        )
    )
    
    return LaunchDescription([
        # Arguments
        enable_auto_restart_arg,
        critical_node_arg,
        max_restart_attempts_arg,
        
        return [
            LogInfo(msg='❌ CRITICAL: Navigation Controller failed!'),
            LogInfo(msg='🚨 SAFETY ALERT: Collision avoidance compromised'),
            LogInfo(msg='🛑 EMERGENCY STOP: Shutting down all robot systems'),
            LogInfo(msg='📞 Emergency contact: Safety Team +1-800-ROBOT-911'),
            EmitEvent(event=Shutdown(reason='Critical safety system failure - navigation')),
        ]
    
    navigation_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=navigation_controller_node,
            on_exit=[OpaqueFunction(function=handle_navigation_exit)]
        )
    )
    
    # Event handler: Task processor lifecycle (non-critical)
    task_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=task_processor_node,
            on_start=[
                LogInfo(msg='✅ Task Processor started successfully'),
            ]
        )
    )
    
    task_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=task_processor_node,
            on_exit=[
                LogInfo(msg='⚠️ Task Processor exited (non-critical)'),
                LogInfo(msg='📋 Order processing paused - robot continues safety operations'),
            ]
        )
    )
    
    return LaunchDescription([
        # Arguments
        enable_auto_restart_arg,
        critical_node_arg,
        max_restart_attempts_arg,
        simulate_failures_arg,
        
        # Startup messages
        LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),
        LogInfo(msg='🔍 Production Robot Monitoring System Starting...'),
        LogInfo(msg=f'📊 Failure tracking: {config_dir}/failure_counter.py'),
        LogInfo(msg='⚙️ Max restart attempts: 3'),
        LogInfo(msg='🚨 Critical nodes: navigation_controller'),
        LogInfo(msg='🔄 Auto-restart enabled: battery_monitor'),
        LogInfo(msg='📋 Non-critical: task_processor'),
        LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),
        
        # Nodes
        battery_monitor_node,
        navigation_controller_node,
        task_processor_node,
        
        # Event handlers
        battery_start_handler,
        battery_exit_handler,
        navigation_start_handler,
        navigation_exit_handler,
        task_start_handler,
        task_exit_handler,
    ])
```

**Key Features of Production-Grade Monitoring:**

1. **Differentiated Criticality Levels:**
   - **CRITICAL** (navigation_controller): Immediate shutdown on failure - safety risk
   - **HIGH** (battery_monitor): Auto-restart up to 3 times, then escalate
   - **NON-CRITICAL** (task_processor): Log only, continue operation

2. **Intelligent Failure Tracking:**
   - Persistent JSON storage in `/tmp/robot_failures/`
   - Tracks failure count per node with timestamps
   - Robot ID tracking for fleet management
   - Automatic escalation after 3 attempts

3. **OpaqueFunction Event Handlers:**
   - Dynamic restart logic based on failure count
   - Conditional escalation to maintenance alerts
   - Customized restart parameters per attempt
   - Emergency shutdown for critical failures

4. **Production Features:**
   - Email/SMS alert simulation on persistent failures
   - Systemd integration for 24/7 operation
   - Configurable via YAML (robot_monitoring.yaml)
   - Failure log for maintenance analytics

**Real-World Comparison:**

| Scenario | Criticality | Behavior |
|----------|-------------|----------|
| Battery Monitor Crash | HIGH | Auto-restart 3x, then alert |
| Navigation Controller Crash | CRITICAL | Immediate emergency stop |
| Task Processor Crash | NON-CRITICAL | Log and continue |

**Testing Failure Recovery:**

```bash
# Test battery monitor auto-restart (will restart 3 times)
ros2 launch ce_robot_launch monitored_system_launch.py simulate_failures:=true

# In another terminal, watch the failure counter
watch -n 1 cat /tmp/robot_failures/failure_counts.json

# View failure log
tail -f /tmp/robot_failures/failure_log.txt
```

### **🧪 Step 4: Build and Test Production System**

**Build the ce_robot package with new nodes:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

**Verify nodes are installed:**
```bash
ros2 pkg executables ce_robot | grep "07_"
```

**Expected output:**
```
ce_robot 07_battery_monitor
ce_robot 07_navigation_controller
ce_robot 07_task_processor
```

---

### **🧪 Testing Exercise 3 Production Monitoring**

#### **Test 1: Normal Operation**

**Launch the monitored system:**
```bash
ros2 launch ce_robot_launch monitored_system_launch.py
```

**Expected startup logs:**
```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🔍 Production Robot Monitoring System Starting...
📊 Failure tracking: /path/to/config/failure_counter.py
⚙️ Max restart attempts: 3
🚨 Critical nodes: navigation_controller
🔄 Auto-restart enabled: battery_monitor
📋 Non-critical: task_processor
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✅ Battery Monitor started successfully
✅ Navigation Controller started successfully
🗺️ Safety systems online - collision avoidance active
✅ Task Processor started successfully
```

**Monitor node outputs:**
```bash
# Terminal 2: Battery status
ros2 topic echo /battery_status --once

# Terminal 3: Navigation status
ros2 topic echo /navigation_status --once

# Terminal 4: Task status
ros2 topic echo /task_status --once
```

#### **Test 2: Battery Monitor Auto-Restart**

**Terminal 1 - Run launch:**
```bash
ros2 launch ce_robot_launch monitored_system_launch.py simulate_failures:=true
```

**Wait for battery_monitor to crash automatically, observe behavior:**
```
⚠️ Battery Monitor exited! Attempt 1/3. Restarting in 3 seconds...
📊 Failure logged with robot tracking ID
[3 second delay]
✅ Battery Monitor started successfully
[continues until 3 failures]
❌ CRITICAL: Battery monitor failed 3 times! Manual intervention required.
⚠️ Alerting maintenance team...
📧 Email sent to: maintenance@warehouse.com
📱 SMS alert sent to on-call engineer
[System shutdown]
```

**Check failure tracking:**
```bash
# View failure count JSON
cat /tmp/robot_failures/failure_counts.json

# View detailed failure log
tail -20 /tmp/robot_failures/failure_log.txt
```

#### **Test 3: Critical Navigation Failure**

**Terminal 1 - Run without simulated failures:**
```bash
ros2 launch ce_robot_launch monitored_system_launch.py
```

**Terminal 2 - Kill navigation controller:**
```bash
pkill -f navigation_controller
```

**Expected immediate shutdown:**
```
❌ CRITICAL: Navigation Controller failed!
🚨 SAFETY ALERT: Collision avoidance compromised
🛑 EMERGENCY STOP: Shutting down all robot systems
📞 Emergency contact: Safety Team +1-800-ROBOT-911
[Immediate system shutdown - no restart attempts]
```

#### **Test 4: Non-Critical Task Processor Failure**

**Terminal 1 - Run launch:**
```bash
ros2 launch ce_robot_launch monitored_system_launch.py
```

**Terminal 2 - Kill task processor:**
```bash
pkill -f task_processor
```

**Expected behavior (no restart, log only):**
```
⚠️ Task Processor exited (non-critical)
📋 Order processing paused - robot continues safety operations
[Battery monitor and navigation controller continue running]
```

#### **Test 5: Service Call to Navigation Controller**

**Terminal 1 - Launch system:**
```bash
ros2 launch ce_robot_launch monitored_system_launch.py
```

**Terminal 2 - Start navigation:**
```bash
ros2 service call /navigation_command example_interfaces/srv/SetBool "{data: true}"
```

**Expected:**
```
response: 
  success: True
  message: 'Navigation started - AMR in motion'
```

**Terminal 3 - Monitor movement:**
```bash
ros2 topic echo /navigation_status
```

**Terminal 2 - Kill service to test critical failure:**
```bash
pkill -f CalRect_server
```

**Expected behavior:**
```
❌ CRITICAL: Rectangle Service failed! Shutting down system...
[Entire launch system shuts down]
```

### **💡 Key Concepts Learned**

1. **RegisterEventHandler** - Monitor node lifecycle events
2. **OnProcessStart** - Trigger actions when node starts
3. **OnProcessExit** - Trigger actions when node exits
4. **TimerAction** - Delay actions (e.g., wait before restart)
5. **EmitEvent** - Trigger system events (like Shutdown)
6. **Fault tolerance** - Build self-healing systems
7. **Critical vs non-critical** - Different handling based on node importance

### **🎯 Challenge: Failure Counter with Escalation**

**Task:** Track node failures and escalate after 3 consecutive crashes:

```python
# In real robots, persistent failures need human intervention
# Track failures using environment variables or state files

import os

def get_failure_count(node_name):
    """Read failure count from file"""
    try:
        with open(f'/tmp/{node_name}_failures.txt', 'r') as f:
            return int(f.read())
    except:
        return 0

def increment_failure_count(node_name):
    """Increment and save failure count"""
    count = get_failure_count(node_name) + 1
    with open(f'/tmp/{node_name}_failures.txt', 'w') as f:
        f.write(str(count))
    return count

# In OnProcessExit handler:
failure_count = increment_failure_count('battery_monitor')
if failure_count >= 3:
    LogInfo(msg='❌️ CRITICAL: Battery monitor failed 3 times! Alert maintenance!')
    EmitEvent(event=Shutdown(reason='Persistent node failure'))
else:
    LogInfo(msg=f'⚠️  Battery monitor crashed (attempt {failure_count}/3). Restarting...')
    # Restart node
```

**Real benefit:** Prevents infinite restart loops, alerts maintenance team when hardware repair is needed.

---

## **Exercise 4: YAML Configuration & Composition (Advanced) 📄**

### **📋 Objective**

Learn to use external YAML configuration files and compose launch files - essential for managing complex parameter sets and reusable launch configurations.

### **🎯 What You'll Learn**

- Load parameters from YAML files
- Use `IncludeLaunchDescription` to compose launch files
- Manage environment variables
- Create modular, reusable launch configurations
- Handle complex parameter hierarchies

### **💡 Real-World Use Case**

**Scenario:** Robot manufacturer deploying to multiple customer sites:

**Customer A (Compact facility):**
- Small robots (25kg capacity)
- Narrow aisles (0.45m width)
- Fast picking (3.0 Hz updates)
- 2-hour charging

**Customer B (Industrial warehouse):**
- Heavy-duty robots (1500kg capacity)
- Wide lanes (1.2m width)
- Stable transport (1.0 Hz updates)
- 6-hour charging

**Configuration files:**
```bash
config/
├── robot_small.yaml      # Customer A specs
├── robot_large.yaml      # Customer B specs
├── robot_simulation.yaml # Testing environment
└── robot_maintenance.yaml # Service mode
```

One codebase, multiple deployments! Change config file = different robot behavior. No code recompilation needed!


### **📁 File: launch/config/robot_small.yaml**

```yaml
# Small robot configuration - Compact picker for narrow aisles
robot_tag_publisher:
  ros__parameters:
    robot_id: "AMR-COMPACT-PICKER-S01"
    robot_type: "picker"
    zone_id: "WAREHOUSE-NARROW-AISLE-B"
    fleet_number: 10
    tag_publish_rate: 3.0  # Fast updates for agile movement
    max_payload_kg: 25.0  # Small capacity
    priority_level: 3
    # Physical dimensions (meters)
    robot_width: 0.45
    robot_length: 0.60
    robot_height: 1.20
    # Performance specs
    max_speed: 1.2  # m/s
    turning_radius: 0.3  # meters
    # Battery specs
    battery_capacity: 50.0  # Amp-hours
    charging_time: 2.5  # hours
    # Operational
    current_location: "AISLE-B-NARROW-12"
    status: "picking"
    assigned_task: "PICK-ORDER-54321"

rect_server:
  ros__parameters:
    max_dimension: 100.0
    min_dimension: 1.0

count_server:
  ros__parameters:
    max_count: 100
    default_period: 0.5
```

### **📁 File: launch/config/robot_large.yaml**

```yaml
# Large robot configuration - Heavy-duty industrial transport
robot_tag_publisher:
  ros__parameters:
    robot_id: "AMR-HEAVY-TRANSPORT-L01"
    robot_type: "transport"
    zone_id: "WAREHOUSE-MAIN-LOADING"
    fleet_number: 100
    tag_publish_rate: 1.0  # Slower updates for stable transport
    max_payload_kg: 1500.0  # Heavy industrial capacity
    priority_level: 8
    # Physical dimensions (meters)
    robot_width: 1.20
    robot_length: 1.80
    robot_height: 0.50  # Low profile for stability
    # Performance specs
    max_speed: 2.0  # m/s
    turning_radius: 1.5  # meters - wider turns
    # Battery specs
    battery_capacity: 200.0  # Amp-hours - large battery
    charging_time: 6.0  # hours
    # Operational
    current_location: "LOADING-DOCK-MAIN-3"
    status: "transporting_pallet"
    assigned_task: "PALLET-MOVE-DOCK3-TO-STORAGE-A"
    # Safety features
    lidar_range: 25.0  # meters
    emergency_stop_distance: 3.0  # meters
    max_slope: 5.0  # degrees

rect_server:
  ros__parameters:
    max_dimension: 1000.0
    min_dimension: 10.0

count_server:
  ros__parameters:
    max_count: 1000
    default_period: 2.0
```

### **📁 File: yaml_config_launch.py**

```python
#!/usr/bin/env python3
"""
Exercise 4: YAML Configuration and Composition
Loads parameters from YAML files and composes other launch files
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description with YAML config"""
    
    # Get package directory
    pkg_share = FindPackageShare('ce_robot_launch').find('ce_robot_launch')
    
    # Declare arguments
    robot_config_arg = DeclareLaunchArgument(
        'robot_config',
        default_value='small',
        description='Robot configuration to load (small or large)',
        choices=['small', 'large']
    )
    
    include_simple_launch_arg = DeclareLaunchArgument(
        'include_simple_launch',
        default_value='false',
        description='Include the simple_launch.py as well'
    )
    
    # Get configurations
    robot_config = LaunchConfiguration('robot_config')
    include_simple = LaunchConfiguration('include_simple_launch')
    
    # Build YAML file path
    config_file = PathJoinSubstitution([
        pkg_share,
        'launch',
        'config',
        ['robot_', robot_config, '.yaml']
    ])
    
    # Set environment variable (optional, but useful for debugging)
    set_env = SetEnvironmentVariable(
        'ROS_DOMAIN_ID', '42'
    )
    
    # Load nodes with YAML parameters
    publisher_node = Node(
        package='ce_robot',
        executable='05_robot_tag_param',
        name='robot_tag_publisher',
        output='screen',
        parameters=[config_file]
    )
    
    service_node = Node(
        package='ce_robot',
        executable='04_CalRect_server',
        name='rect_server',
        output='screen',
        parameters=[config_file]
    )
    
    action_node = Node(
        package='ce_robot',
        executable='06_count_until_server',
        name='count_server',
        output='screen',
        parameters=[config_file]
    )
    
    # Include another launch file conditionally
    simple_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_share,
                'launch',
                'simple_launch.py'
            ])
        ]),
        condition=IfCondition(include_simple)
    )
    
    return LaunchDescription([
        # Environment
        set_env,
        
        # Arguments
        robot_config_arg,
        include_simple_launch_arg,
        
        # Nodes with YAML config
        publisher_node,
        service_node,
        action_node,
        
        # Composed launch file
        simple_launch_include,
    ])
```

### **🧪 Testing Exercise 4**

**First, create the config directory:**
```bash
cd ~/ros2_ws/src/ce_robot_launch/launch
mkdir -p config
# Create robot_small.yaml and robot_large.yaml with content above
```

**Build:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_launch --symlink-install
source install/setup.bash
```

**Test 1 - Small robot config:**
```bash
ros2 launch ce_robot_launch yaml_config_launch.py robot_config:=small
```

**Verify parameters:**
```bash
ros2 param get /robot_tag_publisher robot_id
# Expected: ROBOT-SMALL-S01

ros2 param get /robot_tag_publisher max_payload_kg
# Expected: 50.0
```

**Test 2 - Large robot config:**
```bash
ros2 launch ce_robot_launch yaml_config_launch.py robot_config:=large
```

**Verify parameters:**
```bash
ros2 param get /robot_tag_publisher robot_id
# Expected: ROBOT-LARGE-L01

ros2 param get /robot_tag_publisher max_payload_kg
# Expected: 500.0
```

**Test 3 - Include composition:**
```bash
ros2 launch ce_robot_launch yaml_config_launch.py \
  robot_config:=small \
  include_simple_launch:=true
```

**Expected:** 4 publishers running (3 from yaml_config + 1 from simple_launch)

```bash
ros2 node list
```

### **💡 Key Concepts Learned**

1. **YAML parameters** - External configuration files
2. **PathJoinSubstitution** - Build paths dynamically
3. **FindPackageShare** - Locate package directories
4. **IncludeLaunchDescription** - Compose launch files
5. **SetEnvironmentVariable** - Configure ROS environment
6. **Modular configuration** - Reusable parameter sets
7. **Configuration switching** - Different setups via arguments

### **🎯 Challenge: Simulation vs Hardware Mode**

**Task:** Create configuration switching for testing and deployment:

**File: config/robot_simulation.yaml**
```yaml
# Simulation mode - faster, no real hardware
robot_tag_publisher:
  ros__parameters:
    use_sim_time: true  # ROS 2 simulation time
    robot_id: "SIM-TEST-001"
    zone_id: "GAZEBO-WAREHOUSE"
    tag_publish_rate: 10.0  # Fast for testing
    max_speed: 5.0  # Unrealistic speed OK in sim
    lidar_topic: "/scan_simulated"
    camera_topic: "/camera/image_raw_sim"
    enable_physics: true
    collision_checking: "gazebo"  # Use simulator
```

**File: config/robot_hardware.yaml**
```yaml
# Hardware mode - real robot, safe speeds
robot_tag_publisher:
  ros__parameters:
    use_sim_time: false  # Real system time
    robot_id: "AMR-PROD-001"
    zone_id: "WAREHOUSE-A-REAL"
    tag_publish_rate: 2.0  # Realistic rate
    max_speed: 1.2  # Safe human-compatible speed
    lidar_topic: "/sick_lms_1xx/scan"
    camera_topic: "/realsense/color/image_raw"
    enable_physics: false
    collision_checking: "hardware_estop"  # Physical e-stop
```

**Launch command:**
```bash
# Development testing
ros2 launch ce_robot_launch yaml_config_launch.py robot_config:=simulation

# Deploy to real robot
ros2 launch ce_robot_launch yaml_config_launch.py robot_config:=hardware
```

**Real benefit:** Test algorithms safely in simulation, deploy to hardware with confidence. Same code, different physics!

---

## **📊 Command Reference**

```bash
# Build launch package
colcon build --packages-select ce_robot_launch --symlink-install
source install/setup.bash

# List available launch files
ros2 launch ce_robot_launch --help

# Show launch arguments
ros2 launch ce_robot_launch <launch_file>.py --show-args

# Run with arguments
ros2 launch ce_robot_launch <launch_file>.py arg1:=value1 arg2:=value2

# Debug mode (verbose)
ros2 launch ce_robot_launch <launch_file>.py -d

# Monitor system
ros2 node list
ros2 topic list
ros2 service list
ros2 action list

# Check node parameters
ros2 param list /<node_name>
ros2 param get /<node_name> <param_name>

# Monitor namespaced topics
ros2 topic echo /<namespace>/<topic_name>
ros2 service call /<namespace>/<service_name> <srv_type> <args>
```

---

## **✅ Completion Checklist**

### **Exercise 1: Conditional Launch**
- [ ] Created `conditional_robot_launch.py`
- [ ] Implemented `IfCondition` for optional nodes
- [ ] Implemented `UnlessCondition` for debug mode
- [ ] Tested with different argument combinations
- [ ] Verified only expected nodes launch
- [ ] Understood `PythonExpression` for complex conditions

### **Exercise 2: Multi-Robot Namespaces**
- [ ] Created `multi_robot_launch.py`
- [ ] Implemented `GroupAction` and `PushRosNamespace`
- [ ] Launched 3 robot instances
- [ ] Verified isolated namespaces (no conflicts)
- [ ] Tested namespaced topics and services
- [ ] Understood multi-robot system architecture

### **Exercise 3: Event Handlers**
- [ ] Created `monitored_system_launch.py`
- [ ] Implemented `OnProcessStart` handlers
- [ ] Implemented `OnProcessExit` handlers
- [ ] Tested automatic node restart
- [ ] Tested critical failure shutdown
- [ ] Understood fault-tolerant design patterns
- [ ] Implemented persistent failure tracking

### **Exercise 4: YAML & Composition**
- [ ] Created `robot_small.yaml` and `robot_large.yaml`
- [ ] Created `yaml_config_launch.py`
- [ ] Tested parameter loading from YAML
- [ ] Implemented `IncludeLaunchDescription`
- [ ] Tested launch file composition
- [ ] Understood modular configuration management

### **Overall**
- [ ] All launch files build successfully
- [ ] All tests pass with expected results
- [ ] Understood when to use each technique
- [ ] Can apply concepts to own projects

---

## **💡 Tips & Best Practices**

### **1. Argument Validation**
```python
choices=['option1', 'option2', 'option3']  # Validates user input
```

### **2. Organized Logging**
```python
LogInfo(msg=['🚀 Starting system with ', config, ' configuration'])
```

### **3. Namespace Best Practices**
- Use meaningful names: `robot1`, `robot2` not `ns1`, `ns2`
- Group related nodes together
- Document namespace structure

### **4. Error Handling**
- Mark critical nodes clearly
- Use `OnProcessExit` for all important nodes
- Log failures with context

### **5. YAML Organization**
```yaml
# Group by node name
node_name:
  ros__parameters:
    param1: value1
    param2: value2
```

### **6. Modular Design**
- One launch file = one purpose
- Use composition for complex systems
- Keep configuration in YAML, logic in Python

### **7. Testing Strategy**
1. Test nodes individually first
2. Test launch file with default args
3. Test each argument combination
4. Test failure scenarios
5. Verify all namespaces/remappings

---

## **🎓 What You've Mastered**

Congratulations! You've completed advanced launch file training. You now know:

✅ **Conditional launching** - Flexible system configurations
✅ **Namespace management** - Multi-robot fleet deployment
✅ **Event handling** - Fault-tolerant systems with auto-recovery
✅ **YAML configuration** - Professional parameter management
✅ **Launch composition** - Modular, reusable systems

### **Production-Ready Skills:**

1. **Flexibility** - One launch file, many configurations
2. **Reliability** - Auto-restart and failure handling
3. **Scalability** - Deploy multiple robots easily
4. **Maintainability** - YAML configs, composed modules
5. **Professionalism** - Industry-standard practices

### **Next Steps:**

- Apply these patterns to your robot projects
- Create launch files for 06_Action exercises
- Build a complete robot system launch configuration
- Explore ROS 2 launch documentation for advanced features
- Consider YAML-based launch files for simpler configurations

---

## **📚 Additional Resources**

- [ROS 2 Launch Documentation](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html)
- [Launch File Architecture](https://design.ros2.org/articles/roslaunch.html)
- [Event Handlers Guide](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-Event-Handlers.html)
- [Launch Substitutions](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-Substitutions.html)
- [YAML Launch Files](https://docs.ros.org/en/jazzy/How-To-Guides/Launch-file-different-formats.html)

---

**🎓 Congratulations! You're now a ROS 2 Launch File Expert!** 🚀✨

*You've completed the journey from basic launches to production-ready, fault-tolerant, multi-robot systems. These skills are essential for real-world robotics deployments.*
