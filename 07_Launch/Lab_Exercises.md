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

**Duration:** ~2 hours
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

### **Exercise 4: YAML Config & Composition (20 min)**

```
Configuration Management:
┌─────────────────────────────────────────────────────────────┐
│                  MULTI-SITE DEPLOYMENT                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  config/robot_small.yaml          config/robot_large.yaml   │
│  ┌─────────────────────┐          ┌────────────────────┐    │
│  │ Customer A:         │          │ Customer B:        │    │
│  │ Compact Facility    │          │ Industrial WH      │    │
│  ├─────────────────────┤          ├────────────────────┤    │
│  │ • 25kg capacity     │          │ • 1500kg capacity  │    │
│  │ • 0.45m width       │          │ • 1.2m width       │    │
│  │ • 3.0 Hz updates    │          │ • 1.0 Hz updates   │    │
│  │ • 2 hour charging   │          │ • 6 hour charging  │    │
│  │ • Narrow aisles     │          │ • Wide lanes       │    │
│  │ • Fast picking      │          │ • Heavy pallets    │    │
│  └─────────────────────┘          └────────────────────┘    │
│           │                                │                │
│           └────────────┬───────────────────┘                │
│                        ▼                                    │
│           yaml_config_launch.py                             │
│           ┌──────────────────────┐                          │
│           │ robot_config argument│                          │
│           │ ├─ :=small           │                          │
│           │ └─ :=large           │                          │
│           └──────────────────────┘                          │
│                        │                                    │
│                        ▼                                    │
│           ┌──────────────────────┐                          │
│           │ Load YAML config     │                          │
│           │ Launch nodes with    │                          │
│           │ appropriate params   │                          │
│           └──────────────────────┘                          │
│                                                             │
│  Launch File Composition:                                   │
│  ┌────────────────────────────────────────┐                 │
│  │ yaml_config_launch.py                  │                 │
│  │  ├─ Includes: simple_launch.py         │                 │
│  │  │  └─ Spawns additional robot         │                 │
│  │  └─ SetEnvironmentVariable             │                 │
│  │     └─ ROS_DOMAIN_ID = 42              │                 │
│  └────────────────────────────────────────┘                 │
│                                                             │
└─────────────────────────────────────────────────────────────┘

Deployment Commands:
├─ ros2 launch ce_robot_launch yaml_config_launch.py robot_config:=small
├─ ros2 launch ce_robot_launch yaml_config_launch.py robot_config:=large
├─ ros2 launch ce_robot_launch yaml_config_launch.py robot_config:=simulation
└─ ros2 launch ce_robot_launch yaml_config_launch.py robot_config:=hardware
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

Create the conditional launch file in your launch package:

```bash
cd ~/ros2_ws/src/ce_robot_launch/launch
touch conditional_robot_launch.py
chmod +x conditional_robot_launch.py
```

Open the file in your editor and add the following code:

### **📁 File: conditional_robot_launch.py**

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
        package='ce_robot_launch',
        executable='navigation_service.py',
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
        package='ce_robot_launch',
        executable='task_queue_action.py',
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
        package='ce_robot_launch',
        executable='debug_monitor.py',
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

### **📝 Step 2: Create Support Python Files**

Create the three Python node files that the launch file will use:

```bash
cd ~/ros2_ws/src/ce_robot_launch/launch
touch navigation_service.py task_queue_action.py debug_monitor.py
chmod +x navigation_service.py task_queue_action.py debug_monitor.py
```

Copy the node implementations from the `07_Launch/` directory:
- `navigation_service.py` - Navigation path planning service
- `task_queue_action.py` - Order picking action server  
- `debug_monitor.py` - System diagnostics monitor

Or create them manually with the code from the files we created earlier.

### **📝 Step 4: Update CMakeLists.txt**

Add the launch file to your `ce_robot_launch/CMakeLists.txt`:

```cmake
install(
  PROGRAMS
    launch/conditional_robot_launch.py
    launch/navigation_service.py
    launch/task_queue_action.py
    launch/debug_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)
```

### **🧪 Step 5: Build and Test**

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

---

### **🧪 Testing Exercise 1**

**Test 1 - All nodes enabled (default):**
```bash
ros2 launch ce_robot_launch conditional_robot_launch.py
```

**Expected:** Publisher ✅, Service ✅, Action ❌, Debug ❌

**Test 2 - Enable action server:**
```bash
ros2 launch ce_robot_launch conditional_robot_launch.py \
  enable_action:=true
```

**Expected:** Publisher ✅, Service ✅, Action ✅, Debug ❌

**Test 3 - Disable publisher:**
```bash
ros2 launch ce_robot_launch conditional_robot_launch.py \
  enable_publisher:=false
```

**Expected:** Publisher ❌, Service ✅, Action ❌, Debug ❌

**Test 4 - Development mode (includes debug):**
```bash
ros2 launch ce_robot_launch conditional_robot_launch.py \
  robot_mode:=development
```

**Expected:** Publisher ✅, Service ✅, Action ❌, Debug ✅

**Verify running nodes:**
```bash
ros2 node list
```

---

### **🧪 Test Individual Nodes**

**Test Navigation Service (standalone):**
```bash
# Terminal 1: Run the navigation service
ros2 run ce_robot_launch navigation_service.py --ros-args \
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
ros2 run ce_robot_launch task_queue_action.py --ros-args \
  -p robot_id:=AMR-PICKER-001 \
  -p robot_type:=picker \
  -p zone_id:=PICKING-ZONE \
  -p max_items_per_trip:=20 \
  -p battery_level:=85.0 \
  -p picking_speed_items_per_min:=12.0

# Terminal 2: Send an action goal
ros2 action send_goal /pick_items ce_robot_interfaces/action/PickItems \
  "{target_items: 5, time_per_item: 5.0, order_id: 'ORD-12345', \
    zone_id: 'PICKING-ZONE', priority: 'MEDIUM', max_weight_kg: 25.0}" \
  --feedback

# Expected: Progress updates with items picked, battery consumed, elapsed time
```

**Test Debug Monitor (standalone):**
```bash
# Terminal 1: Run the debug monitor
ros2 run ce_robot_launch debug_monitor.py --ros-args \
  -p robot_id:=AMR-DEBUG-001 \
  -p robot_type:=test \
  -p zone_id:=DEBUG-LAB \
  -p diagnostic_rate_hz:=0.5 \
  -p enable_network_check:=true \
  -p enable_ros_diagnostics:=true

# Monitor will print diagnostic reports every 2 seconds
# Check CPU, memory, disk usage, network status, and issue counts
```

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


### **📁 File: multi_robot_launch.py**

```python
#!/usr/bin/env python3
"""
Exercise 2: Multi-Robot Launch with Namespaces
Launches 3 robot instances with isolated namespaces
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """Generate launch description for multi-robot system"""
    
    # Declare arguments
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='3',
        description='Number of robots to launch'
    )
    
    # Robot 1: Heavy transport robot in loading dock
    robot1_group = GroupAction([
        PushRosNamespace('robot1'),
        Node(
            package='ce_robot',
            executable='05_robot_tag_param',
            name='robot_tag_publisher',
            output='screen',
            parameters=[
                {'robot_id': 'AMR-TRANSPORT-HEAVY-001'},
                {'robot_type': 'transport'},
                {'zone_id': 'WAREHOUSE-A-LOADING-DOCK'},
                {'fleet_number': 1},
                {'tag_publish_rate': 2.0},
                {'max_payload_kg': 1000.0},  # Heavy-duty transport
                {'current_location': 'DOCK-A-STATION-5'},
                {'assigned_task': 'PALLET-TRANSPORT-TO-STORAGE'},
                {'assigned_operator': 'SHIFT-SUPERVISOR-A'},
                {'battery_level': 78.0},
                {'priority_level': 8},  # High priority
                {'safety_certified': True},
            ],
        ),
        Node(
            package='ce_robot',
            executable='04_CalRect_server',
            name='rect_server',
            output='screen',
        ),
    ])
    
    # Robot 2: Picker robot with low battery - returning to charging
    robot2_group = GroupAction([
        PushRosNamespace('robot2'),
        Node(
            package='ce_robot',
            executable='05_robot_tag_param',
            name='robot_tag_publisher',
            output='screen',
            parameters=[
                {'robot_id': 'AMR-PICKER-LIGHT-002'},
                {'robot_type': 'picker'},
                {'zone_id': 'WAREHOUSE-B-PICKING-AREA'},
                {'fleet_number': 2},
                {'tag_publish_rate': 1.5},
                {'max_payload_kg': 50.0},  # Light picker
                {'current_location': 'AISLE-B-12-SHELF-3'},
                {'assigned_task': 'RETURN-TO-CHARGING-STATION'},
                {'assigned_operator': 'AUTO'},  # Autonomous operation
                {'battery_level': 18.5},  # Low battery - needs charging!
                {'priority_level': 3},  # Lower priority - maintenance task
                {'safety_certified': True},
                {'status': 'low_battery_return'},
            ],
        ),
        Node(
            package='ce_robot',
            executable='06_count_until_server',
            name='count_server',
            output='screen',
        ),
    ])
    
    # Robot 3: Multi-function delivery robot at sorting station
    robot3_group = GroupAction([
        PushRosNamespace('robot3'),
        Node(
            package='ce_robot',
            executable='05_robot_tag_param',
            name='robot_tag_publisher',
            output='screen',
            parameters=[
                {'robot_id': 'AMR-DELIVERY-MULTI-003'},
                {'robot_type': 'transport'},
                {'zone_id': 'WAREHOUSE-C-SORTING-STATION'},
                {'fleet_number': 3},
                {'tag_publish_rate': 3.0},  # Fast updates for active operations
                {'max_payload_kg': 300.0},  # Medium capacity
                {'current_location': 'SORT-C-CONVEYOR-7'},
                {'assigned_task': 'PACKAGE-DELIVERY-ROUTE-12'},
                {'assigned_operator': 'DISPATCH-COORD-C'},
                {'battery_level': 92.0},  # Fully charged, ready for operations
                {'priority_level': 9},  # Critical delivery route
                {'safety_certified': True},
                {'status': 'active_delivery'},
                {'operation_hours': 1847.3},  # Veteran robot
                {'firmware_version': 'v3.2.1'},
            ],
        ),
        Node(
            package='ce_robot',
            executable='04_CalRect_server',
            name='rect_server',
            output='screen',
        ),
        Node(
            package='ce_robot',
            executable='06_count_until_server',
            name='count_server',
            output='screen',
        ),
    ])
    
    return LaunchDescription([
        num_robots_arg,
        robot1_group,
        robot2_group,
        robot3_group,
    ])
```

### **🧪 Testing Exercise 2**

**Build and launch:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_launch --symlink-install
source install/setup.bash

ros2 launch ce_robot_launch multi_robot_launch.py
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

**Monitor specific robot:**
```bash
ros2 topic echo /robot1/robot_tag
```

**Call namespaced navigation service:**
```bash
ros2 service call /robot1/navigation_path ce_robot_interfaces/srv/NavigationPath \
  "{obstacle_length: 2.5, obstacle_width: 1.5, robot_x: 5.0, robot_y: 3.0, \
  safety_margin: 0.5, zone_id: 'WAREHOUSE-A-DOCK-3'}"
```

**Test picking action on robot 2:**
```bash
ros2 action send_goal /robot2/pick_items ce_robot_interfaces/action/PickItems \
  "{target_items: 10, time_per_item: 2.0, order_id: 'ORD-2025-001', \
  zone_id: 'ZONE-B-SHELF-42', priority: 8, max_weight_kg: 50.0}" --feedback
```

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


### **📁 File: monitored_system_launch.py**

```python
#!/usr/bin/env python3
"""
Exercise 3: Event Handlers and Monitoring
Implements node failure detection and automatic restart
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    EmitEvent,
    TimerAction
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with event monitoring"""
    
    # Arguments
    enable_auto_restart_arg = DeclareLaunchArgument(
        'enable_auto_restart',
        default_value='true',
        description='Enable automatic node restart on failure'
    )
    
    critical_node_arg = DeclareLaunchArgument(
        'critical_node',
        default_value='publisher',
        description='Critical node that triggers shutdown if it fails',
        choices=['publisher', 'service', 'action', 'none']
    )
    
    # Get configurations
    enable_auto_restart = LaunchConfiguration('enable_auto_restart')
    critical_node = LaunchConfiguration('critical_node')
    
    # Critical publisher node - Battery monitoring system
    # In real robots, losing battery data is dangerous!
    publisher_node = Node(
        package='ce_robot',
        executable='05_robot_tag_param',
        name='robot_tag_publisher',
        output='screen',
        parameters=[
            {'robot_id': 'AMR-BATTERY-MONITOR-001'},
            {'robot_type': 'transport'},
            {'zone_id': 'WAREHOUSE-MAIN-FLOOR'},
            {'fleet_number': 1},
            {'tag_publish_rate': 2.0},
            {'max_payload_kg': 500.0},
            {'current_location': 'MAIN-AISLE-4'},
            {'battery_level': 65.0},
            {'status': 'operational'},
            {'priority_level': 8},
            # Battery monitoring parameters
            {'battery_voltage': 48.2},  # Volts
            {'battery_current': -15.3},  # Amps (negative = discharging)
            {'battery_temp': 32.5},  # Celsius
        ],
    )
    
    # Service node
    service_node = Node(
        package='ce_robot',
        executable='04_CalRect_server',
        name='rect_server',
        output='screen',
    )
    
    # Action node
    action_node = Node(
        package='ce_robot',
        executable='06_count_until_server',
        name='count_server',
        output='screen',
    )
    
    # Event handler: Log when publisher starts
    publisher_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=publisher_node,
            on_start=[
                LogInfo(msg='✅ Robot Tag Publisher started successfully'),
            ]
        )
    )
    
    # Event handler: React when publisher exits
    publisher_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=publisher_node,
            on_exit=[
                LogInfo(msg='⚠️  Robot Tag Publisher exited! Attempting restart in 3 seconds...'),
                TimerAction(
                    period=3.0,
                    actions=[
                        Node(
                            package='ce_robot',
                            executable='05_robot_tag_param',
                            name='robot_tag_publisher_restart',
                            output='screen',
                            parameters=[
                                {'robot_id': 'ROBOT-MONITOR-001-RESTART'},
                                {'robot_type': 'transport'},
                                {'zone_id': 'WAREHOUSE-MONITOR'},
                                {'fleet_number': 1},
                                {'tag_publish_rate': 2.0},
                            ],
                        ),
                    ]
                ),
            ]
        )
    )
    
    # Event handler: Log when service starts
    service_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=service_node,
            on_start=[
                LogInfo(msg='✅ Rectangle Service started successfully'),
            ]
        )
    )
    
    # Event handler: Critical service failure triggers shutdown
    service_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=service_node,
            on_exit=[
                LogInfo(msg='❌ CRITICAL: Rectangle Service failed! Shutting down system...'),
                EmitEvent(event=Shutdown(reason='Critical service failure')),
            ]
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
        
        # Nodes
        publisher_node,
        service_node,
        action_node,
        
        # Event handlers
        publisher_start_handler,
        publisher_exit_handler,
        service_start_handler,
        service_exit_handler,
        action_start_handler,
        action_exit_handler,
    ])
```

### **🧪 Testing Exercise 3**

**Build and launch:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_launch --symlink-install
source install/setup.bash

ros2 launch ce_robot_launch monitored_system_launch.py
```

**Expected startup logs:**
```
✅ Robot Tag Publisher started successfully
✅ Rectangle Service started successfully
✅ Count Action Server started successfully
```

**Test node failure simulation:**

**Terminal 1 - Run launch:**
```bash
ros2 launch ce_robot_launch monitored_system_launch.py
```

**Terminal 2 - Kill publisher to test auto-restart:**
```bash
# Find the process ID
ros2 node list
# Kill the node
ros2 lifecycle set /robot_tag_publisher shutdown
# Or use: pkill -f robot_tag_publisher
```

**Expected behavior:**
```
⚠️  Robot Tag Publisher exited! Attempting restart in 3 seconds...
✅ Robot Tag Publisher started successfully
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
