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

```
From Readme.md (Foundation):
├── simple_launch.py          → 1 node with parameters
└── robot_system_launch.py    → 3 nodes (publisher + service + action)

This Lab (Advanced):
├── Exercise 1: Conditional Launch
│   └── Launch nodes based on runtime conditions
│
├── Exercise 2: Multi-Robot with Namespaces  
│   └── Deploy multiple robot instances without conflicts
│
├── Exercise 3: Event Handlers & Monitoring
│   └── Auto-restart failed nodes, logging, notifications
│
└── Exercise 4: YAML Config & Composition
    └── External config files + include other launch files
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

You're deploying warehouse robots. Some have cameras (`has_camera:=true`), some don't. Your launch file should adapt automatically!


### **📁 File: conditional_robot_launch.py**

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
    
    # Get configurations
    enable_publisher = LaunchConfiguration('enable_publisher')
    enable_service = LaunchConfiguration('enable_service')
    enable_action = LaunchConfiguration('enable_action')
    robot_mode = LaunchConfiguration('robot_mode')
    
    # Conditional publisher (launches only if enabled)
    publisher_node = Node(
        package='ce_robot',
        executable='05_robot_tag_param',
        name='robot_tag_publisher',
        output='screen',
        parameters=[
            {'robot_id': 'ROBOT-COND-001'},
            {'robot_type': 'transport'},
            {'zone_id': 'WAREHOUSE-CONDITIONAL'},
            {'fleet_number': 1},
            {'tag_publish_rate': 2.0},
        ],
        condition=IfCondition(enable_publisher)
    )
    
    # Conditional service (launches only if enabled)
    service_node = Node(
        package='ce_robot',
        executable='04_CalRect_server',
        name='rect_server',
        output='screen',
        condition=IfCondition(enable_service)
    )
    
    # Conditional action (launches only if enabled)
    action_node = Node(
        package='ce_robot',
        executable='06_count_until_server',
        name='count_server',
        output='screen',
        condition=IfCondition(enable_action)
    )
    
    # Node that launches UNLESS in production mode
    debug_node = Node(
        package='ce_robot',
        executable='00_first_node',
        name='debug_monitor',
        output='screen',
        condition=UnlessCondition(
            PythonExpression(["'", robot_mode, "' == 'production'"])
        )
    )
    
    # Log info based on condition
    startup_log = LogInfo(
        msg=['🚀 Starting robot system in ', robot_mode, ' mode'],
    )
    
    return LaunchDescription([
        # Arguments
        enable_publisher_arg,
        enable_service_arg,
        enable_action_arg,
        robot_mode_arg,
        
        # Log
        startup_log,
        
        # Conditional nodes
        publisher_node,
        service_node,
        action_node,
        debug_node,
    ])
```

### **🧪 Testing Exercise 1**

**Test 1 - All nodes enabled (default):**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_launch --symlink-install
source install/setup.bash

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

### **💡 Key Concepts Learned**

1. **IfCondition** - Launch node only if condition is true
2. **UnlessCondition** - Launch node only if condition is false
3. **PythonExpression** - Evaluate Python expressions for complex conditions
4. **LogInfo** - Print messages during launch
5. **choices** parameter - Validate argument values

### **🎯 Challenge**

Add a `warehouse_zone` argument that changes the `zone_id` parameter. Use `PythonExpression` to set different `fleet_number` values based on the zone.

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

You have 3 robots in the same warehouse. Each needs its own publisher, but they all share the same code. Namespaces prevent topic/service name conflicts!


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
    
    # Robot 1 configuration
    robot1_group = GroupAction([
        PushRosNamespace('robot1'),
        Node(
            package='ce_robot',
            executable='05_robot_tag_param',
            name='robot_tag_publisher',
            output='screen',
            parameters=[
                {'robot_id': 'ROBOT-FLEET-001'},
                {'robot_type': 'transport'},
                {'zone_id': 'WAREHOUSE-A'},
                {'fleet_number': 1},
                {'tag_publish_rate': 2.0},
            ],
        ),
        Node(
            package='ce_robot',
            executable='04_CalRect_server',
            name='rect_server',
            output='screen',
        ),
    ])
    
    # Robot 2 configuration
    robot2_group = GroupAction([
        PushRosNamespace('robot2'),
        Node(
            package='ce_robot',
            executable='05_robot_tag_param',
            name='robot_tag_publisher',
            output='screen',
            parameters=[
                {'robot_id': 'ROBOT-FLEET-002'},
                {'robot_type': 'picker'},
                {'zone_id': 'WAREHOUSE-B'},
                {'fleet_number': 2},
                {'tag_publish_rate': 1.5},
            ],
        ),
        Node(
            package='ce_robot',
            executable='06_count_until_server',
            name='count_server',
            output='screen',
        ),
    ])
    
    # Robot 3 configuration
    robot3_group = GroupAction([
        PushRosNamespace('robot3'),
        Node(
            package='ce_robot',
            executable='05_robot_tag_param',
            name='robot_tag_publisher',
            output='screen',
            parameters=[
                {'robot_id': 'ROBOT-FLEET-003'},
                {'robot_type': 'transport'},
                {'zone_id': 'WAREHOUSE-C'},
                {'fleet_number': 3},
                {'tag_publish_rate': 3.0},
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

**Call namespaced service:**
```bash
ros2 service call /robot1/cal_rect ce_robot_interfaces/srv/CalRectangle "{length: 10.0, width: 5.0}"
```

**Test action on robot 2:**
```bash
ros2 action send_goal /robot2/count_until ce_robot_interfaces/action/CountUntil "{target: 5, period: 1.0}" --feedback
```

### **💡 Key Concepts Learned**

1. **GroupAction** - Group nodes together for organized management
2. **PushRosNamespace** - Add namespace prefix to all nodes in group
3. **Node isolation** - Same node names in different namespaces don't conflict
4. **Namespace paths** - Access topics/services via `/namespace/name`
5. **Multi-robot systems** - Deploy fleets without code duplication

### **🎯 Challenge**

Create a launch file that accepts a `robot_id` argument and launches a single robot with that namespace dynamically. Hint: Use `LaunchConfiguration` in `PushRosNamespace`.

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

Your robot's camera driver crashes occasionally. Instead of manual restarts, configure automatic recovery with logging for debugging!


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
    
    # Critical publisher node
    publisher_node = Node(
        package='ce_robot',
        executable='05_robot_tag_param',
        name='robot_tag_publisher',
        output='screen',
        parameters=[
            {'robot_id': 'ROBOT-MONITOR-001'},
            {'robot_type': 'transport'},
            {'zone_id': 'WAREHOUSE-MONITOR'},
            {'fleet_number': 1},
            {'tag_publish_rate': 2.0},
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

### **🎯 Challenge**

Modify the launch file to count failures and only auto-restart up to 3 times. After 3 failures, log an error and shut down the system.

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

You have different robot configurations (small/medium/large robots) with 50+ parameters each. YAML files make management easier than hardcoding parameters!


### **📁 File: launch/config/robot_small.yaml**

```yaml
# Small robot configuration
robot_tag_publisher:
  ros__parameters:
    robot_id: "ROBOT-SMALL-S01"
    robot_type: "picker"
    zone_id: "WAREHOUSE-SMALL"
    fleet_number: 10
    tag_publish_rate: 3.0
    max_payload_kg: 50.0
    priority_level: 3

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
# Large robot configuration
robot_tag_publisher:
  ros__parameters:
    robot_id: "ROBOT-LARGE-L01"
    robot_type: "transport"
    zone_id: "WAREHOUSE-LARGE"
    fleet_number: 100
    tag_publish_rate: 1.0
    max_payload_kg: 500.0
    priority_level: 8

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

### **🎯 Challenge**

Create a `robot_simulation.yaml` config and modify the launch file to accept a `use_sim_time` argument that sets the parameter for all nodes. This is how real robot systems switch between simulation and hardware modes!

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
