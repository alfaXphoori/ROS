# **🎯 ROS 2 Launch Files Fundamentals**

Learn how to automate launching multiple nodes, configure parameters, and manage complex robot systems using ROS 2 Launch files.

---

## **📌 Project Title**

Create and Use ROS 2 Launch Files for Multi-Node System Automation

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🛠 Overview**

Launch files in ROS 2 allow you to:
- **Launch multiple nodes** with a single command
- **Configure parameters** at startup
- **Set up environment variables**
- **Manage node namespaces**
- **Control logging levels**
- **Monitor and restart nodes**
- **Create reusable system configurations**

**Key Features of Launch Files:**
- Written in Python (or YAML in ROS 2)
- Declare nodes, parameters, and remappings
- Conditional logic (if-then for different scenarios)
- Event handlers for node failures
- Structured argument passing

**Duration:** ~1.5 hours  
**Level:** Intermediate  
**Prerequisites:** Parameters lab completed, Actions lab completed, familiar with Python

---

## **🎯 Why Launch Files Matter**

**The Problem:**
Imagine you built a warehouse robot with:
- Battery charging system (server + client = 2 nodes)
- Navigation system (server + client = 2 nodes)
- Gripper control (server + client = 2 nodes)

**Without Launch Files:** Open 6 terminals, run 6 commands, set parameters manually 😫

**With Launch Files:** One command starts everything with correct parameters! 🚀

```bash
# Before: 6 terminals, 6 commands
ros2 run ce_robot 06_battery_charging_server
ros2 run ce_robot 06_battery_charging_client
ros2 run ce_robot 06_navigate_server
# ... 3 more commands

# After: 1 terminal, 1 command
ros2 launch ce_robot_launch robot_system.py
```

---
## **📚 Learning Path**

```
┌─────────────────────────────────────────────────────────┐
│  Step 1: Understand Concepts (15 min)                   │
│  → Read Overview + Architecture                         │
│  → Understand Node(), parameters, arguments             │
└─────────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────────┐
│  Step 2: Create Package (10 min)                        │
│  → Set up ce_robot_launch package                       │
│  → Configure CMakeLists.txt and package.xml             │
└─────────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────────┐
│  Step 3: Simple Launch (20 min)                         │
│  → Create simple_launch.py                              │
│  → Launch 1 node with parameters                        │
│  → Test and verify                                      │
└─────────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────────┐
│  Step 4: Complex Launch (30 min)                        │
│  → Create robot_system_launch.py                        │
│  → Launch multiple nodes                                │
│  → Use launch arguments                                 │
│  → Test with different parameters                       │
└─────────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────────┐
│  Step 5: Advanced Exercises (15 min)                    │
│  → Try Lab_Exercises.md                                 │
│  → Launch your Action nodes from 06_Action              │
└─────────────────────────────────────────────────────────┘
```

---
## **📊 Architecture Diagram**

```
┌─────────────────────────────────────────────────────────┐
│              ROS 2 Launch System Architecture           │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌──────────────────────────────────────────────────┐   │
│  │  Launch File (Python/YAML)                       │   │
│  │  ├─ Node declarations                            │   │
│  │  ├─ Parameter assignments                        │   │
│  │  ├─ Remappings                                   │   │
│  │  └─ Conditional logic                            │   │
│  └────────────────┬─────────────────────────────────┘   │
│                   │                                     │
│                   ▼                                     │
│  ┌──────────────────────────────────────────────────┐   │
│  │  ROS 2 Launch Service                            │   │
│  │  (Parses & executes launch file)                 │   │
│  └────────────────┬─────────────────────────────────┘   │
│                   │                                     │
│    ┌──────┬──────┬──────┬─────────────┐                 │
│    ▼      ▼      ▼      ▼             ▼                 │
│  ┌────┐ ┌────┐ ┌─────┐ ┌─────┐  ┌──────────┐            │
│  │Node│ │Node│ │Node │ │ ... │  │Parameter │            │
│  │ 1  │ │ 2  │ │ N   │ │     │  │  Server  │            │
│  └────┘ └────┘ └─────┘ └─────┘  └──────────┘            │
│                                                         │
│  Publishing:                                            │
│  - Topics from all nodes                                │
│  - Available services                                   │
│  - Parameter values                                     │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

---

## **Step 1: Create Launch Package** ⏱️ 10 min

### **📋 Prerequisites Check**

Before starting, verify you have completed:
- ✅ Parameters module (05_Parameters)
- ✅ Actions module (06_Action)
- ✅ Have `ce_robot` and `ce_robot_interfaces` packages built

### **Create Package Structure**

```bash
cd ~/ros2_ws/src
ros2 pkg create ce_robot_launch
cd ce_robot_launch
rm -rf include src
mkdir launch
```

### **Directory Structure**

```
📁 ros2_ws/
└── 📁 src/
    └── 📁 ce_robot_launch/
        ├── 📁 launch/
        │   ├── 🐍 robot_system_launch.py  ← Create this
        │   ├── 🐍 simple_launch.py        ← Create this
        │   ├── 📁 config/
        │   │   └── 📄 robot_config.yaml
        │   └── 📁 params/
        │       └── 📄 parameters.yaml
        ├── 📄 CMakeLists.txt               ← Update this
        └── 📄 package.xml                  ← Update this
```

### **Update CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(ce_robot_launch)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### **Update package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ce_robot_launch</name>
  <version>0.0.0</version>
  <description>ROS 2 Launch file examples and configurations</description>
  <maintainer email="student@ksu.ac.th">Student</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>ce_robot</exec_depend>
  <exec_depend>ce_robot_interfaces</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## **Step 2: Create Simple Launch File** ⏱️ 20 min

### **📝 Understanding the Basics**

A launch file needs:
1. **Imports** - `LaunchDescription`, `Node`
2. **Function** - `generate_launch_description()`
3. **Node Declaration** - Define what to launch
4. **Return** - Return all nodes in `LaunchDescription([...])`

### **📁 File Location**

```bash
cd ~/ros2_ws/src/ce_robot_launch/launch
touch simple_launch.py
chmod +x simple_launch.py
```

### **File: simple_launch.py**

```python
#!/usr/bin/env python3
"""
Simple Launch File Example
Launches a publisher and subscriber
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description"""
    
    # Define nodes to launch
    publisher_node = Node(
        package='ce_robot',
        executable='05_robot_tag_param',
        name='robot_tag_publisher',
        output='screen',
        parameters=[
            {'robot_id': 'ROBOT-SIMPLE-001'},
            {'robot_type': 'transport'},
            {'zone_id': 'WAREHOUSE-A'},
            {'fleet_number': 1},
            {'tag_publish_rate': 2.0},
        ]
    )
    
    # Return launch description with all nodes
    return LaunchDescription([
        publisher_node,
    ])
```

### **💡 Code Explanation**

```python
# 1. Import required modules
from launch import LaunchDescription      # Container for all launch items
from launch_ros.actions import Node       # ROS 2 node declaration

# 2. Must have this function - ROS 2 looks for it!
def generate_launch_description():
    
    # 3. Declare each node
    publisher_node = Node(
        package='ce_robot',                   # Which package?
        executable='05_robot_tag_param',      # Which node executable? (from setup.py entry_points)
        name='robot_tag_publisher',           # What to call it?
        output='screen',                      # Show output in terminal
        parameters=[                          # Set parameters
            {'robot_id': 'ROBOT-SIMPLE-001'},
            {'robot_type': 'transport'},
            {'zone_id': 'WAREHOUSE-A'},
            {'fleet_number': 1},
            {'tag_publish_rate': 2.0},
        ]
    )
    
    # 4. Return everything to launch
    return LaunchDescription([
        publisher_node,  # Add all nodes here
    ])
```

### **🧪 Test Simple Launch**

```bash
# Build first
cd ~/ros2_ws
colcon build --packages-select ce_robot_launch --symlink-install
source install/setup.bash

# Launch it!
ros2 launch ce_robot_launch simple_launch.py
```

**Expected Output:**
```
[INFO] [robot_tag_publisher]: 🏷️  Robot Tag Publisher initialized
[INFO] [robot_tag_publisher]:    Robot ID: ROBOT-SIMPLE-001
[INFO] [robot_tag_publisher]:    Robot Type: transport
[INFO] [robot_tag_publisher]:    Zone: WAREHOUSE-A
[INFO] [robot_tag_publisher]:    Fleet Number: 1
[INFO] [robot_tag_publisher]:    Publish Rate: 2.0 Hz
[INFO] [robot_tag_publisher]: 🤖 ROBOT-SIMPLE-001 [transport]: Status=active, Zone=WAREHOUSE-A
```

**Verify in another terminal:**
```bash
ros2 node list               # Should see /robot_tag_publisher
ros2 topic list              # Should see /robot_tag
ros2 topic echo /robot_tag   # See RobotTag messages
```

---

## **Step 3: Create Complex Launch File with Configuration** ⏱️ 30 min

### **🎯 New Concepts**

1. **Launch Arguments** - Accept values from command line
2. **Multiple Nodes** - Launch several nodes at once
3. **LaunchConfiguration** - Use command line arguments in nodes

### **📁 File Location**

```bash
cd ~/ros2_ws/src/ce_robot_launch/launch
touch robot_system_launch.py
chmod +x robot_system_launch.py
```

### **File: robot_system_launch.py**

```python
#!/usr/bin/env python3
"""
Robot System Launch File
Launches multiple nodes (publisher, service, action) with configuration
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description"""
    
    # Get package directories
    ce_robot_launch_dir = get_package_share_directory('ce_robot_launch')
    
    # Declare launch arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='ROBOT-BOOT-100',
        description='Robot identifier'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Publishing rate in Hz'
    )
    
    # Get launch configuration values
    robot_id = LaunchConfiguration('robot_id')
    publish_rate = LaunchConfiguration('publish_rate')
    
    # Define nodes
    robot_tag_publisher = Node(
        package='ce_robot',
        executable='05_robot_tag_param',
        name='robot_tag_publisher',
        output='screen',
        parameters=[
            {'robot_id': robot_id},
            {'robot_type': 'transport'},
            {'zone_id': 'WAREHOUSE-A'},
            {'fleet_number': 100},
            {'tag_publish_rate': publish_rate},
        ]
    )
    
    rect_server = Node(
        package='ce_robot',
        executable='04_CalRect_server',
        name='rect_server',
        output='screen'
    )
    
    action_server = Node(
        package='ce_robot',
        executable='06_count_until_server',
        name='count_server',
        output='screen'
    )
    
    # Return launch description
    return LaunchDescription([
        # Arguments
        robot_id_arg,
        publish_rate_arg,
        
        # Nodes
        robot_tag_publisher,
        rect_server,
        action_server,
    ])
```

---

### **File: launch/params/parameters.yaml**

```yaml
# Global parameters configuration
robot_tag_publisher:
  ros__parameters:
    robot_id: "ROBOT-BOOT-100"
    robot_type: "transport"
    zone_id: "WAREHOUSE-A"
    fleet_number: 100
    tag_publish_rate: 2.0

rect_server:
  ros__parameters:
    max_dimension: 1000
    min_dimension: 0.1
```

---

## **Step 4: Build and Run** ⏱️ 5 min

### **Build Package**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_launch --symlink-install
source install/setup.bash
```

### **Launch Simple Example**

```bash
ros2 launch ce_robot_launch simple_launch.py
```

### **Launch with Arguments**

```bash
ros2 launch ce_robot_launch robot_system_launch.py \
  robot_name:=robot_warehouse \
  publish_rate:=5.0
```

### **List Launch Files**

```bash
ros2 launch ce_robot_launch --help
```

---

## **Step 5: Verify Running System** ⏱️ 10 min

### **List Nodes**

```bash
ros2 node list
```

**Expected Output:**
```
/robot_tag_publisher
/rect_server
/count_server
```

### **List Topics**

```bash
ros2 topic list
```

### **List Services**

```bash
ros2 service list
```

### **Test Rectangle Service**

```bash
# Test the CalRectangle service from rect_server
ros2 service call /calculate_rectangle ce_robot_interfaces/srv/CalRectangle "{length: 5.0, width: 3.0}"
```

**Expected Output:**
```
waiting for service to become available...
requester: making request: ce_robot_interfaces.srv.CalRectangle_Request(length=5.0, width=3.0)

response:
ce_robot_interfaces.srv.CalRectangle_Response(area=15.0, perimeter=16.0)
```

### **Test Action Server**

```bash
# List available actions
ros2 action list

# Send goal to CountUntil action from count_server
ros2 action send_goal /count_until ce_robot_interfaces/action/CountUntil "{target_number: 10, period: 1.0}" --feedback
```

**Expected Output:**
```
Waiting for an action server to become available...
Sending goal:
     target_number: 10
     period: 1.0

Goal accepted with ID: ...

Feedback:
    current_number: 0

Feedback:
    current_number: 1

Feedback:
    current_number: 2
    
... (continues until 10)

Result:
    reached_number: 10
    
Goal finished with status: SUCCEEDED
```

### **Monitor Topic Output**

```bash
ros2 topic echo /robot_tag
```

---

## **🎓 What You Learned**

✅ **Launch files automate multi-node startup**
- Before: 6 terminals for 6 nodes
- After: 1 command for all nodes

✅ **Three key components:**
1. `Node()` - Declares what to launch
2. `DeclareLaunchArgument()` - Accepts command line args
3. `LaunchConfiguration()` - Uses arg values in nodes

✅ **Parameters can be:**
- Hardcoded: `{'param': value}`
- From arguments: `{'param': LaunchConfiguration('arg')}`
- From YAML files: `parameters=[yaml_file]`

✅ **Common patterns:**
- Single node: `simple_launch.py`
- Multiple nodes: `robot_system_launch.py`
- With arguments: `robot_name:=value`
- Show args: `--show-args`

---

## **📝 Key Concepts**

### **Launch File Structure**

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    # Create nodes
    # Return LaunchDescription
    return LaunchDescription([...])
```

### **Node Configuration**

| Parameter | Purpose | Example |
|-----------|---------|---------|
| `package` | Package name | `ce_robot` |
| `executable` | Node executable | `hw_status_param_pub` |
| `name` | Node name | `hw_publisher` |
| `output` | Output handling | `screen`, `log` |
| `parameters` | Runtime parameters | `{'param': value}` |

### **Launch Arguments**

```python
# Declare argument
robot_arg = DeclareLaunchArgument(
    'robot_name',
    default_value='robot_default',
    description='Robot identifier'
)

# Use in node
robot_name = LaunchConfiguration('robot_name')
```

### **Remapping Topics**

```python
Node(
    package='package',
    executable='node',
    remappings=[
        ('/input_topic', '/renamed_input'),
        ('/output_topic', '/renamed_output'),
    ]
)
```

---

## **⚠️ Common Mistakes & Troubleshooting**

### **❌ Mistake 1: Forgot to build**
```bash
# Error: Package 'ce_robot_launch' not found
```
**Fix:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_launch
source install/setup.bash
```

### **❌ Mistake 2: Wrong executable name**
```bash
# Error: Executable 'hw_status_pub' not found
```
**Fix:** Check your `setup.py` in ce_robot package:

**Required entry points for 07_Launch (11 total from modules 00-06):**
```python
entry_points={
    'console_scripts': [
        '00_first_node = ce_robot.first_node:main',
        "01_first_pub = ce_robot.first_publisher:main",
        "01_first_sub = ce_robot.first_subscriber:main",
        "02_add_two_server = ce_robot.add_two_ints_server:main",
        "02_add_two_client = ce_robot.add_two_ints_client:main",
        "03_hw_status_publisher = ce_robot.HardwareStatus_publish:main",
        "04_CalRect_server = ce_robot.CalRect_server:main",
        "04_CalRect_client = ce_robot.CalRect_client:main",
        "05_robot_tag_param = ce_robot.robot_tag_param_pub:main",
        "06_count_until_server = ce_robot.count_until_server:main",
        "06_count_until_client = ce_robot.count_until_client:main",
    ],
},

```

### **💡 Debugging Tips**

1. **Check if launch file is found:**
```bash
ros2 launch ce_robot_launch --help
```

2. **See available arguments:**
```bash
ros2 launch ce_robot_launch robot_system_launch.py --show-args
```

3. **Test nodes individually first:**
```bash
# Before launch file, make sure nodes work:
ros2 run ce_robot 05_robot_tag_param
```

4. **Check build output:**
```bash
colcon build --packages-select ce_robot_launch --event-handlers console_direct+
```

---

## **⚠️ Troubleshooting**

### **Issue: "Package not found"**
- **Cause:** Package not built or sourced
- **Solution:** `colcon build` and `source install/setup.bash`

### **Issue: "Node failed to start"**
- **Cause:** Executable not found
- **Solution:** Verify executable name in setup.py

### **Issue: "Parameter not found"**
- **Cause:** Parameter name mismatch
- **Solution:** Check parameter names in node code

### **Issue: "Service call fails"**
- **Cause:** Service name mismatch or node not running
- **Solution:** Use `ros2 service list` to verify

---

## **📚 Resources**

- [ROS 2 Launch Documentation](https://docs.ros.org/en/jazzy/Concepts/Intermediate/Launch-system.html)
- [ROS 2 Launch Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch-system.html)
- [Launch File Substitutions](https://docs.ros.org/en/jazzy/How-To-Guides/Launch-file-different-formats.html)
- [ROS 2 Launch Commands](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Launch.html)

---

## **✅ Completion Checklist**

Mark each item as you complete it:

### **Package Setup**
- [ ] Created `ce_robot_launch` package
- [ ] Updated `CMakeLists.txt` to install launch directory
- [ ] Updated `package.xml` with dependencies
- [ ] Created `launch/` directory
- [ ] Package builds without errors

### **Simple Launch (simple_launch.py)**
- [ ] Created file with correct imports
- [ ] Defined `generate_launch_description()` function
- [ ] Declared one Node with parameters
- [ ] Returns LaunchDescription
- [ ] Builds successfully
- [ ] Launches and node runs correctly
- [ ] Can verify with `ros2 node list`

### **Complex Launch (robot_system_launch.py)**
- [ ] Created file with argument imports
- [ ] Declared launch arguments (robot_name, publish_rate)
- [ ] Used LaunchConfiguration to get arg values
- [ ] Declared multiple nodes (3+ nodes)
- [ ] Passed arguments to node parameters
- [ ] Returns LaunchDescription with args and nodes
- [ ] Builds successfully
- [ ] Launches with default values
- [ ] Can override arguments from command line
- [ ] `--show-args` displays arguments correctly

### **Verification**
- [ ] All nodes appear in `ros2 node list`
- [ ] Topics are publishing (`ros2 topic list`)
- [ ] Services are available (`ros2 service list`)
- [ ] Parameters applied correctly
- [ ] Output visible in terminal
- [ ] Can stop all nodes with Ctrl+C

### **Understanding**
- [ ] Understand purpose of launch files
- [ ] Know how to declare nodes
- [ ] Know how to pass parameters
- [ ] Know how to use launch arguments
- [ ] Can modify launch files confidently
- [ ] Ready for Lab_Exercises.md

---

## **✅ Verification Checklist**

- [ ] Launch package created
- [ ] CMakeLists.txt configured
- [ ] package.xml with dependencies
- [ ] Simple launch file working
- [ ] Complex launch file working
- [ ] Arguments passing correctly
- [ ] All nodes start without errors
- [ ] Topics and services available
- [ ] Parameters applied correctly
- [ ] Output visible in terminal

---

