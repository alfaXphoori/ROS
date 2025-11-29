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

---

## **� Architecture Diagram**

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
│                   │                                       │
│                   ▼                                       │
│  ┌──────────────────────────────────────────────────┐   │
│  │  ROS 2 Launch Service                            │   │
│  │  (Parses & executes launch file)                 │   │
│  └────────────────┬─────────────────────────────────┘   │
│                   │                                       │
│    ┌──────┬──────┬──────┬──────────────────────┐         │
│    ▼      ▼      ▼      ▼                      ▼         │
│  ┌────┐ ┌────┐ ┌─────┐ ┌─────┐  ┌──────────┐           │
│  │Node│ │Node│ │Node │ │ ... │  │Parameter │           │
│  │ 1  │ │ 2  │ │ N   │ │     │  │  Server  │           │
│  └────┘ └────┘ └─────┘ └─────┘  └──────────┘           │
│                                                         │
│  Publishing:                                           │
│  - Topics from all nodes                               │
│  - Available services                                  │
│  - Parameter values                                    │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

---

## **Step 1: Create Launch Package**

### **Create Package Structure**

```bash
cd ~/ros2_ws/src
ros2 pkg create ce_robot_launch
cd ce_robot_launch
rm -rf include src
mkdir launch
```

### **Directory Structure**

```bash
ce_robot_launch/
├── launch/
│   ├── ce_boot_launch.py
│   ├── simple_launch.py
│   ├── config/
│   │   └── robot_config.yaml
│   └── params/
│       └── parameters.yaml
├── CMakeLists.txt
└── package.xml
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

## **Step 2: Create Simple Launch File**

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
        executable='hw_status_param_pub',
        name='hw_publisher',
        output='screen',
        parameters=[
            {'robot_name': 'robot_simple'},
            {'robot_number': 1},
            {'publish_rate': 2.0},
        ]
    )
    
    # Return launch description with all nodes
    return LaunchDescription([
        publisher_node,
    ])
```

---

## **Step 3: Create Complex Launch File with Configuration**

### **File: ce_boot_launch.py**

```python
#!/usr/bin/env python3
"""
Boot Launch File
Launches multiple nodes with configuration from YAML
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
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot_bootup',
        description='Name of the robot'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Publishing rate in Hz'
    )
    
    # Get launch configuration values
    robot_name = LaunchConfiguration('robot_name')
    publish_rate = LaunchConfiguration('publish_rate')
    
    # Define nodes
    hw_publisher = Node(
        package='ce_robot',
        executable='hw_status_param_pub',
        name='hw_publisher',
        output='screen',
        parameters=[
            {'robot_name': robot_name},
            {'robot_number': 100},
            {'publish_rate': publish_rate},
        ]
    )
    
    hw_subscriber = Node(
        package='ce_robot',
        executable='hw_status_callback_pub',
        name='hw_subscriber',
        output='screen',
        parameters=[
            {'robot_name': 'hw_monitor'},
            {'debug_mode': True},
        ]
    )
    
    rect_server = Node(
        package='ce_robot',
        executable='cal_rect_server',
        name='rect_server',
        output='screen'
    )
    
    # Return launch description
    return LaunchDescription([
        # Arguments
        robot_name_arg,
        publish_rate_arg,
        
        # Nodes
        hw_publisher,
        hw_subscriber,
        rect_server,
    ])
```

### **File: launch/params/parameters.yaml**

```yaml
# Global parameters configuration
hw_publisher:
  ros__parameters:
    robot_name: "robot_bootup"
    robot_number: 100
    publish_rate: 2.0
    debug_mode: false

hw_subscriber:
  ros__parameters:
    robot_name: "hw_monitor"
    debug_mode: true
    temperature_offset: 0

rect_server:
  ros__parameters:
    max_dimension: 1000
    min_dimension: 0.1
```

---

## **Step 4: Build and Run**

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
ros2 launch ce_robot_launch ce_boot_launch.py \
  robot_name:=robot_warehouse \
  publish_rate:=5.0
```

### **List Launch Files**

```bash
ros2 launch ce_robot_launch --help
```

---

## **Step 5: Verify Running System**

### **List Nodes**

```bash
ros2 node list
```

**Expected Output:**
```
/hw_publisher
/hw_subscriber
/rect_server
```

### **List Topics**

```bash
ros2 topic list
```

### **List Services**

```bash
ros2 service list
```

### **Monitor Topic Output**

```bash
ros2 topic echo /hardware_status
```

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

## **🚀 Next Steps**

After mastering launch files, continue with:

### **Option 1: Simulation (8_Simulation) - Recommended**
**Test your systems in realistic 3D environments**
- 🤖 Webots simulator integration
- 🎮 Physics simulation and visualization
- 🔄 ROS 2 middleware connection
- 🎯 Virtual testing before hardware
- **Duration:** ~3 hours | **Level:** Advanced

### **Recommended Learning Path:**
```
Parameters ➜ Actions ➜ Launch Files ➜ Simulation
           (Done)  (Done)   (Done)      (Next!)
```

---

**🎓 Congratulations! You've learned ROS 2 Launch Files!** 🚀✨
