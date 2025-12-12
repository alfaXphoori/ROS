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
│  → Read Overview + Architecture                          │
│  → Understand Node(), parameters, arguments              │
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
│  → Create ce_boot_launch.py                             │
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
│                   │                                     │
│                   ▼                                     │
│  ┌──────────────────────────────────────────────────┐   │
│  │  ROS 2 Launch Service                            │   │
│  │  (Parses & executes launch file)                 │   │
│  └────────────────┬─────────────────────────────────┘   │
│                   │                                     │
│    ┌──────┬──────┬──────┬──────────────────────┐        │
│    ▼      ▼      ▼      ▼                      ▼        │
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
        │   ├── 🐍 ce_boot_launch.py       ← Create this
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

### **💡 Code Explanation**

```python
# 1. Import required modules
from launch import LaunchDescription      # Container for all launch items
from launch_ros.actions import Node       # ROS 2 node declaration

# 2. Must have this function - ROS 2 looks for it!
def generate_launch_description():
    
    # 3. Declare each node
    publisher_node = Node(
        package='ce_robot',              # Which package?
        executable='hw_status_param_pub', # Which node executable?
        name='hw_publisher',              # What to call it?
        output='screen',                  # Show output in terminal
        parameters=[                      # Set parameters
            {'robot_name': 'robot_simple'},
            {'robot_number': 1},
            {'publish_rate': 2.0},
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
[INFO] [hw_publisher]: Robot Name: robot_simple
[INFO] [hw_publisher]: Robot Number: 1
[INFO] [hw_publisher]: Publishing at 2.0 Hz
```

**Verify in another terminal:**
```bash
ros2 node list          # Should see /hw_publisher
ros2 topic list         # Should see /hardware_status
ros2 topic echo /hardware_status  # See messages
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
touch ce_boot_launch.py
chmod +x ce_boot_launch.py
```

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

### **💡 Understanding Launch Arguments**

```python
# Step 1: Declare what arguments you accept
robot_name_arg = DeclareLaunchArgument(
    'robot_name',                    # Argument name
    default_value='robot_bootup',    # Default if not provided
    description='Name of the robot'  # Help text
)

# Step 2: Get the value (even if from command line)
robot_name = LaunchConfiguration('robot_name')

# Step 3: Use it in your node
Node(
    package='ce_robot',
    executable='hw_status_param_pub',
    parameters=[
        {'robot_name': robot_name},  # Uses the argument value!
    ]
)
```

### **🧪 Test Complex Launch**

```bash
# Rebuild
cd ~/ros2_ws
colcon build --packages-select ce_robot_launch --symlink-install
source install/setup.bash

# Test 1: Use default values
ros2 launch ce_robot_launch ce_boot_launch.py
```

**Expected: 3 nodes start (hw_publisher, hw_subscriber, rect_server)**

```bash
# Test 2: Override arguments
ros2 launch ce_robot_launch ce_boot_launch.py \
  robot_name:=warehouse_bot \
  publish_rate:=5.0
```

**Expected: Same nodes, but robot_name="warehouse_bot", rate=5.0 Hz**

```bash
# Test 3: Check what arguments are available
ros2 launch ce_robot_launch ce_boot_launch.py --show-args
```

**Expected Output:**
```
Arguments (pass arguments as '<name>:=<value>'):
    'robot_name':
        Name of the robot
        (default: 'robot_bootup')
    'publish_rate':
        Publishing rate in Hz
        (default: '1.0')
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

## **💪 Challenge: Launch Your Action Nodes!**

Now that you know launch files, try creating a launch file for your 06_Action nodes:

```python
#!/usr/bin/env python3
"""
Robot Actions Launch
Launches battery, navigation, and gripper systems
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch all robot action servers and clients"""
    
    # Battery charging system
    battery_server = Node(
        package='ce_robot',
        executable='06_battery_charging_server',
        name='battery_server',
        output='screen'
    )
    
    battery_client = Node(
        package='ce_robot',
        executable='06_battery_charging_client',
        name='battery_client',
        output='screen'
    )
    
    # Navigation system
    navigate_server = Node(
        package='ce_robot',
        executable='06_navigate_server',
        name='navigate_server',
        output='screen'
    )
    
    navigate_client = Node(
        package='ce_robot',
        executable='06_navigate_client',
        name='navigate_client',
        output='screen'
    )
    
    # Gripper system
    gripper_server = Node(
        package='ce_robot',
        executable='06_gripper_server',
        name='gripper_server',
        output='screen'
    )
    
    gripper_client = Node(
        package='ce_robot',
        executable='06_gripper_client',
        name='gripper_client',
        output='screen'
    )
    
    return LaunchDescription([
        battery_server,
        battery_client,
        navigate_server,
        navigate_client,
        gripper_server,
        gripper_client,
    ])
```

**Save as:** `launch/robot_actions_launch.py`

**Test:**
```bash
colcon build --packages-select ce_robot_launch --symlink-install
source install/setup.bash
ros2 launch ce_robot_launch robot_actions_launch.py
```

**Result:** All 6 action nodes start with one command! 🎉

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
ros2 launch ce_robot_launch ce_boot_launch.py \
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
- Multiple nodes: `ce_boot_launch.py`
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

**Required entry points for 07_Launch (55 total):**
```python
entry_points={
    'console_scripts': [
        # 05_Parameters (required for launch files)
        'hw_status_param_pub = ce_robot.hw_status_param_pub:main',
        'hw_status_callback_pub = ce_robot.hw_status_callback_pub:main',
        
        # 04_Service (alias for launch files)
        'cal_rect_server = ce_robot.CalRect_server:main',
        
        # ... plus 52 other entry points for modules 00-06
    ],
}
```

📝 **Complete setup.py available:** See `99_Test/setup.py` for reference with all 55 entry points

### **❌ Mistake 3: Python syntax error in launch file**
```bash
# Error: SyntaxError: invalid syntax
```
**Fix:** Check:
- Missing commas between parameters
- Missing closing brackets `]` or `)`
- Correct indentation (Python is strict!)

### **❌ Mistake 4: Argument not working**
```bash
# You pass robot_name:=test but it still uses default
```
**Fix:** Make sure you're using `LaunchConfiguration`:
```python
robot_name = LaunchConfiguration('robot_name')  # Get value
Node(
    parameters=[{'robot_name': robot_name}]  # Use value
)
```

### **💡 Debugging Tips**

1. **Check if launch file is found:**
```bash
ros2 launch ce_robot_launch --help
```

2. **See available arguments:**
```bash
ros2 launch ce_robot_launch ce_boot_launch.py --show-args
```

3. **Test nodes individually first:**
```bash
# Before launch file, make sure nodes work:
ros2 run ce_robot hw_status_param_pub
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

### **Complex Launch (ce_boot_launch.py)**
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

## **🚀 Next Steps**

### **Immediate Practice:**
1. **Complete Lab_Exercises.md** - More advanced launch patterns
2. **Create your own launch file** - For your 06_Action robot systems
3. **Experiment** - Try launching different node combinations

### **After Mastering Launch Files:**

### **Option 1: Simulation (08_Simulation) - Highly Recommended Next! 🎮**
**Test your complete robot system in 3D virtual environment**

**What you'll learn:**
- 🤖 **Webots Integration** - Professional robot simulator
- 🎮 **Physics Simulation** - Realistic robot behavior
- 📊 **Sensor Data** - Virtual cameras, lidars, IMU
- 🗺️ **Environment Design** - Create custom worlds
- 🔄 **ROS 2 Bridge** - Connect simulation to your nodes
- 🎯 **Safe Testing** - Break things without breaking things!

**Perfect For:**
- Testing your battery, navigation, and gripper systems
- Experimenting before buying hardware
- Debugging robot logic safely
- Learning advanced robotics concepts

**Duration:** ~3 hours | **Level:** Advanced | **Prerequisites:** Launch files ✅

### **Why Simulation Next?**
```
You've built:  Parameters → Actions → Launch Files
Now use them:  🤖 Simulated Robot → Your Code → Real Results
```

Launch files + Simulation = Complete robot system testing! 🚀

---

### **Alternative Path: Advanced Topics**

If you need specific skills first:
- **tf2 (Transforms)** - Robot coordinate frames
- **Navigation Stack** - Autonomous navigation
- **Computer Vision** - Camera processing
- **Custom Interfaces** - Advanced msgs/srvs

### **Recommended Learning Path:**
```
✅ 01_Basics ➜ ✅ 02_Topics ➜ ✅ 03_Services ➜ ✅ 04_Interfaces
           ↓
✅ 05_Parameters ➜ ✅ 06_Actions ➜ ✅ 07_Launch (You are here!)
           ↓
🎯 08_Simulation ← Next recommended step
           ↓
    Real Hardware / Advanced Topics
```

---

**🎓 Congratulations! You've mastered ROS 2 Launch Files!** 🚀

**You can now:**
- ✅ Start multiple nodes with one command
- ✅ Configure complex robot systems
- ✅ Use launch arguments dynamically
- ✅ Build reusable robot configurations
- ✅ Ready for simulation and real robots!

**Your Progress:**
```
[████████████████████░░] 90% Complete
05_Parameters ✅ → 06_Actions ✅ → 07_Launch ✅ → 08_Simulation 🎯
```

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
