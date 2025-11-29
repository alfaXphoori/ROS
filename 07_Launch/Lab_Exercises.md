# **🎯 ROS 2 Launch Files Lab Exercises**

Master launch file creation, configuration management, and multi-node system automation through progressive hands-on exercises.

---

## **📌 Project Title**

Create and Use ROS 2 Launch Files for Multi-Node System Automation

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🛠 Lab Overview**

This lab provides hands-on exercises to master launch file creation, parameter configuration, and multi-node system management. Each exercise builds upon the previous one, progressing from simple single-node launches through complex multi-node system configurations.

**Duration:** ~2.5 hours
**Level:** Beginner to Intermediate
**Prerequisites:** ROS 2 Jazzy installed, Actions lab completed, familiar with Python

---

## **🎯 Learning Objectives**

By completing this lab, you will be able to:

- ✅ Create basic launch files in Python
- ✅ Declare launch arguments
- ✅ Configure node parameters at startup
- ✅ Use substitutions and compositions
- ✅ Launch multiple nodes with different configurations
- ✅ Manage namespaces and remappings
- ✅ Handle node failures and monitoring
- ✅ Use YAML configuration files
- ✅ Create conditional launch logic
- ✅ Debug launch files with ROS 2 tools

---

## **📊 Lab Architecture**

```
┌─────────────────────────────────────────────────────┐
│ Exercise 1: Basic Launch File                       │
│ (Single node, output to terminal)                   │
└──────────────────┬──────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────┐
│ Exercise 2: Multi-Node Launch with Parameters       │
│ (Multiple nodes, parameter configuration)           │
└──────────────────┬──────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────┐
│ Exercise 3: Advanced Launch with Arguments & Logic  │
│ (Arguments, conditions, remappings)                 │
└─────────────────────────────────────────────────────┘
```

---

## **📚 Learning Path Overview**

| Exercise | Title | Level | Duration | Focus |
|----------|-------|-------|----------|-------|
| 1 | Basic Single-Node Launch | Beginner | 30 min | Fundamentals |
| 2 | Multi-Node with Parameters | Beginner-Intermediate | 40 min | Configuration |
| 3 | Advanced Launch with Arguments | Intermediate | 50 min | Advanced features |

---

## **Exercise 1: Basic Single-Node Launch (Beginner) 🚀**

### **📋 Task**

Create a simple launch file that starts a single node with direct console output.

### **File: simple_publisher_launch.py**

```python
#!/usr/bin/env python3
"""
Exercise 1: Basic Single-Node Launch File
Launches a simple publisher node
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for basic publisher"""
    
    # Create publisher node
    publisher_node = Node(
        package='ce_robot',
        executable='hw_status_param_pub',
        name='publisher_ex1',
        output='screen'
    )
    
    # Return launch description
    return LaunchDescription([
        publisher_node,
    ])
```

### **File: simple_subscriber_launch.py**

```python
#!/usr/bin/env python3
"""
Exercise 1: Basic Subscriber Launch File
Launches a simple subscriber node
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for basic subscriber"""
    
    # Create subscriber node
    subscriber_node = Node(
        package='ce_robot',
        executable='hw_status_callback_pub',
        name='subscriber_ex1',
        output='screen'
    )
    
    # Return launch description
    return LaunchDescription([
        subscriber_node,
    ])
```

### **File: combined_launch.py**

```python
#!/usr/bin/env python3
"""
Exercise 1: Combined Publisher-Subscriber Launch
Launches both publisher and subscriber
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description"""
    
    # Publisher node
    publisher_node = Node(
        package='ce_robot',
        executable='hw_status_param_pub',
        name='publisher_ex1',
        output='screen'
    )
    
    # Subscriber node
    subscriber_node = Node(
        package='ce_robot',
        executable='hw_status_callback_pub',
        name='subscriber_ex1',
        output='screen'
    )
    
    # Return both nodes
    return LaunchDescription([
        publisher_node,
        subscriber_node,
    ])
```

### **Testing Exercise 1**

**Build:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_launch --symlink-install
source install/setup.bash
```

**Run combined launch:**
```bash
ros2 launch ce_robot_launch combined_launch.py
```

**Expected Output (Terminal):**
```
[INFO] [publisher_ex1]: Hardware Status Publisher initialized
[INFO] [publisher_ex1]: Robot: robot_default (ID: 1)
[INFO] [publisher_ex1]: Publish Rate: 1.0 Hz
[INFO] [publisher_ex1]: Published: robot_default - Temp: 25°C - Count: 1
[INFO] [subscriber_ex1]: Hardware Status Publisher with Callbacks started
[INFO] [subscriber_ex1]: === Current Parameters ===
[INFO] [subscriber_ex1]: debug_mode: False
```

**Verify in another terminal:**
```bash
ros2 node list
ros2 topic list
ros2 topic echo /hardware_status
```

### **Key Concepts**

- Basic launch file structure with `generate_launch_description()`
- Node declaration with package, executable, and name
- Output configuration (`screen`, `log`)
- Returning LaunchDescription with node list

---

## **Exercise 2: Multi-Node with Parameters (Beginner-Intermediate) ⚙️**

### **📋 Task**

Create a launch file that starts multiple nodes with configured parameters.

### **File: multi_node_launch.py**

```python
#!/usr/bin/env python3
"""
Exercise 2: Multi-Node Launch with Parameters
Launches publisher, subscriber, and server with configured parameters
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with multiple nodes and parameters"""
    
    # Publisher with custom parameters
    publisher_node = Node(
        package='ce_robot',
        executable='hw_status_param_pub',
        name='hw_publisher_ex2',
        output='screen',
        parameters=[
            {'robot_name': 'robot_lab2'},
            {'robot_number': 200},
            {'publish_rate': 2.0},
            {'debug_mode': False},
        ]
    )
    
    # Subscriber with custom parameters
    subscriber_node = Node(
        package='ce_robot',
        executable='hw_status_callback_pub',
        name='hw_subscriber_ex2',
        output='screen',
        parameters=[
            {'robot_name': 'hw_monitor'},
            {'robot_number': 0},
            {'publish_rate': 2.0},
            {'debug_mode': True},
            {'temperature_offset': 5},
        ]
    )
    
    # Server node
    service_server = Node(
        package='ce_robot',
        executable='cal_rect_server',
        name='rect_server_ex2',
        output='screen'
    )
    
    # Return all nodes
    return LaunchDescription([
        publisher_node,
        subscriber_node,
        service_server,
    ])
```

### **File: launch/config/system_params.yaml**

```yaml
# System-wide parameter configuration
hw_publisher_ex2:
  ros__parameters:
    robot_name: "robot_lab2"
    robot_number: 200
    publish_rate: 2.0
    debug_mode: false

hw_subscriber_ex2:
  ros__parameters:
    robot_name: "hw_monitor"
    robot_number: 0
    publish_rate: 2.0
    debug_mode: true
    temperature_offset: 5

rect_server_ex2:
  ros__parameters:
    max_dimension: 1000.0
    min_dimension: 0.1
```

### **Testing Exercise 2**

**Run launch:**
```bash
ros2 launch ce_robot_launch multi_node_launch.py
```

**Verify nodes:**
```bash
ros2 node list
```

**Expected output:**
```
/hw_publisher_ex2
/hw_subscriber_ex2
/rect_server_ex2
```

**Check parameters:**
```bash
ros2 param list /hw_publisher_ex2
ros2 param get /hw_publisher_ex2 robot_name
ros2 param get /hw_publisher_ex2 publish_rate
```

**Test service:**
```bash
ros2 service call /rect_server_ex2/cal_rectangle \
  ce_robot_interfaces/srv/CalRectangle \
  "{length: 10.0, width: 5.0}"
```

### **Key Concepts**

- Multiple Node declarations
- Parameter configuration with dictionary
- Package dependencies in launch
- Service availability after launch

---

## **Exercise 3: Advanced Launch with Arguments & Logic (Intermediate) 🎨**

### **📋 Task**

Create advanced launch file with configurable arguments, conditional logic, and remappings.

### **File: advanced_launch.py**

```python
#!/usr/bin/env python3
"""
Exercise 3: Advanced Launch File with Arguments and Logic
Demonstrates arguments, substitutions, and conditional node launching
"""

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate advanced launch description"""
    
    # Declare launch arguments with defaults
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot_advanced',
        description='Name of the robot'
    )
    
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='300',
        description='Robot ID number'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Publishing rate in Hz'
    )
    
    enable_monitoring_arg = DeclareLaunchArgument(
        'enable_monitoring',
        default_value='true',
        description='Enable monitoring subscriber'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable debug logging'
    )
    
    # Get launch configurations
    robot_name = LaunchConfiguration('robot_name')
    robot_id = LaunchConfiguration('robot_id')
    publish_rate = LaunchConfiguration('publish_rate')
    enable_monitoring = LaunchConfiguration('enable_monitoring')
    debug_mode = LaunchConfiguration('debug_mode')
    
    # Publisher node with dynamic parameters
    publisher_node = Node(
        package='ce_robot',
        executable='hw_status_param_pub',
        name='hw_publisher_ex3',
        output='screen',
        parameters=[
            {'robot_name': robot_name},
            {'robot_number': robot_id},
            {'publish_rate': publish_rate},
            {'debug_mode': debug_mode},
        ],
        remappings=[
            ('/hardware_status', '/robot_hw_status'),
        ]
    )
    
    # Conditional subscriber (only launched if enabled)
    subscriber_node = Node(
        package='ce_robot',
        executable='hw_status_callback_pub',
        name='hw_monitor_ex3',
        output='screen',
        parameters=[
            {'robot_name': 'system_monitor'},
            {'debug_mode': True},
        ],
        remappings=[
            ('/hardware_status', '/robot_hw_status'),
        ],
        condition=IfCondition(enable_monitoring)
    )
    
    # Service server
    service_server = Node(
        package='ce_robot',
        executable='cal_rect_server',
        name='geometry_server_ex3',
        output='screen'
    )
    
    # Return all components
    return LaunchDescription([
        # Arguments
        robot_name_arg,
        robot_id_arg,
        publish_rate_arg,
        enable_monitoring_arg,
        debug_mode_arg,
        
        # Nodes
        publisher_node,
        subscriber_node,
        service_server,
    ])
```

### **Testing Exercise 3**

**Build:**
```bash
colcon build --packages-select ce_robot_launch --symlink-install
source install/setup.bash
```

**Test 1 - Default arguments:**
```bash
ros2 launch ce_robot_launch advanced_launch.py
```

**Test 2 - Custom robot name:**
```bash
ros2 launch ce_robot_launch advanced_launch.py \
  robot_name:=robot_warehouse \
  robot_id:=500 \
  publish_rate:=5.0
```

**Test 3 - Disable monitoring:**
```bash
ros2 launch ce_robot_launch advanced_launch.py \
  enable_monitoring:=false
```

**Test 4 - Enable debug mode:**
```bash
ros2 launch ce_robot_launch advanced_launch.py \
  debug_mode:=true
```

**Expected Output:**
```
[INFO] [hw_publisher_ex3]: Robot: robot_warehouse (ID: 500)
[INFO] [hw_publisher_ex3]: Publish Rate: 5.0 Hz
[INFO] [geometry_server_ex3]: Rectangle Service Server started
```

**Verify remappings:**
```bash
ros2 topic list | grep robot_hw_status
ros2 topic echo /robot_hw_status
```

### **Key Concepts**

- Launch arguments with `DeclareLaunchArgument`
- Launch configurations with `LaunchConfiguration`
- Topic remapping with `remappings`
- Conditional node launching with `IfCondition`
- Parameter substitution in node configuration

---

## **Commands Reference**

```bash
# List available launch files
ros2 launch package_name --help

# Run launch file
ros2 launch package_name launch_file.py

# Run with arguments
ros2 launch package_name launch_file.py arg1:=value1 arg2:=value2

# Debug launch file (verbose output)
ros2 launch package_name launch_file.py -d

# List nodes from launch
ros2 node list

# Monitor topics
ros2 topic list
ros2 topic echo /topic_name

# Check parameters
ros2 param list /node_name
ros2 param get /node_name param_name
```

---

## **✅ Completion Checklist**

- [ ] Exercise 1: Basic Single-Node Launch
  - [ ] simple_publisher_launch.py created
  - [ ] simple_subscriber_launch.py created
  - [ ] combined_launch.py created and working
  - [ ] Output visible in terminal

- [ ] Exercise 2: Multi-Node with Parameters
  - [ ] multi_node_launch.py created
  - [ ] Multiple nodes launch simultaneously
  - [ ] Parameters applied correctly
  - [ ] Service server accessible
  - [ ] Nodes appear in `ros2 node list`

- [ ] Exercise 3: Advanced Launch
  - [ ] advanced_launch.py created
  - [ ] Arguments work with default values
  - [ ] Custom arguments override defaults
  - [ ] Conditional launching works
  - [ ] Remappings redirect topics correctly

- [ ] All packages build successfully
- [ ] All launch files run without errors
- [ ] Arguments and parameters work
- [ ] Multi-node systems start correctly
- [ ] Services and topics accessible

---

## **💡 Tips & Tricks**

1. **Always use screen output for debugging:**
   ```python
   output='screen'  # Shows logs in terminal
   ```

2. **Use substitutions for flexibility:**
   ```python
   robot_name = LaunchConfiguration('robot_name')
   {'robot_name': robot_name}
   ```

3. **Remap topics to avoid conflicts:**
   ```python
   remappings=[
       ('/old_topic', '/new_topic'),
   ]
   ```

4. **Use conditions for optional nodes:**
   ```python
   condition=IfCondition(enable_flag)
   ```

5. **Test launch files incrementally:**
   - Start with single node
   - Add nodes one by one
   - Verify each step

---

**🎓 Congratulations! You've completed the ROS 2 Launch Files Lab!** 🚀✨
