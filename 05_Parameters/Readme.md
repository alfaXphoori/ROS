# **⚙️ ROS 2 Parameters Fundamentals**

Learn how to use parameters for dynamic node configuration and runtime customization in ROS 2.

---

## **📌 Project Title**

Dynamic Node Configuration with ROS 2 Parameters

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🛠 Overview**

Parameters in ROS 2 provide a powerful mechanism for configuring nodes without recompiling code. Unlike topics (streaming data) and services (request-response), parameters are persistent configuration values that control node behavior and can be modified at runtime.

**Key Capabilities:**
- ✅ Set default values at node initialization
- ✅ Override parameters via command line
- ✅ Load configurations from YAML files
- ✅ Modify parameters while nodes are running
- ✅ Validate parameter changes with callbacks
- ✅ Save and restore parameter configurations

**What You'll Learn:**
- Declaring parameters with types and defaults
- Reading and writing parameter values
- Using parameter callbacks for validation
- Creating and loading parameter files (.yaml)
- Runtime parameter modification and introspection
- Best practices for parameter management

---

## **📊 Architecture Diagram**

```
┌─────────────────────────────────────────────┐
│   ROS 2 Node with Parameters                │
│                                             │
│  ┌──────────────────────────────────────┐   │
│  │ Parameter Declarations:              │   │
│  │ • robot_name: string = "robot_001"   │   │
│  │ • robot_number: int = 1              │   │
│  │ • publish_rate: double = 1.0         │   │
│  │ • debug_mode: bool = false           │   │
│  └──────────────────────────────────────┘   │
│               │                             │
│               ▼                             │
│  ┌──────────────────────────────────────┐   │
│  │ Parameter Callbacks:                 │   │
│  │ • Validate new values                │   │
│  │ • Update node behavior               │   │
│  │ • Reconfigure timers/state           │   │
│  └──────────────────────────────────────┘   │
└──────────────┬──────────────────────────────┘
               │
    Access via ROS 2 CLI / Parameter Files
               │
               ▼
┌─────────────────────────────────────────────┐
│   Parameter Management                      │
│   • ros2 param list                         │
│   • ros2 param get/set                      │
│   • YAML configuration files                │
│   • Runtime introspection                   │
└─────────────────────────────────────────────┘
```

---

## **Example: Hardware Status Publisher with Parameters**

This example demonstrates practical parameter usage: a publisher node that reports robot hardware status with configurable identification, timing, and debug settings.

### **Configuration Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot_name` | string | `"robot_default"` | Robot identifier/name |
| `robot_number` | int | `1` | Unique robot ID number |
| `publish_rate` | double | `1.0` | Publishing frequency (Hz) |
| `debug_mode` | bool | `false` | Enable verbose logging |

**Use Case:** Configure different robots with unique identities and adjust behavior without code changes.

---

## **Step 1: Create Parameterized Publisher Node**

The publisher node declares parameters, reads their values, and responds to runtime changes through callbacks.

### **File: HwStatus_para_publish.py**

```python
#!/usr/bin/env python3
"""
Parameterized Hardware Status Publisher
Demonstrates parameter declaration, access, and callbacks
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from ce_robot_interfaces.msg import HardwareStatus


class HardwareStatusPublisher(Node):
    def __init__(self):
        super().__init__('hw_status_publisher')
        
        # Declare parameters with defaults and descriptors
        self.declare_parameter(
            'robot_name',
            'robot_default',
            ParameterDescriptor(
                description='Name/identifier of the robot'
            )
        )
        
        self.declare_parameter(
            'robot_number',
            1,
            ParameterDescriptor(
                description='Robot ID number (1-1000)'
            )
        )
        
        self.declare_parameter(
            'publish_rate',
            1.0,
            ParameterDescriptor(
                description='Publishing frequency in Hz'
            )
        )
        
        self.declare_parameter(
            'debug_mode',
            False,
            ParameterDescriptor(
                description='Enable debug logging'
            )
        )
        
        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.robot_number = self.get_parameter('robot_number').value
        publish_rate = self.get_parameter('publish_rate').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # Create publisher
        self.publisher = self.create_publisher(
            HardwareStatus,
            'hardware_status_para',
            10
        )
        
        # Create timer based on publish_rate
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(
            timer_period,
            self.timer_callback
        )
        
        # Add callback for parameter changes
        self.add_on_set_parameters_callback(
            self.parameter_callback
        )
        
        self.message_count = 0
        
        self.get_logger().info('⚙️  Hardware Status Publisher initialized')
        self.get_logger().info(f'   Robot: {self.robot_name} (ID: {self.robot_number})')
        self.get_logger().info(f'   Publish Rate: {publish_rate} Hz')
        self.get_logger().info(f'   Debug Mode: {self.debug_mode}')

    def timer_callback(self):
        """Publish HardwareStatus message"""
        msg = HardwareStatus()
        msg.name_robot = self.robot_name
        msg.number_robot = self.robot_number
        msg.temperature = 25 + (self.message_count % 5)
        msg.motor_ready = True
        msg.debug_message = f'Message #{self.message_count} from {self.robot_name}'
        
        self.publisher.publish(msg)
        self.message_count += 1
        
        if self.debug_mode:
            self.get_logger().info(
                f'📡 Published: {msg.name_robot} - Temp: {msg.temperature}°C'
            )

    def parameter_callback(self, params):
        """Handle parameter changes at runtime"""
        for param in params:
            if param.name == 'robot_name':
                self.robot_name = param.value
                self.get_logger().info(f'✅ Updated robot_name = {self.robot_name}')
                
            elif param.name == 'robot_number':
                self.robot_number = param.value
                self.get_logger().info(f'✅ Updated robot_number = {self.robot_number}')
                
            elif param.name == 'debug_mode':
                self.debug_mode = param.value
                self.get_logger().info(f'✅ Updated debug_mode = {self.debug_mode}')
                
            elif param.name == 'publish_rate':
                publish_rate = param.value
                timer_period = 1.0 / publish_rate
                self.timer.cancel()
                self.timer = self.create_timer(timer_period, self.timer_callback)
                self.get_logger().info(f'✅ Updated publish_rate = {publish_rate} Hz')
        
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPublisher()
    
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

---

## **Step 2: Package Configuration**

### **Update setup.py**

Add the entry point for the parameterized publisher:

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
            '05_hw_para = ce_robot.HwStatus_para_publish:main',
        ],
    },
)
```

### **Update package.xml**

Ensure dependencies are included:

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

### **Build the Package**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

---

## **Step 3: Running with Different Parameter Configurations**

### **Method 1: Default Parameters**

Run the node with built-in default values:

**Terminal 1:**
```bash
ros2 run ce_robot 05_hw_para
```

**Expected Output:**
```
[INFO] [hw_status_publisher]: ⚙️  Hardware Status Publisher initialized
[INFO] [hw_status_publisher]:    Robot: robot_default (ID: 1)
[INFO] [hw_status_publisher]:    Publish Rate: 1.0 Hz
[INFO] [hw_status_publisher]:    Debug Mode: False
```

### **Method 2: Command-Line Parameters**

Override parameters directly from the command line:

**Terminal 1:**
```bash
ros2 run ce_robot 05_hw_para --ros-args \
  -p robot_name:=robot_ce6541 \
  -p robot_number:=6541 \
  -p publish_rate:=2.0 \
  -p debug_mode:=true
```

**Expected Output:**
```
[INFO] [hw_status_publisher]: ⚙️  Hardware Status Publisher initialized
[INFO] [hw_status_publisher]:    Robot: robot_ce6541 (ID: 6541)
[INFO] [hw_status_publisher]:    Publish Rate: 2.0 Hz
[INFO] [hw_status_publisher]:    Debug Mode: True
[INFO] [hw_status_publisher]: 📡 Published: robot_ce6541 - Temp: 25°C
[INFO] [hw_status_publisher]: 📡 Published: robot_ce6541 - Temp: 26°C
```

### **Method 3: Parameter File (.yaml)**

Create a configuration file for reusable parameter sets.

**Create file: robot_config.yaml**
```yaml
hw_status_publisher:
  ros__parameters:
    robot_name: "robot_warehouse"
    robot_number: 100
    publish_rate: 5.0
    debug_mode: true
```

**Run with parameter file:**
```bash
ros2 run ce_robot 05_hw_para --ros-args --params-file robot_config.yaml
```

**Expected Output:**
```
[INFO] [hw_status_publisher]: ⚙️  Hardware Status Publisher initialized
[INFO] [hw_status_publisher]:    Robot: robot_warehouse (ID: 100)
[INFO] [hw_status_publisher]:    Publish Rate: 5.0 Hz
[INFO] [hw_status_publisher]:    Debug Mode: True
[INFO] [hw_status_publisher]: 📡 Published: robot_warehouse - Temp: 25°C
```

---

## **Step 4: Runtime Parameter Management**

### **List All Parameters**

View all parameters for the running node:

```bash
ros2 param list
```

**Output:**
```
/hw_status_publisher:
  debug_mode
  publish_rate
  robot_name
  robot_number
  use_sim_time
```

### **Get Parameter Value**

Retrieve the current value of a specific parameter:

```bash
ros2 param get /hw_status_publisher robot_name
```

**Output:**
```
String value is: robot_ce6541
```

### **Set Parameter at Runtime**

Modify parameter while node is running:

**Terminal 2:**
```bash
ros2 param set /hw_status_publisher debug_mode true
```

**Output:**
```
Set parameter successful
```

**Node Output (Terminal 1):**
```
[INFO] [hw_status_publisher]: ✅ Updated debug_mode = True
[INFO] [hw_status_publisher]: 📡 Published: robot_ce6541 - Temp: 27°C
```

### **Change Publishing Rate at Runtime**

```bash
ros2 param set /hw_status_publisher publish_rate 10.0
```

**Node Output:**
```
[INFO] [hw_status_publisher]: ✅ Updated publish_rate = 10.0 Hz
```

### **Get Parameter Description**

View parameter metadata:

```bash
ros2 param describe /hw_status_publisher robot_name
```

**Output:**
```
Parameter name: robot_name
  Type: string
  Description: Name/identifier of the robot
  Constraints: none
```

### **Dump All Parameters to File**

Save current configuration for later use:

```bash
ros2 param dump /hw_status_publisher > current_params.yaml
```

---

## **📝 Key Concepts**

### **Parameter Types**

ROS 2 supports the following parameter types:

| Type | Python Type | Example |
|------|-------------|---------|
| `bool` | `bool` | `True`, `False` |
| `integer` | `int` | `1`, `42`, `-10` |
| `double` | `float` | `1.5`, `3.14` |
| `string` | `str` | `"robot_001"` |
| `byte_array` | `bytes` | `b'\x00\x01'` |

### **Parameter Declaration**

Parameters must be declared before use:

```python
self.declare_parameter(
    'parameter_name',        # Parameter name
    default_value,          # Default value (defines type)
    ParameterDescriptor(
        description='Human-readable description',
        read_only=False     # Allow runtime modification
    )
)
```

### **Parameter Callbacks**

Handle parameter changes at runtime:

```python
def parameter_callback(self, params):
    """Called when parameters are modified"""
    for param in params:
        if param.name == 'my_param':
            self.my_value = param.value
            # Update node behavior
    
    return SetParametersResult(successful=True)

# Register callback
self.add_on_set_parameters_callback(self.parameter_callback)
```

### **Parameter Scope**

- **Node-level:** Parameters belong to specific nodes
- **Namespace:** Nodes can have namespaces (`/ns/node_name`)
- **Full path:** `/namespace/node_name:parameter_name`

---

## **📊 Parameters vs Topics vs Services**

| Feature | Parameters | Topics | Services |
|---------|------------|--------|----------|
| **Purpose** | Configuration | Data streaming | Request-response |
| **Persistence** | ✅ Persistent | ❌ Transient | ❌ One-time |
| **Runtime Modify** | ✅ Yes | ❌ No | ❌ No |
| **Callbacks** | On-set | On-receive | On-request |
| **Use Case** | Settings, config | Sensor data | Commands, queries |

---

## **🔍 Useful ROS 2 Parameter Commands**

### **Parameter Inspection**

```bash
# List all parameters
ros2 param list

# Get parameter value
ros2 param get /node_name parameter_name

# Get parameter description
ros2 param describe /node_name parameter_name

# Dump parameters to file
ros2 param dump /node_name > params.yaml
```

### **Parameter Modification**

```bash
# Set parameter at runtime
ros2 param set /node_name parameter_name value

# Load parameters from file
ros2 param load /node_name params.yaml
```

### **Running with Parameters**

```bash
# Single parameter
ros2 run package node --ros-args -p param:=value

# Multiple parameters
ros2 run package node --ros-args \
  -p param1:=value1 \
  -p param2:=value2

# From YAML file
ros2 run package node --ros-args --params-file config.yaml
```

---

## **💡 Best Practices**

1. **Always declare parameters before use**
2. **Provide descriptive parameter descriptions**
3. **Validate parameter values in callbacks**
4. **Use YAML files for complex configurations**
5. **Group related parameters with common prefixes**
6. **Set reasonable default values**
7. **Document parameter units and valid ranges**

---

## **⚠️ Troubleshooting**

### **Issue: "Parameter not declared"**
- **Cause:** Trying to access parameter before declaring it
- **Solution:** Always call `declare_parameter()` before `get_parameter()`
```python
# ✅ Correct order
self.declare_parameter('speed', 1.0)
speed = self.get_parameter('speed').value
```

### **Issue: "Parameter file not found"**
- **Cause:** Incorrect file path or missing file
- **Solution:** Use absolute path or verify file location
```bash
# Check file exists
ls -la robot_config.yaml

# Use absolute path
ros2 run ce_robot 05_hw_para --ros-args \
  --params-file /absolute/path/to/robot_config.yaml
```

### **Issue: "Parameter type mismatch"**
- **Cause:** YAML value type doesn't match declared type
- **Solution:** Ensure YAML types match parameter declarations
```yaml
# ❌ Wrong - string for int parameter
robot_number: "100"

# ✅ Correct - int for int parameter
robot_number: 100
```

### **Issue: "Cannot modify parameter at runtime"**
- **Cause:** Parameter callback not implemented or returns False
- **Solution:** Add parameter callback that accepts changes
```python
self.add_on_set_parameters_callback(self.parameter_callback)
```

### **Issue: "Parameter changes don't take effect"**
- **Cause:** Node doesn't use updated parameter value
- **Solution:** Read parameter value in callback and update node behavior
```python
def parameter_callback(self, params):
    for param in params:
        if param.name == 'my_param':
            self.my_value = param.value  # Update internal state
    return SetParametersResult(successful=True)
```

---

## **📚 Additional Learning Resources**

### **Official ROS 2 Documentation**
- [ROS 2 Parameters Concepts](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Parameters.html) - Deep dive into parameter architecture
- [Using Parameters (Python)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html) - Step-by-step Python tutorial
- [Creating Parameter Files](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Params-File.html) - YAML configuration guide
- [Parameter Callbacks](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Parameters.html#setting-parameters) - Advanced callback patterns

### **Video Tutorials**
- [ROS 2 Parameters Explained](https://www.youtube.com/results?search_query=ros2+parameters+tutorial) - Visual learning resources
- [Parameter Files in Practice](https://www.youtube.com/results?search_query=ros2+parameter+files) - Real-world examples

### **Community Resources**
- [ROS Answers](https://answers.ros.org/questions/scope:all/sort:activity-desc/tags:ros2/page:1/) - Community Q&A
- [ROS Discourse](https://discourse.ros.org/) - Discussion forum
- [GitHub ROS 2 Examples](https://github.com/ros2/examples) - Official code samples

### **Practice Exercises**

**Beginner Level:**
1. Create a node with 5 different parameter types
2. Modify parameters using command line while node runs
3. Create a YAML file with 10+ parameters
4. Implement parameter validation (reject invalid values)

**Intermediate Level:**
5. Create parameter namespaces (`/robot/navigation/speed`)
6. Build a multi-node system sharing parameter files
7. Implement read-only parameters
8. Add parameter range constraints (min/max values)

**Advanced Level:**
9. Create dynamic reconfiguration system
10. Build parameter inheritance system
11. Implement parameter templates for robot fleets
12. Create parameter monitoring and logging system

---

## **🎯 How to Learn More**

### **Progressive Learning Path**

#### **Phase 1: Foundation (You are here! ✅)**
- ✅ Completed Readme.md basic examples
- ✅ Understand parameter declaration
- ✅ Know how to use command-line parameters
- ✅ Can create and load YAML files

**Next Steps:**
- [ ] Complete Lab_Exercises.md for advanced scenarios
- [ ] Practice with different parameter types
- [ ] Experiment with parameter callbacks

#### **Phase 2: Practical Application**
**Focus:** Apply parameters to real robot scenarios

1. **Configure Robot Behavior**
   - Speed limits, safety zones, operational modes
   - Sensor calibration values
   - Control loop gains (PID tuning)

2. **Multi-Robot Systems**
   - Unique ID assignment
   - Role-based configuration
   - Fleet coordination parameters

3. **Development vs Production**
   - Debug mode toggles
   - Logging levels
   - Performance tuning parameters

**Recommended Lab:** `Lab_Exercises.md` in this directory

#### **Phase 3: Integration with Other Concepts**
**Combine parameters with:**

1. **Launch Files (07_Launch)**
   - Load parameters automatically on startup
   - Configure multiple nodes simultaneously
   - Environment-specific configurations

2. **Actions (06_Action)**
   - Configure action goals with parameters
   - Adjust feedback rates dynamically
   - Set timeout and retry policies

3. **Services (04_Service)**
   - Service behavior configuration
   - Request/response timeout settings
   - Retry and error handling parameters

#### **Phase 4: Advanced Patterns**

1. **Parameter Validation**
```python
def parameter_callback(self, params):
    for param in params:
        if param.name == 'speed':
            if param.value < 0 or param.value > 5.0:
                return SetParametersResult(
                    successful=False,
                    reason='Speed must be 0-5.0 m/s'
                )
    return SetParametersResult(successful=True)
```

2. **Parameter Persistence**
```bash
# Save current config
ros2 param dump /node > robot_20241211_config.yaml

# Version control your configs
git add configs/robot_config.yaml
git commit -m "Updated speed parameters"
```

3. **Parameter Templates**
```yaml
# Template for warehouse robots
warehouse_robot:
  ros__parameters:
    max_speed: 2.5
    safety_distance: 0.5
    charging_threshold: 20.0

# Template for delivery robots  
delivery_robot:
  ros__parameters:
    max_speed: 1.5
    safety_distance: 1.0
    delivery_timeout: 300.0
```

### **Hands-On Projects**

**Project 1: Robot Configuration Manager**
- Create a node that manages multiple robot profiles
- Switch between profiles via service calls
- Validate all parameters on profile change

**Project 2: Fleet Configuration System**
- Single YAML file configures entire robot fleet
- Each robot gets unique parameters based on ID
- Central parameter server for shared configs

**Project 3: Parameter Monitoring Dashboard**
- Subscribe to parameter change events
- Log all parameter modifications
- Create visualization of parameter history

---

## **📖 Extended Reading**

### **Design Patterns**
- **Parameter Namespacing:** Organize related parameters (`/robot/navigation/*`, `/robot/sensors/*`)
- **Default Hierarchies:** System defaults → Robot defaults → User overrides
- **Configuration Profiles:** Dev, staging, production parameter sets
- **Hot Reloading:** Design nodes to handle parameter changes gracefully

### **Real-World Use Cases**
- **Autonomous Vehicles:** Dynamic speed limits based on conditions
- **Manufacturing Robots:** Tool-specific parameters for different operations
- **Drone Swarms:** Individual drone tuning within coordinated system
- **Research Platforms:** Experiment parameters for reproducible results

### **Best Practices from Industry**
- Document all parameters with units and ranges
- Version control parameter files with code
- Create parameter migration scripts for updates
- Test parameter validation thoroughly
- Use parameter namespaces for scalability

---

## **✅ Verification Checklist**

**Basic Implementation:**
- [ ] Parameterized node created
- [ ] Parameters declared with types and defaults
- [ ] Parameter descriptors added
- [ ] Node builds successfully
- [ ] Node runs with default parameters

**Parameter Access:**
- [ ] Default parameters work correctly
- [ ] Command-line parameters override defaults
- [ ] Parameter file (.yaml) loads successfully
- [ ] `ros2 param list` shows all parameters
- [ ] `ros2 param get` retrieves correct values

**Runtime Modification:**
- [ ] Parameter callback implemented
- [ ] `ros2 param set` modifies values successfully
- [ ] Node responds to parameter changes
- [ ] Timer/behavior updates when parameters change
- [ ] Debug output shows parameter updates

**Advanced Features:**
- [ ] Parameter validation implemented
- [ ] Invalid values rejected appropriately
- [ ] Parameter dump/load works
- [ ] Multiple parameter configurations tested
- [ ] Documentation complete

---

## **🚀 Next Steps**

After mastering parameters, continue your ROS 2 journey:

### **Recommended: 6_Action - Long-Running Tasks**
Learn asynchronous tasks with progress feedback and cancellation.
- **Why Next:** Actions build on topics, services, and parameters
- **What You'll Learn:** Goal-based task execution, feedback mechanisms
- **Real-World Use:** Robot navigation, object manipulation, long computations
- **Duration:** ~2-3 hours | **Level:** ⭐⭐⭐

### **Alternative: 7_Launch - System Orchestration**
Master launching multiple nodes with configurations.
- **Why Useful:** Run complex systems with one command
- **What You'll Learn:** Launch files, parameter loading, node coordination
- **Real-World Use:** Starting complete robot applications
- **Duration:** ~1.5 hours | **Level:** ⭐⭐

### **Learning Path:**
```
Parameters ➜ Actions ➜ Launch Files ➜ Simulation
(Completed!)  (Next)    (Then)        (Advanced)
```

---

**🎓 Congratulations! You've mastered ROS 2 parameters!** 🚀✨

You now understand:
- ✅ Parameter declaration and types
- ✅ Runtime parameter modification
- ✅ Parameter callbacks and validation
- ✅ YAML configuration files
- ✅ Parameter introspection tools

**Ready to tackle Actions!** 💪
