# **⚙️ ROS 2 Parameters Fundamentals**

Learn how to use parameters for dynamic node configuration and runtime customization in ROS 2.

---

## **📌 Project Title**

Create and Use ROS 2 Parameters for Node Configuration

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🛠 Overview**

Parameters in ROS 2 allow nodes to store and dynamically modify configuration values without recompiling code. Unlike topics (continuous data flow) and services (request-response), parameters are persistent configuration values that can be:

- Set before a node starts
- Modified while a node is running
- Saved to files for later reuse
- Declared with default values and types
- Monitored for changes via callbacks

**What You'll Learn:**
- Parameter declaration and initialization
- Reading and writing parameter values
- Using parameter callbacks for dynamic updates
- Parameter files (.yaml) for configuration management
- Parameter introspection and debugging
- Best practices for parameter validation

---

## **📊 Architecture Diagram**

```
┌────────────────────────────────────────────┐
│   Node with Parameters                     │
│                                            │
│  ┌─────────────────────────────────────┐   │
│  │ Declared Parameters:                │   │
│  │ - robot_name (string)               │   │
│  │ - max_speed (double)                │   │
│  │ - debug_mode (bool)                 │   │
│  └─────────────────────────────────────┘   │
│                                            │
│  ┌─────────────────────────────────────┐   │
│  │ Parameter Callbacks:                │   │
│  │ - On parameter set/modified         │   │
│  │ - Validate new values               │   │
│  │ - Update internal state             │   │
│  └─────────────────────────────────────┘   │
└──────────────┬─────────────────────────────┘
               │
        Set via CLI / File / ROS Service
               │
               ▼
┌─────────────────────────────────────────────┐
│   Parameter Server (ROS 2)                  │
│   - Stores all parameter values             │
│   - Broadcasts changes                      │
│   - Provides parameter introspection        │
└─────────────────────────────────────────────┘
```

---

## **Example: Hardware Status Publisher with Parameters**

This lab demonstrates a practical parameter implementation: a publisher node with configurable robot identification and publishing behavior.

### **Parameters Used:**

```python
robot_name: str         # Robot identifier (default: "robot_default")
robot_number: int       # Robot ID number (default: 1)
publish_rate: float     # Publishing frequency in Hz (default: 1.0)
debug_mode: bool        # Enable debug logging (default: false)
```

---

## **Step 1: Create Parameterized Node**

### **File: HwStatus_para_publish.py**

```python
#!/usr/bin/env python3
"""
Parameterized Publisher Node
Publishes HardwareStatus with configurable parameters
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
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
        
        self.get_logger().info(
            f'Hardware Status Publisher initialized'
        )
        self.get_logger().info(
            f'Robot: {self.robot_name} (ID: {self.robot_number})'
        )
        self.get_logger().info(
            f'Publish Rate: {publish_rate} Hz'
        )
        self.get_logger().info(
            f'Debug Mode: {self.debug_mode}'
        )

    def timer_callback(self):
        """Publish HardwareStatus message"""
        msg = HardwareStatus()
        msg.name_robot = self.robot_name
        msg.number_robot = self.robot_number
        msg.temperature = 25 + (self.message_count % 5)
        msg.motor_ready = True
        msg.debug_message = (
            f'Message #{self.message_count} from {self.robot_name}'
        )
        
        self.publisher.publish(msg)
        self.message_count += 1
        
        if self.debug_mode:
            self.get_logger().info(
                f'Published: {msg.name_robot} - Temp: {msg.temperature}°C'
            )

    def parameter_callback(self, params):
        """Handle parameter changes"""
        for param in params:
            if param.name == 'robot_name':
                self.robot_name = param.value
                self.get_logger().info(
                    f'Parameter updated: robot_name = {self.robot_name}'
                )
            elif param.name == 'robot_number':
                self.robot_number = param.value
                self.get_logger().info(
                    f'Parameter updated: robot_number = {self.robot_number}'
                )
            elif param.name == 'debug_mode':
                self.debug_mode = param.value
                self.get_logger().info(
                    f'Parameter updated: debug_mode = {self.debug_mode}'
                )
            elif param.name == 'publish_rate':
                publish_rate = param.value
                timer_period = 1.0 / publish_rate
                self.timer.cancel()
                self.timer = self.create_timer(
                    timer_period,
                    self.timer_callback
                )
                self.get_logger().info(
                    f'Parameter updated: publish_rate = {publish_rate} Hz'
                )
        
        return rcl_interfaces.msg.SetParametersResult(successful=True)


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

## **Step 2: Run the Parameterized Node**

### **Build the Package**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

### **Run with Default Parameters**

```bash
ros2 run ce_robot hw_para
```

**Expected Output:**
```
[INFO] [hw_status_publisher]: Hardware Status Publisher initialized
[INFO] [hw_status_publisher]: Robot: robot_default (ID: 1)
[INFO] [hw_status_publisher]: Publish Rate: 1.0 Hz
[INFO] [hw_status_publisher]: Debug Mode: False
```

### **Run with Custom Parameters (Command Line)**

```bash
ros2 run ce_robot hw_para --ros-args \
  -p robot_name:=robot_ce6541 \
  -p robot_number:=6541 \
  -p publish_rate:=2.0 \
  -p debug_mode:=true
```

**Expected Output:**
```
[INFO] [hw_status_publisher]: Hardware Status Publisher initialized
[INFO] [hw_status_publisher]: Robot: robot_ce6541 (ID: 6541)
[INFO] [hw_status_publisher]: Publish Rate: 2.0 Hz
[INFO] [hw_status_publisher]: Debug Mode: True
[INFO] [hw_status_publisher]: Published: robot_ce6541 - Temp: 25°C
[INFO] [hw_status_publisher]: Published: robot_ce6541 - Temp: 26°C
```

---

## **Step 3: Parameter Introspection & Modification**

### **List All Parameters**

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
```

### **Get Parameter Value**

```bash
ros2 param get /hw_status_publisher robot_name
```

**Output:**
```
String value is: robot_ce6541
```

### **Set Parameter Value at Runtime**

```bash
ros2 param set /hw_status_publisher debug_mode true
```

**Expected Node Output:**
```
[INFO] [hw_status_publisher]: Parameter updated: debug_mode = True
```

### **Get Parameter Descriptor**

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

---

## **Step 4: Using Parameter Files (.yaml)**

### **Create Configuration File: robot_config.yaml**

```yaml
hw_status_publisher:
  ros__parameters:
    robot_name: "robot_warehouse"
    robot_number: 100
    publish_rate: 5.0
    debug_mode: true

# Advanced example with multiple nodes:
# node1:
#   ros__parameters:
#     param1: value1
#     param2: value2
# 
# node2:
#   ros__parameters:
#     param_a: value_a
#     param_b: value_b
```

### **Run with Parameter File**

```bash
ros2 run ce_robot hw_para --ros-args \
  --params-file robot_config.yaml
```

**Expected Output:**
```
[INFO] [hw_status_publisher]: Hardware Status Publisher initialized
[INFO] [hw_status_publisher]: Robot: robot_warehouse (ID: 100)
[INFO] [hw_status_publisher]: Publish Rate: 5.0 Hz
[INFO] [hw_status_publisher]: Debug Mode: True
```

### **Dump Parameters to File**

```bash
ros2 param dump /hw_status_publisher > current_params.yaml
```

---

## **Step 5: Package Configuration**

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
            'hw_para = ce_robot.HwStatus_para_publish:main',
        ],
    },
)
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

---

## **📝 Key Concepts**

### **Parameter Types**
- `bool` - Boolean values (true/false)
- `int` - Integer values
- `double` - Floating-point values
- `string` - Text values
- `byte_array` - Binary data

### **Parameter Declaration**
```python
self.declare_parameter(
    'parameter_name',
    default_value,
    ParameterDescriptor(
        description='Human-readable description'
    )
)
```

### **Parameter Callbacks**
- Triggered when parameter is modified
- Can validate new values
- Can update internal state or timers
- Must return `SetParametersResult`

### **Parameter Scope**
- Node-level: Parameters are scoped to individual nodes
- Namespace: Parameters include node namespace
- Full path: `/namespace/node_name:param_name`

---

## **📊 Parameter vs Topic vs Service**

| Aspect | Parameter | Topic | Service |
|--------|-----------|-------|---------|
| **Purpose** | Configuration | Data stream | One-time request |
| **Persistence** | Persistent | Transient | N/A |
| **Modification** | Runtime | N/A | N/A |
| **Callbacks** | On-set | Subscription | Handler |
| **Use Case** | Settings | Sensors | Queries |

---

## **⚠️ Troubleshooting**

### **Issue: "Parameter not found"**
- **Cause:** Parameter not declared before accessing
- **Solution:** Use `declare_parameter()` before `get_parameter()`

### **Issue: "Parameter file not found"**
- **Cause:** Incorrect file path
- **Solution:** Use absolute path or relative to current directory

### **Issue: "Parameter value type mismatch"**
- **Cause:** Trying to set wrong type
- **Solution:** Ensure YAML types match declared parameter types

### **Issue: "Cannot set parameter on running node"**
- **Cause:** Parameter not declared with proper callback
- **Solution:** Add `add_on_set_parameters_callback()`

---

## **📚 Resources**

- [ROS 2 Parameters Documentation](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Parameters.html)
- [ROS 2 Using Parameters](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
- [ROS 2 Parameter Files](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Params-File.html)
- [ROS 2 Dynamic Reconfiguration](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Parameters.html#setting-parameters)

---

## **✅ Verification Checklist**

- [ ] Parameterized node created
- [ ] Parameters declared with defaults
- [ ] Node runs with default parameters
- [ ] Node runs with command-line parameters
- [ ] Parameter callbacks implemented
- [ ] Runtime parameter modification works
- [ ] Parameter file (.yaml) works
- [ ] `ros2 param list` shows all parameters
- [ ] `ros2 param get` retrieves values correctly
- [ ] `ros2 param set` modifies values at runtime
- [ ] Debug output shows parameter changes

---

## **🚀 Next Steps**

After mastering parameters, you're ready for the next advanced topics:

### **Option 1: Actions (6_Action) - Recommended Next**
**Learn long-running asynchronous tasks with feedback**
- 🎯 For operations that take time and need progress feedback
- ✅ Goal submission, feedback, and result cancellation
- 📊 Real-world examples: robotic manipulation, navigation, image processing
- **Duration:** ~2 hours | **Level:** Intermediate to Advanced

### **Option 2: Launch Files (7_Launch)**
**Automate launching multiple nodes with parameters**
- 🚀 Launch complex systems with one command
- ⚙️ Configure parameters at launch time
- 📝 Create reusable launch configurations
- **Duration:** ~1.5 hours | **Level:** Beginner to Intermediate

### **Option 3: Simulation (8_Simulation)**
**Test your nodes in Webots robot simulator**
- 🤖 Virtual robot testing environment
- 🎮 3D simulation with physics
- 🔄 Integration with ROS 2 nodes
- **Duration:** ~3 hours | **Level:** Advanced

### **Recommended Learning Path:**
```
Parameters ➜ Actions ➜ Launch Files ➜ Simulation
(You are here!)  (Next)  (Then)      (Advanced)
```

---

**🎓 Congratulations! You've learned ROS 2 parameters!** 🚀✨
