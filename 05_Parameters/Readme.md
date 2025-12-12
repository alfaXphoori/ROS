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
┌───────────────────────────────────────────────────────────┐
│   Step 1: Message Definition (ce_robot_interfaces)        │
│   ┌──────────────────────────────────────────────────┐    │
│   │  RobotTag.msg                                    │    │
│   │  • robot_id, robot_type, zone_id                 │    │
│   │  • status, priority_level, max_payload_kg        │    │
│   │  • current_location, assigned_task               │    │
│   │  • operation_hours, firmware_version             │    │
│   │  • safety_certified, error_code                  │    │
│   └──────────────────────────────────────────────────┘    │
└───────────────────────────────────────────────────────────┘
                          ⬇️
┌────────────────────────────────────────────────────────────┐
│   Step 2: Warehouse Robot Node (robot_tag_publisher)       │
│                                                            │
│  ┌────────────────────────────────────────────────────┐    │
│  │ 📋 Parameter Declarations with Validation:         │    │
│  │                                                    │    │
│  │ • robot_id: string = "warehouse_bot_001"           │    │
│  │ • max_speed: double = 2.5 m/s (0.1-5.0)            │    │
│  │ • battery_warning_level: int = 20% (10-50)         │    │
│  │ • status_publish_rate: double = 1.0 Hz (0.1-10.0)  │    │
│  │ • enable_safety_features: bool = true              │    │
│  │ • operation_mode: string = "autonomous"            │    │
│  │   (autonomous | manual | standby)                  │    │
│  └────────────────────────────────────────────────────┘    │
│                          ⬇️                                │
│                                                            │
│  ┌────────────────────────────────────────────────────┐    │
│  │ ⚙️  Parameter Callbacks (Runtime Updates):         │    │
│  │                                                    │    │
│  │ • Validate ranges (speed, battery, rate)           │    │
│  │ • Verify operation modes                           │    │
│  │ • Update robot behavior immediately                │    │
│  │ • Reconfigure timers dynamically                   │    │
│  │ • Log changes with success/error messages          │    │
│  └────────────────────────────────────────────────────┘    │
│                         ⬇️                                 │
│                                                            │
│  ┌────────────────────────────────────────────────────┐    │
│  │ 🤖 Robot Status Publishing:                        │    │
│  │                                                    │    │
│  │ • Battery monitoring (simulated drain)             │    │
│  │ • Temperature tracking (25-35°C)                   │    │
│  │ • Motor readiness (based on mode & safety)         │    │
│  │ • Warning on low battery                           │    │
│  │ • Speed limit enforcement                          │    │
│  └────────────────────────────────────────────────────┘    │
└──────────────┬─────────────────────────────────────────────┘
               │
     ⬆️ Parameter Access Methods ⬇️
               │
    ┌──────────┴──────────┬──────────────────────┐
    │                     │                      │
    ▼                     ▼                      ▼
┌─────────────┐  ┌──────────────────┐  ┌─────────────────┐
│ Command Line│  │  YAML Config     │  │ Runtime (CLI)   │
│             │  │                  │  │                 │
│ --ros-args  │  │ robot_config.yaml│  │ ros2 param set  │
│ -p param:=  │  │                  │  │ ros2 param get  │
│    value    │  │ Load at startup  │  │ ros2 param list │
└─────────────┘  └──────────────────┘  └─────────────────┘
```

---

## **Step 1: Create Custom Message**

Before creating the publisher node, we need to define the custom message type that will carry robot status information.

### **Create Message Interface Package**

First, create a separate package for message definitions:

```bash
cd ~/ros2_ws/src/ce_robot_interfaces/msg/
```

### **Define Custom Messages**

Create the message definition file:

#### **File: ce_robot_interfaces/msg/RobotTag.msg**

```
# Robot Identification Tag
string robot_id                    # Unique robot identifier (e.g., "WH-BOT-042")
string robot_type                  # Robot type: "transport", "delivery", "inspection", "loader"
string zone_id                     # Current operational zone (e.g., "ZONE-A", "LOADING-BAY-3")
int32 fleet_number                 # Fleet assignment number (1-999)

# Operational Status
string status                      # Status: "active", "idle", "charging", "maintenance", "error"
int32 priority_level              # Task priority: 0=lowest, 10=highest
float32 max_payload_kg            # Maximum payload capacity in kilograms

# Location & Assignment
string current_location           # Current location (e.g., "SHELF-A-12", "DOCK-5")
string assigned_task              # Current task ID or description
string assigned_operator          # Operator/supervisor ID (e.g., "OPR-001", "AUTO")

# Timestamps
builtin_interfaces/Time last_maintenance    # Last maintenance timestamp
builtin_interfaces/Time deployment_date     # Robot deployment date
float32 operation_hours                     # Total operation hours

# Safety & Compliance
bool safety_certified             # Safety certification status
string firmware_version           # Current firmware version (e.g., "v2.3.1")
int32 error_code                  # Error code (0 = no error)
```

**RobotTag Message Fields:**

**Identification:**
- `robot_id`: Unique identifier following naming convention (e.g., "WH-BOT-042", "DLV-003")
- `robot_type`: Classification - transport (heavy loads), delivery (fast movement), inspection (sensors), loader (specialized)
- `zone_id`: Current operational zone for fleet management
- `fleet_number`: Fleet group assignment for coordinated operations

**Operational Status:**
- `status`: Real-time operational state for task allocation
- `priority_level`: Task priority for scheduling (0-10 scale)
- `max_payload_kg`: Physical capacity limit for load planning

**Location & Assignment:**
- `current_location`: Precise location identifier in warehouse grid
- `assigned_task`: Active task ID for tracking and coordination
- `assigned_operator`: Human supervisor or "AUTO" for autonomous mode

**Timestamps:**
- `last_maintenance`: Track maintenance schedules and compliance
- `deployment_date`: Robot age for lifecycle management
- `operation_hours`: Usage tracking for predictive maintenance

**Safety & Compliance:**
- `safety_certified`: Safety inspection status for compliance
- `firmware_version`: Software version for compatibility checks
- `error_code`: Diagnostic code for troubleshooting (0 = operational)

### **Configure Message Package**

**Update CMakeLists.txt:**

```cmake
cmake_minimum_required(VERSION 3.8)
project(ce_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotTag.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
```

**Update package.xml:**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ce_robot_interfaces</name>
  <version>0.0.0</version>
  <description>Custom message and service interfaces for CE Robot</description>
  <maintainer email="student@ksu.ac.th">Student</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>builtin_interfaces</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### **Build Message Package**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
source install/setup.bash
```

### **Verify Message Creation**

Check that the message was created successfully:

```bash
# Verify RobotTag message
ros2 interface show ce_robot_interfaces/msg/RobotTag
```

**Expected Output:**
```
string robot_id
string robot_type
string zone_id
int32 fleet_number
string status
int32 priority_level
float32 max_payload_kg
string current_location
string assigned_task
string assigned_operator
builtin_interfaces/Time last_maintenance
builtin_interfaces/Time deployment_date
float32 operation_hours
bool safety_certified
string firmware_version
int32 error_code
```

---

## **Example Use Case: RobotTag for Fleet Management**

The `RobotTag` message is ideal for warehouse fleet management systems. Here's how it would be populated for different robot types:

### **Transport Robot Example:**
```python
from ce_robot_interfaces.msg import RobotTag
from builtin_interfaces.msg import Time

tag = RobotTag()
tag.robot_id = "WH-TRP-042"
tag.robot_type = "transport"
tag.zone_id = "WAREHOUSE-A"
tag.fleet_number = 42

tag.status = "active"
tag.priority_level = 7
tag.max_payload_kg = 500.0

tag.current_location = "SHELF-A-15"
tag.assigned_task = "TASK-8821"
tag.assigned_operator = "AUTO"

tag.last_maintenance.sec = 1733875200  # Unix timestamp
tag.deployment_date.sec = 1704067200
tag.operation_hours = 2847.5

tag.safety_certified = True
tag.firmware_version = "v2.3.1"
tag.error_code = 0
```

### **Delivery Robot Example:**
```python
tag = RobotTag()
tag.robot_id = "DLV-FST-018"
tag.robot_type = "delivery"
tag.zone_id = "LOADING-BAY-3"
tag.fleet_number = 18

tag.status = "active"
tag.priority_level = 9  # High priority delivery
tag.max_payload_kg = 50.0

tag.current_location = "DOCK-5"
tag.assigned_task = "EXPRESS-DELIVERY-445"
tag.assigned_operator = "OPR-007"

tag.operation_hours = 1523.8
tag.safety_certified = True
tag.firmware_version = "v3.0.2"
tag.error_code = 0
```

### **Inspection Robot Example:**
```python
tag = RobotTag()
tag.robot_id = "INSP-001"
tag.robot_type = "inspection"
tag.zone_id = "QUALITY-CONTROL"
tag.fleet_number = 1

tag.status = "idle"
tag.priority_level = 3
tag.max_payload_kg = 10.0  # Sensor equipment only

tag.current_location = "QC-STATION-2"
tag.assigned_task = ""  # No current task
tag.assigned_operator = "OPR-015"

tag.operation_hours = 894.2
tag.safety_certified = True
tag.firmware_version = "v2.1.5"
tag.error_code = 0
```

### **Maintenance Status Example:**
```python
tag = RobotTag()
tag.robot_id = "WH-TRP-009"
tag.robot_type = "transport"
tag.zone_id = "MAINTENANCE-AREA"
tag.fleet_number = 9

tag.status = "maintenance"  # Under maintenance
tag.priority_level = 0  # Not operational
tag.max_payload_kg = 500.0

tag.current_location = "MAINT-BAY-1"
tag.assigned_task = "MAINT-SCHEDULED-2024"
tag.assigned_operator = "TECH-003"

tag.operation_hours = 5240.0  # High hours, needs service
tag.safety_certified = False  # Pending recertification
tag.firmware_version = "v2.2.8"
tag.error_code = 204  # Scheduled maintenance code
```

---

## **Example: Robot Tag Publisher with Parameters**

This example demonstrates practical parameter usage: a publisher node that reports robot fleet management information with configurable identification, timing, and operational settings.

### **Configuration Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot_id` | string | `"WH-BOT-001"` | Unique robot identifier |
| `robot_type` | string | `"transport"` | Robot type: transport/delivery/inspection/loader |
| `zone_id` | string | `"WAREHOUSE-A"` | Current operational zone |
| `fleet_number` | int | `1` | Fleet assignment number (1-999) |
| `max_payload_kg` | double | `500.0` | Maximum payload capacity in kg |
| `priority_level` | int | `5` | Task priority (0-10) |
| `tag_publish_rate` | double | `1.0` | Tag update frequency (Hz) |
| `firmware_version` | string | `"v2.3.1"` | Current firmware version |

**Use Case:** Configure warehouse robots with comprehensive fleet management parameters - identification, operational zones, payload limits, and task priorities.

---

## **Step 2: Create Parameterized Publisher Node**

The publisher node declares parameters, reads their values, and responds to runtime changes through callbacks.

### **📁 File Location**

Navigate to your ROS 2 workspace and create the Python file:

```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
touch robot_tag_publisher.py
chmod +x robot_tag_publisher.py
```

**Directory Structure:**
```
📁 ros2_ws/
└── 📁 src/
    └── 📁 ce_robot/
        └── 📁 ce_robot/
            ├── 📄 __init__.py
            └── 🐍 robot_tag_publisher.py    ← Create this file
```

### **File: robot_tag_publisher.py**

```python
#!/usr/bin/env python3
"""
Parameterized Robot Tag Publisher
Demonstrates parameter declaration, access, and callbacks for fleet management
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from ce_robot_interfaces.msg import RobotTag
from builtin_interfaces.msg import Time


class RobotTagPublisher(Node):
    def __init__(self):
        super().__init__('robot_tag_publisher')
        
        # Declare parameters with defaults and descriptors
        self.declare_parameter(
            'robot_id',
            'WH-BOT-001',
            ParameterDescriptor(
                description='Unique robot identifier (e.g., WH-BOT-042, DLV-003)'
            )
        )
        
        self.declare_parameter(
            'robot_type',
            'transport',
            ParameterDescriptor(
                description='Robot type: transport, delivery, inspection, loader'
            )
        )
        
        self.declare_parameter(
            'zone_id',
            'WAREHOUSE-A',
            ParameterDescriptor(
                description='Current operational zone (e.g., ZONE-A, LOADING-BAY-3)'
            )
        )
        
        self.declare_parameter(
            'fleet_number',
            1,
            ParameterDescriptor(
                description='Fleet assignment number (1-999)'
            )
        )
        
        self.declare_parameter(
            'max_payload_kg',
            500.0,
            ParameterDescriptor(
                description='Maximum payload capacity in kilograms (10.0-1000.0)'
            )
        )
        
        self.declare_parameter(
            'priority_level',
            5,
            ParameterDescriptor(
                description='Task priority level (0=lowest, 10=highest)'
            )
        )
        
        self.declare_parameter(
            'tag_publish_rate',
            1.0,
            ParameterDescriptor(
                description='Tag publishing frequency in Hz (0.1-10.0 Hz)'
            )
        )
        
        self.declare_parameter(
            'firmware_version',
            'v2.3.1',
            ParameterDescriptor(
                description='Current firmware version'
            )
        )
        
        # Get parameter values
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.zone_id = self.get_parameter('zone_id').value
        self.fleet_number = self.get_parameter('fleet_number').value
        self.max_payload_kg = self.get_parameter('max_payload_kg').value
        self.priority_level = self.get_parameter('priority_level').value
        self.tag_rate = self.get_parameter('tag_publish_rate').value
        self.firmware_version = self.get_parameter('firmware_version').value
        
        # Create publisher
        self.publisher = self.create_publisher(
            RobotTag,
            'robot_tag',
            10
        )
        
        # Create timer based on tag_publish_rate
        timer_period = 1.0 / self.tag_rate
        self.timer = self.create_timer(
            timer_period,
            self.timer_callback
        )
        
        # Add callback for parameter changes
        self.add_on_set_parameters_callback(
            self.parameter_callback
        )
        
        self.message_count = 0
        self.operation_hours = 0.0
        
        self.get_logger().info('🏷️  Robot Tag Publisher initialized')
        self.get_logger().info(f'   Robot ID: {self.robot_id}')
        self.get_logger().info(f'   Robot Type: {self.robot_type}')
        self.get_logger().info(f'   Zone: {self.zone_id}')
        self.get_logger().info(f'   Fleet Number: {self.fleet_number}')
        self.get_logger().info(f'   Max Payload: {self.max_payload_kg} kg')
        self.get_logger().info(f'   Priority Level: {self.priority_level}')
        self.get_logger().info(f'   Tag Rate: {self.tag_rate} Hz')
        self.get_logger().info(f'   Firmware: {self.firmware_version}')

    def timer_callback(self):
        """Publish robot tag message"""
        tag = RobotTag()
        
        # Robot identification
        tag.robot_id = self.robot_id
        tag.robot_type = self.robot_type
        tag.zone_id = self.zone_id
        tag.fleet_number = self.fleet_number
        
        # Operational status (simulate different states)
        status_cycle = self.message_count % 100
        if status_cycle < 80:
            tag.status = "active"
        elif status_cycle < 90:
            tag.status = "idle"
        elif status_cycle < 95:
            tag.status = "charging"
        else:
            tag.status = "maintenance"
        
        tag.priority_level = self.priority_level
        tag.max_payload_kg = self.max_payload_kg
        
        # Location & Assignment (simulate)
        if tag.status == "active":
            tag.current_location = f"SHELF-A-{self.message_count % 20 + 1}"
            tag.assigned_task = f"TASK-{8800 + self.message_count}"
            tag.assigned_operator = "AUTO"
        elif tag.status == "maintenance":
            tag.current_location = "MAINT-BAY-1"
            tag.assigned_task = "MAINT-SCHEDULED"
            tag.assigned_operator = "TECH-003"
        else:
            tag.current_location = f"DOCK-{self.fleet_number}"
            tag.assigned_task = ""
            tag.assigned_operator = "AUTO"
        
        # Timestamps
        current_time = self.get_clock().now().to_msg()
        tag.last_maintenance.sec = current_time.sec - 86400  # 1 day ago
        tag.deployment_date.sec = current_time.sec - 7776000  # 90 days ago
        
        # Increment operation hours (simulated)
        self.operation_hours += (1.0 / self.tag_rate) / 3600.0  # Convert seconds to hours
        tag.operation_hours = self.operation_hours
        
        # Safety & Compliance
        tag.safety_certified = (tag.status != "maintenance")
        tag.firmware_version = self.firmware_version
        tag.error_code = 0 if tag.status != "maintenance" else 204
        
        self.publisher.publish(tag)
        self.message_count += 1
        
        # Log status periodically
        if self.message_count % 5 == 0:
            self.get_logger().info(
                f'🤖 {self.robot_id} [{self.robot_type}]: '
                f'Status={tag.status}, Zone={self.zone_id}, '
                f'Location={tag.current_location}, Hours={tag.operation_hours:.1f}'
            )

    def parameter_callback(self, params):
        """Handle parameter changes at runtime with validation"""
        for param in params:
            if param.name == 'robot_id':
                self.robot_id = param.value
                self.get_logger().info(f'✅ Updated robot_id = {self.robot_id}')
                
            elif param.name == 'robot_type':
                # Validate robot type
                valid_types = ['transport', 'delivery', 'inspection', 'loader']
                if param.value in valid_types:
                    self.robot_type = param.value
                    self.get_logger().info(f'✅ Updated robot_type = {self.robot_type}')
                else:
                    self.get_logger().error(f'❌ Invalid type: {param.value} (must be {valid_types})')
                    return SetParametersResult(successful=False, reason='Invalid robot type')
                    
            elif param.name == 'zone_id':
                self.zone_id = param.value
                self.get_logger().info(f'✅ Updated zone_id = {self.zone_id}')
                
            elif param.name == 'fleet_number':
                # Validate fleet number range
                if 1 <= param.value <= 999:
                    self.fleet_number = param.value
                    self.get_logger().info(f'✅ Updated fleet_number = {self.fleet_number}')
                else:
                    self.get_logger().error(f'❌ Invalid fleet number: {param.value} (must be 1-999)')
                    return SetParametersResult(successful=False, reason='Fleet number out of range')
                    
            elif param.name == 'max_payload_kg':
                # Validate payload range
                if 10.0 <= param.value <= 1000.0:
                    self.max_payload_kg = param.value
                    self.get_logger().info(f'✅ Updated max_payload_kg = {self.max_payload_kg} kg')
                else:
                    self.get_logger().error(f'❌ Invalid payload: {param.value} (must be 10.0-1000.0 kg)')
                    return SetParametersResult(successful=False, reason='Payload out of range')
                    
            elif param.name == 'priority_level':
                # Validate priority range
                if 0 <= param.value <= 10:
                    self.priority_level = param.value
                    self.get_logger().info(f'✅ Updated priority_level = {self.priority_level}')
                else:
                    self.get_logger().error(f'❌ Invalid priority: {param.value} (must be 0-10)')
                    return SetParametersResult(successful=False, reason='Priority out of range')
                    
            elif param.name == 'firmware_version':
                self.firmware_version = param.value
                self.get_logger().info(f'✅ Updated firmware_version = {self.firmware_version}')
                    
            elif param.name == 'tag_publish_rate':
                # Validate publish rate
                if 0.1 <= param.value <= 10.0:
                    self.tag_rate = param.value
                    timer_period = 1.0 / self.tag_rate
                    self.timer.cancel()
                    self.timer = self.create_timer(timer_period, self.timer_callback)
                    self.get_logger().info(f'✅ Updated tag_publish_rate = {self.tag_rate} Hz')
                else:
                    self.get_logger().error(f'❌ Invalid rate: {param.value} (must be 0.1-10.0 Hz)')
                    return SetParametersResult(successful=False, reason='Publish rate out of range')
        
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = RobotTagPublisher()
    
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

## **Step 3: Package Configuration**

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
            "05_robot_tag_param = ce_robot.robot_tag_param_pub:main",
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

## **Step 4: Running with Different Parameter Configurations**

### **Method 1: Default Parameters**

Run the node with built-in default values:

**Terminal 1:**
```bash
ros2 run ce_robot 05_robot_tag
```

**Expected Output:**
```
[INFO] [robot_tag_publisher]: 🏷️  Robot Tag Publisher initialized
[INFO] [robot_tag_publisher]:    Robot ID: WH-BOT-001
[INFO] [robot_tag_publisher]:    Robot Type: transport
[INFO] [robot_tag_publisher]:    Zone: WAREHOUSE-A
[INFO] [robot_tag_publisher]:    Fleet Number: 1
[INFO] [robot_tag_publisher]:    Max Payload: 500.0 kg
[INFO] [robot_tag_publisher]:    Priority Level: 5
[INFO] [robot_tag_publisher]:    Tag Rate: 1.0 Hz
[INFO] [robot_tag_publisher]:    Firmware: v2.3.1
[INFO] [robot_tag_publisher]: 🤖 WH-BOT-001 [transport]: Status=active, Zone=WAREHOUSE-A, Location=SHELF-A-1, Hours=0.0
```

### **Method 2: Command-Line Parameters**

Override parameters directly from the command line:

**Terminal 1:**
```bash
ros2 run ce_robot 05_robot_tag --ros-args \
  -p robot_id:=DLV-FST-042 \
  -p robot_type:=delivery \
  -p zone_id:=LOADING-BAY-3 \
  -p fleet_number:=42 \
  -p max_payload_kg:=50.0 \
  -p priority_level:=9 \
  -p tag_publish_rate:=2.0
```

**Expected Output:**
```
[INFO] [robot_tag_publisher]: 🏷️  Robot Tag Publisher initialized
[INFO] [robot_tag_publisher]:    Robot ID: DLV-FST-042
[INFO] [robot_tag_publisher]:    Robot Type: delivery
[INFO] [robot_tag_publisher]:    Zone: LOADING-BAY-3
[INFO] [robot_tag_publisher]:    Fleet Number: 42
[INFO] [robot_tag_publisher]:    Max Payload: 50.0 kg
[INFO] [robot_tag_publisher]:    Priority Level: 9
[INFO] [robot_tag_publisher]:    Tag Rate: 2.0 Hz
[INFO] [robot_tag_publisher]:    Firmware: v2.3.1
```

### **Method 3: Parameter File (.yaml)**

Create a configuration file for reusable parameter sets.

**Create file: robot_config.yaml**
```yaml
robot_tag_publisher:
  ros__parameters:
    robot_id: "INSP-001"
    robot_type: "inspection"
    zone_id: "QUALITY-CONTROL"
    fleet_number: 1
    max_payload_kg: 10.0
    priority_level: 3
    tag_publish_rate: 5.0
    firmware_version: "v2.1.5"
```

![Parameter Configuration](imgs/01_Param.png)

**Run with parameter file:**
```bash
ros2 run ce_robot 05_robot_tag --ros-args --params-file robot_config.yaml
```

**Expected Output:**
```
[INFO] [robot_tag_publisher]: 🏷️  Robot Tag Publisher initialized
[INFO] [robot_tag_publisher]:    Robot ID: INSP-001
[INFO] [robot_tag_publisher]:    Robot Type: inspection
[INFO] [robot_tag_publisher]:    Zone: QUALITY-CONTROL
[INFO] [robot_tag_publisher]:    Fleet Number: 1
[INFO] [robot_tag_publisher]:    Max Payload: 10.0 kg
[INFO] [robot_tag_publisher]:    Priority Level: 3
[INFO] [robot_tag_publisher]:    Tag Rate: 5.0 Hz
[INFO] [robot_tag_publisher]:    Firmware: v2.1.5
[INFO] [robot_tag_publisher]: 🤖 INSP-001 [inspection]: Status=active, Zone=QUALITY-CONTROL, Location=SHELF-A-1, Hours=0.0
```

---

## **Step 5: Runtime Parameter Management**

### **List All Parameters**

View all parameters for the running node:

```bash
ros2 param list
```

**Output:**
```
/robot_tag_publisher:
  firmware_version
  fleet_number
  max_payload_kg
  priority_level
  robot_id
  robot_type
  tag_publish_rate
  use_sim_time
  zone_id
```

### **Get Parameter Value**

Retrieve the current value of a specific parameter:

```bash
ros2 param get /robot_tag_publisher robot_id
```

**Output:**
```
String value is: WH-BOT-001
```

### **Set Parameter at Runtime**

Modify parameter while node is running:

**Terminal 2:**
```bash
ros2 param set /robot_tag_publisher zone_id LOADING-BAY-5
```

**Output:**
```
Set parameter successful
```

**Node Output (Terminal 1):**
```
[INFO] [robot_tag_publisher]: ✅ Updated zone_id = LOADING-BAY-5
[INFO] [robot_tag_publisher]: 🤖 WH-BOT-001 [transport]: Status=active, Zone=LOADING-BAY-5, Location=SHELF-A-12, Hours=0.5
```

### **Change Priority Level at Runtime**

```bash
ros2 param set /robot_tag_publisher priority_level 8
```

**Node Output:**
```
[INFO] [robot_tag_publisher]: ✅ Updated priority_level = 8
```

### **Test Parameter Validation**

Try setting an invalid robot type:

```bash
ros2 param set /robot_tag_publisher robot_type invalid_type
```

**Node Output:**
```
[ERROR] [robot_tag_publisher]: ❌ Invalid type: invalid_type (must be ['transport', 'delivery', 'inspection', 'loader'])
```

### **Get Parameter Description**

View parameter metadata:

```bash
ros2 param describe /robot_tag_publisher max_payload_kg
```

**Output:**
```
Parameter name: max_payload_kg
  Type: double
  Description: Maximum payload capacity in kilograms (10.0-1000.0)
  Constraints: none
```

### **Dump All Parameters to File**

Save current configuration for later use:

```bash
ros2 param dump /robot_tag_publisher > current_params.yaml
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
battery_warning_level: "25"

# ✅ Correct - int for int parameter
battery_warning_level: 25

# ❌ Wrong - string for double parameter  
max_speed: "2.5"

# ✅ Correct - double for double parameter
max_speed: 2.5
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
        if param.name == 'max_speed':
            if param.value < 0.1 or param.value > 5.0:
                return SetParametersResult(
                    successful=False,
                    reason='Max speed must be 0.1-5.0 m/s'
                )
        elif param.name == 'operation_mode':
            valid_modes = ['autonomous', 'manual', 'standby']
            if param.value not in valid_modes:
                return SetParametersResult(
                    successful=False,
                    reason=f'Mode must be one of {valid_modes}'
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
# Template for warehouse transport robots
warehouse_transport_robot:
  ros__parameters:
    robot_id: "transport_bot_xxx"
    max_speed: 2.5
    battery_warning_level: 20
    status_publish_rate: 1.0
    enable_safety_features: true
    operation_mode: "autonomous"
    payload_capacity: 500.0  # kg

# Template for high-speed delivery robots  
delivery_robot:
  ros__parameters:
    robot_id: "delivery_bot_xxx"
    max_speed: 4.0
    battery_warning_level: 30
    status_publish_rate: 2.0
    enable_safety_features: true
    operation_mode: "autonomous"
    delivery_timeout: 300.0  # seconds
    
# Template for inspection robots
inspection_robot:
  ros__parameters:
    robot_id: "inspect_bot_xxx"
    max_speed: 1.0
    battery_warning_level: 15
    status_publish_rate: 5.0
    enable_safety_features: true
    operation_mode: "manual"
    camera_resolution: "1080p"
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
