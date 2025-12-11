# **⚙️ ROS 2 Parameters Lab Exercises**

## **📌 Project Title**

Hands-On Lab: Master ROS 2 Parameters for Dynamic Node Configuration

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

## **🛠 Overview**

This comprehensive lab demonstrates **parameter management and dynamic configuration** in ROS 2:
- **Parameter Declaration** - Define configurable node parameters with types and defaults
- **Runtime Modification** - Change parameter values without restarting nodes
- **Parameter Callbacks** - Respond to parameter changes with validation
- **YAML Configuration** - Load and save parameter sets from files
- **Fleet Management** - Real-world warehouse robot fleet configuration system

**Duration:** ~2 hours  
**Level:** Beginner to Intermediate  
**Prerequisites:** ROS 2 Jazzy installed, Publisher/Subscriber lab completed, RobotTag message created

---

## **📊 Architecture**

```
┌─────────────────────────────────────────────┐
│ Exercise 1: Basic Parameter Publisher       │
│ (Declare & read fleet parameters)           │
│ • robot_id, robot_type, zone_id             │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 2: Dynamic Fleet Configuration     │
│ (Runtime parameter updates with callbacks)  │
│ • Change zones, priorities, payloads        │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 3: Fleet Management with Validation│
│ (YAML configs & parameter constraints)      │
│ • Validate robot types, ranges, thresholds  │
└─────────────────────────────────────────────┘
```

---

## **Understanding Parameters 📖**

ROS 2 parameters provide runtime configuration for nodes without code recompilation. Unlike topics (streaming data) and services (request-response), parameters are persistent settings that control node behavior.

### 📝 Task
1. List all parameter commands:
```bash
ros2 param --help
```

2. Examine parameter operations:
```bash
ros2 param list
ros2 param get /node_name parameter_name
ros2 param set /node_name parameter_name value
ros2 param describe /node_name parameter_name
```

3. Explore parameter types and understand when to use them

### 💡 Key Concepts
- **Parameter Types**: bool, int, double, string, byte_array
- **Declaration**: Parameters must be declared before use
- **Descriptors**: Documentation and metadata for parameters
- **Callbacks**: Functions triggered when parameters change
- **YAML Files**: Store and load parameter configurations
- **Naming Conventions**: Use snake_case for parameter names

### 🔍 Expected Output
```
Comparison of Parameter vs Topic vs Service:
- Parameters: Persistent configuration (robot_id, max_speed, zones)
- Topics: Streaming data (sensor readings, robot status)
- Services: Request-reply (calculate path, emergency stop)
```

---

## **⚙️ Package Configuration**

Before starting the exercises, ensure your package is properly configured:

```bash
cd ~/ros2_ws/src/ce_robot
```

### 📌 Updating `package.xml`

Verify dependencies exist in `package.xml`:

```xml
<depend>rclpy</depend>
<depend>ce_robot_interfaces</depend>
```

### 📌 Updating `setup.py`

Add entry points for all three exercises under `console_scripts`:

```python
entry_points={
    'console_scripts': [
        '05_robot_tag_param = ce_robot.robot_tag_param_pub:main',
        '05_robot_tag_callback = ce_robot.robot_tag_callback_pub:main',
        '05_robot_tag_validated = ce_robot.robot_tag_validated_pub:main',
    ],
},
```

---

## **📚 Learning Path Overview**| Exercise | Title | Level | Duration |
|----------|-------|-------|----------|
| 1 | Basic Parameter Publisher | Beginner | 25 min |
| 2 | Dynamic Fleet Configuration | Intermediate | 30 min |
| 3 | Fleet Management with Validation | Intermediate | 35 min |

---

## **Exercise 1: Basic Parameter Publisher (Beginner) 📝**

### **📋 Task**

Create a publisher node with configurable parameters for warehouse robot fleet identification and publishing rate.

### **📁 File Location**

Navigate to your ROS 2 workspace and create the Python file:

```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
touch robot_tag_param_pub.py
chmod +x robot_tag_param_pub.py
```

**Directory Structure:**
```
📁 ros2_ws/
└── 📁 src/
    └── 📁 ce_robot/
        └── 📁 ce_robot/
            ├── 📄 __init__.py
            └── 🐍 robot_tag_param_pub.py    ← Create this file
```

### **File: robot_tag_param_pub.py**

```python
#!/usr/bin/env python3
"""
Exercise 1: Basic RobotTag Parameter Publisher
Publishes RobotTag with configurable fleet parameters
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from ce_robot_interfaces.msg import RobotTag
from builtin_interfaces.msg import Time


class RobotTagParamPublisher(Node):
    def __init__(self):
        super().__init__('robot_tag_param_pub')
        
        # Declare parameters
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
            'tag_publish_rate',
            1.0,
            ParameterDescriptor(
                description='Tag publishing frequency in Hz (0.1-10.0)'
            )
        )
        
        # Get parameter values
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.zone_id = self.get_parameter('zone_id').value
        self.fleet_number = self.get_parameter('fleet_number').value
        publish_rate = self.get_parameter('tag_publish_rate').value
        
        # Create publisher
        self.publisher = self.create_publisher(
            RobotTag,
            'robot_tag',
            10
        )
        
        # Create timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(
            timer_period,
            self.timer_callback
        )
        
        self.message_count = 0
        self.operation_hours = 0.0
        
        self.get_logger().info('🏷️  Robot Tag Publisher initialized')
        self.get_logger().info(f'   Robot ID: {self.robot_id}')
        self.get_logger().info(f'   Robot Type: {self.robot_type}')
        self.get_logger().info(f'   Zone: {self.zone_id}')
        self.get_logger().info(f'   Fleet Number: {self.fleet_number}')
        self.get_logger().info(f'   Publish Rate: {publish_rate} Hz')

    def timer_callback(self):
        """Publish RobotTag message"""
        tag = RobotTag()
        
        # Robot identification
        tag.robot_id = self.robot_id
        tag.robot_type = self.robot_type
        tag.zone_id = self.zone_id
        tag.fleet_number = self.fleet_number
        
        # Operational status (simple cycle)
        status_cycle = self.message_count % 4
        if status_cycle == 0:
            tag.status = "active"
            tag.current_location = f"SHELF-A-{self.message_count % 20 + 1}"
        elif status_cycle == 1:
            tag.status = "idle"
            tag.current_location = f"DOCK-{self.fleet_number}"
        elif status_cycle == 2:
            tag.status = "charging"
            tag.current_location = "CHARGING-STATION-1"
        else:
            tag.status = "maintenance"
            tag.current_location = "MAINT-BAY-1"
        
        tag.priority_level = 5
        tag.max_payload_kg = 500.0
        tag.assigned_task = f"TASK-{8800 + self.message_count}"
        tag.assigned_operator = "AUTO"
        
        # Timestamps
        current_time = self.get_clock().now().to_msg()
        tag.last_maintenance.sec = current_time.sec - 86400
        tag.deployment_date.sec = current_time.sec - 7776000
        
        # Operation hours
        self.operation_hours += 0.001
        tag.operation_hours = self.operation_hours
        
        # Safety & Compliance
        tag.safety_certified = True
        tag.firmware_version = "v2.3.1"
        tag.error_code = 0
        
        self.publisher.publish(tag)
        self.message_count += 1
        
        self.get_logger().info(
            f'🤖 {tag.robot_id} [{tag.robot_type}]: '
            f'Status={tag.status}, Zone={tag.zone_id}, Location={tag.current_location}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotTagParamPublisher()
    
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

## **🔨 Building the Package with Colcon**

Once Exercise 1 is created, compile the package:

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

---

## **🚀 Running and Testing**

### **Step 1: Terminal 1 - Run with Default Parameters**

```bash
source ~/.bashrc
ros2 run ce_robot 05_robot_tag_param
```

**Expected Output:**
```
[INFO] [robot_tag_param_pub]: 🏷️  Robot Tag Publisher initialized
[INFO] [robot_tag_param_pub]:    Robot ID: WH-BOT-001
[INFO] [robot_tag_param_pub]:    Robot Type: transport
[INFO] [robot_tag_param_pub]:    Zone: WAREHOUSE-A
[INFO] [robot_tag_param_pub]:    Fleet Number: 1
[INFO] [robot_tag_param_pub]:    Publish Rate: 1.0 Hz
[INFO] [robot_tag_param_pub]: 🤖 WH-BOT-001 [transport]: Status=active, Zone=WAREHOUSE-A, Location=SHELF-A-1
[INFO] [robot_tag_param_pub]: 🤖 WH-BOT-001 [transport]: Status=idle, Zone=WAREHOUSE-A, Location=DOCK-1
```

### **Step 2: Terminal 1 - Run with Command-Line Parameters**

Stop the previous node (Ctrl+C), then run:

```bash
ros2 run ce_robot 05_robot_tag_param --ros-args \
  -p robot_id:=DLV-FST-042 \
  -p robot_type:=delivery \
  -p zone_id:=LOADING-BAY-3 \
  -p fleet_number:=42 \
  -p tag_publish_rate:=2.0
```

**Expected Output:**
```
[INFO] [robot_tag_param_pub]: 🏷️  Robot Tag Publisher initialized
[INFO] [robot_tag_param_pub]:    Robot ID: DLV-FST-042
[INFO] [robot_tag_param_pub]:    Robot Type: delivery
[INFO] [robot_tag_param_pub]:    Zone: LOADING-BAY-3
[INFO] [robot_tag_param_pub]:    Fleet Number: 42
[INFO] [robot_tag_param_pub]:    Publish Rate: 2.0 Hz
[INFO] [robot_tag_param_pub]: 🤖 DLV-FST-042 [delivery]: Status=active, Zone=LOADING-BAY-3, Location=SHELF-A-1
```

### **Step 3: Terminal 2 - Inspect Parameters**

While the node is running, open another terminal:

```bash
# List all parameters
ros2 param list

# Get specific parameter value
ros2 param get /robot_tag_param_pub robot_id

# Describe parameter
ros2 param describe /robot_tag_param_pub robot_id
```

**Expected Output:**
```
String value is: DLV-FST-042
```

### **Step 4: Terminal 3 - Monitor Topic**

```bash
ros2 topic echo /robot_tag
```

You should see RobotTag messages displayed with parameter values:
```
robot_id: DLV-FST-042
robot_type: delivery
zone_id: LOADING-BAY-3
fleet_number: 42
status: active
---
```

### **💡 Key Concepts - Exercise 1**

- ✅ Parameter declaration with descriptors for fleet management
- ✅ Reading fleet parameter values (robot_id, robot_type, zone_id)
- ✅ Using parameters in warehouse robot logic
- ✅ RobotTag message population
- ✅ Parameter naming conventions for fleet systems
- ✅ Default parameter values for transport robots
- ✅ Command-line parameter overrides with `--ros-args`

---

## **Exercise 2: Dynamic Fleet Configuration (Intermediate) 🔄**

### **📋 Task**

Create a warehouse robot node with parameter callbacks that dynamically update fleet configuration when parameters change at runtime.

### **📁 File Location**

Navigate to your ROS 2 workspace and create the Python file:

```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
touch robot_tag_callback_pub.py
chmod +x robot_tag_callback_pub.py
```

**Directory Structure:**
```
📁 ros2_ws/
└── 📁 src/
    └── 📁 ce_robot/
        └── 📁 ce_robot/
            ├── 📄 __init__.py
            ├── 🐍 robot_tag_param_pub.py
            └── 🐍 robot_tag_callback_pub.py    ← Create this file
```

### **File: robot_tag_callback_pub.py**

```python
#!/usr/bin/env python3
"""
Exercise 2: Dynamic Fleet Configuration Publisher
Updates robot behavior dynamically when fleet parameters change
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from ce_robot_interfaces.msg import RobotTag
from builtin_interfaces.msg import Time


class RobotTagCallbackPublisher(Node):
    def __init__(self):
        super().__init__('robot_tag_callback_pub')
        
        # Declare fleet management parameters
        self.declare_parameter(
            'robot_id',
            'WH-BOT-001',
            ParameterDescriptor(description='Unique robot identifier')
        )
        
        self.declare_parameter(
            'robot_type',
            'transport',
            ParameterDescriptor(description='Robot type: transport/delivery/inspection/loader')
        )
        
        self.declare_parameter(
            'zone_id',
            'WAREHOUSE-A',
            ParameterDescriptor(description='Current operational zone')
        )
        
        self.declare_parameter(
            'fleet_number',
            1,
            ParameterDescriptor(description='Fleet assignment number (1-999)')
        )
        
        self.declare_parameter(
            'max_payload_kg',
            500.0,
            ParameterDescriptor(description='Maximum payload capacity in kg')
        )
        
        self.declare_parameter(
            'priority_level',
            5,
            ParameterDescriptor(description='Task priority (0-10)')
        )
        
        self.declare_parameter(
            'tag_publish_rate',
            1.0,
            ParameterDescriptor(description='Publishing rate in Hz (0.1-10.0)')
        )
        
        self.declare_parameter(
            'debug_mode',
            False,
            ParameterDescriptor(description='Enable debug logging')
        )
        
        # Get initial values
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.zone_id = self.get_parameter('zone_id').value
        self.fleet_number = self.get_parameter('fleet_number').value
        self.max_payload_kg = self.get_parameter('max_payload_kg').value
        self.priority_level = self.get_parameter('priority_level').value
        self.tag_rate = self.get_parameter('tag_publish_rate').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # Create publisher
        self.publisher = self.create_publisher(RobotTag, 'robot_tag', 10)
        
        # Create timer
        self.timer = self.create_timer(1.0 / self.tag_rate, self.timer_callback)
        
        # Register parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.message_count = 0
        self.operation_hours = 0.0
        
        self.get_logger().info('🏷️  Dynamic Fleet Configuration Publisher started')
        self.log_parameters()

    def log_parameters(self):
        """Log current fleet parameters"""
        self.get_logger().info('=== Current Fleet Configuration ===')
        self.get_logger().info(f'robot_id: {self.robot_id}')
        self.get_logger().info(f'robot_type: {self.robot_type}')
        self.get_logger().info(f'zone_id: {self.zone_id}')
        self.get_logger().info(f'fleet_number: {self.fleet_number}')
        self.get_logger().info(f'max_payload_kg: {self.max_payload_kg} kg')
        self.get_logger().info(f'priority_level: {self.priority_level}')
        self.get_logger().info(f'tag_publish_rate: {self.tag_rate} Hz')
        self.get_logger().info(f'debug_mode: {self.debug_mode}')

    def parameters_callback(self, params):
        """Handle fleet parameter changes"""
        for param in params:
            if param.name == 'robot_id':
                self.robot_id = param.value
                self.get_logger().info(f'✅ Updated robot_id = {self.robot_id}')
                
            elif param.name == 'robot_type':
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
                if 1 <= param.value <= 999:
                    self.fleet_number = param.value
                    self.get_logger().info(f'✅ Updated fleet_number = {self.fleet_number}')
                else:
                    self.get_logger().error(f'❌ Fleet number must be 1-999')
                    return SetParametersResult(successful=False, reason='Fleet number out of range')
                    
            elif param.name == 'max_payload_kg':
                if 10.0 <= param.value <= 1000.0:
                    self.max_payload_kg = param.value
                    self.get_logger().info(f'✅ Updated max_payload_kg = {self.max_payload_kg} kg')
                else:
                    self.get_logger().error(f'❌ Payload must be 10.0-1000.0 kg')
                    return SetParametersResult(successful=False, reason='Payload out of range')
                    
            elif param.name == 'priority_level':
                if 0 <= param.value <= 10:
                    self.priority_level = param.value
                    self.get_logger().info(f'✅ Updated priority_level = {self.priority_level}')
                else:
                    self.get_logger().error(f'❌ Priority must be 0-10')
                    return SetParametersResult(successful=False, reason='Priority out of range')
                    
            elif param.name == 'tag_publish_rate':
                if 0.1 <= param.value <= 10.0:
                    self.tag_rate = param.value
                    self.timer.cancel()
                    self.timer = self.create_timer(1.0 / self.tag_rate, self.timer_callback)
                    self.get_logger().info(f'✅ Updated tag_publish_rate = {self.tag_rate} Hz')
                else:
                    self.get_logger().error(f'❌ Rate must be 0.1-10.0 Hz')
                    return SetParametersResult(successful=False, reason='Rate out of range')
                    
            elif param.name == 'debug_mode':
                self.debug_mode = param.value
                self.get_logger().info(f'✅ Updated debug_mode = {self.debug_mode}')
        
        return SetParametersResult(successful=True)

    def timer_callback(self):
        """Publish RobotTag message"""
        tag = RobotTag()
        
        # Robot identification
        tag.robot_id = self.robot_id
        tag.robot_type = self.robot_type
        tag.zone_id = self.zone_id
        tag.fleet_number = self.fleet_number
        
        # Operational status
        status_cycle = self.message_count % 4
        if status_cycle == 0:
            tag.status = "active"
            tag.current_location = f"SHELF-A-{self.message_count % 20 + 1}"
        elif status_cycle == 1:
            tag.status = "idle"
            tag.current_location = f"DOCK-{self.fleet_number}"
        elif status_cycle == 2:
            tag.status = "charging"
            tag.current_location = "CHARGING-STATION-1"
        else:
            tag.status = "maintenance"
            tag.current_location = "MAINT-BAY-1"
        
        tag.priority_level = self.priority_level
        tag.max_payload_kg = self.max_payload_kg
        tag.assigned_task = f"TASK-{8800 + self.message_count}"
        tag.assigned_operator = "AUTO"
        
        # Timestamps
        current_time = self.get_clock().now().to_msg()
        tag.last_maintenance.sec = current_time.sec - 86400
        tag.deployment_date.sec = current_time.sec - 7776000
        
        self.operation_hours += (1.0 / self.tag_rate) / 3600.0
        tag.operation_hours = self.operation_hours
        
        # Safety & Compliance
        tag.safety_certified = (tag.status != "maintenance")
        tag.firmware_version = "v2.3.1"
        tag.error_code = 0 if tag.status != "maintenance" else 204
        
        self.publisher.publish(tag)
        self.message_count += 1
        
        if self.debug_mode:
            self.get_logger().info(
                f'🤖 {tag.robot_id} [{tag.robot_type}]: Status={tag.status}, '
                f'Zone={tag.zone_id}, Priority={tag.priority_level}, Payload={tag.max_payload_kg}kg'
            )


def main(args=None):
    rclpy.init(args=args)
    node = RobotTagCallbackPublisher()
    
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

### **Testing Exercise 2**

**Terminal 1 - Run node:**
```bash
ros2 run ce_robot 05_robot_tag_callback --ros-args -p debug_mode:=false
```

**Terminal 2 - Set fleet parameters at runtime:**
```bash
# Enable debug mode
ros2 param set /robot_tag_callback_pub debug_mode true

# Change robot to delivery type
ros2 param set /robot_tag_callback_pub robot_type delivery

# Move to different zone
ros2 param set /robot_tag_callback_pub zone_id LOADING-BAY-5

# Increase priority for urgent tasks
ros2 param set /robot_tag_callback_pub priority_level 9

# Reduce payload for lighter tasks
ros2 param set /robot_tag_callback_pub max_payload_kg 250.0

# Increase publish rate for real-time tracking
ros2 param set /robot_tag_callback_pub tag_publish_rate 3.0

# View all parameters
ros2 param list
```

**Expected Output (Terminal 1):**
```
[INFO] [robot_tag_callback_pub]: 🏷️  Dynamic Fleet Configuration Publisher started
[INFO] [robot_tag_callback_pub]: === Current Fleet Configuration ===
[INFO] [robot_tag_callback_pub]: robot_id: WH-BOT-001
[INFO] [robot_tag_callback_pub]: robot_type: transport
[INFO] [robot_tag_callback_pub]: zone_id: WAREHOUSE-A
[INFO] [robot_tag_callback_pub]: fleet_number: 1
[INFO] [robot_tag_callback_pub]: max_payload_kg: 500.0 kg
[INFO] [robot_tag_callback_pub]: priority_level: 5
[INFO] [robot_tag_callback_pub]: tag_publish_rate: 1.0 Hz
[INFO] [robot_tag_callback_pub]: debug_mode: False

[After setting debug_mode to true]
[INFO] [robot_tag_callback_pub]: ✅ Updated debug_mode = True
[INFO] [robot_tag_callback_pub]: 🤖 WH-BOT-001 [transport]: Status=active, Zone=WAREHOUSE-A, Priority=5, Payload=500.0kg

[After changing robot_type to delivery]
[INFO] [robot_tag_callback_pub]: ✅ Updated robot_type = delivery
[INFO] [robot_tag_callback_pub]: 🤖 WH-BOT-001 [delivery]: Status=idle, Zone=WAREHOUSE-A, Priority=5, Payload=500.0kg

[After changing zone_id to LOADING-BAY-5]
[INFO] [robot_tag_callback_pub]: ✅ Updated zone_id = LOADING-BAY-5
[INFO] [robot_tag_callback_pub]: 🤖 WH-BOT-001 [delivery]: Status=charging, Zone=LOADING-BAY-5, Priority=5, Payload=500.0kg

[After setting priority_level to 9]
[INFO] [robot_tag_callback_pub]: ✅ Updated priority_level = 9
[INFO] [robot_tag_callback_pub]: 🤖 WH-BOT-001 [delivery]: Status=maintenance, Zone=LOADING-BAY-5, Priority=9, Payload=500.0kg

[After setting max_payload_kg to 250.0]
[INFO] [robot_tag_callback_pub]: ✅ Updated max_payload_kg = 250.0 kg
[INFO] [robot_tag_callback_pub]: 🤖 WH-BOT-001 [delivery]: Status=active, Zone=LOADING-BAY-5, Priority=9, Payload=250.0kg
```

### **Key Concepts**

- Parameter callbacks with `add_on_set_parameters_callback()` for fleet management
- Dynamic timer recreation for tag publish rate changes
- Parameter validation (robot type enum, fleet ranges, payload limits, priority levels)
- Real-time fleet configuration updates without node restart
- SetParametersResult for validation responses
- Robot type validation (transport, delivery, inspection, loader)
- Fleet number range checking (1-999)
- Payload capacity validation (10-1000 kg)
- Priority level validation (0-10)

---

## **Exercise 3: Fleet Management with Validation (Intermediate) 📋**

### **📋 Task**

Create a warehouse fleet management node with comprehensive parameter validation and YAML configuration support for multi-robot deployments.

### **📁 File Location**

Navigate to your ROS 2 workspace and create the Python file:

```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
touch robot_tag_validated_pub.py
chmod +x robot_tag_validated_pub.py
```

**Directory Structure:**
```
📁 ros2_ws/
└── 📁 src/
    └── 📁 ce_robot/
        └── 📁 ce_robot/
            ├── 📄 __init__.py
            ├── 🐍 robot_tag_param_pub.py
            ├── 🐍 robot_tag_callback_pub.py
            └── 🐍 robot_tag_validated_pub.py    ← Create this file
```

### **File: robot_tag_validated_pub.py**

```python
#!/usr/bin/env python3
"""
Exercise 3: Fleet Management with Validation
Validates fleet parameters and uses YAML configuration for warehouse robots
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from ce_robot_interfaces.msg import RobotTag
from builtin_interfaces.msg import Time


class RobotTagValidatedPublisher(Node):
    def __init__(self):
        super().__init__('robot_tag_validated_pub')
        
        # Declare parameters with constraints documentation
        self.declare_parameter(
            'robot_id',
            'WH-BOT-001',
            ParameterDescriptor(description='Robot ID (non-empty, format: XXX-XXX-NNN)')
        )
        
        self.declare_parameter(
            'robot_type',
            'transport',
            ParameterDescriptor(description='Robot type: transport, delivery, inspection, loader')
        )
        
        self.declare_parameter(
            'zone_id',
            'WAREHOUSE-A',
            ParameterDescriptor(description='Operational zone (non-empty string)')
        )
        
        self.declare_parameter(
            'fleet_number',
            1,
            ParameterDescriptor(description='Fleet number (1-999)')
        )
        
        self.declare_parameter(
            'max_payload_kg',
            500.0,
            ParameterDescriptor(description='Maximum payload (10.0-1000.0 kg)')
        )
        
        self.declare_parameter(
            'priority_level',
            5,
            ParameterDescriptor(description='Priority level (0-10)')
        )
        
        self.declare_parameter(
            'tag_publish_rate',
            1.0,
            ParameterDescriptor(description='Publishing rate (0.1-10.0 Hz)')
        )
        
        self.declare_parameter(
            'firmware_version',
            'v2.3.1',
            ParameterDescriptor(description='Firmware version string')
        )
        
        self.declare_parameter(
            'safety_check_enabled',
            True,
            ParameterDescriptor(description='Enable safety parameter validation')
        )
        
        # Get and validate parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.zone_id = self.get_parameter('zone_id').value
        self.fleet_number = self.get_parameter('fleet_number').value
        self.max_payload_kg = self.get_parameter('max_payload_kg').value
        self.priority_level = self.get_parameter('priority_level').value
        self.tag_rate = self.get_parameter('tag_publish_rate').value
        self.firmware_version = self.get_parameter('firmware_version').value
        self.safety_check_enabled = self.get_parameter('safety_check_enabled').value
        
        # Validate on startup
        if not self.validate_parameters():
            self.get_logger().error('❌ Invalid fleet parameters on startup!')
            return
        
        # Create publisher
        self.publisher = self.create_publisher(RobotTag, 'robot_tag', 10)
        
        # Create timer
        self.timer = self.create_timer(1.0 / self.tag_rate, self.timer_callback)
        
        # Register parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.message_count = 0
        self.operation_hours = 0.0
        self.safety_violations = 0
        
        self.get_logger().info('🏷️  Fleet Management with Validation started')
        self.log_parameters()

    def validate_parameters(self):
        """Validate all fleet parameters with detailed error reporting"""
        errors = []
        
        # Validate robot_id (non-empty)
        if not self.robot_id or len(self.robot_id.strip()) == 0:
            errors.append('robot_id: Must be non-empty')
        
        # Validate robot_type (enum)
        valid_types = ['transport', 'delivery', 'inspection', 'loader']
        if self.robot_type not in valid_types:
            errors.append(f'robot_type: Must be one of {valid_types}')
        
        # Validate zone_id (non-empty)
        if not self.zone_id or len(self.zone_id.strip()) == 0:
            errors.append('zone_id: Must be non-empty')
        
        # Validate fleet_number (1-999)
        if self.fleet_number < 1 or self.fleet_number > 999:
            errors.append('fleet_number: Must be between 1-999')
        
        # Validate max_payload_kg (10.0-1000.0)
        if self.max_payload_kg < 10.0 or self.max_payload_kg > 1000.0:
            errors.append('max_payload_kg: Must be between 10.0-1000.0 kg')
        
        # Validate priority_level (0-10)
        if self.priority_level < 0 or self.priority_level > 10:
            errors.append('priority_level: Must be between 0-10')
        
        # Validate tag_publish_rate (0.1-10.0)
        if self.tag_rate < 0.1 or self.tag_rate > 10.0:
            errors.append('tag_publish_rate: Must be between 0.1-10.0 Hz')
        
        # Type-specific payload validation
        if self.robot_type == 'delivery' and self.max_payload_kg > 100.0:
            errors.append('delivery robots: max_payload_kg should be ≤100.0 kg')
        elif self.robot_type == 'inspection' and self.max_payload_kg > 50.0:
            errors.append('inspection robots: max_payload_kg should be ≤50.0 kg')
        elif self.robot_type == 'transport' and self.max_payload_kg < 100.0:
            errors.append('transport robots: max_payload_kg should be ≥100.0 kg')
        
        if errors:
            self.get_logger().error('Fleet parameter validation errors:')
            for error in errors:
                self.get_logger().error(f'  • {error}')
            return False
        
        return True

    def parameters_callback(self, params):
        """Handle parameter changes with full fleet validation"""
        # Store old values for rollback
        old_robot_id = self.robot_id
        old_robot_type = self.robot_type
        old_zone_id = self.zone_id
        old_fleet_number = self.fleet_number
        old_max_payload_kg = self.max_payload_kg
        old_priority_level = self.priority_level
        old_tag_rate = self.tag_rate
        old_firmware_version = self.firmware_version
        old_safety_check = self.safety_check_enabled
        
        # Update temporary values
        for param in params:
            if param.name == 'robot_id':
                self.robot_id = param.value
            elif param.name == 'robot_type':
                self.robot_type = param.value
            elif param.name == 'zone_id':
                self.zone_id = param.value
            elif param.name == 'fleet_number':
                self.fleet_number = param.value
            elif param.name == 'max_payload_kg':
                self.max_payload_kg = param.value
            elif param.name == 'priority_level':
                self.priority_level = param.value
            elif param.name == 'tag_publish_rate':
                self.tag_rate = param.value
            elif param.name == 'firmware_version':
                self.firmware_version = param.value
            elif param.name == 'safety_check_enabled':
                self.safety_check_enabled = param.value
        
        # Validate new parameters
        if self.safety_check_enabled and not self.validate_parameters():
            # Restore old values if validation fails
            self.robot_id = old_robot_id
            self.robot_type = old_robot_type
            self.zone_id = old_zone_id
            self.fleet_number = old_fleet_number
            self.max_payload_kg = old_max_payload_kg
            self.priority_level = old_priority_level
            self.tag_rate = old_tag_rate
            self.firmware_version = old_firmware_version
            self.safety_check_enabled = old_safety_check
            
            self.safety_violations += 1
            self.get_logger().error(f'❌ Parameter validation failed (violations: {self.safety_violations})')
            return SetParametersResult(successful=False, reason='Fleet parameter validation failed')
        
        # If tag_publish_rate changed, recreate timer
        if self.tag_rate != old_tag_rate:
            self.timer.cancel()
            self.timer = self.create_timer(1.0 / self.tag_rate, self.timer_callback)
            self.get_logger().info(f'✅ Publishing rate updated: {self.tag_rate} Hz')
        
        # Log other changes
        if self.robot_type != old_robot_type:
            self.get_logger().info(f'✅ Robot type changed: {old_robot_type} → {self.robot_type}')
        if self.zone_id != old_zone_id:
            self.get_logger().info(f'✅ Zone changed: {old_zone_id} → {self.zone_id}')
        if self.priority_level != old_priority_level:
            self.get_logger().info(f'✅ Priority changed: {old_priority_level} → {self.priority_level}')
        
        return SetParametersResult(successful=True)

    def log_parameters(self):
        """Log current fleet parameter values"""
        self.get_logger().info('=== Fleet Configuration ===')
        self.get_logger().info(f'robot_id: {self.robot_id}')
        self.get_logger().info(f'robot_type: {self.robot_type}')
        self.get_logger().info(f'zone_id: {self.zone_id}')
        self.get_logger().info(f'fleet_number: {self.fleet_number}')
        self.get_logger().info(f'max_payload_kg: {self.max_payload_kg} kg')
        self.get_logger().info(f'priority_level: {self.priority_level}')
        self.get_logger().info(f'tag_publish_rate: {self.tag_rate} Hz')
        self.get_logger().info(f'firmware_version: {self.firmware_version}')
        self.get_logger().info(f'safety_check_enabled: {self.safety_check_enabled}')

    def timer_callback(self):
        """Publish validated RobotTag message"""
        tag = RobotTag()
        
        # Robot identification
        tag.robot_id = self.robot_id
        tag.robot_type = self.robot_type
        tag.zone_id = self.zone_id
        tag.fleet_number = self.fleet_number
        
        # Operational status with payload checking
        status_cycle = self.message_count % 4
        if status_cycle == 0:
            tag.status = "active"
            tag.current_location = f"SHELF-A-{self.message_count % 20 + 1}"
        elif status_cycle == 1:
            tag.status = "idle"
            tag.current_location = f"DOCK-{self.fleet_number}"
        elif status_cycle == 2:
            tag.status = "charging"
            tag.current_location = "CHARGING-STATION-1"
        else:
            tag.status = "maintenance"
            tag.current_location = "MAINT-BAY-1"
        
        tag.priority_level = self.priority_level
        tag.max_payload_kg = self.max_payload_kg
        tag.assigned_task = f"TASK-{8800 + self.message_count}"
        tag.assigned_operator = "AUTO"
        
        # Timestamps
        current_time = self.get_clock().now().to_msg()
        tag.last_maintenance.sec = current_time.sec - 86400
        tag.deployment_date.sec = current_time.sec - 7776000
        
        self.operation_hours += (1.0 / self.tag_rate) / 3600.0
        tag.operation_hours = self.operation_hours
        
        # Safety & Compliance
        tag.safety_certified = self.safety_check_enabled and (tag.status != "maintenance")
        tag.firmware_version = self.firmware_version
        tag.error_code = 0 if tag.status != "maintenance" else 204
        
        # Safety alert for high priority maintenance
        if tag.status == "maintenance" and self.priority_level > 7:
            alert = '⚠️  HIGH PRIORITY MAINTENANCE'
        elif tag.status == "active":
            alert = '✓ OPERATIONAL'
        else:
            alert = f'• {tag.status.upper()}'
        
        self.publisher.publish(tag)
        self.message_count += 1
        
        self.get_logger().info(
            f'🤖 {tag.robot_id} [{tag.robot_type}] {alert} | '
            f'Zone={tag.zone_id}, Priority={tag.priority_level}, '
            f'Payload={tag.max_payload_kg}kg, Violations={self.safety_violations}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotTagValidatedPublisher()
    
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

### **📁 Configuration File Location**

Create the YAML configuration file in your workspace root or a config directory:

```bash
cd ~/ros2_ws
touch fleet_config.yaml
```

**Recommended Directory Structure:**
```
📁 ros2_ws/
├── 📄 fleet_config.yaml    ← Create this file (option 1)
└── 📁 src/
    └── 📁 ce_robot/
        ├── 📁 config/
        │   └── 📄 fleet_config.yaml    ← Or create here (option 2)
        └── 📁 ce_robot/
            └── 🐍 robot_tag_validated_pub.py
```

### **Create Configuration File: fleet_config.yaml**

```yaml
robot_tag_validated_pub:
  ros__parameters:
    robot_id: "WH-TRP-100"
    robot_type: "transport"
    zone_id: "WAREHOUSE-FLOOR-2"
    fleet_number: 100
    max_payload_kg: 750.0
    priority_level: 7
    tag_publish_rate: 2.0
    firmware_version: "v2.5.0"
    safety_check_enabled: true
```

### **Testing Exercise 3**

**Test 1 - Run with YAML configuration file:**
```bash
ros2 run ce_robot 05_robot_tag_validated --ros-args \
  --params-file fleet_config.yaml
```

**Expected Output:**
```
[INFO] [robot_tag_validated_pub]: 🏷️  Fleet Management with Validation started
[INFO] [robot_tag_validated_pub]: === Fleet Configuration ===
[INFO] [robot_tag_validated_pub]: robot_id: WH-TRP-100
[INFO] [robot_tag_validated_pub]: robot_type: transport
[INFO] [robot_tag_validated_pub]: zone_id: WAREHOUSE-FLOOR-2
[INFO] [robot_tag_validated_pub]: fleet_number: 100
[INFO] [robot_tag_validated_pub]: max_payload_kg: 750.0 kg
[INFO] [robot_tag_validated_pub]: priority_level: 7
[INFO] [robot_tag_validated_pub]: tag_publish_rate: 2.0 Hz
[INFO] [robot_tag_validated_pub]: firmware_version: v2.5.0
[INFO] [robot_tag_validated_pub]: safety_check_enabled: True
[INFO] [robot_tag_validated_pub]: 🤖 WH-TRP-100 [transport] ✓ OPERATIONAL | Zone=WAREHOUSE-FLOOR-2, Priority=7, Payload=750.0kg, Violations=0
```

**Test 2 - Try invalid robot type (should fail):**
```bash
ros2 param set /robot_tag_validated_pub robot_type invalid_type
```

**Expected Output:**
```
[ERROR] [robot_tag_validated_pub]: Fleet parameter validation errors:
[ERROR] [robot_tag_validated_pub]:   • robot_type: Must be one of ['transport', 'delivery', 'inspection', 'loader']
[ERROR] [robot_tag_validated_pub]: ❌ Parameter validation failed (violations: 1)
Warning: Setting parameter failed
```

**Test 3 - Try invalid payload for delivery robot:**
```bash
# First change to delivery type
ros2 param set /robot_tag_validated_pub robot_type delivery

# Then try to set payload too high for delivery
ros2 param set /robot_tag_validated_pub max_payload_kg 500.0
```

**Expected Output:**
```
[INFO] [robot_tag_validated_pub]: ✅ Robot type changed: transport → delivery
[ERROR] [robot_tag_validated_pub]: Fleet parameter validation errors:
[ERROR] [robot_tag_validated_pub]:   • delivery robots: max_payload_kg should be ≤100.0 kg
[ERROR] [robot_tag_validated_pub]: ❌ Parameter validation failed (violations: 1)
Warning: Setting parameter failed
```

**Test 4 - Set valid parameters:**
```bash
# Change to inspection robot with appropriate payload
ros2 param set /robot_tag_validated_pub robot_type inspection
ros2 param set /robot_tag_validated_pub max_payload_kg 30.0
ros2 param set /robot_tag_validated_pub zone_id QUALITY-CONTROL
ros2 param set /robot_tag_validated_pub priority_level 3
```

**Expected Output:**
```
[INFO] [robot_tag_validated_pub]: ✅ Robot type changed: transport → inspection
[INFO] [robot_tag_validated_pub]: 🤖 WH-TRP-100 [inspection] ✓ OPERATIONAL | Zone=WAREHOUSE-FLOOR-2, Priority=7, Payload=30.0kg, Violations=0
[INFO] [robot_tag_validated_pub]: ✅ Zone changed: WAREHOUSE-FLOOR-2 → QUALITY-CONTROL
[INFO] [robot_tag_validated_pub]: ✅ Priority changed: 7 → 3
[INFO] [robot_tag_validated_pub]: 🤖 WH-TRP-100 [inspection] ✓ OPERATIONAL | Zone=QUALITY-CONTROL, Priority=3, Payload=30.0kg, Violations=0
```

**Test 5 - Save current parameters:**
```bash
ros2 param dump /robot_tag_validated_pub > current_fleet_config.yaml
cat current_fleet_config.yaml
```

### **Key Concepts**

- Comprehensive fleet parameter validation with detailed error reporting
- YAML configuration files for multi-robot deployment
- Parameter callbacks with validation and rollback on failure
- Robot type-specific validation (payload constraints per robot type)
- Fleet number range validation (1-999)
- Payload capacity validation with type-specific limits
- Priority level validation (0-10 scale)
- Safety check toggle for validation bypass
- Validation violation tracking
- Parameter persistence and configuration management

---

## **📂 Directory Structure**

```
📁 ros2_ws/
├── 📄 fleet_config.yaml        ← YAML configuration file
└── 📁 src/
    └── 📁 ce_robot/
        ├── 📁 ce_robot/
        │   ├── 📄 __init__.py
        │   ├── 🐍 robot_tag_param_pub.py         (Exercise 1)
        │   ├── 🐍 robot_tag_callback_pub.py      (Exercise 2)
        │   └── 🐍 robot_tag_validated_pub.py     (Exercise 3)
        ├── 📄 package.xml
        ├── 📄 setup.cfg
        └── 📄 setup.py
```

---

## **🔍 Parameter Inspection Commands**

### **List All Parameters**
```bash
ros2 param list
```

### **Get Parameter Value**
```bash
ros2 param get /node_name param_name
```

### **Set Parameter Value**
```bash
ros2 param set /node_name param_name new_value
```

### **Describe Parameter**
```bash
ros2 param describe /node_name param_name
```

### **Load Parameters from File**
```bash
ros2 run ce_robot 05_robot_tag_validated --ros-args --params-file fleet_config.yaml
```

### **Save Parameters to File**
```bash
ros2 param dump /node_name > saved_params.yaml
```

### **View Parameter File**
```bash
cat fleet_config.yaml
```

---

## **🎯 Key Concepts**

### **Parameter Architecture**
- **Parameter Declaration**: Must declare parameters before use with type and default
- **Parameter Types**: bool, int (int64), double, string, byte_array
- **Descriptors**: Add documentation and constraints to parameters
- **Callbacks**: Functions triggered when parameters change at runtime
- **Validation**: Ensure parameter values meet requirements before applying

### **Communication Pattern**
- **Node-Level**: Parameters belong to specific nodes
- **Runtime Modification**: Change parameters without restarting nodes
- **YAML Configuration**: Store and load parameter sets from files
- **Quality of Service**: Parameters persist throughout node lifecycle

### **Advantages**
- ✅ Runtime configuration without code recompilation
- ✅ Validation and error handling
- ✅ Configuration file support (YAML)
- ✅ Parameter introspection and debugging
- ✅ Dynamic behavior changes

### **Disadvantages**
- ❌ Requires careful validation logic
- ❌ Can be overridden unexpectedly
- ❌ No automatic synchronization across nodes
- ❌ Type safety only at runtime

---

## **⚠️ Troubleshooting**

### **Issue: Parameter not declared error**
**Solution:** Always declare parameters before using them
```python
self.declare_parameter('param_name', default_value, descriptor)
value = self.get_parameter('param_name').value
```

### **Issue: TypeError when setting parameter**
**Solution:** Ensure parameter value matches declared type
```bash
# Wrong: String value for int parameter
ros2 param set /node fleet_number "100"

# Correct: Integer value
ros2 param set /node fleet_number 100
```

### **Issue: YAML file not loading**
**Solution:** Check YAML syntax and file path
```bash
# Verify file exists
ls -la fleet_config.yaml

# Check YAML syntax
cat fleet_config.yaml

# Use absolute path if needed
ros2 run ce_robot 05_robot_tag_validated --ros-args --params-file /absolute/path/to/fleet_config.yaml
```

### **Issue: Parameter callback not triggered**
**Solution:** Ensure callback is registered
```python
self.add_on_set_parameters_callback(self.parameters_callback)
```

### **Issue: Validation always fails**
**Solution:** Check validation logic and return SetParametersResult correctly
```python
return SetParametersResult(successful=True)  # On success
return SetParametersResult(successful=False, reason='Error message')  # On failure
```

---

## **📚 Resources**

- [ROS 2 Parameters Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
- [ROS 2 Parameter Concepts](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Parameters.html)
- [Parameter Files Documentation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Params-File.html)
- [ROS 2 Command-Line Tools](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html)

---

## **✅ Verification Checklist**

- [ ] **Exercise 1: Basic Parameter Publisher**
  - [ ] Fleet parameters declared with descriptors
  - [ ] Node runs with default parameters
  - [ ] Node runs with command-line fleet parameters
  - [ ] RobotTag message published successfully
  - [ ] Parameters readable via `ros2 param get`

- [ ] **Exercise 2: Dynamic Fleet Configuration**
  - [ ] Parameter callbacks implemented for fleet management
  - [ ] Fleet parameters can be modified at runtime
  - [ ] Timer updates when tag_publish_rate changes
  - [ ] Robot type, zone, priority, payload updates work
  - [ ] Validation prevents invalid robot types and ranges
  - [ ] Debug output shows fleet configuration changes

- [ ] **Exercise 3: Fleet Management with Validation**
  - [ ] YAML fleet configuration file works
  - [ ] Comprehensive parameter validation working
  - [ ] Invalid parameters rejected with detailed errors
  - [ ] Robot type-specific payload validation
  - [ ] Fleet number range checking (1-999)
  - [ ] Priority level validation (0-10)
  - [ ] Validation violations counted and logged
  - [ ] Safety check toggle works
  - [ ] Parameter rollback on validation failure

- [ ] **Package Configuration**
  - [ ] All nodes build successfully
  - [ ] All nodes run without errors
  - [ ] Fleet parameter modifications work
  - [ ] YAML fleet configuration files load correctly
  - [ ] Validation prevents invalid fleet configurations
  - [ ] RobotTag messages published with correct fleet data

---

## **🔗 Related Topics**

- Parameters (runtime configuration)
- Topics (asynchronous communication)
- Services (synchronous request-reply)
- Actions (long-running tasks with feedback)
- Launch Files (system orchestration)
- YAML Configuration Files

---

## **Commands to Practice**

```bash
# List all parameters
ros2 param list

# Get parameter value
ros2 param get /node_name param_name

# Set parameter value
ros2 param set /node_name param_name new_value

# Get parameter descriptor
ros2 param describe /node_name param_name

# Load parameters from YAML file
ros2 run package_name node_name --ros-args --params-file config.yaml

# Save current parameters to file
ros2 param dump /node_name > saved_params.yaml

# View parameter file content
cat config.yaml
```

---

## **✅ Completion Checklist**

- [ ] Exercise 1: Basic Parameter Publisher completed
  - [ ] Fleet parameters declared with descriptors
  - [ ] Node runs with default parameters
  - [ ] Node runs with command-line fleet parameters
  - [ ] RobotTag message published successfully
  - [ ] Parameters readable via `ros2 param get`

- [ ] Exercise 2: Dynamic Fleet Configuration completed
  - [ ] Parameter callbacks implemented for fleet management
  - [ ] Fleet parameters can be modified at runtime
  - [ ] Timer updates when tag_publish_rate changes
  - [ ] Robot type, zone, priority, payload updates work
  - [ ] Validation prevents invalid robot types and ranges
  - [ ] Debug output shows fleet configuration changes

- [ ] Exercise 3: Fleet Management with Validation completed
  - [ ] YAML fleet configuration file works
  - [ ] Comprehensive parameter validation working
  - [ ] Invalid parameters rejected with detailed errors
  - [ ] Robot type-specific payload validation
  - [ ] Fleet number range checking (1-999)
  - [ ] Priority level validation (0-10)
  - [ ] Validation violations counted and logged
  - [ ] Safety check toggle works
  - [ ] Parameter rollback on validation failure

- [ ] All nodes build successfully
- [ ] All nodes run without errors
- [ ] Fleet parameter modifications work
- [ ] YAML fleet configuration files load correctly
- [ ] Validation prevents invalid fleet configurations
- [ ] RobotTag messages published with correct fleet data

---

## **💡 Tips & Best Practices**

1. **Always declare parameters early in `__init__`:**
   ```python
   self.declare_parameter('param_name', default_value, descriptor)
   ```

2. **Use ParameterDescriptor for documentation:**
   ```python
   ParameterDescriptor(description='Clear description of parameter')
   ```

3. **Always validate parameters in callbacks:**
   ```python
   if not self.validate_parameters():
       return SetParametersResult(successful=False, reason='Validation failed')
   ```

4. **Create organized YAML files for fleet configuration:**
   ```yaml
   node_name:
     ros__parameters:
       robot_id: "WH-TRP-001"
       robot_type: "transport"
       zone_id: "WAREHOUSE-A"
       fleet_number: 1
   ```

5. **Save and restore fleet configuration:**
   ```bash
   ros2 param dump /node_name > fleet_backup.yaml
   ```

6. **Use meaningful fleet parameter names:**
   - `snake_case` for parameter names
   - Descriptive names (e.g., `robot_id` not `id`, `zone_id` not `zone`)
   - Group related fleet parameters together
   - Use robot type prefixes for clarity (WH-TRP for warehouse transport)

7. **Test validation logic thoroughly:**
   - Test boundary values (min/max)
   - Test invalid types and values
   - Ensure rollback works on validation failure

8. **Document parameter constraints:**
   - Valid ranges (e.g., 1-999 for fleet_number)
   - Enum values (e.g., transport, delivery, inspection, loader)
   - Required vs optional parameters

---

**✅ ROS 2 Parameters Lab Complete!** 🎓🚀✨

You've successfully mastered:
- ✅ Parameter declaration with types and defaults
- ✅ Runtime parameter modification
- ✅ Parameter callbacks and validation
- ✅ YAML configuration files
- ✅ Fleet management with comprehensive validation

**Next Steps:**
- Proceed to **06_Action** for long-running tasks with feedback
- Explore **07_Launch** for system orchestration
- Build complete robot fleet management systems
