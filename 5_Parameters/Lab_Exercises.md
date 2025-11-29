# **⚙️ ROS 2 Parameters Lab Exercises**

Master parameter management and dynamic configuration in ROS 2 through progressive hands-on exercises.

---

## **📌 Project Title**

Create and Use ROS 2 Parameters for Node Configuration

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🛠 Lab Overview**

This lab provides hands-on exercises to master parameter declaration, modification, and management in ROS 2. Each exercise builds upon the previous one, progressing from basic parameter usage through advanced configuration patterns with callbacks and parameter files.

**Duration:** ~2 hours
**Level:** Beginner to Intermediate
**Prerequisites:** ROS 2 Jazzy installed, Publisher/Subscriber lab completed

---

## **🎯 Learning Objectives**

By completing this lab, you will be able to:

- ✅ Declare and initialize parameters in ROS 2 nodes
- ✅ Read parameter values at runtime
- ✅ Modify parameters from command line
- ✅ Implement parameter callbacks for dynamic updates
- ✅ Use parameter descriptors for documentation
- ✅ Create and use parameter files (.yaml)
- ✅ Validate parameter values
- ✅ Save and load parameter configurations
- ✅ Debug parameters with ROS 2 tools
- ✅ Implement best practices for parameter management

---

## **📊 Lab Architecture**

```
┌─────────────────────────────────────────────┐
│ Exercise 1: Basic Parameter Publisher       │
│ (Declare & read parameters)                 │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 2: Parameter Callbacks             │
│ (Modify parameters at runtime)              │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 3: Parameter Files & Validation    │
│ (YAML configs and parameter validation)     │
└─────────────────────────────────────────────┘
```

---

## **📚 Learning Path Overview**

| Exercise | Title | Level | Duration |
|----------|-------|-------|----------|
| 1 | Basic Parameter Publisher | Beginner | 25 min |
| 2 | Parameter Callbacks | Intermediate | 30 min |
| 3 | Parameter Files & Validation | Intermediate | 35 min |

---

## **Exercise 1: Basic Parameter Publisher (Beginner) 📝**

### **📋 Task**

Create a publisher node with configurable parameters for robot identification and publishing rate.

### **File: hw_status_param_pub.py**

```python
#!/usr/bin/env python3
"""
Exercise 1: Basic Parameter Publisher
Publishes HardwareStatus with configurable parameters
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from ce_robot_interfaces.msg import HardwareStatus


class HardwareStatusParamPublisher(Node):
    def __init__(self):
        super().__init__('hw_status_param_pub')
        
        # Declare parameters
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
        
        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.robot_number = self.get_parameter('robot_number').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Create publisher
        self.publisher = self.create_publisher(
            HardwareStatus,
            'hardware_status',
            10
        )
        
        # Create timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(
            timer_period,
            self.timer_callback
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
        
        self.get_logger().info(
            f'Published: {msg.name_robot} - '
            f'Temp: {msg.temperature}°C - '
            f'Count: {self.message_count}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusParamPublisher()
    
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

### **Testing Exercise 1**

**Build:**
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

**Run with defaults:**
```bash
ros2 run ce_robot hw_status_param_pub
```

**Expected Output:**
```
[INFO] [hw_status_param_pub]: Hardware Status Publisher initialized
[INFO] [hw_status_param_pub]: Robot: robot_default (ID: 1)
[INFO] [hw_status_param_pub]: Publish Rate: 1.0 Hz
[INFO] [hw_status_param_pub]: Published: robot_default - Temp: 25°C - Count: 1
[INFO] [hw_status_param_pub]: Published: robot_default - Temp: 26°C - Count: 2
```

**Run with command-line parameters:**
```bash
ros2 run ce_robot hw_status_param_pub --ros-args \
  -p robot_name:=robot_ce6541 \
  -p robot_number:=6541 \
  -p publish_rate:=2.0
```

**Expected Output:**
```
[INFO] [hw_status_param_pub]: Hardware Status Publisher initialized
[INFO] [hw_status_param_pub]: Robot: robot_ce6541 (ID: 6541)
[INFO] [hw_status_param_pub]: Publish Rate: 2.0 Hz
[INFO] [hw_status_param_pub]: Published: robot_ce6541 - Temp: 25°C - Count: 1
[INFO] [hw_status_param_pub]: Published: robot_ce6541 - Temp: 26°C - Count: 2
```

**Check parameters (in another terminal):**
```bash
ros2 param list
```

**Get parameter value:**
```bash
ros2 param get /hw_status_param_pub robot_name
```

**Output:**
```
String value is: robot_ce6541
```

### **Key Concepts**

- Parameter declaration with descriptors
- Reading parameter values
- Using parameters in node logic
- Parameter naming conventions
- Default parameter values

---

## **Exercise 2: Parameter Callbacks (Intermediate) 🔄**

### **📋 Task**

Create a node with parameter callbacks that dynamically update behavior when parameters change at runtime.

### **File: hw_status_callback_pub.py**

```python
#!/usr/bin/env python3
"""
Exercise 2: Parameter Callbacks Publisher
Updates behavior dynamically when parameters change
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from ce_robot_interfaces.msg import HardwareStatus


class HardwareStatusCallbackPublisher(Node):
    def __init__(self):
        super().__init__('hw_status_callback_pub')
        
        # Declare parameters
        self.declare_parameter(
            'robot_name',
            'robot_default',
            ParameterDescriptor(description='Robot name')
        )
        
        self.declare_parameter(
            'robot_number',
            1,
            ParameterDescriptor(description='Robot ID')
        )
        
        self.declare_parameter(
            'publish_rate',
            1.0,
            ParameterDescriptor(description='Publishing rate in Hz')
        )
        
        self.declare_parameter(
            'debug_mode',
            False,
            ParameterDescriptor(description='Enable debug logging')
        )
        
        self.declare_parameter(
            'temperature_offset',
            0,
            ParameterDescriptor(description='Temperature offset')
        )
        
        # Get initial values
        self.robot_name = self.get_parameter('robot_name').value
        self.robot_number = self.get_parameter('robot_number').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.temperature_offset = self.get_parameter('temperature_offset').value
        
        # Create publisher
        self.publisher = self.create_publisher(
            HardwareStatus,
            'hardware_status',
            10
        )
        
        # Create timer
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )
        
        # Register parameter callback
        self.add_on_set_parameters_callback(
            self.parameters_callback
        )
        
        self.message_count = 0
        
        self.get_logger().info('Hardware Status Publisher with Callbacks started')
        self.log_parameters()

    def log_parameters(self):
        """Log current parameter values"""
        self.get_logger().info('=== Current Parameters ===')
        self.get_logger().info(f'robot_name: {self.robot_name}')
        self.get_logger().info(f'robot_number: {self.robot_number}')
        self.get_logger().info(f'publish_rate: {self.publish_rate} Hz')
        self.get_logger().info(f'debug_mode: {self.debug_mode}')
        self.get_logger().info(f'temperature_offset: {self.temperature_offset}°C')

    def parameters_callback(self, params):
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
            
            elif param.name == 'publish_rate':
                self.publish_rate = param.value
                # Recreate timer with new rate
                self.timer.cancel()
                self.timer = self.create_timer(
                    1.0 / self.publish_rate,
                    self.timer_callback
                )
                self.get_logger().info(
                    f'Parameter updated: publish_rate = {self.publish_rate} Hz'
                )
            
            elif param.name == 'debug_mode':
                self.debug_mode = param.value
                self.get_logger().info(
                    f'Parameter updated: debug_mode = {self.debug_mode}'
                )
            
            elif param.name == 'temperature_offset':
                self.temperature_offset = param.value
                self.get_logger().info(
                    f'Parameter updated: temperature_offset = {self.temperature_offset}°C'
                )
        
        return SetParametersResult(successful=True)

    def timer_callback(self):
        """Publish HardwareStatus message"""
        base_temp = 25 + (self.message_count % 5)
        temperature = base_temp + self.temperature_offset
        
        msg = HardwareStatus()
        msg.name_robot = self.robot_name
        msg.number_robot = self.robot_number
        msg.temperature = temperature
        msg.motor_ready = True
        msg.debug_message = (
            f'#{self.message_count} | {self.robot_name} | {temperature}°C'
        )
        
        self.publisher.publish(msg)
        self.message_count += 1
        
        if self.debug_mode:
            self.get_logger().info(
                f'Published: {msg.name_robot} - Temp: {temperature}°C'
            )


def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusCallbackPublisher()
    
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
ros2 run ce_robot hw_status_callback_pub --ros-args -p debug_mode:=false
```

**Terminal 2 - Set parameters at runtime:**
```bash
# Enable debug mode
ros2 param set /hw_status_callback_pub debug_mode true

# Change robot name
ros2 param set /hw_status_callback_pub robot_name robot_warehouse

# Change temperature offset
ros2 param set /hw_status_callback_pub temperature_offset 5

# Increase publish rate
ros2 param set /hw_status_callback_pub publish_rate 2.0

# View all parameters
ros2 param list
```

**Expected Output (Terminal 1):**
```
[INFO] [hw_status_callback_pub]: Hardware Status Publisher with Callbacks started
[INFO] [hw_status_callback_pub]: === Current Parameters ===
[INFO] [hw_status_callback_pub]: robot_name: robot_default
[INFO] [hw_status_callback_pub]: robot_number: 1
[INFO] [hw_status_callback_pub]: publish_rate: 1.0 Hz
[INFO] [hw_status_callback_pub]: debug_mode: False
[INFO] [hw_status_callback_pub]: temperature_offset: 0°C

[After setting debug_mode to true]
[INFO] [hw_status_callback_pub]: Parameter updated: debug_mode = True
[INFO] [hw_status_callback_pub]: Published: robot_default - Temp: 25°C
[INFO] [hw_status_callback_pub]: Published: robot_default - Temp: 26°C

[After changing robot_name]
[INFO] [hw_status_callback_pub]: Parameter updated: robot_name = robot_warehouse
[INFO] [hw_status_callback_pub]: Published: robot_warehouse - Temp: 26°C

[After changing temperature_offset to 5]
[INFO] [hw_status_callback_pub]: Parameter updated: temperature_offset = 5°C
[INFO] [hw_status_callback_pub]: Published: robot_warehouse - Temp: 31°C
```

### **Key Concepts**

- Parameter callbacks with `add_on_set_parameters_callback()`
- Dynamic timer recreation for rate changes
- Parameter validation
- Real-time updates without node restart
- SetParametersResult for callback responses

---

## **Exercise 3: Parameter Files & Validation (Intermediate) 📋**

### **📋 Task**

Create a node with parameter validation and use YAML configuration files for parameter management.

### **File: hw_status_validated_pub.py**

```python
#!/usr/bin/env python3
"""
Exercise 3: Validated Parameter Publisher
Validates parameters and uses YAML configuration
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from ce_robot_interfaces.msg import HardwareStatus


class HardwareStatusValidatedPublisher(Node):
    def __init__(self):
        super().__init__('hw_status_validated_pub')
        
        # Declare parameters with constraints
        self.declare_parameter(
            'robot_name',
            'robot_default',
            ParameterDescriptor(description='Robot name (non-empty string)')
        )
        
        self.declare_parameter(
            'robot_number',
            1,
            ParameterDescriptor(description='Robot ID (1-9999)')
        )
        
        self.declare_parameter(
            'publish_rate',
            1.0,
            ParameterDescriptor(description='Publishing rate (0.1-10.0 Hz)')
        )
        
        self.declare_parameter(
            'temperature_min',
            -40,
            ParameterDescriptor(description='Minimum temperature threshold')
        )
        
        self.declare_parameter(
            'temperature_max',
            100,
            ParameterDescriptor(description='Maximum temperature threshold')
        )
        
        # Get and validate parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.robot_number = self.get_parameter('robot_number').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.temperature_min = self.get_parameter('temperature_min').value
        self.temperature_max = self.get_parameter('temperature_max').value
        
        # Validate on startup
        if not self.validate_parameters():
            self.get_logger().error('Invalid parameters on startup!')
            return
        
        # Create publisher
        self.publisher = self.create_publisher(
            HardwareStatus,
            'hardware_status',
            10
        )
        
        # Create timer
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )
        
        # Register parameter callback
        self.add_on_set_parameters_callback(
            self.parameters_callback
        )
        
        self.message_count = 0
        self.temperature_violations = 0
        
        self.get_logger().info('Validated Parameter Publisher started')
        self.log_parameters()

    def validate_parameters(self):
        """Validate all parameters"""
        errors = []
        
        # Validate robot_name
        if not self.robot_name or len(self.robot_name.strip()) == 0:
            errors.append('robot_name: Must be non-empty')
        
        # Validate robot_number
        if self.robot_number < 1 or self.robot_number > 9999:
            errors.append('robot_number: Must be between 1-9999')
        
        # Validate publish_rate
        if self.publish_rate < 0.1 or self.publish_rate > 10.0:
            errors.append('publish_rate: Must be between 0.1-10.0 Hz')
        
        # Validate temperature thresholds
        if self.temperature_min >= self.temperature_max:
            errors.append(
                'temperature_min must be less than temperature_max'
            )
        
        if errors:
            self.get_logger().error('Parameter validation errors:')
            for error in errors:
                self.get_logger().error(f'  - {error}')
            return False
        
        return True

    def parameters_callback(self, params):
        """Handle parameter changes with validation"""
        # Store old values in case validation fails
        old_robot_name = self.robot_name
        old_robot_number = self.robot_number
        old_publish_rate = self.publish_rate
        old_temperature_min = self.temperature_min
        old_temperature_max = self.temperature_max
        
        # Update temporary values
        for param in params:
            if param.name == 'robot_name':
                self.robot_name = param.value
            elif param.name == 'robot_number':
                self.robot_number = param.value
            elif param.name == 'publish_rate':
                self.publish_rate = param.value
            elif param.name == 'temperature_min':
                self.temperature_min = param.value
            elif param.name == 'temperature_max':
                self.temperature_max = param.value
        
        # Validate new parameters
        if not self.validate_parameters():
            # Restore old values if validation fails
            self.robot_name = old_robot_name
            self.robot_number = old_robot_number
            self.publish_rate = old_publish_rate
            self.temperature_min = old_temperature_min
            self.temperature_max = old_temperature_max
            
            return SetParametersResult(successful=False)
        
        # If publish_rate changed, recreate timer
        if self.publish_rate != old_publish_rate:
            self.timer.cancel()
            self.timer = self.create_timer(
                1.0 / self.publish_rate,
                self.timer_callback
            )
            self.get_logger().info(
                f'Publishing rate updated: {self.publish_rate} Hz'
            )
        
        return SetParametersResult(successful=True)

    def log_parameters(self):
        """Log current parameter values"""
        self.get_logger().info('=== Current Parameters ===')
        self.get_logger().info(f'robot_name: {self.robot_name}')
        self.get_logger().info(f'robot_number: {self.robot_number}')
        self.get_logger().info(f'publish_rate: {self.publish_rate} Hz')
        self.get_logger().info(
            f'temperature range: [{self.temperature_min}, {self.temperature_max}]°C'
        )

    def timer_callback(self):
        """Publish HardwareStatus message"""
        base_temp = 25 + (self.message_count % 5)
        
        # Check temperature thresholds
        if base_temp < self.temperature_min or base_temp > self.temperature_max:
            self.temperature_violations += 1
            alert = '⚠️ ALERT'
        else:
            alert = '✓'
        
        msg = HardwareStatus()
        msg.name_robot = self.robot_name
        msg.number_robot = self.robot_number
        msg.temperature = base_temp
        msg.motor_ready = True
        msg.debug_message = (
            f'#{self.message_count} {alert} {base_temp}°C'
        )
        
        self.publisher.publish(msg)
        self.message_count += 1
        
        self.get_logger().info(
            f'{msg.debug_message} | Violations: {self.temperature_violations}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusValidatedPublisher()
    
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

### **Create Configuration File: robot_config.yaml**

```yaml
hw_status_validated_pub:
  ros__parameters:
    robot_name: "robot_warehouse"
    robot_number: 100
    publish_rate: 2.0
    temperature_min: 20
    temperature_max: 35
```

### **Testing Exercise 3**

**Test 1 - Run with YAML configuration file:**
```bash
ros2 run ce_robot hw_status_validated_pub --ros-args \
  --params-file robot_config.yaml
```

**Expected Output:**
```
[INFO] [hw_status_validated_pub]: Validated Parameter Publisher started
[INFO] [hw_status_validated_pub]: === Current Parameters ===
[INFO] [hw_status_validated_pub]: robot_name: robot_warehouse
[INFO] [hw_status_validated_pub]: robot_number: 100
[INFO] [hw_status_validated_pub]: publish_rate: 2.0 Hz
[INFO] [hw_status_validated_pub]: temperature range: [20, 35]°C
[INFO] [hw_status_validated_pub]: #1 ✓ 25°C | Violations: 0
[INFO] [hw_status_validated_pub]: #2 ✓ 26°C | Violations: 0
```

**Test 2 - Try invalid parameter (should fail):**
```bash
ros2 param set /hw_status_validated_pub publish_rate 15.0
```

**Expected Output:**
```
Warning: Setting parameter failed
```

**Test 3 - Set valid parameter:**
```bash
ros2 param set /hw_status_validated_pub temperature_max 25
```

**Expected Output:**
```
[INFO] [hw_status_validated_pub]: #3 ⚠️ ALERT 25°C | Violations: 1
[INFO] [hw_status_validated_pub]: #4 ⚠️ ALERT 26°C | Violations: 2
```

**Test 4 - Save current parameters:**
```bash
ros2 param dump /hw_status_validated_pub > current_config.yaml
```

### **Key Concepts**

- Parameter validation with constraints
- YAML configuration files
- Parameter callbacks with validation
- Rollback on validation failure
- Temperature threshold monitoring
- Parameter persistence

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
  - [ ] Parameters declared with descriptors
  - [ ] Node runs with default parameters
  - [ ] Node runs with command-line parameters
  - [ ] Parameters readable via `ros2 param get`

- [ ] Exercise 2: Parameter Callbacks completed
  - [ ] Parameter callbacks implemented
  - [ ] Parameters can be modified at runtime
  - [ ] Timer updates when publish_rate changes
  - [ ] Debug output shows parameter changes

- [ ] Exercise 3: Parameter Files & Validation completed
  - [ ] YAML configuration file works
  - [ ] Parameter validation working
  - [ ] Invalid parameters rejected
  - [ ] Temperature threshold monitoring
  - [ ] Violations counted and logged

- [ ] All nodes build successfully
- [ ] All nodes run without errors
- [ ] Parameter modifications work
- [ ] YAML files load correctly
- [ ] Validation prevents invalid configurations

---

## **💡 Tips & Tricks**

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
       return SetParametersResult(successful=False)
   ```

4. **Create organized YAML files:**
   ```yaml
   node_name:
     ros__parameters:
       param1: value1
       param2: value2
   ```

5. **Save and restore parameter state:**
   ```bash
   ros2 param dump /node_name > backup.yaml
   ```

6. **Use meaningful parameter names:**
   - `snake_case` for parameter names
   - Descriptive names (e.g., `publish_rate` not `rate`)
   - Group related parameters together

---

**🎓 Congratulations! You've completed the ROS 2 Parameters Lab!** 🚀✨
