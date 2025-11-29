# **ROS 2 Server & Client Lab Exercises**

## **📚 Lab Overview**

This lab provides hands-on exercises to master Service-based (request-reply) communication patterns in ROS 2. Each exercise builds upon the previous one, progressing from basic to advanced concepts.

**Duration:** 2-3 hours  
**Level:** Beginner to Intermediate  
**Prerequisites:** ROS 2 Jazzy installed, Publisher/Subscriber lab completed

---

## **🎯 Learning Objectives**

By completing this lab, you will be able to:
- ✅ Create basic Server nodes with service handlers
- ✅ Create basic Client nodes that make requests
- ✅ Understand synchronous request-reply communication
- ✅ Work with built-in and custom service types
- ✅ Implement error handling and validation
- ✅ Debug and monitor services
- ✅ Handle multiple services in one node

---

## **📋 Lab Exercises**

### **Exercise 1: Temperature Conversion Service (Beginner)**

**Objective:** Create a service that converts temperature between units

**Tasks:**
1. Create `temp_converter_server.py`
2. Implement conversion: Celsius ↔ Fahrenheit ↔ Kelvin
3. Create `temp_converter_client.py` with user input
4. Add input validation for temperature ranges

**File: temp_converter_server.py**

```python
#!/usr/bin/env python3
"""
Exercise 1: Temperature Conversion Server
Converts temperature between Celsius, Fahrenheit, and Kelvin
Service type: AddTwoInts (reused for simplicity - a=temp, b=unit)
Note: In real scenario, would create custom .srv file
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class TempConverterServer(Node):
    def __init__(self):
        super().__init__('temp_converter_server')
        
        # Create service
        self.srv = self.create_service(
            AddTwoInts,
            'convert_temperature',
            self.convert_callback
        )
        
        self.get_logger().info('Temperature Converter Server started')
        self.get_logger().info('Service: /convert_temperature')
        self.get_logger().info('Units: 1=Celsius, 2=Fahrenheit, 3=Kelvin')

    def convert_callback(self, request, response):
        """
        Convert temperature
        request.a = temperature value
        request.b = unit (1=C, 2=F, 3=K)
        response.sum = result
        """
        temp = request.a
        unit = request.b
        
        # Validate temperature
        if unit == 1:  # Celsius input
            if temp < -273.15:
                self.get_logger().warn('Temperature below absolute zero!')
                response.sum = -999
                return response
            
            # Convert to Fahrenheit and Kelvin for demo
            fahrenheit = (temp * 9/5) + 32
            kelvin = temp + 273.15
            
            self.get_logger().info(
                f'Converting {temp}°C → F: {fahrenheit:.2f}°F, K: {kelvin:.2f}K'
            )
            response.sum = int(fahrenheit)
        
        elif unit == 2:  # Fahrenheit input
            celsius = (temp - 32) * 5/9
            kelvin = celsius + 273.15
            
            self.get_logger().info(
                f'Converting {temp}°F → C: {celsius:.2f}°C, K: {kelvin:.2f}K'
            )
            response.sum = int(celsius)
        
        elif unit == 3:  # Kelvin input
            celsius = temp - 273.15
            fahrenheit = (celsius * 9/5) + 32
            
            self.get_logger().info(
                f'Converting {temp}K → C: {celsius:.2f}°C, F: {fahrenheit:.2f}°F'
            )
            response.sum = int(celsius)
        
        else:
            self.get_logger().error('Invalid unit. Use 1=C, 2=F, 3=K')
            response.sum = -999
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TempConverterServer()
    
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

**File: temp_converter_client.py**

```python
#!/usr/bin/env python3
"""
Exercise 1: Temperature Conversion Client
Converts temperature using the server
Usage: ros2 run ce_robot temp_converter_client <temp> <unit>
Units: 1=Celsius, 2=Fahrenheit, 3=Kelvin
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class TempConverterClient(Node):
    def __init__(self):
        super().__init__('temp_converter_client')

    def convert(self, temp, unit):
        """Request temperature conversion"""
        
        client = self.create_client(AddTwoInts, 'convert_temperature')
        
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        request = AddTwoInts.Request()
        request.a = temp
        request.b = unit
        
        self.get_logger().info(f'Converting {temp} (unit={unit})')
        future = client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 3:
        print('Usage: temp_converter_client <temperature> <unit>')
        print('Units: 1=Celsius, 2=Fahrenheit, 3=Kelvin')
        sys.exit(1)
    
    temp = int(sys.argv[1])
    unit = int(sys.argv[2])
    
    node = TempConverterClient()
    response = node.convert(temp, unit)
    
    units = {1: '°C', 2: '°F', 3: 'K'}
    unit_names = {1: 'Celsius', 2: 'Fahrenheit', 3: 'Kelvin'}
    
    if response.sum == -999:
        node.get_logger().error('Conversion failed')
    else:
        node.get_logger().info(
            f'Converted: {temp}{units[unit]} → {response.sum}°C'
        )
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Expected Output (Server):**

```
[INFO] [temp_converter_server]: Temperature Converter Server started
[INFO] [temp_converter_server]: Service: /convert_temperature
[INFO] [temp_converter_server]: Units: 1=Celsius, 2=Fahrenheit, 3=Kelvin
[INFO] [temp_converter_server]: Converting 25°C → F: 77.00°F, K: 298.15K
[INFO] [temp_converter_server]: Converting 77°F → C: 25.00°C, K: 298.15K
```

**Expected Output (Client):**

```
[INFO] [temp_converter_client]: Converting 25 (unit=1)
[INFO] [temp_converter_client]: Converted: 25°C → 77°C
```

**Key Concepts:**
- Input validation for temperature ranges
- Multiple request types handled by same service
- Error handling and status codes (-999)
- Type conversions and calculations
- Command-line argument parsing

---

### **Exercise 2: String Processing Service (Beginner)**

**Objective:** Create a service that processes text strings with multiple operations

**Tasks:**
1. Create `string_processor_server.py`
2. Support operations: reverse, uppercase, lowercase, count words, count chars
3. Create `string_processor_client.py`
4. Implement operation selection

**File: string_processor_server.py**

```python
#!/usr/bin/env python3
"""
Exercise 2: String Processing Server
Processes strings with various operations
Operation codes: 1=reverse, 2=uppercase, 3=lowercase, 4=word_count, 5=char_count
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class StringProcessorServer(Node):
    def __init__(self):
        super().__init__('string_processor_server')
        
        self.srv = self.create_service(
            AddTwoInts,
            'process_string',
            self.process_callback
        )
        
        self.get_logger().info('String Processor Server started')

    def process_callback(self, request, response):
        """
        Process string operations
        Note: Using AddTwoInts as placeholder (would use custom .srv in production)
        """
        # For this exercise, we'll use a simple approach
        # In real scenario, create custom .srv file with string fields
        
        self.get_logger().info('Service called')
        response.sum = 42  # Placeholder
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = StringProcessorServer()
    
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

**Alternative: Using Custom Service Type (.srv file)**

For better implementation, create a custom service type. First, create `StringProcess.srv` file:

```
# StringProcess.srv
# Request
string text
int32 operation  # 1=reverse, 2=upper, 3=lower, 4=word_count, 5=char_count
---
# Response
string result
int32 status  # 0=success, -1=error
```

**Updated server_processor_server.py (with custom .srv):**

```python
#!/usr/bin/env python3
"""
Exercise 2: String Processing Server (with Custom Service)
"""

import rclpy
from rclpy.node import Node
from ce_robot.srv import StringProcess


class StringProcessorServer(Node):
    def __init__(self):
        super().__init__('string_processor_server')
        
        self.srv = self.create_service(
            StringProcess,
            'process_string',
            self.process_callback
        )
        
        self.get_logger().info('String Processor Server started')

    def process_callback(self, request, response):
        """Process string operations"""
        
        text = request.text
        operation = request.operation
        
        try:
            if operation == 1:  # Reverse
                response.result = text[::-1]
                self.get_logger().info(f'Reversed: {text} → {response.result}')
            
            elif operation == 2:  # Uppercase
                response.result = text.upper()
                self.get_logger().info(f'Uppercase: {text} → {response.result}')
            
            elif operation == 3:  # Lowercase
                response.result = text.lower()
                self.get_logger().info(f'Lowercase: {text} → {response.result}')
            
            elif operation == 4:  # Word count
                word_count = len(text.split())
                response.result = f'Word count: {word_count}'
                self.get_logger().info(f'Words in "{text}": {word_count}')
            
            elif operation == 5:  # Character count
                char_count = len(text)
                response.result = f'Char count: {char_count}'
                self.get_logger().info(f'Characters in "{text}": {char_count}')
            
            else:
                response.result = 'Invalid operation'
                response.status = -1
                self.get_logger().error('Invalid operation code')
                return response
            
            response.status = 0  # Success
        
        except Exception as e:
            response.result = f'Error: {str(e)}'
            response.status = -1
            self.get_logger().error(f'Processing failed: {str(e)}')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = StringProcessorServer()
    
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

**File: string_processor_client.py**

```python
#!/usr/bin/env python3
"""
Exercise 2: String Processor Client
Operations: 1=reverse, 2=upper, 3=lower, 4=word_count, 5=char_count
Usage: ros2 run ce_robot string_processor_client "hello world" 1
"""

import sys
import rclpy
from rclpy.node import Node
from ce_robot.srv import StringProcess


class StringProcessorClient(Node):
    def __init__(self):
        super().__init__('string_processor_client')

    def process(self, text, operation):
        """Request string processing"""
        
        client = self.create_client(StringProcess, 'process_string')
        
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        request = StringProcess.Request()
        request.text = text
        request.operation = operation
        
        self.get_logger().info(f'Processing: "{text}" (op={operation})')
        future = client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 3:
        print('Usage: string_processor_client "text" <operation>')
        print('Operations: 1=reverse, 2=upper, 3=lower, 4=word_count, 5=char_count')
        sys.exit(1)
    
    text = sys.argv[1]
    operation = int(sys.argv[2])
    
    node = StringProcessorClient()
    response = node.process(text, operation)
    
    if response.status == 0:
        node.get_logger().info(f'Result: {response.result}')
    else:
        node.get_logger().error(f'Failed: {response.result}')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Expected Output (Server):**

```
[INFO] [string_processor_server]: String Processor Server started
[INFO] [string_processor_server]: Reversed: hello world → dlrow olleh
[INFO] [string_processor_server]: Uppercase: hello world → HELLO WORLD
[INFO] [string_processor_server]: Words in "hello world": 2
```

**Expected Output (Client):**

```
[INFO] [string_processor_client]: Processing: "hello world" (op=1)
[INFO] [string_processor_client]: Result: dlrow olleh
```

**Key Concepts:**
- Creating custom `.srv` files
- String manipulation methods (reverse, upper, lower)
- String methods (split, len)
- Status codes for error handling
- Operation selection with integer codes

---

### **Exercise 3: Database Query Service (Intermediate)**

**Objective:** Create a service that queries a database of records

**Tasks:**
1. Create `database_server.py` with student records
2. Support query by ID, name, or all records
3. Create `database_client.py` with query options
4. Implement filtering and sorting

**File: database_server.py**

```python
#!/usr/bin/env python3
"""
Exercise 3: Database Query Server
Maintains a student database and handles queries
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class DatabaseServer(Node):
    def __init__(self):
        super().__init__('database_server')
        
        # Initialize student database
        self.students = {
            1: {'name': 'Alice', 'gpa': 3.8, 'major': 'CS'},
            2: {'name': 'Bob', 'gpa': 3.5, 'major': 'EE'},
            3: {'name': 'Charlie', 'gpa': 3.9, 'major': 'CS'},
            4: {'name': 'Diana', 'gpa': 3.6, 'major': 'ME'},
            5: {'name': 'Eve', 'gpa': 3.7, 'major': 'CS'},
        }
        
        self.srv = self.create_service(
            AddTwoInts,
            'query_database',
            self.query_callback
        )
        
        self.get_logger().info('Database Server started')
        self.get_logger().info(f'Database loaded: {len(self.students)} records')

    def query_callback(self, request, response):
        """Handle database queries"""
        
        query_type = request.a  # 0=all, 1=by_id
        query_id = request.b
        
        if query_type == 0:  # Get all
            count = len(self.students)
            self.get_logger().info(f'Query: All records ({count} total)')
            response.sum = count
        
        elif query_type == 1:  # Get by ID
            if query_id in self.students:
                student = self.students[query_id]
                self.get_logger().info(
                    f'Query: ID {query_id} → {student["name"]} '
                    f'(GPA: {student["gpa"]}, Major: {student["major"]})'
                )
                response.sum = query_id  # Return success
            else:
                self.get_logger().warn(f'Query: ID {query_id} not found')
                response.sum = -1  # Not found
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DatabaseServer()
    
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

**File: database_client.py**

```python
#!/usr/bin/env python3
"""
Exercise 3: Database Client
Queries the student database
Usage: ros2 run ce_robot database_client <query_type> [id]
query_type: 0=all, 1=by_id
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class DatabaseClient(Node):
    def __init__(self):
        super().__init__('database_client')

    def query(self, query_type, query_id=0):
        """Query the database"""
        
        client = self.create_client(AddTwoInts, 'query_database')
        
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        request = AddTwoInts.Request()
        request.a = query_type
        request.b = query_id
        
        self.get_logger().info(f'Querying database...')
        future = client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print('Usage: database_client <query_type> [id]')
        print('query_type: 0=all, 1=by_id')
        sys.exit(1)
    
    query_type = int(sys.argv[1])
    query_id = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    
    node = DatabaseClient()
    response = node.query(query_type, query_id)
    
    if response.sum >= 0:
        node.get_logger().info(f'Query successful: {response.sum}')
    else:
        node.get_logger().warn('Query returned no results')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Expected Output (Server):**

```
[INFO] [database_server]: Database Server started
[INFO] [database_server]: Database loaded: 5 records
[INFO] [database_server]: Query: All records (5 total)
[INFO] [database_server]: Query: ID 1 → Alice (GPA: 3.8, Major: CS)
[INFO] [database_server]: Query: ID 10 not found
```

**Expected Output (Client):**

```
[INFO] [database_client]: Querying database...
[INFO] [database_client]: Query successful: 5
```

**Key Concepts:**
- Data structures (dictionaries) for storage
- Query patterns (get all, filter by ID)
- Status codes and error handling
- In-memory database simulation

---

### **Exercise 4: Multi-Service Robot Controller (Intermediate)**

**Objective:** Create a robot controller with multiple related services

**Tasks:**
1. Create `robot_controller_server.py` with 3 services:
   - Movement (forward, backward, turn)
   - Sensor reading
   - LED control
2. Create `robot_controller_client.py` to coordinate calls
3. Implement state management

**File: robot_controller_server.py**

```python
#!/usr/bin/env python3
"""
Exercise 4: Robot Controller Server
Manages multiple robot control services
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class RobotControllerServer(Node):
    def __init__(self):
        super().__init__('robot_controller_server')
        
        # Robot state
        self.position_x = 0.0
        self.position_y = 0.0
        self.heading = 0.0  # degrees
        self.battery = 100  # percentage
        self.led_color = 'off'  # off, red, green, blue
        
        # Create services
        self.move_srv = self.create_service(
            AddTwoInts, 'robot/move', self.move_callback
        )
        self.sensor_srv = self.create_service(
            AddTwoInts, 'robot/sensor', self.sensor_callback
        )
        self.led_srv = self.create_service(
            AddTwoInts, 'robot/led', self.led_callback
        )
        
        self.get_logger().info('Robot Controller Server started')

    def move_callback(self, request, response):
        """Handle movement commands"""
        command = request.a  # 1=forward, 2=backward, 3=turn_left, 4=turn_right
        distance = request.b
        
        if command == 1:  # Forward
            self.position_x += distance
            self.get_logger().info(f'Moving forward {distance}m')
        elif command == 2:  # Backward
            self.position_x -= distance
            self.get_logger().info(f'Moving backward {distance}m')
        elif command == 3:  # Turn left
            self.heading += distance
            self.get_logger().info(f'Turning left {distance}°')
        elif command == 4:  # Turn right
            self.heading -= distance
            self.get_logger().info(f'Turning right {distance}°')
        
        self.battery -= 5  # Battery drain
        response.sum = int(self.battery)
        
        return response

    def sensor_callback(self, request, response):
        """Return sensor data"""
        sensor_type = request.a  # 1=position, 2=battery, 3=heading
        
        if sensor_type == 1:
            response.sum = int(self.position_x)
            self.get_logger().info(f'Position: ({self.position_x:.1f}, {self.position_y:.1f})')
        elif sensor_type == 2:
            response.sum = int(self.battery)
            self.get_logger().info(f'Battery: {self.battery}%')
        elif sensor_type == 3:
            response.sum = int(self.heading)
            self.get_logger().info(f'Heading: {self.heading}°')
        
        return response

    def led_callback(self, request, response):
        """Handle LED control"""
        color_code = request.a  # 0=off, 1=red, 2=green, 3=blue
        colors = {0: 'off', 1: 'red', 2: 'green', 3: 'blue'}
        
        self.led_color = colors.get(color_code, 'off')
        self.get_logger().info(f'LED set to: {self.led_color}')
        response.sum = color_code
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerServer()
    
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

**File: robot_controller_client.py**

```python
#!/usr/bin/env python3
"""
Exercise 4: Robot Controller Client
Coordinates multiple service calls to control robot
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class RobotControllerClient(Node):
    def __init__(self):
        super().__init__('robot_controller_client')
        self.move_client = self.create_client(AddTwoInts, 'robot/move')
        self.sensor_client = self.create_client(AddTwoInts, 'robot/sensor')
        self.led_client = self.create_client(AddTwoInts, 'robot/led')

    def move(self, command, distance):
        """Move the robot"""
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Move service not available...')
        
        request = AddTwoInts.Request()
        request.a = command
        request.b = distance
        
        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()

    def read_sensor(self, sensor_type):
        """Read sensor data"""
        while not self.sensor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Sensor service not available...')
        
        request = AddTwoInts.Request()
        request.a = sensor_type
        request.b = 0
        
        future = self.sensor_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()

    def set_led(self, color_code):
        """Set LED color"""
        while not self.led_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('LED service not available...')
        
        request = AddTwoInts.Request()
        request.a = color_code
        request.b = 0
        
        future = self.led_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()

    def execute_mission(self):
        """Execute a robot mission"""
        self.get_logger().info('Starting robot mission...')
        
        # Set LED to green (starting)
        self.set_led(2)
        
        # Move forward 10 units
        self.move(1, 10)
        
        # Check battery
        battery = self.read_sensor(2)
        self.get_logger().info(f'Battery level: {battery.sum}%')
        
        # Turn left 45 degrees
        self.move(3, 45)
        
        # Move forward 5 units
        self.move(1, 5)
        
        # Check position
        position = self.read_sensor(1)
        self.get_logger().info(f'Position: {position.sum}')
        
        # Set LED to blue (done)
        self.set_led(3)
        
        self.get_logger().info('Mission complete!')


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerClient()
    
    try:
        node.execute_mission()
    except Exception as e:
        node.get_logger().error(f'Mission failed: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Expected Output (Server):**

```
[INFO] [robot_controller_server]: Robot Controller Server started
[INFO] [robot_controller_server]: LED set to: green
[INFO] [robot_controller_server]: Moving forward 10m
[INFO] [robot_controller_server]: Battery: 95%
[INFO] [robot_controller_server]: Turning left 45°
[INFO] [robot_controller_server]: Moving forward 5m
[INFO] [robot_controller_server]: Position: (15.0, 0.0)
[INFO] [robot_controller_server]: LED set to: blue
```

**Expected Output (Client):**

```
[INFO] [robot_controller_client]: Starting robot mission...
[INFO] [robot_controller_client]: Battery level: 95
[INFO] [robot_controller_client]: Position: 15
[INFO] [robot_controller_client]: Mission complete!
```

**Key Concepts:**
- Multiple services in one node
- State management across services
- Sequential service calls from client
- Coordinated multi-service workflows
- Battery/resource management

---

## **Commands to Practice**

```bash
# List all active services
ros2 service list

# View service type
ros2 service type /add_numbers

# View service definition
ros2 service find example_interfaces/srv/AddTwoInts

# Call service from command line
ros2 service call /add_numbers example_interfaces/srv/AddTwoInts "{a: 10, b: 20}"

# Monitor service calls (requires separate window)
ros2 service call /add_numbers example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

# View node graph
rqt_graph

# List all nodes
ros2 node list

# Get node info
ros2 node info /calculator_server
```

---

## **✅ Completion Checklist**

- [ ] Exercise 1: Temperature Conversion Service completed
- [ ] Exercise 2: String Processing Service completed
- [ ] Exercise 3: Database Query Service completed
- [ ] Exercise 4: Multi-Service Robot Controller completed
- [ ] All servers start without errors
- [ ] All clients connect and receive responses
- [ ] Command-line service calls work
- [ ] rqt_graph shows proper connections
- [ ] Error handling tested

---

## **💡 Tips & Tricks**

1. **Always check service availability:**
   ```python
   while not client.wait_for_service(timeout_sec=1.0):
       self.get_logger().info('Waiting for service...')
   ```

2. **Use status codes for error handling:**
   ```python
   if response.sum == -1:
       self.get_logger().error('Operation failed')
   ```

3. **List services in real-time:**
   ```bash
   ros2 service list
   ```

4. **Test services from command line:**
   ```bash
   ros2 service call /service_name package/srv/ServiceType "{field: value}"
   ```

5. **Debug service connections:**
   ```bash
   rqt_graph
   ```

6. **Create custom .srv files for better type safety:**
   ```
   # In srv/ folder
   CustomService.srv
   ```

---

**🎓 Congratulations! You've completed the ROS 2 Server & Client Lab!** 🚀✨
