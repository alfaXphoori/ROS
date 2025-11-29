# **🔧 Custom Service Lab Exercises**

Master custom service creation and usage in ROS 2 through progressive hands-on exercises.

---

## **📌 Project Title**

Create and Use Custom Service Types in ROS 2

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🛠 Lab Overview**

This lab provides hands-on exercises to master custom service definitions and request-response communication patterns in ROS 2. Each exercise builds upon the previous one, progressing from basic service definition through production-quality multi-service coordination.

**Duration:** ~2 hours
**Level:** Beginner to Intermediate
**Prerequisites:** ROS 2 Jazzy installed, Publisher/Subscriber and Server/Client labs completed

---

## **🎯 Learning Objectives**

By completing this lab, you will be able to:

- ✅ Create custom .srv files with request and response fields
- ✅ Build service packages following ROS 2 conventions
- ✅ Implement robust server nodes with proper error handling
- ✅ Create client nodes that make service requests
- ✅ Define complex data structures in service definitions
- ✅ Implement state management in servers
- ✅ Handle multiple service calls sequentially
- ✅ Use service introspection tools for debugging
- ✅ Validate input and handle edge cases
- ✅ Coordinate multi-service workflows from clients

---

## **📊 Lab Architecture**

```
┌─────────────────────────────────────────────┐
│ Exercise 1: Basic Rectangle Calculator      │
│ (Simple service with calculation)           │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 2: Distance Calculator Service     │
│ (Multiple calculations, error handling)     │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 3: Shape Analyzer Service          │
│ (Complex calculations, multiple shapes)     │
└─────────────────────────────────────────────┘
```

---

## **📚 Learning Path Overview**

| Exercise | Title | Level | Duration |
|----------|-------|-------|----------|
| 1 | Rectangle Area Calculator | Beginner | 25 min |
| 2 | Distance Calculator | Intermediate | 30 min |
| 3 | Shape Analyzer | Advanced | 35 min |

---

## **Exercise 1: Rectangle Area Calculator Service (Beginner) 📐**

### **📝 Task**

Create a custom service that calculates the area of a rectangle given length and width.

### **Service Definition: CalRectangle.srv**

```srv
float64 length          # Rectangle length
float64 width           # Rectangle width
---
float64 area_rectangle  # Calculated area
```

### **File: CalRect_server.py**

```python
#!/usr/bin/env python3
"""
Exercise 1: Rectangle Calculator Server
Calculates rectangle area from length and width
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.srv import CalRectangle


class RectangleCalculatorServer(Node):
    def __init__(self):
        super().__init__('calrect_server')
        
        self.srv = self.create_service(
            CalRectangle,
            'cal_rect',
            self.calculate_area_callback
        )
        
        self.get_logger().info('Rectangle Calculator Server started')
        self.get_logger().info('Service: /cal_rect')
        self.get_logger().info('Waiting for requests...')

    def calculate_area_callback(self, request, response):
        """Calculate rectangle area"""
        length = request.length
        width = request.width
        
        # Validate input
        if length <= 0 or width <= 0:
            self.get_logger().warn(
                f'Invalid input: length={length}, width={width}'
            )
            response.area_rectangle = 0
            return response
        
        area = length * width
        response.area_rectangle = area
        
        self.get_logger().info(
            f'Calculated: {length} × {width} = {area} m²'
        )
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RectangleCalculatorServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        self.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **File: CalRect_client.py**

```python
#!/usr/bin/env python3
"""
Exercise 1: Rectangle Calculator Client
Sends area calculation requests
Usage: ros2 run ce_robot cal_rect_client <length> <width>
"""

import sys
import rclpy
from rclpy.node import Node
from ce_robot_interfaces.srv import CalRectangle


class RectangleCalculatorClient(Node):
    def __init__(self):
        super().__init__('calrect_client')

    def send_request(self, length, width):
        """Send rectangle area calculation request"""
        
        client = self.create_client(CalRectangle, 'cal_rect')
        
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        
        request = CalRectangle.Request()
        request.length = length
        request.width = width
        
        self.get_logger().info(
            f'Requesting area for: {length} × {width}'
        )
        future = client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 3:
        print('Usage: cal_rect_client <length> <width>')
        print('Example: cal_rect_client 22.22 33.34')
        sys.exit(1)
    
    try:
        length = float(sys.argv[1])
        width = float(sys.argv[2])
    except ValueError:
        print('Error: length and width must be numbers')
        sys.exit(1)
    
    node = RectangleCalculatorClient()
    response = node.send_request(length, width)
    
    if response.area_rectangle > 0:
        node.get_logger().info(
            f'Result: Area = {response.area_rectangle:.2f} m²'
        )
    else:
        node.get_logger().error('Invalid dimensions provided')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **Testing Exercise 1**

**Terminal 1 - Server:**
```bash
ros2 run ce_robot cal_rect_server
```

**Terminal 2 - Client:**
```bash
ros2 run ce_robot cal_rect_client 22.22 33.34
```

**Expected Output (Server):**
```
[INFO] [calrect_server]: Rectangle Calculator Server started
[INFO] [calrect_server]: Service: /cal_rect
[INFO] [calrect_server]: Waiting for requests...
[INFO] [calrect_server]: Calculated: 22.22 × 33.34 = 741.1248 m²
```

**Expected Output (Client):**
```
[INFO] [calrect_client]: Requesting area for: 22.22 × 33.34
[INFO] [calrect_client]: Result: Area = 741.12 m²
```

### **Key Concepts**

- Service request/response structure
- Input validation in callbacks
- Basic calculations in services
- Error checking before processing

---

## **Exercise 2: Distance Calculator Service (Intermediate) 📍**

### **📝 Task**

Create a custom service that calculates distances between two points (Euclidean and Manhattan distance).

### **Service Definition: DistanceCalculator.srv**

```srv
float64 x1
float64 y1
float64 x2
float64 y2
int32 distance_type  # 1=Euclidean, 2=Manhattan
---
float64 distance
string distance_name
```

### **File: distance_calc_server.py**

```python
#!/usr/bin/env python3
"""
Exercise 2: Distance Calculator Server
Calculates Euclidean and Manhattan distances between two points
"""

import rclpy
import math
from rclpy.node import Node
from ce_robot_interfaces.srv import DistanceCalculator


class DistanceCalculatorServer(Node):
    def __init__(self):
        super().__init__('distance_server')
        
        self.srv = self.create_service(
            DistanceCalculator,
            'calculate_distance',
            self.calculate_distance_callback
        )
        
        self.call_count = 0
        self.get_logger().info('Distance Calculator Server started')
        self.get_logger().info('Service: /calculate_distance')

    def calculate_distance_callback(self, request, response):
        """Calculate distance between two points"""
        self.call_count += 1
        
        x1, y1 = request.x1, request.y1
        x2, y2 = request.x2, request.y2
        distance_type = request.distance_type
        
        self.get_logger().info(
            f'Request #{self.call_count}: '
            f'Point1({x1}, {y1}) → Point2({x2}, {y2}), '
            f'Type: {distance_type}'
        )
        
        if distance_type == 1:  # Euclidean
            distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            response.distance_name = 'Euclidean'
        elif distance_type == 2:  # Manhattan
            distance = abs(x2 - x1) + abs(y2 - y1)
            response.distance_name = 'Manhattan'
        else:
            self.get_logger().warn(f'Unknown distance type: {distance_type}')
            distance = 0
            response.distance_name = 'Unknown'
        
        response.distance = distance
        
        self.get_logger().info(
            f'{response.distance_name} distance: {distance:.4f}'
        )
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DistanceCalculatorServer()
    
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

### **File: distance_calc_client.py**

```python
#!/usr/bin/env python3
"""
Exercise 2: Distance Calculator Client
Usage: ros2 run ce_robot distance_calc_client <x1> <y1> <x2> <y2> <type>
Types: 1=Euclidean, 2=Manhattan
"""

import sys
import rclpy
from rclpy.node import Node
from ce_robot_interfaces.srv import DistanceCalculator


class DistanceCalculatorClient(Node):
    def __init__(self):
        super().__init__('distance_client')

    def calculate_distance(self, x1, y1, x2, y2, distance_type):
        """Request distance calculation"""
        
        client = self.create_client(
            DistanceCalculator,
            'calculate_distance'
        )
        
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        
        request = DistanceCalculator.Request()
        request.x1 = x1
        request.y1 = y1
        request.x2 = x2
        request.y2 = y2
        request.distance_type = distance_type
        
        self.get_logger().info(f'Requesting distance calculation...')
        future = client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 6:
        print('Usage: distance_calc_client <x1> <y1> <x2> <y2> <type>')
        print('Types: 1=Euclidean, 2=Manhattan')
        print('Example: distance_calc_client 0 0 3 4 1')
        sys.exit(1)
    
    try:
        x1 = float(sys.argv[1])
        y1 = float(sys.argv[2])
        x2 = float(sys.argv[3])
        y2 = float(sys.argv[4])
        distance_type = int(sys.argv[5])
    except ValueError:
        print('Error: Invalid input values')
        sys.exit(1)
    
    node = DistanceCalculatorClient()
    response = node.calculate_distance(x1, y1, x2, y2, distance_type)
    
    node.get_logger().info(
        f'{response.distance_name} Distance: {response.distance:.4f} units'
    )
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **Testing Exercise 2**

**Terminal 1 - Server:**
```bash
ros2 run ce_robot distance_calc_server
```

**Terminal 2 - Client (Euclidean distance 3-4-5 triangle):**
```bash
ros2 run ce_robot distance_calc_client 0 0 3 4 1
```

**Expected Output:**
```
[INFO] [distance_server]: Request #1: Point1(0, 0) → Point2(3, 4), Type: 1
[INFO] [distance_server]: Euclidean distance: 5.0000

[distance_client]: Euclidean Distance: 5.0000 units
```

**Terminal 2 - Client (Manhattan distance):**
```bash
ros2 run ce_robot distance_calc_client 0 0 3 4 2
```

**Expected Output:**
```
[INFO] [distance_server]: Request #2: Point1(0, 0) → Point2(3, 4), Type: 2
[INFO] [distance_server]: Manhattan distance: 7.0000

[distance_client]: Manhattan Distance: 7.0000 units
```

### **Key Concepts**

- Complex request structures with multiple fields
- Conditional logic in service handlers
- Returning different response types
- Math operations in services
- Request counter/logging

---

## **Exercise 3: Shape Analyzer Service (Advanced) 🔷**

### **📝 Task**

Create a custom service that analyzes different geometric shapes and calculates their properties.

### **Service Definition: ShapeAnalyzer.srv**

```srv
int32 shape_type      # 1=Circle, 2=Triangle, 3=Ellipse
float64 param1         # Radius/Side1/SemiMajor
float64 param2         # unused/Side2/SemiMinor
float64 param3         # unused/Side3/unused
---
float64 area
float64 perimeter
string shape_name
string properties
```

### **File: shape_analyzer_server.py**

```python
#!/usr/bin/env python3
"""
Exercise 3: Shape Analyzer Server
Analyzes geometric shapes and calculates properties
"""

import rclpy
import math
from rclpy.node import Node
from ce_robot_interfaces.srv import ShapeAnalyzer


class ShapeAnalyzerServer(Node):
    def __init__(self):
        super().__init__('shape_analyzer_server')
        
        self.srv = self.create_service(
            ShapeAnalyzer,
            'analyze_shape',
            self.analyze_shape_callback
        )
        
        self.shape_count = 0
        self.get_logger().info('Shape Analyzer Server started')
        self.get_logger().info('Service: /analyze_shape')

    def analyze_shape_callback(self, request, response):
        """Analyze geometric shape properties"""
        self.shape_count += 1
        
        shape_type = request.shape_type
        param1 = request.param1
        param2 = request.param2
        param3 = request.param3
        
        self.get_logger().info(
            f'Request #{self.shape_count}: Type={shape_type}, '
            f'Params=({param1}, {param2}, {param3})'
        )
        
        if shape_type == 1:  # Circle
            response.shape_name = 'Circle'
            response.area = math.pi * param1**2
            response.perimeter = 2 * math.pi * param1
            response.properties = f'Radius: {param1}'
        
        elif shape_type == 2:  # Triangle
            response.shape_name = 'Triangle'
            # Using Heron's formula
            s = (param1 + param2 + param3) / 2
            area = math.sqrt(s * (s - param1) * (s - param2) * (s - param3))
            response.area = area
            response.perimeter = param1 + param2 + param3
            response.properties = (
                f'Sides: {param1}, {param2}, {param3} | '
                f'Semi-perimeter: {s}'
            )
        
        elif shape_type == 3:  # Ellipse
            response.shape_name = 'Ellipse'
            response.area = math.pi * param1 * param2
            # Approximate perimeter using Ramanujan's formula
            h = ((param1 - param2)**2) / ((param1 + param2)**2)
            perimeter = math.pi * (param1 + param2) * (
                1 + (3*h) / (10 + math.sqrt(4 - 3*h))
            )
            response.perimeter = perimeter
            response.properties = (
                f'Semi-major axis: {param1}, '
                f'Semi-minor axis: {param2}'
            )
        
        else:
            self.get_logger().warn(f'Unknown shape type: {shape_type}')
            response.shape_name = 'Unknown'
            response.area = 0
            response.perimeter = 0
            response.properties = 'Invalid shape type'
        
        self.get_logger().info(
            f'{response.shape_name}: '
            f'Area={response.area:.4f}, '
            f'Perimeter={response.perimeter:.4f}'
        )
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ShapeAnalyzerServer()
    
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

### **File: shape_analyzer_client.py**

```python
#!/usr/bin/env python3
"""
Exercise 3: Shape Analyzer Client
Usage: ros2 run ce_robot shape_analyzer_client <shape_type> <param1> [param2] [param3]
Types: 1=Circle (r), 2=Triangle (a,b,c), 3=Ellipse (a,b)
"""

import sys
import rclpy
from rclpy.node import Node
from ce_robot_interfaces.srv import ShapeAnalyzer


class ShapeAnalyzerClient(Node):
    def __init__(self):
        super().__init__('shape_analyzer_client')

    def analyze_shape(self, shape_type, param1, param2=0, param3=0):
        """Request shape analysis"""
        
        client = self.create_client(ShapeAnalyzer, 'analyze_shape')
        
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        
        request = ShapeAnalyzer.Request()
        request.shape_type = shape_type
        request.param1 = param1
        request.param2 = param2
        request.param3 = param3
        
        self.get_logger().info('Analyzing shape...')
        future = client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 3:
        print('Usage: shape_analyzer_client <shape_type> <param1> [param2] [param3]')
        print('Types:')
        print('  1 = Circle (requires: radius)')
        print('  2 = Triangle (requires: side1 side2 side3)')
        print('  3 = Ellipse (requires: semi_major semi_minor)')
        print('Examples:')
        print('  shape_analyzer_client 1 5')
        print('  shape_analyzer_client 2 3 4 5')
        print('  shape_analyzer_client 3 5 3')
        sys.exit(1)
    
    try:
        shape_type = int(sys.argv[1])
        param1 = float(sys.argv[2])
        param2 = float(sys.argv[3]) if len(sys.argv) > 3 else 0
        param3 = float(sys.argv[4]) if len(sys.argv) > 4 else 0
    except ValueError:
        print('Error: Invalid input values')
        sys.exit(1)
    
    node = ShapeAnalyzerClient()
    response = node.analyze_shape(shape_type, param1, param2, param3)
    
    node.get_logger().info(f'\n=== Shape Analysis Results ===')
    node.get_logger().info(f'Shape: {response.shape_name}')
    node.get_logger().info(f'Area: {response.area:.4f}')
    node.get_logger().info(f'Perimeter: {response.perimeter:.4f}')
    node.get_logger().info(f'Properties: {response.properties}')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **Testing Exercise 3**

**Terminal 1 - Server:**
```bash
ros2 run ce_robot shape_analyzer_server
```

**Terminal 2 - Client (Circle with radius 5):**
```bash
ros2 run ce_robot shape_analyzer_client 1 5
```

**Expected Output:**
```
[INFO] [shape_analyzer_server]: Request #1: Type=1, Params=(5, 0, 0)
[INFO] [shape_analyzer_server]: Circle: Area=78.5398, Perimeter=31.4159

[INFO] [shape_analyzer_client]: === Shape Analysis Results ===
[INFO] [shape_analyzer_client]: Shape: Circle
[INFO] [shape_analyzer_client]: Area: 78.5398
[INFO] [shape_analyzer_client]: Perimeter: 31.4159
[INFO] [shape_analyzer_client]: Properties: Radius: 5
```

**Terminal 2 - Client (Triangle with sides 3, 4, 5):**
```bash
ros2 run ce_robot shape_analyzer_client 2 3 4 5
```

**Expected Output:**
```
[INFO] [shape_analyzer_server]: Request #2: Type=2, Params=(3, 4, 5)
[INFO] [shape_analyzer_server]: Triangle: Area=6.0000, Perimeter=12.0000

[INFO] [shape_analyzer_client]: === Shape Analysis Results ===
[INFO] [shape_analyzer_client]: Shape: Triangle
[INFO] [shape_analyzer_client]: Area: 6.0000
[INFO] [shape_analyzer_client]: Perimeter: 12.0000
[INFO] [shape_analyzer_client]: Properties: Sides: 3, 4, 5 | Semi-perimeter: 6
```

### **Key Concepts**

- Complex service definitions with multiple parameters
- Conditional shape-specific calculations
- Mathematical formulas (Heron's, ellipse approximation)
- Rich response data structures
- Request/response logging and metrics

---

## **Commands to Practice**

```bash
# List all active services
ros2 service list

# View service type
ros2 service type /cal_rect

# View service definition
ros2 interface show ce_robot_interfaces/srv/CalRectangle

# Call service from command line
ros2 service call /cal_rect ce_robot_interfaces/srv/CalRectangle "{length: 5.5, width: 3.2}"

# View service information
ros2 service info /cal_rect

# Visualize service connections
rqt_graph

# List all nodes
ros2 node list

# Get node information
ros2 node info /calrect_server
```

---

## **✅ Completion Checklist**

- [ ] Exercise 1: Rectangle Calculator completed
  - [ ] CalRectangle.srv defined
  - [ ] Server runs successfully
  - [ ] Client connects and receives responses
  - [ ] Input validation working

- [ ] Exercise 2: Distance Calculator completed
  - [ ] DistanceCalculator.srv defined
  - [ ] Euclidean distance calculation correct
  - [ ] Manhattan distance calculation correct
  - [ ] Multiple request types handled

- [ ] Exercise 3: Shape Analyzer completed
  - [ ] ShapeAnalyzer.srv defined
  - [ ] Circle analysis working
  - [ ] Triangle analysis working
  - [ ] Ellipse analysis working
  - [ ] Complex response structures handled

- [ ] All services built successfully
- [ ] All clients connect and receive responses
- [ ] Command-line service calls work
- [ ] rqt_graph shows proper connections
- [ ] All error cases handled gracefully

---

## **💡 Tips & Tricks**

1. **Always source setup.bash before running:**
   ```bash
   source ~/.bashrc
   ```

2. **Rebuild services when .srv files change:**
   ```bash
   colcon build --packages-select ce_robot_interfaces
   source install/setup.bash
   ```

3. **Test services from command line before running clients:**
   ```bash
   ros2 service call /service_name package/srv/ServiceType "{field: value}"
   ```

4. **Use rqt_graph to visualize connections:**
   ```bash
   rqt_graph
   ```

5. **Monitor service calls in real-time:**
   ```bash
   ros2 service call /service_name --verbose
   ```

6. **Always wait for service availability before calling:**
   ```python
   while not client.wait_for_service(timeout_sec=1.0):
       self.get_logger().info('Waiting for service...')
   ```

---

**🎓 Congratulations! You've completed the ROS 2 Custom Service Lab!** 🚀✨
