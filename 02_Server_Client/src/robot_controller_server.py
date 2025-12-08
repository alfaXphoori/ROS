#!/usr/bin/env python3
"""
Exercise 3: Robot Controller Server
Manages multiple robot control services
Service type: AddTwoInts (from example_interfaces)
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
