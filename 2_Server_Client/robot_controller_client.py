#!/usr/bin/env python3
"""
Exercise 3: Robot Controller Client
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
