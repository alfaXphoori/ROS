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
