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
