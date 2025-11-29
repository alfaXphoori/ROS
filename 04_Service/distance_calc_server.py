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
