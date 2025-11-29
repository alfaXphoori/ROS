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
