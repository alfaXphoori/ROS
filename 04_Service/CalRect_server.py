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
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
