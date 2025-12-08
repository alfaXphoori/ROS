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
