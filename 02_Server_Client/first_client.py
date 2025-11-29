#!/usr/bin/env python3
"""
First Client Node - Basic Service Client
Simple example to demonstrate ROS 2 Service (Client side)
Service type: AddTwoInts (from example_interfaces)
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class FirstClientNode(Node):
    def __init__(self):
        super().__init__('first_client_node')

    def send_request(self, a, b):
        """Send addition request to server"""
        client = self.create_client(AddTwoInts, 'add_two_ints')
        
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        self.get_logger().info(f'Sending request: {a} + {b}')
        future = client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 3:
        print('Usage: first_client <a> <b>')
        sys.exit(1)
    
    a = int(sys.argv[1])
    b = int(sys.argv[2])
    
    node = FirstClientNode()
    response = node.send_request(a, b)
    
    node.get_logger().info(f'Result: {a} + {b} = {response.sum}')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
