#!/usr/bin/env python3
"""
First Server Node - Basic Addition Service
Simple example to demonstrate ROS 2 Service (Server side)
Service type: AddTwoInts (from example_interfaces)
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class FirstServerNode(Node):
    def __init__(self):
        super().__init__('first_server_node')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )
        self.get_logger().info('First Server Node started')
        self.get_logger().info('Service: /add_two_ints')

    def add_two_ints_callback(self, request, response):
        """Handle addition requests"""
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Incoming request: a={request.a}, b={request.b}'
        )
        self.get_logger().info(
            f'Sending response: {request.a} + {request.b} = {response.sum}'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FirstServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
