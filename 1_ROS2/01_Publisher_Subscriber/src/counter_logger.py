#!/usr/bin/env python3
"""
Exercise 4b: Counter Logger
Subscribes to counter and only processes odd numbers
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class CounterLogger(Node):
    def __init__(self):
        super().__init__('counter_logger')
        # Subscribe to counter topic
        self.sub = self.create_subscription(
            Int32,
            'counter_topic',
            self.callback,
            10
        )
        self.get_logger().info('Counter Logger started - filtering odd numbers')

    def callback(self, msg):
        # Check if number is odd
        if msg.data % 2 != 0:
            self.get_logger().info(f'Odd: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = CounterLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
