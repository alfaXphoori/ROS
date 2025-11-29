#!/usr/bin/env python3
"""
Exercise 4a: Counter Processor
Subscribes to counter and only processes even numbers
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class CounterProcessor(Node):
    def __init__(self):
        super().__init__('counter_processor')
        # Subscribe to counter topic
        self.sub = self.create_subscription(
            Int32,
            'counter_topic',
            self.callback,
            10
        )
        self.get_logger().info('Counter Processor started - filtering even numbers')

    def callback(self, msg):
        # Check if number is even
        if msg.data % 2 == 0:
            self.get_logger().info(f'Even: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = CounterProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
