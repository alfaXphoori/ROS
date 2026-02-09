#!/usr/bin/env python3
"""
Exercise 2: Basic Subscriber
Listens to messages from simple_publisher and prints them with timestamp
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime


class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        # Subscribe to the same topic as the publisher
        self.sub = self.create_subscription(
            String,              # Message type
            'hello_topic',       # Topic name
            self.callback,       # Callback function
            10                   # Queue size
        )
        self.get_logger().info('Simple Subscriber started - waiting for messages...')

    def callback(self, msg):
        # Get current time
        timestamp = datetime.now().strftime('%H:%M:%S')
        
        # Log received message with timestamp
        self.get_logger().info(f'[{timestamp}] Received: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
