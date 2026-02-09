#!/usr/bin/env python3
"""
Exercise 1: Basic Publisher
Sends "Hello from Publisher" message every 1 second
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        # Create publisher with String message type
        # Topic name: 'hello_topic'
        # Queue size: 10
        self.pub = self.create_publisher(String, 'hello_topic', 10)
        
        # Create timer to publish every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Simple Publisher started')

    def timer_callback(self):
        # Create message
        msg = String()
        msg.data = "Hello from Publisher"
        
        # Publish message
        self.pub.publish(msg)
        
        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
