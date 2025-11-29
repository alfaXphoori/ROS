#!/usr/bin/env python3
"""
Exercise 3: Counter Publisher
Publishes incrementing counter from 0 to 100 every 500ms
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class CounterPublisher(Node):
    def __init__(self):
        super().__init__('counter_publisher')
        # Create publisher with Int32 message type
        self.pub = self.create_publisher(Int32, 'counter_topic', 10)
        
        # Create timer to publish every 500ms (0.5 seconds)
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # Initialize counter
        self.counter = 0
        self.get_logger().info('Counter Publisher started')

    def timer_callback(self):
        # Create Int32 message
        msg = Int32()
        msg.data = self.counter
        
        # Publish message
        self.pub.publish(msg)
        
        # Log counter value
        self.get_logger().info(f'Counter: {self.counter}')
        
        # Increment counter
        self.counter += 1
        
        # Stop after reaching 100
        if self.counter > 100:
            self.get_logger().info('Finished! Counter reached 100')
            exit()


def main(args=None):
    rclpy.init(args=args)
    node = CounterPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
