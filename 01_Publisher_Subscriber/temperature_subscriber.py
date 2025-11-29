#!/usr/bin/env python3
"""
Exercise 5: Temperature Subscriber - View published temperature data
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class TemperatureSubscriber(Node):
    def __init__(self):
        super().__init__('temperature_subscriber')
        # Subscribe to temperature topic
        self.sub = self.create_subscription(
            Float32,
            'temperature',
            self.callback,
            10
        )
        self.min_temp = 30.0
        self.max_temp = 20.0
        self.count = 0
        self.get_logger().info('Temperature Subscriber started')

    def callback(self, msg):
        temp = msg.data
        self.count += 1
        
        # Track min and max
        if temp < self.min_temp:
            self.min_temp = temp
        if temp > self.max_temp:
            self.max_temp = temp
        
        # Display temperature
        self.get_logger().info(
            f'Received #{self.count}: {temp:.2f}°C '
            f'(Min: {self.min_temp:.2f}°C, Max: {self.max_temp:.2f}°C)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
