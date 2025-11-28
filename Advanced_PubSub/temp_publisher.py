#!/usr/bin/env python3
"""
Temperature Publisher Node
Publishes temperature data at regular intervals
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random
import time


class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher = self.create_publisher(Float64, 'sensor/temperature', 10)
        self.timer = self.create_timer(1.0, self.publish_temperature)
        self.get_logger().info('Temperature Publisher started')

    def publish_temperature(self):
        # Simulate temperature data (20-30°C)
        temperature = random.uniform(20.0, 30.0)
        msg = Float64()
        msg.data = temperature
        self.publisher.publish(msg)
        self.get_logger().info(f'Published Temperature: {temperature:.2f}°C')


def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
