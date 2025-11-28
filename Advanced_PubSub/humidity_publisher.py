#!/usr/bin/env python3
"""
Humidity Publisher Node
Publishes humidity data at regular intervals
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random
import time


class HumidityPublisher(Node):
    def __init__(self):
        super().__init__('humidity_publisher')
        self.publisher = self.create_publisher(Float64, 'sensor/humidity', 10)
        self.timer = self.create_timer(1.0, self.publish_humidity)
        self.get_logger().info('Humidity Publisher started')

    def publish_humidity(self):
        # Simulate humidity data (30-80%)
        humidity = random.uniform(30.0, 80.0)
        msg = Float64()
        msg.data = humidity
        self.publisher.publish(msg)
        self.get_logger().info(f'Published Humidity: {humidity:.2f}%')


def main(args=None):
    rclpy.init(args=args)
    node = HumidityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
