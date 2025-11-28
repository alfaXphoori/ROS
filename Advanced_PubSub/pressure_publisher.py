#!/usr/bin/env python3
"""
Pressure Publisher Node
Publishes pressure data at regular intervals
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random
import time


class PressurePublisher(Node):
    def __init__(self):
        super().__init__('pressure_publisher')
        self.publisher = self.create_publisher(Float64, 'sensor/pressure', 10)
        self.timer = self.create_timer(1.0, self.publish_pressure)
        self.get_logger().info('Pressure Publisher started')

    def publish_pressure(self):
        # Simulate pressure data (950-1050 hPa)
        pressure = random.uniform(950.0, 1050.0)
        msg = Float64()
        msg.data = pressure
        self.publisher.publish(msg)
        self.get_logger().info(f'Published Pressure: {pressure:.2f} hPa')


def main(args=None):
    rclpy.init(args=args)
    node = PressurePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
