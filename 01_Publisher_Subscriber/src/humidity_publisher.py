#!/usr/bin/env python3
"""
Exercise 6: Humidity Publisher - Test publisher for sensor monitor
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class HumidityPublisher(Node):
    def __init__(self):
        super().__init__('humidity_publisher')
        self.pub = self.create_publisher(Float32, 'humidity', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.base_humid = 50.0
        self.count = 0

    def timer_callback(self):
        humidity = self.base_humid + random.uniform(-5, 5)
        humidity = max(30.0, min(80.0, humidity))
        msg = Float32()
        msg.data = humidity
        self.pub.publish(msg)
        self.count += 1
        self.get_logger().info(f'[#{self.count}] Humidity: {humidity:.2f}%')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(HumidityPublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
