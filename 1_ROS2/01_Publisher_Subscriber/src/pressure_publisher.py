#!/usr/bin/env python3
"""
Exercise 6: Pressure Publisher - Test publisher for sensor monitor
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class PressurePublisher(Node):
    def __init__(self):
        super().__init__('pressure_publisher')
        self.pub = self.create_publisher(Float32, 'pressure', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.base_press = 1013.0
        self.count = 0

    def timer_callback(self):
        pressure = self.base_press + random.uniform(-10, 10)
        pressure = max(950.0, min(1050.0, pressure))
        msg = Float32()
        msg.data = pressure
        self.pub.publish(msg)
        self.count += 1
        self.get_logger().info(f'[#{self.count}] Pressure: {pressure:.2f} hPa')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PressurePublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
