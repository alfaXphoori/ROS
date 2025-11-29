#!/usr/bin/env python3
"""
Exercise 3: Hardware Status Subscriber
Subscribes to hardware_status and displays received messages
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus


class HardwareStatusSubscriber(Node):
    def __init__(self):
        super().__init__('hardware_status_subscriber')
        self.subscriber = self.create_subscription(
            HardwareStatus, 'hardware_status', self.status_callback, 10
        )
        self.received_count = 0

    def status_callback(self, msg):
        self.received_count += 1
        self.get_logger().info(
            f'[{self.received_count}] Received: {msg.name_robot} | '
            f'Temp: {msg.temperature}°C | '
            f'Motor: {msg.motor_ready} | '
            f'Message: {msg.debug_message}'
        )


def main(args=None):
    rclpy.init(args=args)
    subscriber = HardwareStatusSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
