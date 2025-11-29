#!/usr/bin/env python3
"""
Exercise 3: Hardware Status Aggregator
Subscribes to hardware_status and aggregates statistics
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
from collections import deque


class HardwareStatusAggregator(Node):
    def __init__(self):
        super().__init__('hardware_status_aggregator')
        self.subscriber = self.create_subscription(
            HardwareStatus, 'hardware_status', self.status_callback, 10
        )
        self.timer = self.create_timer(5.0, self.print_stats)
        
        # Keep history of last 10 messages
        self.temperature_history = deque(maxlen=10)
        self.motor_statuses = []
        self.message_count = 0

    def status_callback(self, msg):
        self.temperature_history.append(msg.temperature)
        self.motor_statuses.append(msg.motor_ready)
        self.message_count += 1
        
        self.get_logger().info(
            f'Received: {msg.name_robot} | '
            f'Temp: {msg.temperature}°C | '
            f'Motor: {msg.motor_ready} | '
            f'Message: {msg.debug_message}'
        )

    def print_stats(self):
        if len(self.temperature_history) == 0:
            return
        
        avg_temp = sum(self.temperature_history) / len(self.temperature_history)
        max_temp = max(self.temperature_history)
        min_temp = min(self.temperature_history)
        motor_on_count = sum(1 for status in self.motor_statuses if status)
        
        self.get_logger().info(
            f'\n=== Statistics (Last {len(self.temperature_history)} messages) ===\n'
            f'Total Messages: {self.message_count}\n'
            f'Temperature - Avg: {avg_temp:.1f}°C, Max: {max_temp}°C, Min: {min_temp}°C\n'
            f'Motor Status - On: {motor_on_count}/{len(self.motor_statuses)}\n'
            f'================================\n'
        )


def main(args=None):
    rclpy.init(args=args)
    aggregator = HardwareStatusAggregator()
    rclpy.spin(aggregator)
    aggregator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
