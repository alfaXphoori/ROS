#!/usr/bin/env python3
"""
Filtering Hardware Status Subscriber
Filters and tracks high-temperature messages
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
from collections import deque


class FilteringSubscriber(Node):
    def __init__(self):
        super().__init__('filtering_subscriber')
        self.subscription = self.create_subscription(
            HardwareStatus,
            'hardware_status',
            self.status_callback,
            10
        )
        
        # Configuration
        self.temp_threshold = 55  # Alert if temperature > 55°C
        
        # Statistics
        self.total_messages = 0
        self.high_temp_count = 0
        self.high_temp_values = deque(maxlen=10)  # Keep last 10 high temps
        
        self.get_logger().info(
            f'Filtering Subscriber started! Temperature threshold: {self.temp_threshold}°C'
        )

    def status_callback(self, msg):
        """Process incoming messages with filtering"""
        self.total_messages += 1
        
        # Filter: Only process high temperature messages
        if msg.temperature > self.temp_threshold:
            self.high_temp_count += 1
            self.high_temp_values.append(msg.temperature)
            
            # Calculate statistics on filtered data
            avg_high = sum(self.high_temp_values) / len(self.high_temp_values)
            max_high = max(self.high_temp_values)
            percentage = (self.high_temp_count / self.total_messages) * 100
            
            self.get_logger().warn(
                f'🔥 HIGH TEMP ALERT: {msg.temperature}°C from {msg.name_robot} | '
                f'Count: {self.high_temp_count}/{self.total_messages} ({percentage:.1f}%) | '
                f'Avg: {avg_high:.1f}°C | Max: {max_high}°C'
            )
        else:
            # Log normal messages at INFO level
            self.get_logger().info(
                f'✓ Normal temp: {msg.temperature}°C from {msg.name_robot} | '
                f'Total: {self.total_messages}, High: {self.high_temp_count}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = FilteringSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'\n📊 Filter Statistics:\n'
            f'  Total Messages: {node.total_messages}\n'
            f'  High Temp Messages: {node.high_temp_count}\n'
            f'  Percentage: {(node.high_temp_count/node.total_messages*100):.1f}%'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
