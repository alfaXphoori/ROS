#!/usr/bin/env python3
"""
Exercise 4: Validated Hardware Status Subscriber
Subscribes with error handling and data validation
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus


class ValidatedSubscriber(Node):
    def __init__(self):
        super().__init__('validated_hw_subscriber')
        self.subscriber = self.create_subscription(
            HardwareStatus, 'hardware_status', self.status_callback, 10
        )
        self.received_count = 0
        self.error_count = 0

    def status_callback(self, msg):
        try:
            # Basic validation check
            if msg.temperature > 75:
                self.get_logger().warn(
                    f'⚠️  HIGH TEMPERATURE ALERT: {msg.name_robot} reached {msg.temperature}°C'
                )
            
            if not msg.motor_ready:
                self.get_logger().info(f'🛑 {msg.name_robot} motors are offline')
            
            self.received_count += 1
            self.get_logger().info(
                f'[{self.received_count}] Received: {msg.name_robot} - '
                f'Temp: {msg.temperature}°C, Motor: {msg.motor_ready}'
            )
        
        except AttributeError as e:
            self.error_count += 1
            self.get_logger().error(f'Missing message field: {str(e)}')
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Processing error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    subscriber = ValidatedSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
