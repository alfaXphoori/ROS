#!/usr/bin/env python3
"""
Validated Hardware Status Publisher
Adds input validation and error handling
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
import random


class ValidatedHwPublisher(Node):
    def __init__(self):
        super().__init__('validated_hw_publisher')
        self.publisher_ = self.create_publisher(HardwareStatus, 'hardware_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)
        
        # Statistics tracking
        self.total_attempts = 0
        self.successful_publishes = 0
        self.failed_publishes = 0
        
        self.get_logger().info('Validated HW Publisher started!')

    def validate_message(self, msg):
        """
        Validate message fields
        Returns: (is_valid, error_message)
        """
        # Check name_robot
        if not msg.name_robot or len(msg.name_robot) == 0:
            return False, "name_robot cannot be empty"
        
        # Check number_robot range
        if msg.number_robot < 1 or msg.number_robot > 9999:
            return False, f"number_robot must be 1-9999, got {msg.number_robot}"
        
        # Check temperature range
        if msg.temperature < -40 or msg.temperature > 100:
            return False, f"temperature out of range (-40 to 100°C): {msg.temperature}°C"
        
        # Check debug_message length
        if len(msg.debug_message) > 200:
            return False, "debug_message exceeds 200 characters"
        
        return True, "OK"

    def publish_status(self):
        """Create, validate, and publish message"""
        self.total_attempts += 1
        
        # Create message with random data
        msg = HardwareStatus()
        msg.name_robot = 'CE-ROBOT'
        msg.number_robot = random.randint(1000, 1010)  # Valid range
        msg.temperature = random.randint(30, 70)  # Mostly valid, some edge cases
        msg.motor_ready = True
        msg.debug_message = f'Status update #{self.total_attempts}'
        
        # Occasionally generate invalid data for testing
        if self.total_attempts % 10 == 0:
            msg.temperature = 150  # Invalid temperature!
        
        # Validate before publishing
        is_valid, error_msg = self.validate_message(msg)
        
        if is_valid:
            self.publisher_.publish(msg)
            self.successful_publishes += 1
            self.get_logger().info(
                f'✓ Published: {msg.name_robot} | Temp: {msg.temperature}°C | '
                f'Success Rate: {self.get_success_rate():.1f}%'
            )
        else:
            self.failed_publishes += 1
            self.get_logger().error(
                f'❌ Validation FAILED: {error_msg} | '
                f'Failed: {self.failed_publishes}/{self.total_attempts}'
            )

    def get_success_rate(self):
        """Calculate success percentage"""
        if self.total_attempts == 0:
            return 0.0
        return (self.successful_publishes / self.total_attempts) * 100


def main(args=None):
    rclpy.init(args=args)
    node = ValidatedHwPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'\n📊 Final Statistics:\n'
            f'  Total Attempts: {node.total_attempts}\n'
            f'  Successful: {node.successful_publishes}\n'
            f'  Failed: {node.failed_publishes}\n'
            f'  Success Rate: {node.get_success_rate():.1f}%'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
