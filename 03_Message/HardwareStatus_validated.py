#!/usr/bin/env python3
"""
Exercise 4: Validated Hardware Status Publisher
Publishes messages with input validation and error handling
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
import random


class ValidatedHardwarePublisher(Node):
    def __init__(self):
        super().__init__('validated_hw_publisher')
        self.publisher = self.create_publisher(HardwareStatus, 'hardware_status', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.robot_id = 1
        self.count = 0
        self.error_count = 0

    def validate_message(self, msg):
        """Validate message fields before publishing"""
        errors = []
        
        # Validate name_robot
        if not msg.name_robot or len(msg.name_robot) == 0:
            errors.append("name_robot cannot be empty")
        
        # Validate number_robot
        if msg.number_robot < 1 or msg.number_robot > 1000:
            errors.append(f"number_robot must be 1-1000, got {msg.number_robot}")
        
        # Validate temperature
        if msg.temperature < -40 or msg.temperature > 100:
            errors.append(f"temperature out of range: {msg.temperature}°C")
        
        # Validate debug_message
        if len(msg.debug_message) > 200:
            errors.append("debug_message exceeds 200 characters")
        
        return errors

    def timer_callback(self):
        msg = HardwareStatus()
        msg.name_robot = f'Robot-{self.robot_id}'
        msg.number_robot = self.robot_id
        msg.temperature = random.randint(20, 80)
        msg.motor_ready = random.choice([True, False])
        msg.debug_message = f'Cycle #{self.count}: {"Motors running" if msg.motor_ready else "Motors stopped"}'
        
        # Validate before publishing
        errors = self.validate_message(msg)
        
        if errors:
            self.error_count += 1
            self.get_logger().error(f'Validation failed: {", ".join(errors)}')
            return
        
        try:
            self.publisher.publish(msg)
            self.get_logger().info(
                f'✓ Published [{self.count}]: {msg.name_robot} | '
                f'Temp: {msg.temperature}°C | Motor: {msg.motor_ready}'
            )
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Publishing error: {str(e)}')
        
        self.count += 1
        
        # Report stats every 10 cycles
        if self.count % 10 == 0:
            success_rate = ((self.count - self.error_count) / self.count) * 100
            self.get_logger().info(
                f'Stats: Published {self.count - self.error_count}/{self.count} '
                f'({success_rate:.1f}% success rate)'
            )


def main(args=None):
    rclpy.init(args=args)
    publisher = ValidatedHardwarePublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
