#!/usr/bin/env python3
"""
Multi-Robot Publisher
Simulates multiple robots publishing to the same topic
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
import random


class MultiRobotPublisher(Node):
    def __init__(self):
        super().__init__('multi_robot_publisher')
        self.publisher_ = self.create_publisher(HardwareStatus, 'hardware_status', 10)
        self.timer = self.create_timer(0.5, self.publish_status)  # Publish twice per second
        
        # Simulate 3 different robots
        self.robots = [
            {'name': 'CE-ROBOT-01', 'number': 1001, 'temp_base': 45},
            {'name': 'CE-ROBOT-02', 'number': 1002, 'temp_base': 55},
            {'name': 'CE-ROBOT-03', 'number': 1003, 'temp_base': 40},
        ]
        
        self.publish_count = 0
        self.get_logger().info('Multi-Robot Publisher started! Simulating 3 robots...')

    def publish_status(self):
        """Publish messages from different robots in rotation"""
        self.publish_count += 1
        
        # Round-robin through robots
        robot = self.robots[self.publish_count % len(self.robots)]
        
        msg = HardwareStatus()
        msg.name_robot = robot['name']
        msg.number_robot = robot['number']
        msg.temperature = robot['temp_base'] + random.randint(-5, 15)
        msg.motor_ready = random.choice([True, True, True, False])  # 75% ready
        msg.debug_message = f'Status update #{self.publish_count}'
        
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published: {msg.name_robot} | Temp: {msg.temperature}°C | Motor: {msg.motor_ready}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
