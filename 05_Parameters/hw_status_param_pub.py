#!/usr/bin/env python3
"""
Exercise 1: Basic Parameter Publisher
Publishes HardwareStatus with configurable parameters
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from ce_robot_interfaces.msg import HardwareStatus


class HardwareStatusParamPublisher(Node):
    def __init__(self):
        super().__init__('hw_status_param_pub')
        
        # Declare parameters
        self.declare_parameter(
            'robot_name',
            'robot_default',
            ParameterDescriptor(
                description='Name/identifier of the robot'
            )
        )
        
        self.declare_parameter(
            'robot_number',
            1,
            ParameterDescriptor(
                description='Robot ID number (1-1000)'
            )
        )
        
        self.declare_parameter(
            'publish_rate',
            1.0,
            ParameterDescriptor(
                description='Publishing frequency in Hz'
            )
        )
        
        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.robot_number = self.get_parameter('robot_number').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Create publisher
        self.publisher = self.create_publisher(
            HardwareStatus,
            'hardware_status',
            10
        )
        
        # Create timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(
            timer_period,
            self.timer_callback
        )
        
        self.message_count = 0
        
        self.get_logger().info(
            f'Hardware Status Publisher initialized'
        )
        self.get_logger().info(
            f'Robot: {self.robot_name} (ID: {self.robot_number})'
        )
        self.get_logger().info(
            f'Publish Rate: {publish_rate} Hz'
        )

    def timer_callback(self):
        """Publish HardwareStatus message"""
        msg = HardwareStatus()
        msg.name_robot = self.robot_name
        msg.number_robot = self.robot_number
        msg.temperature = 25 + (self.message_count % 5)
        msg.motor_ready = True
        msg.debug_message = (
            f'Message #{self.message_count} from {self.robot_name}'
        )
        
        self.publisher.publish(msg)
        self.message_count += 1
        
        self.get_logger().info(
            f'Published: {msg.name_robot} - '
            f'Temp: {msg.temperature}°C - '
            f'Count: {self.message_count}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusParamPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
