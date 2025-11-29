#!/usr/bin/env python3
"""
Exercise 2: Parameter Callbacks Publisher
Updates behavior dynamically when parameters change
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from ce_robot_interfaces.msg import HardwareStatus


class HardwareStatusCallbackPublisher(Node):
    def __init__(self):
        super().__init__('hw_status_callback_pub')
        
        # Declare parameters
        self.declare_parameter(
            'robot_name',
            'robot_default',
            ParameterDescriptor(description='Robot name')
        )
        
        self.declare_parameter(
            'robot_number',
            1,
            ParameterDescriptor(description='Robot ID')
        )
        
        self.declare_parameter(
            'publish_rate',
            1.0,
            ParameterDescriptor(description='Publishing rate in Hz')
        )
        
        self.declare_parameter(
            'debug_mode',
            False,
            ParameterDescriptor(description='Enable debug logging')
        )
        
        self.declare_parameter(
            'temperature_offset',
            0,
            ParameterDescriptor(description='Temperature offset')
        )
        
        # Get initial values
        self.robot_name = self.get_parameter('robot_name').value
        self.robot_number = self.get_parameter('robot_number').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.temperature_offset = self.get_parameter('temperature_offset').value
        
        # Create publisher
        self.publisher = self.create_publisher(
            HardwareStatus,
            'hardware_status',
            10
        )
        
        # Create timer
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )
        
        # Register parameter callback
        self.add_on_set_parameters_callback(
            self.parameters_callback
        )
        
        self.message_count = 0
        
        self.get_logger().info('Hardware Status Publisher with Callbacks started')
        self.log_parameters()

    def log_parameters(self):
        """Log current parameter values"""
        self.get_logger().info('=== Current Parameters ===')
        self.get_logger().info(f'robot_name: {self.robot_name}')
        self.get_logger().info(f'robot_number: {self.robot_number}')
        self.get_logger().info(f'publish_rate: {self.publish_rate} Hz')
        self.get_logger().info(f'debug_mode: {self.debug_mode}')
        self.get_logger().info(f'temperature_offset: {self.temperature_offset}°C')

    def parameters_callback(self, params):
        """Handle parameter changes"""
        for param in params:
            if param.name == 'robot_name':
                self.robot_name = param.value
                self.get_logger().info(
                    f'Parameter updated: robot_name = {self.robot_name}'
                )
            
            elif param.name == 'robot_number':
                self.robot_number = param.value
                self.get_logger().info(
                    f'Parameter updated: robot_number = {self.robot_number}'
                )
            
            elif param.name == 'publish_rate':
                self.publish_rate = param.value
                # Recreate timer with new rate
                self.timer.cancel()
                self.timer = self.create_timer(
                    1.0 / self.publish_rate,
                    self.timer_callback
                )
                self.get_logger().info(
                    f'Parameter updated: publish_rate = {self.publish_rate} Hz'
                )
            
            elif param.name == 'debug_mode':
                self.debug_mode = param.value
                self.get_logger().info(
                    f'Parameter updated: debug_mode = {self.debug_mode}'
                )
            
            elif param.name == 'temperature_offset':
                self.temperature_offset = param.value
                self.get_logger().info(
                    f'Parameter updated: temperature_offset = {self.temperature_offset}°C'
                )
        
        return SetParametersResult(successful=True)

    def timer_callback(self):
        """Publish HardwareStatus message"""
        base_temp = 25 + (self.message_count % 5)
        temperature = base_temp + self.temperature_offset
        
        msg = HardwareStatus()
        msg.name_robot = self.robot_name
        msg.number_robot = self.robot_number
        msg.temperature = temperature
        msg.motor_ready = True
        msg.debug_message = (
            f'#{self.message_count} | {self.robot_name} | {temperature}°C'
        )
        
        self.publisher.publish(msg)
        self.message_count += 1
        
        if self.debug_mode:
            self.get_logger().info(
                f'Published: {msg.name_robot} - Temp: {temperature}°C'
            )


def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusCallbackPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
