#!/usr/bin/env python3
"""
Exercise 3: Validated Parameter Publisher
Validates parameters and uses YAML configuration
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from ce_robot_interfaces.msg import HardwareStatus


class HardwareStatusValidatedPublisher(Node):
    def __init__(self):
        super().__init__('hw_status_validated_pub')
        
        # Declare parameters with constraints
        self.declare_parameter(
            'robot_name',
            'robot_default',
            ParameterDescriptor(description='Robot name (non-empty string)')
        )
        
        self.declare_parameter(
            'robot_number',
            1,
            ParameterDescriptor(description='Robot ID (1-9999)')
        )
        
        self.declare_parameter(
            'publish_rate',
            1.0,
            ParameterDescriptor(description='Publishing rate (0.1-10.0 Hz)')
        )
        
        self.declare_parameter(
            'temperature_min',
            -40,
            ParameterDescriptor(description='Minimum temperature threshold')
        )
        
        self.declare_parameter(
            'temperature_max',
            100,
            ParameterDescriptor(description='Maximum temperature threshold')
        )
        
        # Get and validate parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.robot_number = self.get_parameter('robot_number').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.temperature_min = self.get_parameter('temperature_min').value
        self.temperature_max = self.get_parameter('temperature_max').value
        
        # Validate on startup
        if not self.validate_parameters():
            self.get_logger().error('Invalid parameters on startup!')
            return
        
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
        self.temperature_violations = 0
        
        self.get_logger().info('Validated Parameter Publisher started')
        self.log_parameters()

    def validate_parameters(self):
        """Validate all parameters"""
        errors = []
        
        # Validate robot_name
        if not self.robot_name or len(self.robot_name.strip()) == 0:
            errors.append('robot_name: Must be non-empty')
        
        # Validate robot_number
        if self.robot_number < 1 or self.robot_number > 9999:
            errors.append('robot_number: Must be between 1-9999')
        
        # Validate publish_rate
        if self.publish_rate < 0.1 or self.publish_rate > 10.0:
            errors.append('publish_rate: Must be between 0.1-10.0 Hz')
        
        # Validate temperature thresholds
        if self.temperature_min >= self.temperature_max:
            errors.append(
                'temperature_min must be less than temperature_max'
            )
        
        if errors:
            self.get_logger().error('Parameter validation errors:')
            for error in errors:
                self.get_logger().error(f'  - {error}')
            return False
        
        return True

    def parameters_callback(self, params):
        """Handle parameter changes with validation"""
        # Store old values in case validation fails
        old_robot_name = self.robot_name
        old_robot_number = self.robot_number
        old_publish_rate = self.publish_rate
        old_temperature_min = self.temperature_min
        old_temperature_max = self.temperature_max
        
        # Update temporary values
        for param in params:
            if param.name == 'robot_name':
                self.robot_name = param.value
            elif param.name == 'robot_number':
                self.robot_number = param.value
            elif param.name == 'publish_rate':
                self.publish_rate = param.value
            elif param.name == 'temperature_min':
                self.temperature_min = param.value
            elif param.name == 'temperature_max':
                self.temperature_max = param.value
        
        # Validate new parameters
        if not self.validate_parameters():
            # Restore old values if validation fails
            self.robot_name = old_robot_name
            self.robot_number = old_robot_number
            self.publish_rate = old_publish_rate
            self.temperature_min = old_temperature_min
            self.temperature_max = old_temperature_max
            
            return SetParametersResult(successful=False)
        
        # If publish_rate changed, recreate timer
        if self.publish_rate != old_publish_rate:
            self.timer.cancel()
            self.timer = self.create_timer(
                1.0 / self.publish_rate,
                self.timer_callback
            )
            self.get_logger().info(
                f'Publishing rate updated: {self.publish_rate} Hz'
            )
        
        return SetParametersResult(successful=True)

    def log_parameters(self):
        """Log current parameter values"""
        self.get_logger().info('=== Current Parameters ===')
        self.get_logger().info(f'robot_name: {self.robot_name}')
        self.get_logger().info(f'robot_number: {self.robot_number}')
        self.get_logger().info(f'publish_rate: {self.publish_rate} Hz')
        self.get_logger().info(
            f'temperature range: [{self.temperature_min}, {self.temperature_max}]°C'
        )

    def timer_callback(self):
        """Publish HardwareStatus message"""
        base_temp = 25 + (self.message_count % 5)
        
        # Check temperature thresholds
        if base_temp < self.temperature_min or base_temp > self.temperature_max:
            self.temperature_violations += 1
            alert = '⚠️ ALERT'
        else:
            alert = '✓'
        
        msg = HardwareStatus()
        msg.name_robot = self.robot_name
        msg.number_robot = self.robot_number
        msg.temperature = base_temp
        msg.motor_ready = True
        msg.debug_message = (
            f'#{self.message_count} {alert} {base_temp}°C'
        )
        
        self.publisher.publish(msg)
        self.message_count += 1
        
        self.get_logger().info(
            f'{msg.debug_message} | Violations: {self.temperature_violations}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusValidatedPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
