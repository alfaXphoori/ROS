#!/usr/bin/env python3
"""
Exercise 3: Fleet Management with Validation
Validates fleet parameters and uses YAML configuration for warehouse robots
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from ce_robot_interfaces.msg import RobotTag
from builtin_interfaces.msg import Time


class RobotTagValidatedPublisher(Node):
    def __init__(self):
        super().__init__('robot_tag_validated_pub')
        
        # Declare parameters with constraints documentation
        self.declare_parameter(
            'robot_id',
            'WH-BOT-001',
            ParameterDescriptor(description='Robot ID (non-empty, format: XXX-XXX-NNN)')
        )
        
        self.declare_parameter(
            'robot_type',
            'transport',
            ParameterDescriptor(description='Robot type: transport, delivery, inspection, loader')
        )
        
        self.declare_parameter(
            'zone_id',
            'WAREHOUSE-A',
            ParameterDescriptor(description='Operational zone (non-empty string)')
        )
        
        self.declare_parameter(
            'fleet_number',
            1,
            ParameterDescriptor(description='Fleet number (1-999)')
        )
        
        self.declare_parameter(
            'max_payload_kg',
            500.0,
            ParameterDescriptor(description='Maximum payload (10.0-1000.0 kg)')
        )
        
        self.declare_parameter(
            'priority_level',
            5,
            ParameterDescriptor(description='Priority level (0-10)')
        )
        
        self.declare_parameter(
            'tag_publish_rate',
            1.0,
            ParameterDescriptor(description='Publishing rate (0.1-10.0 Hz)')
        )
        
        self.declare_parameter(
            'firmware_version',
            'v2.3.1',
            ParameterDescriptor(description='Firmware version string')
        )
        
        self.declare_parameter(
            'safety_check_enabled',
            True,
            ParameterDescriptor(description='Enable safety parameter validation')
        )
        
        # Get and validate parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.zone_id = self.get_parameter('zone_id').value
        self.fleet_number = self.get_parameter('fleet_number').value
        self.max_payload_kg = self.get_parameter('max_payload_kg').value
        self.priority_level = self.get_parameter('priority_level').value
        self.tag_rate = self.get_parameter('tag_publish_rate').value
        self.firmware_version = self.get_parameter('firmware_version').value
        self.safety_check_enabled = self.get_parameter('safety_check_enabled').value
        
        # Validate on startup
        if not self.validate_parameters():
            self.get_logger().error('❌ Invalid fleet parameters on startup!')
            return
        
        # Create publisher
        self.publisher = self.create_publisher(RobotTag, 'robot_tag', 10)
        
        # Create timer
        self.timer = self.create_timer(1.0 / self.tag_rate, self.timer_callback)
        
        # Register parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.message_count = 0
        self.operation_hours = 0.0
        self.safety_violations = 0
        
        self.get_logger().info('🏷️  Fleet Management with Validation started')
        self.log_parameters()

    def validate_parameters(self):
        """Validate all fleet parameters with detailed error reporting"""
        errors = []
        
        # Validate robot_id (non-empty)
        if not self.robot_id or len(self.robot_id.strip()) == 0:
            errors.append('robot_id: Must be non-empty')
        
        # Validate robot_type (enum)
        valid_types = ['transport', 'delivery', 'inspection', 'loader']
        if self.robot_type not in valid_types:
            errors.append(f'robot_type: Must be one of {valid_types}')
        
        # Validate zone_id (non-empty)
        if not self.zone_id or len(self.zone_id.strip()) == 0:
            errors.append('zone_id: Must be non-empty')
        
        # Validate fleet_number (1-999)
        if self.fleet_number < 1 or self.fleet_number > 999:
            errors.append('fleet_number: Must be between 1-999')
        
        # Validate max_payload_kg (10.0-1000.0)
        if self.max_payload_kg < 10.0 or self.max_payload_kg > 1000.0:
            errors.append('max_payload_kg: Must be between 10.0-1000.0 kg')
        
        # Validate priority_level (0-10)
        if self.priority_level < 0 or self.priority_level > 10:
            errors.append('priority_level: Must be between 0-10')
        
        # Validate tag_publish_rate (0.1-10.0)
        if self.tag_rate < 0.1 or self.tag_rate > 10.0:
            errors.append('tag_publish_rate: Must be between 0.1-10.0 Hz')
        
        # Type-specific payload validation
        if self.robot_type == 'delivery' and self.max_payload_kg > 100.0:
            errors.append('delivery robots: max_payload_kg should be ≤100.0 kg')
        elif self.robot_type == 'inspection' and self.max_payload_kg > 50.0:
            errors.append('inspection robots: max_payload_kg should be ≤50.0 kg')
        elif self.robot_type == 'transport' and self.max_payload_kg < 100.0:
            errors.append('transport robots: max_payload_kg should be ≥100.0 kg')
        
        if errors:
            self.get_logger().error('Fleet parameter validation errors:')
            for error in errors:
                self.get_logger().error(f'  • {error}')
            return False
        
        return True

    def parameters_callback(self, params):
        """Handle parameter changes with full fleet validation"""
        # Store old values for rollback
        old_robot_id = self.robot_id
        old_robot_type = self.robot_type
        old_zone_id = self.zone_id
        old_fleet_number = self.fleet_number
        old_max_payload_kg = self.max_payload_kg
        old_priority_level = self.priority_level
        old_tag_rate = self.tag_rate
        old_firmware_version = self.firmware_version
        old_safety_check = self.safety_check_enabled
        
        # Update temporary values
        for param in params:
            if param.name == 'robot_id':
                self.robot_id = param.value
            elif param.name == 'robot_type':
                self.robot_type = param.value
            elif param.name == 'zone_id':
                self.zone_id = param.value
            elif param.name == 'fleet_number':
                self.fleet_number = param.value
            elif param.name == 'max_payload_kg':
                self.max_payload_kg = param.value
            elif param.name == 'priority_level':
                self.priority_level = param.value
            elif param.name == 'tag_publish_rate':
                self.tag_rate = param.value
            elif param.name == 'firmware_version':
                self.firmware_version = param.value
            elif param.name == 'safety_check_enabled':
                self.safety_check_enabled = param.value
        
        # Validate new parameters
        if self.safety_check_enabled and not self.validate_parameters():
            # Restore old values if validation fails
            self.robot_id = old_robot_id
            self.robot_type = old_robot_type
            self.zone_id = old_zone_id
            self.fleet_number = old_fleet_number
            self.max_payload_kg = old_max_payload_kg
            self.priority_level = old_priority_level
            self.tag_rate = old_tag_rate
            self.firmware_version = old_firmware_version
            self.safety_check_enabled = old_safety_check
            
            self.safety_violations += 1
            self.get_logger().error(f'❌ Parameter validation failed (violations: {self.safety_violations})')
            return SetParametersResult(successful=False, reason='Fleet parameter validation failed')
        
        # If tag_publish_rate changed, recreate timer
        if self.tag_rate != old_tag_rate:
            self.timer.cancel()
            self.timer = self.create_timer(1.0 / self.tag_rate, self.timer_callback)
            self.get_logger().info(f'✅ Publishing rate updated: {self.tag_rate} Hz')
        
        # Log other changes
        if self.robot_type != old_robot_type:
            self.get_logger().info(f'✅ Robot type changed: {old_robot_type} → {self.robot_type}')
        if self.zone_id != old_zone_id:
            self.get_logger().info(f'✅ Zone changed: {old_zone_id} → {self.zone_id}')
        if self.priority_level != old_priority_level:
            self.get_logger().info(f'✅ Priority changed: {old_priority_level} → {self.priority_level}')
        
        return SetParametersResult(successful=True)

    def log_parameters(self):
        """Log current fleet parameter values"""
        self.get_logger().info('=== Fleet Configuration ===')
        self.get_logger().info(f'robot_id: {self.robot_id}')
        self.get_logger().info(f'robot_type: {self.robot_type}')
        self.get_logger().info(f'zone_id: {self.zone_id}')
        self.get_logger().info(f'fleet_number: {self.fleet_number}')
        self.get_logger().info(f'max_payload_kg: {self.max_payload_kg} kg')
        self.get_logger().info(f'priority_level: {self.priority_level}')
        self.get_logger().info(f'tag_publish_rate: {self.tag_rate} Hz')
        self.get_logger().info(f'firmware_version: {self.firmware_version}')
        self.get_logger().info(f'safety_check_enabled: {self.safety_check_enabled}')

    def timer_callback(self):
        """Publish validated RobotTag message"""
        tag = RobotTag()
        
        # Robot identification
        tag.robot_id = self.robot_id
        tag.robot_type = self.robot_type
        tag.zone_id = self.zone_id
        tag.fleet_number = self.fleet_number
        
        # Operational status with payload checking
        status_cycle = self.message_count % 4
        if status_cycle == 0:
            tag.status = "active"
            tag.current_location = f"SHELF-A-{self.message_count % 20 + 1}"
        elif status_cycle == 1:
            tag.status = "idle"
            tag.current_location = f"DOCK-{self.fleet_number}"
        elif status_cycle == 2:
            tag.status = "charging"
            tag.current_location = "CHARGING-STATION-1"
        else:
            tag.status = "maintenance"
            tag.current_location = "MAINT-BAY-1"
        
        tag.priority_level = self.priority_level
        tag.max_payload_kg = self.max_payload_kg
        tag.assigned_task = f"TASK-{8800 + self.message_count}"
        tag.assigned_operator = "AUTO"
        
        # Timestamps
        current_time = self.get_clock().now().to_msg()
        tag.last_maintenance.sec = current_time.sec - 86400
        tag.deployment_date.sec = current_time.sec - 7776000
        
        self.operation_hours += (1.0 / self.tag_rate) / 3600.0
        tag.operation_hours = self.operation_hours
        
        # Safety & Compliance
        tag.safety_certified = self.safety_check_enabled and (tag.status != "maintenance")
        tag.firmware_version = self.firmware_version
        tag.error_code = 0 if tag.status != "maintenance" else 204
        
        # Safety alert for high priority maintenance
        if tag.status == "maintenance" and self.priority_level > 7:
            alert = '⚠️  HIGH PRIORITY MAINTENANCE'
        elif tag.status == "active":
            alert = '✓ OPERATIONAL'
        else:
            alert = f'• {tag.status.upper()}'
        
        self.publisher.publish(tag)
        self.message_count += 1
        
        self.get_logger().info(
            f'🤖 {tag.robot_id} [{tag.robot_type}] {alert} | '
            f'Zone={tag.zone_id}, Priority={tag.priority_level}, '
            f'Payload={tag.max_payload_kg}kg, Violations={self.safety_violations}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotTagValidatedPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
