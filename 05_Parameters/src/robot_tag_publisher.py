#!/usr/bin/env python3
"""
Parameterized Robot Tag Publisher
Demonstrates parameter declaration, access, and callbacks for fleet management
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from ce_robot_interfaces.msg import RobotTag
from builtin_interfaces.msg import Time


class RobotTagPublisher(Node):
    def __init__(self):
        super().__init__('robot_tag_publisher')
        
        # Declare parameters with descriptors
        self.declare_parameter(
            'robot_id',
            'WH-BOT-001',
            ParameterDescriptor(description='Unique robot identifier (e.g., WH-BOT-042)')
        )
        
        self.declare_parameter(
            'robot_type',
            'transport',
            ParameterDescriptor(description='Robot type: transport, delivery, inspection, loader')
        )
        
        self.declare_parameter(
            'zone_id',
            'WAREHOUSE-A',
            ParameterDescriptor(description='Current operational zone (e.g., ZONE-A, LOADING-BAY-3)')
        )
        
        self.declare_parameter(
            'fleet_number',
            1,
            ParameterDescriptor(description='Fleet assignment number (1-999)')
        )
        
        self.declare_parameter(
            'max_payload_kg',
            500.0,
            ParameterDescriptor(description='Maximum payload capacity in kilograms (10.0-1000.0)')
        )
        
        self.declare_parameter(
            'priority_level',
            5,
            ParameterDescriptor(description='Task priority level (0-10, 0=lowest, 10=highest)')
        )
        
        self.declare_parameter(
            'tag_publish_rate',
            1.0,
            ParameterDescriptor(description='Tag update frequency in Hz (0.1-10.0)')
        )
        
        self.declare_parameter(
            'firmware_version',
            'v2.3.1',
            ParameterDescriptor(description='Current firmware version (e.g., v2.3.1)')
        )
        
        # Get initial parameter values
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.zone_id = self.get_parameter('zone_id').value
        self.fleet_number = self.get_parameter('fleet_number').value
        self.max_payload_kg = self.get_parameter('max_payload_kg').value
        self.priority_level = self.get_parameter('priority_level').value
        self.tag_rate = self.get_parameter('tag_publish_rate').value
        self.firmware_version = self.get_parameter('firmware_version').value
        
        # Register parameter callback for runtime changes
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Create publisher
        self.publisher = self.create_publisher(RobotTag, 'robot_tag', 10)
        
        # Create timer
        self.timer = self.create_timer(1.0 / self.tag_rate, self.timer_callback)
        
        # Message counter and operation hours tracker
        self.message_count = 0
        self.operation_hours = 0.0
        
        # Print initialization info
        self.get_logger().info('🏷️  Robot Tag Publisher initialized')
        self.get_logger().info(f'   Robot ID: {self.robot_id}')
        self.get_logger().info(f'   Robot Type: {self.robot_type}')
        self.get_logger().info(f'   Zone: {self.zone_id}')
        self.get_logger().info(f'   Fleet Number: {self.fleet_number}')
        self.get_logger().info(f'   Max Payload: {self.max_payload_kg} kg')
        self.get_logger().info(f'   Priority Level: {self.priority_level}')
        self.get_logger().info(f'   Tag Rate: {self.tag_rate} Hz')
        self.get_logger().info(f'   Firmware: {self.firmware_version}')

    def timer_callback(self):
        """Publish robot tag message"""
        tag = RobotTag()
        
        # Robot identification
        tag.robot_id = self.robot_id
        tag.robot_type = self.robot_type
        tag.zone_id = self.zone_id
        tag.fleet_number = self.fleet_number
        
        # Operational status (simulate different states)
        status_cycle = self.message_count % 100
        if status_cycle < 80:
            tag.status = "active"
        elif status_cycle < 90:
            tag.status = "idle"
        elif status_cycle < 95:
            tag.status = "charging"
        else:
            tag.status = "maintenance"
        
        tag.priority_level = self.priority_level
        tag.max_payload_kg = self.max_payload_kg
        
        # Location & Assignment (simulate)
        if tag.status == "active":
            tag.current_location = f"SHELF-A-{self.message_count % 20 + 1}"
            tag.assigned_task = f"TASK-{8800 + self.message_count}"
            tag.assigned_operator = "AUTO"
        elif tag.status == "maintenance":
            tag.current_location = "MAINT-BAY-1"
            tag.assigned_task = "MAINT-SCHEDULED-2024"
            tag.assigned_operator = "TECH-003"
        else:
            tag.current_location = f"DOCK-{self.fleet_number}"
            tag.assigned_task = ""
            tag.assigned_operator = "AUTO"
        
        # Timestamps
        current_time = self.get_clock().now().to_msg()
        tag.last_maintenance.sec = current_time.sec - 86400  # 1 day ago
        tag.deployment_date.sec = current_time.sec - 7776000  # 90 days ago
        
        # Increment operation hours (simulated)
        self.operation_hours += (1.0 / self.tag_rate) / 3600.0  # Convert seconds to hours
        tag.operation_hours = self.operation_hours
        
        # Safety & Compliance
        tag.safety_certified = (tag.status != "maintenance")
        tag.firmware_version = self.firmware_version
        tag.error_code = 0 if tag.status != "maintenance" else 204
        
        self.publisher.publish(tag)
        self.message_count += 1
        
        # Log status periodically
        if self.message_count % 5 == 0:
            self.get_logger().info(
                f'🤖 {tag.robot_id} [{tag.robot_type}]: '
                f'Status={tag.status}, Zone={tag.zone_id}, '
                f'Location={tag.current_location}, Hours={tag.operation_hours:.1f}'
            )

    def parameter_callback(self, params):
        """Handle parameter changes at runtime with validation"""
        for param in params:
            if param.name == 'robot_id':
                self.robot_id = param.value
                self.get_logger().info(f'✅ Updated robot_id = {self.robot_id}')
                
            elif param.name == 'robot_type':
                valid_types = ['transport', 'delivery', 'inspection', 'loader']
                if param.value in valid_types:
                    self.robot_type = param.value
                    self.get_logger().info(f'✅ Updated robot_type = {self.robot_type}')
                else:
                    self.get_logger().error(f'❌ Invalid type: {param.value} (must be {valid_types})')
                    return SetParametersResult(successful=False)
                    
            elif param.name == 'zone_id':
                self.zone_id = param.value
                self.get_logger().info(f'✅ Updated zone_id = {self.zone_id}')
                
            elif param.name == 'fleet_number':
                if 1 <= param.value <= 999:
                    self.fleet_number = param.value
                    self.get_logger().info(f'✅ Updated fleet_number = {self.fleet_number}')
                else:
                    self.get_logger().error(f'❌ Invalid fleet number: {param.value} (must be 1-999)')
                    return SetParametersResult(successful=False)
                    
            elif param.name == 'max_payload_kg':
                if 10.0 <= param.value <= 1000.0:
                    self.max_payload_kg = param.value
                    self.get_logger().info(f'✅ Updated max_payload_kg = {self.max_payload_kg}')
                else:
                    self.get_logger().error(f'❌ Invalid payload: {param.value} (must be 10.0-1000.0)')
                    return SetParametersResult(successful=False)
                    
            elif param.name == 'priority_level':
                if 0 <= param.value <= 10:
                    self.priority_level = param.value
                    self.get_logger().info(f'✅ Updated priority_level = {self.priority_level}')
                else:
                    self.get_logger().error(f'❌ Invalid priority: {param.value} (must be 0-10)')
                    return SetParametersResult(successful=False)
                    
            elif param.name == 'firmware_version':
                self.firmware_version = param.value
                self.get_logger().info(f'✅ Updated firmware_version = {self.firmware_version}')
                    
            elif param.name == 'tag_publish_rate':
                if 0.1 <= param.value <= 10.0:
                    self.tag_rate = param.value
                    # Recreate timer with new rate
                    self.timer.cancel()
                    self.timer = self.create_timer(1.0 / self.tag_rate, self.timer_callback)
                    self.get_logger().info(f'✅ Updated tag_publish_rate = {self.tag_rate} Hz')
                else:
                    self.get_logger().error(f'❌ Invalid rate: {param.value} (must be 0.1-10.0)')
                    return SetParametersResult(successful=False)
        
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = RobotTagPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
