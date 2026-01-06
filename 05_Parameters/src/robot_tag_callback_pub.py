#!/usr/bin/env python3
"""
Exercise 2: Dynamic Fleet Configuration Publisher
Updates robot behavior dynamically when fleet parameters change
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from ce_robot_interfaces.msg import RobotTag
from builtin_interfaces.msg import Time


class RobotTagCallbackPublisher(Node):
    def __init__(self):
        super().__init__('robot_tag_callback_pub')
        
        # Declare fleet management parameters
        self.declare_parameter(
            'robot_id',
            'WH-BOT-001',
            ParameterDescriptor(description='Unique robot identifier')
        )
        
        self.declare_parameter(
            'robot_type',
            'transport',
            ParameterDescriptor(description='Robot type: transport/delivery/inspection/loader')
        )
        
        self.declare_parameter(
            'zone_id',
            'WAREHOUSE-A',
            ParameterDescriptor(description='Current operational zone')
        )
        
        self.declare_parameter(
            'fleet_number',
            1,
            ParameterDescriptor(description='Fleet assignment number (1-999)')
        )
        
        self.declare_parameter(
            'max_payload_kg',
            500.0,
            ParameterDescriptor(description='Maximum payload capacity in kg')
        )
        
        self.declare_parameter(
            'priority_level',
            5,
            ParameterDescriptor(description='Task priority (0-10)')
        )
        
        self.declare_parameter(
            'tag_publish_rate',
            1.0,
            ParameterDescriptor(description='Publishing rate in Hz (0.1-10.0)')
        )
        
        self.declare_parameter(
            'debug_mode',
            False,
            ParameterDescriptor(description='Enable debug logging')
        )
        
        # Get initial values
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.zone_id = self.get_parameter('zone_id').value
        self.fleet_number = self.get_parameter('fleet_number').value
        self.max_payload_kg = self.get_parameter('max_payload_kg').value
        self.priority_level = self.get_parameter('priority_level').value
        self.tag_rate = self.get_parameter('tag_publish_rate').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # Create publisher
        self.publisher = self.create_publisher(RobotTag, 'robot_tag', 10)
        
        # Create timer
        self.timer = self.create_timer(1.0 / self.tag_rate, self.timer_callback)
        
        # Register parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.message_count = 0
        self.operation_hours = 0.0
        
        self.get_logger().info('🏷️  Dynamic Fleet Configuration Publisher started')
        self.log_parameters()

    def log_parameters(self):
        """Log current fleet parameters"""
        self.get_logger().info('=== Current Fleet Configuration ===')
        self.get_logger().info(f'robot_id: {self.robot_id}')
        self.get_logger().info(f'robot_type: {self.robot_type}')
        self.get_logger().info(f'zone_id: {self.zone_id}')
        self.get_logger().info(f'fleet_number: {self.fleet_number}')
        self.get_logger().info(f'max_payload_kg: {self.max_payload_kg} kg')
        self.get_logger().info(f'priority_level: {self.priority_level}')
        self.get_logger().info(f'tag_publish_rate: {self.tag_rate} Hz')
        self.get_logger().info(f'debug_mode: {self.debug_mode}')

    def parameters_callback(self, params):
        """Handle fleet parameter changes"""
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
                    return SetParametersResult(successful=False, reason='Invalid robot type')
                    
            elif param.name == 'zone_id':
                self.zone_id = param.value
                self.get_logger().info(f'✅ Updated zone_id = {self.zone_id}')
                
            elif param.name == 'fleet_number':
                if 1 <= param.value <= 999:
                    self.fleet_number = param.value
                    self.get_logger().info(f'✅ Updated fleet_number = {self.fleet_number}')
                else:
                    self.get_logger().error(f'❌ Fleet number must be 1-999')
                    return SetParametersResult(successful=False, reason='Fleet number out of range')
                    
            elif param.name == 'max_payload_kg':
                if 10.0 <= param.value <= 1000.0:
                    self.max_payload_kg = param.value
                    self.get_logger().info(f'✅ Updated max_payload_kg = {self.max_payload_kg} kg')
                else:
                    self.get_logger().error(f'❌ Payload must be 10.0-1000.0 kg')
                    return SetParametersResult(successful=False, reason='Payload out of range')
                    
            elif param.name == 'priority_level':
                if 0 <= param.value <= 10:
                    self.priority_level = param.value
                    self.get_logger().info(f'✅ Updated priority_level = {self.priority_level}')
                else:
                    self.get_logger().error(f'❌ Priority must be 0-10')
                    return SetParametersResult(successful=False, reason='Priority out of range')
                    
            elif param.name == 'tag_publish_rate':
                if 0.1 <= param.value <= 10.0:
                    self.tag_rate = param.value
                    self.timer.cancel()
                    self.timer = self.create_timer(1.0 / self.tag_rate, self.timer_callback)
                    self.get_logger().info(f'✅ Updated tag_publish_rate = {self.tag_rate} Hz')
                else:
                    self.get_logger().error(f'❌ Rate must be 0.1-10.0 Hz')
                    return SetParametersResult(successful=False, reason='Rate out of range')
                    
            elif param.name == 'debug_mode':
                self.debug_mode = param.value
                self.get_logger().info(f'✅ Updated debug_mode = {self.debug_mode}')
        
        return SetParametersResult(successful=True)

    def timer_callback(self):
        """Publish RobotTag message"""
        tag = RobotTag()
        
        # Robot identification
        tag.robot_id = self.robot_id
        tag.robot_type = self.robot_type
        tag.zone_id = self.zone_id
        tag.fleet_number = self.fleet_number
        
        # Operational status
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
        tag.safety_certified = (tag.status != "maintenance")
        tag.firmware_version = "v2.3.1"
        tag.error_code = 0 if tag.status != "maintenance" else 204
        
        self.publisher.publish(tag)
        self.message_count += 1
        
        if self.debug_mode:
            self.get_logger().info(
                f'🤖 {tag.robot_id} [{tag.robot_type}]: Status={tag.status}, '
                f'Zone={tag.zone_id}, Priority={tag.priority_level}, Payload={tag.max_payload_kg}kg'
            )


def main(args=None):
    rclpy.init(args=args)
    node = RobotTagCallbackPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
