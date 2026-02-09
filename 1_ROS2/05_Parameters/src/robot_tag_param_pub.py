#!/usr/bin/env python3
"""
Exercise 1: Basic RobotTag Parameter Publisher
Publishes RobotTag with configurable fleet parameters
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from ce_robot_interfaces.msg import RobotTag
from builtin_interfaces.msg import Time


class RobotTagParamPublisher(Node):
    def __init__(self):
        super().__init__('robot_tag_param_pub')
        
        # Declare parameters
        self.declare_parameter(
            'robot_id',
            'WH-BOT-001',
            ParameterDescriptor(
                description='Unique robot identifier (e.g., WH-BOT-042, DLV-003)'
            )
        )
        
        self.declare_parameter(
            'robot_type',
            'transport',
            ParameterDescriptor(
                description='Robot type: transport, delivery, inspection, loader'
            )
        )
        
        self.declare_parameter(
            'zone_id',
            'WAREHOUSE-A',
            ParameterDescriptor(
                description='Current operational zone (e.g., ZONE-A, LOADING-BAY-3)'
            )
        )
        
        self.declare_parameter(
            'fleet_number',
            1,
            ParameterDescriptor(
                description='Fleet assignment number (1-999)'
            )
        )
        
        self.declare_parameter(
            'tag_publish_rate',
            1.0,
            ParameterDescriptor(
                description='Tag publishing frequency in Hz (0.1-10.0)'
            )
        )
        
        # Get parameter values
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.zone_id = self.get_parameter('zone_id').value
        self.fleet_number = self.get_parameter('fleet_number').value
        publish_rate = self.get_parameter('tag_publish_rate').value
        
        # Create publisher
        self.publisher = self.create_publisher(
            RobotTag,
            'robot_tag',
            10
        )
        
        # Create timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(
            timer_period,
            self.timer_callback
        )
        
        self.message_count = 0
        self.operation_hours = 0.0
        
        self.get_logger().info('🏷️  Robot Tag Publisher initialized')
        self.get_logger().info(f'   Robot ID: {self.robot_id}')
        self.get_logger().info(f'   Robot Type: {self.robot_type}')
        self.get_logger().info(f'   Zone: {self.zone_id}')
        self.get_logger().info(f'   Fleet Number: {self.fleet_number}')
        self.get_logger().info(f'   Publish Rate: {publish_rate} Hz')

    def timer_callback(self):
        """Publish RobotTag message"""
        tag = RobotTag()
        
        # Robot identification
        tag.robot_id = self.robot_id
        tag.robot_type = self.robot_type
        tag.zone_id = self.zone_id
        tag.fleet_number = self.fleet_number
        
        # Operational status (simple cycle)
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
        
        tag.priority_level = 5
        tag.max_payload_kg = 500.0
        tag.assigned_task = f"TASK-{8800 + self.message_count}"
        tag.assigned_operator = "AUTO"
        
        # Timestamps
        current_time = self.get_clock().now().to_msg()
        tag.last_maintenance.sec = current_time.sec - 86400
        tag.deployment_date.sec = current_time.sec - 7776000
        
        # Operation hours
        self.operation_hours += 0.001
        tag.operation_hours = self.operation_hours
        
        # Safety & Compliance
        tag.safety_certified = True
        tag.firmware_version = "v2.3.1"
        tag.error_code = 0
        
        self.publisher.publish(tag)
        self.message_count += 1
        
        self.get_logger().info(
            f'🤖 {tag.robot_id} [{tag.robot_type}]: '
            f'Status={tag.status}, Zone={tag.zone_id}, Location={tag.current_location}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotTagParamPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
