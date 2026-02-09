#!/usr/bin/env python3
"""
Robot Status Publisher for Multi-Robot System
Publishes detailed robot status with namespace-aware information
Real-world use: Fleet management and robot coordination in warehouses
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class RobotStatusPublisher(Node):
    """
    Publishes comprehensive robot status information
    Designed for multi-robot systems with namespace isolation
    """
    
    def __init__(self):
        super().__init__('robot_status_publisher')
        
        # Declare parameters
        self.declare_parameter('robot_id', 'AMR-UNKNOWN-000')
        self.declare_parameter('robot_type', 'transport')
        self.declare_parameter('zone_id', 'UNKNOWN-ZONE')
        self.declare_parameter('fleet_number', 0)
        self.declare_parameter('max_payload_kg', 100.0)
        self.declare_parameter('battery_level', 100.0)
        self.declare_parameter('current_location', 'UNKNOWN')
        self.declare_parameter('assigned_task', 'IDLE')
        self.declare_parameter('status_rate_hz', 1.0)
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.zone_id = self.get_parameter('zone_id').value
        self.fleet_number = self.get_parameter('fleet_number').value
        self.max_payload = self.get_parameter('max_payload_kg').value
        self.battery_level = self.get_parameter('battery_level').value
        self.current_location = self.get_parameter('current_location').value
        self.assigned_task = self.get_parameter('assigned_task').value
        status_rate = self.get_parameter('status_rate_hz').value
        
        # Create publisher
        self.publisher = self.create_publisher(
            String,
            'robot_status',
            10
        )
        
        # Create timer
        timer_period = 1.0 / status_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.publish_count = 0
        
        # Get namespace (important for multi-robot systems)
        self.namespace = self.get_namespace()
        
        self.get_logger().info('🤖 Robot Status Publisher Started')
        self.get_logger().info(f'Namespace: {self.namespace}')
        self.get_logger().info(f'Robot ID: {self.robot_id}')
        self.get_logger().info(f'Robot Type: {self.robot_type}')
        self.get_logger().info(f'Zone: {self.zone_id}')
        self.get_logger().info(f'Fleet Number: {self.fleet_number}')
        self.get_logger().info(f'Max Payload: {self.max_payload}kg')
        self.get_logger().info(f'Battery: {self.battery_level}%')
        self.get_logger().info(f'Location: {self.current_location}')
        self.get_logger().info(f'Task: {self.assigned_task}')
        self.get_logger().info(f'Publishing to: {self.namespace}/robot_status')
    
    def timer_callback(self):
        """Publish robot status"""
        # Create status message as JSON
        status = {
            'namespace': self.namespace,
            'robot_id': self.robot_id,
            'robot_type': self.robot_type,
            'zone_id': self.zone_id,
            'fleet_number': self.fleet_number,
            'max_payload_kg': self.max_payload,
            'battery_level': self.battery_level,
            'current_location': self.current_location,
            'assigned_task': self.assigned_task,
            'publish_count': self.publish_count,
            'operational_status': self.get_operational_status()
        }
        
        msg = String()
        msg.data = json.dumps(status, indent=2)
        self.publisher.publish(msg)
        
        # Log status periodically
        if self.publish_count % 10 == 0:
            self.get_logger().info(
                f'[{self.namespace}] Status #{self.publish_count}: '
                f'{self.robot_id} | {self.operational_status()} | '
                f'Battery: {self.battery_level}% | Task: {self.assigned_task}'
            )
        
        self.publish_count += 1
    
    def get_operational_status(self):
        """Determine operational status based on battery and task"""
        if self.battery_level < 15:
            return 'CRITICAL_BATTERY'
        elif self.battery_level < 30:
            return 'LOW_BATTERY'
        elif self.assigned_task == 'IDLE':
            return 'IDLE'
        elif 'CHARGING' in self.assigned_task.upper():
            return 'CHARGING'
        elif 'MAINTENANCE' in self.assigned_task.upper():
            return 'MAINTENANCE'
        else:
            return 'OPERATIONAL'


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'🔴 [{node.namespace}] Robot Status Publisher shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
