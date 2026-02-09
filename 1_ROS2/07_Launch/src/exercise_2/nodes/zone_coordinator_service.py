#!/usr/bin/env python3
"""
Zone Coordinator Service for Multi-Robot System
Manages robot assignments and coordinates tasks within warehouse zones
Real-world use: Warehouse zone management and robot task allocation
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
import random


class ZoneCoordinatorService(Node):
    """
    Service for coordinating robots within a specific warehouse zone
    Handles task assignments and zone-specific operations
    """
    
    def __init__(self):
        super().__init__('zone_coordinator')
        
        # Declare parameters
        self.declare_parameter('robot_id', 'AMR-UNKNOWN-000')
        self.declare_parameter('zone_id', 'UNKNOWN-ZONE')
        self.declare_parameter('robot_type', 'transport')
        self.declare_parameter('max_capacity', 100.0)
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.zone_id = self.get_parameter('zone_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.max_capacity = self.get_parameter('max_capacity').value
        
        # Get namespace
        self.namespace = self.get_namespace()
        
        # Create service
        self.service = self.create_service(
            SetBool,
            'request_task',
            self.request_task_callback
        )
        
        self.tasks_assigned = 0
        
        self.get_logger().info('🎯 Zone Coordinator Service Started')
        self.get_logger().info(f'Namespace: {self.namespace}')
        self.get_logger().info(f'Robot ID: {self.robot_id}')
        self.get_logger().info(f'Zone: {self.zone_id}')
        self.get_logger().info(f'Robot Type: {self.robot_type}')
        self.get_logger().info(f'Max Capacity: {self.max_capacity}kg')
        self.get_logger().info(f'Service: {self.namespace}/request_task')
    
    def request_task_callback(self, request, response):
        """
        Handle task request from fleet management
        Request.data = True: Robot is ready for new task
        Request.data = False: Robot requests to return to base
        """
        
        if request.data:
            # Robot requesting new task
            task = self.assign_task()
            response.success = True
            response.message = f'Task assigned: {task}'
            self.tasks_assigned += 1
            
            self.get_logger().info(
                f'✅ [{self.namespace}] Task assigned to {self.robot_id}:\n'
                f'   Zone: {self.zone_id}\n'
                f'   Task: {task}\n'
                f'   Total tasks assigned: {self.tasks_assigned}'
            )
        else:
            # Robot requesting return to base
            response.success = True
            response.message = f'Approved: Return to base in {self.zone_id}'
            
            self.get_logger().info(
                f'🔙 [{self.namespace}] {self.robot_id} returning to base\n'
                f'   Zone: {self.zone_id}\n'
                f'   Reason: Low battery or maintenance'
            )
        
        return response
    
    def assign_task(self):
        """Assign appropriate task based on robot type and zone"""
        
        if self.robot_type == 'transport':
            tasks = [
                f'TRANSPORT-PALLET-{random.randint(1000, 9999)}',
                f'MOVE-CARGO-TO-{self.zone_id}',
                f'DELIVER-TO-ZONE-{chr(random.randint(65, 68))}',
                'LOAD-TRUCK-BAY-3',
                'UNLOAD-DELIVERY-DOCK-5'
            ]
        elif self.robot_type == 'picker':
            tasks = [
                f'PICK-ORDER-{random.randint(10000, 99999)}',
                f'COLLECT-ITEMS-AISLE-{random.randint(1, 20)}',
                f'FULFILL-ORDER-ZONE-{self.zone_id}',
                'RESTOCK-SHELF-A12',
                'INVENTORY-COUNT-SECTION-B'
            ]
        elif self.robot_type == 'sorter':
            tasks = [
                f'SORT-PACKAGES-BATCH-{random.randint(100, 999)}',
                'ORGANIZE-INCOMING-SHIPMENT',
                f'CATEGORIZE-ITEMS-{self.zone_id}',
                'CONSOLIDATE-RETURNS',
                'PROCESS-EXPRESS-ORDERS'
            ]
        else:
            tasks = [
                f'GENERAL-TASK-{random.randint(1, 100)}',
                f'ZONE-OPERATION-{self.zone_id}'
            ]
        
        return random.choice(tasks)


def main(args=None):
    rclpy.init(args=args)
    node = ZoneCoordinatorService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'🔴 [{node.namespace}] Zone Coordinator shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
