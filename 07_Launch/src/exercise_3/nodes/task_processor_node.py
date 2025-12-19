#!/usr/bin/env python3
"""
Exercise 3: Task Processor Node
NON-CRITICAL - Real-world robot task queue processing
Handles warehouse tasks: picking, packing, transport, inventory
Failure is logged but system continues operating
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random


class TaskProcessorNode(Node):
    """
    Task queue processor for warehouse operations
    Processes: pick orders, transport tasks, inventory updates
    Non-critical: Can fail without stopping other robot systems
    """
    
    def __init__(self):
        super().__init__('task_processor')
        
        # Declare parameters
        self.declare_parameter('robot_id', 'AMR-TASK-001')
        self.declare_parameter('robot_type', 'picker')
        self.declare_parameter('zone_id', 'WAREHOUSE-PICKING-AREA')
        self.declare_parameter('max_tasks_per_hour', 50)
        self.declare_parameter('process_rate_hz', 0.5)  # Process tasks every 2 seconds
        self.declare_parameter('simulate_failure', False)
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.zone_id = self.get_parameter('zone_id').value
        self.max_tasks = self.get_parameter('max_tasks_per_hour').value
        self.rate = self.get_parameter('process_rate_hz').value
        self.simulate_failure = self.get_parameter('simulate_failure').value
        
        # Task queue state
        self.tasks_completed = 0
        self.tasks_pending = 12
        self.tasks_failed = 0
        self.current_task = None
        self.processing = False
        self.queue_status = 'ready'
        
        # Task types
        self.task_types = [
            'PICK_ORDER',
            'TRANSPORT_ITEM',
            'INVENTORY_CHECK',
            'RESTOCK_SHELF',
            'PACK_BOX',
            'LABEL_ITEM'
        ]
        
        # Publisher for task status
        self.publisher = self.create_publisher(
            String,
            'task_status',
            10
        )
        
        # Timer for task processing
        self.timer = self.create_timer(
            1.0 / self.rate,
            self.process_task
        )
        
        self.iteration = 0
        
        self.get_logger().info(f'📋 Task Processor Node Started')
        self.get_logger().info(f'   Robot ID: {self.robot_id}')
        self.get_logger().info(f'   Robot Type: {self.robot_type}')
        self.get_logger().info(f'   Zone: {self.zone_id}')
        self.get_logger().info(f'   Max Tasks/Hour: {self.max_tasks}')
        self.get_logger().info(f'   Processing Rate: {self.rate} Hz')
        
    def process_task(self):
        """Process tasks from the queue"""
        
        self.iteration += 1
        
        # Check if there are tasks to process
        if self.tasks_pending > 0 and not self.processing:
            # Start processing a new task
            self.current_task = {
                'task_id': f'TASK-{self.tasks_completed + 1:04d}',
                'type': random.choice(self.task_types),
                'priority': random.randint(1, 10),
                'items': random.randint(1, 5),
                'weight_kg': round(random.uniform(1.0, 25.0), 2)
            }
            self.processing = True
            self.queue_status = 'processing'
            
            self.get_logger().info(
                f'🔄 Processing {self.current_task["type"]}: '
                f'{self.current_task["task_id"]} '
                f'(Priority: {self.current_task["priority"]}, '
                f'Items: {self.current_task["items"]})'
            )
        
        elif self.processing:
            # Simulate task completion
            success_rate = 0.95  # 95% success rate
            
            if random.random() < success_rate:
                # Task completed successfully
                self.tasks_completed += 1
                self.tasks_pending -= 1
                self.processing = False
                self.queue_status = 'ready'
                
                self.get_logger().info(
                    f'✅ Task {self.current_task["task_id"]} completed! '
                    f'Total: {self.tasks_completed} '
                    f'(Pending: {self.tasks_pending})'
                )
                self.current_task = None
            else:
                # Task failed
                self.tasks_failed += 1
                self.tasks_pending -= 1
                self.processing = False
                self.queue_status = 'error'
                
                self.get_logger().warn(
                    f'❌ Task {self.current_task["task_id"]} FAILED! '
                    f'(Total failures: {self.tasks_failed})'
                )
                self.current_task = None
        
        else:
            # No tasks pending
            if self.iteration % int(self.rate * 10) == 0:  # Every 10 seconds
                self.get_logger().info('⏸️  No tasks pending. Waiting for new tasks...')
            
            # Randomly add new tasks
            if random.random() < 0.3:  # 30% chance of new task
                new_tasks = random.randint(1, 3)
                self.tasks_pending += new_tasks
                self.get_logger().info(f'📥 New tasks received: +{new_tasks} (Total pending: {self.tasks_pending})')
        
        # Create task status message
        task_data = {
            'robot_id': self.robot_id,
            'robot_type': self.robot_type,
            'zone_id': self.zone_id,
            'timestamp': self.get_clock().now().to_msg().sec,
            'queue': {
                'tasks_completed': self.tasks_completed,
                'tasks_pending': self.tasks_pending,
                'tasks_failed': self.tasks_failed,
                'status': self.queue_status,
                'processing': self.processing
            },
            'current_task': self.current_task,
            'performance': {
                'success_rate': round(
                    (self.tasks_completed / max(1, self.tasks_completed + self.tasks_failed)) * 100, 1
                ),
                'tasks_per_hour': round(self.tasks_completed * (3600 / max(1, self.iteration / self.rate)), 1)
            }
        }
        
        # Publish
        msg = String()
        msg.data = json.dumps(task_data)
        self.publisher.publish(msg)
        
        # Performance summary every 30 seconds
        if self.iteration % int(self.rate * 30) == 0 and self.tasks_completed > 0:
            success_rate = (self.tasks_completed / (self.tasks_completed + self.tasks_failed)) * 100
            self.get_logger().info(
                f'📊 Performance: {self.tasks_completed} completed | '
                f'{self.tasks_failed} failed | '
                f'{success_rate:.1f}% success rate | '
                f'{self.tasks_pending} pending'
            )
        
        # Simulate failure for testing event handlers
        if self.simulate_failure and self.iteration > 25:
            self.get_logger().error('💥 Task processor crashed (non-critical)')
            raise RuntimeError('Simulated task processor failure - non-critical')


def main(args=None):
    rclpy.init(args=args)
    node = TaskProcessorNode()
    
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Task processor exception: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
