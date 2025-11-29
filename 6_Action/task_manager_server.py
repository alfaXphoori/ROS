#!/usr/bin/env python3
"""
Exercise 3: Task Manager Action Server
Manages task queue with priorities and tracking
"""

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from ce_robot_interfaces.action import TaskManagement
import time
from collections import deque


class TaskManagerServer(Node):
    def __init__(self):
        super().__init__('task_manager_server')
        self._action_server = ActionServer(
            self,
            TaskManagement,
            'task_management',
            self.execute_callback
        )
        self.task_queue = deque()
        self.tasks_completed = 0
        self.get_logger().info('Task Manager Server initialized')

    def execute_callback(self, goal_handle):
        """Execute task management goal"""
        task_name = goal_handle.request.task_name
        priority = goal_handle.request.priority
        duration = goal_handle.request.estimated_duration
        
        self.get_logger().info(
            f'New task: {task_name} (priority={priority}, '
            f'duration={duration}s)'
        )
        
        # Validate task
        if not task_name or duration <= 0:
            self.get_logger().warn('Invalid task parameters')
            goal_handle.abort()
            result_msg = TaskManagement.Result()
            result_msg.success = False
            result_msg.actual_duration = 0
            result_msg.completion_message = 'Invalid parameters'
            return result_msg
        
        # Add to queue
        self.task_queue.append((priority, task_name))
        queue_pos = len(self.task_queue)
        
        feedback_msg = TaskManagement.Feedback()
        result_msg = TaskManagement.Result()
        
        # Simulate task execution
        start_time = time.time()
        steps = int(duration * 10)  # 10 steps per second
        
        for step in range(steps):
            # Check cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().info(f'Task {task_name} cancelled')
                goal_handle.canceled()
                result_msg.success = False
                result_msg.actual_duration = time.time() - start_time
                result_msg.completion_message = 'Cancelled'
                return result_msg
            
            # Calculate progress
            progress = int((step / steps) * 100)
            remaining_queue = max(0, len(self.task_queue) - 1)
            
            feedback_msg.progress_percentage = progress
            feedback_msg.queue_position = queue_pos
            feedback_msg.current_status = f'Processing: {task_name}'
            feedback_msg.tasks_completed = self.tasks_completed
            
            self.get_logger().info(
                f'Task {task_name}: {progress}% '
                f'(Queue: {remaining_queue} tasks remaining)'
            )
            
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)
        
        # Task completed
        goal_handle.succeed()
        self.tasks_completed += 1
        actual_duration = time.time() - start_time
        
        result_msg.success = True
        result_msg.actual_duration = actual_duration
        result_msg.completion_message = (
            f'Task {task_name} completed in {actual_duration:.2f}s'
        )
        
        self.get_logger().info(
            f'Task completed: {task_name} '
            f'(actual: {actual_duration:.2f}s)'
        )
        
        # Remove from queue
        if (priority, task_name) in self.task_queue:
            self.task_queue.remove((priority, task_name))
        
        return result_msg


def main(args=None):
    rclpy.init(args=args)
    server = TaskManagerServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
