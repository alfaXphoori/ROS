#!/usr/bin/env python3
"""
Exercise 3: Task Manager Action Client
Submits tasks and monitors execution
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ce_robot_interfaces.action import TaskManagement
import sys


class TaskManagerClient(Node):
    def __init__(self):
        super().__init__('task_manager_client')
        self._action_client = ActionClient(
            self,
            TaskManagement,
            'task_management'
        )
        self.task_count = 0

    def send_task(self, task_name, priority=1, duration=5.0):
        """Submit a task"""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Task manager not available')
            return
        
        goal_msg = TaskManagement.Goal()
        goal_msg.task_name = task_name
        goal_msg.priority = priority
        goal_msg.estimated_duration = duration
        
        self.get_logger().info(
            f'Submitting task: {task_name} '
            f'(priority={priority}, duration={duration}s)'
        )
        
        self.task_count += 1
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        """Handle goal acceptance"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Task rejected')
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            self.get_result_callback
        )

    def feedback_callback(self, feedback_msg):
        """Handle feedback"""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Progress: {fb.progress_percentage}% | '
            f'Queue: {fb.queue_position} | '
            f'Status: {fb.current_status} | '
            f'Completed: {fb.tasks_completed}'
        )

    def get_result_callback(self, future):
        """Handle task completion"""
        result = future.result().result
        self.get_logger().info(
            f'Task result: success={result.success}, '
            f'duration={result.actual_duration:.2f}s, '
            f'message={result.completion_message}'
        )


def main(args=None):
    rclpy.init(args=args)
    client = TaskManagerClient()
    
    # Submit multiple tasks
    client.send_task('data_processing', priority=1, duration=3.0)
    client.send_task('file_backup', priority=2, duration=2.0)
    client.send_task('system_check', priority=3, duration=2.5)
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
