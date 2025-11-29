#!/usr/bin/env python3
"""
Exercise 1: Basic Counter Action Client
Submits counting goals and monitors progress
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ce_robot_interfaces.action import CountUntil


class CountUntilClient(Node):
    def __init__(self):
        super().__init__('count_action_client')
        self._action_client = ActionClient(
            self,
            CountUntil,
            'count_until'
        )

    def send_goal(self, target):
        """Send a counting goal"""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return
        
        goal_msg = CountUntil.Goal()
        goal_msg.target = target
        
        self.get_logger().info(f'Sending goal: target={target}')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            self.get_result_callback
        )

    def feedback_callback(self, feedback_msg):
        """Handle periodic feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback received: current count = {feedback.current}'
        )

    def get_result_callback(self, future):
        """Handle final result"""
        result = future.result().result
        self.get_logger().info(
            f'Result: total_count={result.total_count}, '
            f'success={result.success}'
        )


def main(args=None):
    rclpy.init(args=args)
    client = CountUntilClient()
    
    # Send goal to count until 5
    client.send_goal(5)
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
