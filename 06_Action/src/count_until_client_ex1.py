#!/usr/bin/env python3
"""
Exercise 1: Basic Count Until Client
Sends goal and receives feedback and result
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ce_robot_interfaces.action import CountUntil


class CountUntilActionClient(Node):
    def __init__(self):
        super().__init__('count_until_client_ex1')
        self._action_client = ActionClient(
            self,
            CountUntil,
            'count_until_ex1'
        )
        self.get_logger().info('Count Until Action Client (Exercise 1) initialized')

    def send_goal(self, target, period):
        """Send goal to server"""
        self.get_logger().info(f'[EX1] Waiting for server...')
        self._action_client.wait_for_server()
        
        goal_msg = CountUntil.Goal()
        goal_msg.target = target
        goal_msg.period = period
        
        self.get_logger().info(f'[EX1] Sending goal: target={target}, period={period}s')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('[EX1] Goal rejected!')
            return
        
        self.get_logger().info('[EX1] Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Receive feedback"""
        count = feedback_msg.feedback.current_count
        self.get_logger().info(f'[EX1] Feedback: count={count}')

    def result_callback(self, future):
        """Receive result"""
        result = future.result().result
        self.get_logger().info(f'[EX1] Result: total_count={result.total_count}')


def main(args=None):
    rclpy.init(args=args)
    client = CountUntilActionClient()
    client.send_goal(target=5, period=1)
    rclpy.spin(client)


if __name__ == '__main__':
    main()
