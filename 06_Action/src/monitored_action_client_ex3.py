#!/usr/bin/env python3
"""
Exercise 3: Monitored Action Client with Cancellation
Tests cancellation handling and goal tracking
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ce_robot_interfaces.action import CountUntil
import threading
import time


class MonitoredActionClient(Node):
    def __init__(self):
        super().__init__('monitored_action_client_ex3')
        self._action_client = ActionClient(
            self,
            CountUntil,
            'monitored_action_ex3'
        )
        self.goal_handle = None
        self.get_logger().info('Monitored Action Client (Exercise 3) initialized')

    def send_goal(self, target, period):
        """Send goal to server"""
        self.get_logger().info(f'[EX3] Waiting for server...')
        self._action_client.wait_for_server()
        
        goal_msg = CountUntil.Goal()
        goal_msg.target = target
        goal_msg.period = period
        
        self.get_logger().info(f'[EX3] Sending goal: target={target}, period={period}s')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error('[EX3] Goal rejected!')
            return
        
        self.get_logger().info('[EX3] Goal accepted!')
        
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)
        
        # Schedule cancellation after 3 seconds (for testing)
        threading.Thread(target=self._cancel_after_delay, daemon=True).start()

    def _cancel_after_delay(self):
        """Cancel goal after delay for testing"""
        time.sleep(3)
        if self.goal_handle:
            self.get_logger().info('[EX3] Requesting cancellation...')
            self._cancel_future = self.goal_handle.cancel_goal_async()
            self._cancel_future.add_done_callback(self.cancel_callback)

    def cancel_callback(self, future):
        """Handle cancellation response"""
        cancel_response = future.result()
        if cancel_response.return_code == 0:
            self.get_logger().info('[EX3] Cancellation accepted')
        else:
            self.get_logger().warn('[EX3] Cancellation rejected')

    def feedback_callback(self, feedback_msg):
        """Receive feedback"""
        count = feedback_msg.feedback.current_count
        self.get_logger().info(f'[EX3] Feedback: count={count}')

    def result_callback(self, future):
        """Receive result or cancellation status"""
        result = future.result()
        
        if result.status == 4:  # CANCELED
            self.get_logger().info(f'[EX3] Goal was CANCELLED')
        elif result.status == 3:  # ABORTED
            self.get_logger().info(f'[EX3] Goal was ABORTED')
        else:  # SUCCEEDED
            total = result.result.total_count
            self.get_logger().info(f'[EX3] Goal SUCCEEDED: total_count={total}')


def main(args=None):
    rclpy.init(args=args)
    client = MonitoredActionClient()
    client.send_goal(target=10, period=1)
    rclpy.spin(client)


if __name__ == '__main__':
    main()
