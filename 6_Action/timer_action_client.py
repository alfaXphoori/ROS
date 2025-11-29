#!/usr/bin/env python3
"""
Exercise 2: Timer Action Client
Submits timer goals and monitors status
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ce_robot_interfaces.action import TimerControl


class TimerClient(Node):
    def __init__(self):
        super().__init__('timer_action_client')
        self._action_client = ActionClient(
            self,
            TimerControl,
            'timer_control'
        )

    def send_goal(self, duration, auto_start=True):
        """Send a timer goal"""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return
        
        goal_msg = TimerControl.Goal()
        goal_msg.duration = duration
        goal_msg.auto_start = auto_start
        
        self.get_logger().info(
            f'Starting timer: {duration}s (auto_start={auto_start})'
        )
        
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
            self.get_logger().error('Goal rejected')
            return
        
        self.get_logger().info('Timer goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            self.get_result_callback
        )

    def feedback_callback(self, feedback_msg):
        """Handle feedback"""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Timer: {fb.elapsed_time:.1f}s elapsed, '
            f'{fb.remaining_time:.1f}s remaining '
            f'[{fb.state}]'
        )

    def get_result_callback(self, future):
        """Handle result"""
        result = future.result().result
        self.get_logger().info(
            f'Timer result: total_elapsed={result.total_elapsed:.1f}s, '
            f'completed={result.completed}, reason={result.reason}'
        )


def main(args=None):
    rclpy.init(args=args)
    client = TimerClient()
    
    # Send 5-second timer
    client.send_goal(5.0)
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
