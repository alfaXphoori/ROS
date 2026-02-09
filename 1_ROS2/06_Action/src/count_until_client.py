#!/usr/bin/env python3
"""
Action Client: Count Until
Sends goal to server and monitors progress
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ce_robot_interfaces.action import CountUntil


class CountUntilActionClient(Node):
    def __init__(self):
        super().__init__('count_until_client')
        
        self._action_client = ActionClient(
            self,
            CountUntil,
            'count_until'
        )
        
        self.get_logger().info('Count Until Action Client initialized')

    def send_goal(self, target, period):
        """Send goal to action server"""
        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()
        
        goal_msg = CountUntil.Goal()
        goal_msg.target = target
        goal_msg.period = period
        
        self.get_logger().info(f'Sending goal: target={target}, period={period}s')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle server's response to goal"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return
        
        self.get_logger().info('Goal accepted by server')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Receive feedback during execution"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: current_count = {feedback.current_count}')

    def get_result_callback(self, future):
        """Handle final result"""
        result = future.result().result
        self.get_logger().info(f'Result: total_count = {result.total_count}')


def main(args=None):
    rclpy.init(args=args)
    action_client = CountUntilActionClient()
    
    # Send goal with target=5 and period=1 second
    action_client.send_goal(target=5, period=1)
    
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
