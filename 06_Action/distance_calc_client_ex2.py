#!/usr/bin/env python3
"""
Exercise 2: Distance Calculator Client
Tests validation and error handling
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ce_robot_interfaces.action import DistanceCalc


class DistanceCalcActionClient(Node):
    def __init__(self):
        super().__init__('distance_calc_client_ex2')
        self._action_client = ActionClient(
            self,
            DistanceCalc,
            'distance_calc_ex2'
        )
        self.get_logger().info('Distance Calculator Client (Exercise 2) initialized')

    def send_goal(self, x1, y1, x2, y2):
        """Send goal with validation test"""
        self.get_logger().info(f'[EX2] Waiting for server...')
        self._action_client.wait_for_server()
        
        goal_msg = DistanceCalc.Goal()
        goal_msg.x1 = x1
        goal_msg.y1 = y1
        goal_msg.x2 = x2
        goal_msg.y2 = y2
        
        self.get_logger().info(
            f'[EX2] Sending goal: ({x1}, {y1}) to ({x2}, {y2})'
        )
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('[EX2] Goal rejected by server!')
            return
        
        self.get_logger().info('[EX2] Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Receive feedback"""
        fb = feedback_msg.feedback
        self.get_logger().info(f'[EX2] {fb.status} ({fb.progress_percent:.0f}%)')

    def result_callback(self, future):
        """Receive result or error"""
        result = future.result().result
        if result.distance < 0:
            self.get_logger().error('[EX2] Goal failed or aborted')
        else:
            self.get_logger().info(f'[EX2] Distance: {result.distance:.2f} units')


def main(args=None):
    rclpy.init(args=args)
    client = DistanceCalcActionClient()
    
    # Test 1: Valid goal
    client.send_goal(0.0, 0.0, 3.0, 4.0)
    rclpy.spin(client)


if __name__ == '__main__':
    main()
