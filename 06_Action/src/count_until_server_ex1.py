#!/usr/bin/env python3
"""
Exercise 1: Basic Count Until Server
Counts from 1 to target with simple feedback
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ce_robot_interfaces.action import CountUntil
import time


class CountUntilActionServer(Node):
    def __init__(self):
        super().__init__('count_until_server_ex1')
        
        self._action_server = ActionServer(
            self,
            CountUntil,
            'count_until_ex1',
            self.execute_callback
        )
        
        self.get_logger().info('Count Until Action Server (Exercise 1) started')

    def execute_callback(self, goal_handle):
        """Execute the counting task"""
        target = goal_handle.request.target
        period = goal_handle.request.period
        
        self.get_logger().info(
            f'[EX1] Executing: count to {target} with {period}s period'
        )
        
        feedback_msg = CountUntil.Feedback()
        
        # Count from 1 to target
        for count in range(1, target + 1):
            # Publish feedback
            feedback_msg.current_count = count
            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().info(f'[EX1] Count: {count}/{target}')
            time.sleep(period)
        
        # Succeed and return result
        goal_handle.succeed()
        result = CountUntil.Result()
        result.total_count = target
        
        self.get_logger().info(f'[EX1] Goal succeeded! Total: {target}')
        return result


def main(args=None):
    rclpy.init(args=args)
    server = CountUntilActionServer()
    rclpy.spin(server)


if __name__ == '__main__':
    main()
