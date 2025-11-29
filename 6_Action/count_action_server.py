#!/usr/bin/env python3
"""
Exercise 1: Basic Counter Action Server
Counts from 1 to target with feedback
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ce_robot_interfaces.action import CountUntil
import time


class CountUntilServer(Node):
    def __init__(self):
        super().__init__('count_action_server')
        self._action_server = ActionServer(
            self,
            CountUntil,
            'count_until',
            self.execute_callback
        )
        self.get_logger().info('Count Until Action Server initialized')

    def execute_callback(self, goal_handle):
        """Execute counting goal"""
        self.get_logger().info(f'Received goal: target={goal_handle.request.target}')
        
        # Validate goal
        if goal_handle.request.target <= 0:
            self.get_logger().warn('Goal target must be positive')
            goal_handle.abort()
            result_msg = CountUntil.Result()
            result_msg.total_count = 0
            result_msg.success = False
            return result_msg
        
        # Initialize feedback and result
        feedback_msg = CountUntil.Feedback()
        result_msg = CountUntil.Result()
        
        # Count from 1 to target
        for count in range(1, goal_handle.request.target + 1):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().info(f'Goal cancelled at count={count}')
                goal_handle.canceled()
                result_msg.total_count = count - 1
                result_msg.success = False
                return result_msg
            
            # Send feedback
            feedback_msg.current = count
            self.get_logger().info(f'Count: {count}/{goal_handle.request.target}')
            goal_handle.publish_feedback(feedback_msg)
            
            # Simulate work with delay
            time.sleep(0.5)
        
        # Goal succeeded
        goal_handle.succeed()
        result_msg.total_count = goal_handle.request.target
        result_msg.success = True
        
        self.get_logger().info(
            f'Goal succeeded! Counted to {result_msg.total_count}'
        )
        return result_msg


def main(args=None):
    rclpy.init(args=args)
    server = CountUntilServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
