#!/usr/bin/env python3
"""
Action Server: Count Until
Counts from 1 to target with feedback
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ce_robot_interfaces.action import CountUntil
import time


class CountUntilActionServer(Node):
    def __init__(self):
        super().__init__('count_until_server')
        
        self._action_server = ActionServer(
            self,
            CountUntil,
            'count_until',
            self.execute_callback
        )
        
        self.get_logger().info('Count Until Action Server started')

    def execute_callback(self, goal_handle):
        """Execute the counting task"""
        self.get_logger().info(
            f'Executing goal: count to {goal_handle.request.target} '
            f'with {goal_handle.request.period}s period'
        )
        
        # Initialize feedback
        feedback_msg = CountUntil.Feedback()
        
        # Count from 1 to target
        for count in range(1, goal_handle.request.target + 1):
            # Check if goal was cancelled
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled')
                goal_handle.canceled()
                return CountUntil.Result(total_count=count - 1)
            
            # Update feedback
            feedback_msg.current_count = count
            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().info(f'Counting: {count}/{goal_handle.request.target}')
            
            # Wait for period
            time.sleep(goal_handle.request.period)
        
        # Goal succeeded
        goal_handle.succeed()
        
        result = CountUntil.Result()
        result.total_count = goal_handle.request.target
        
        self.get_logger().info('Goal completed successfully')
        
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = CountUntilActionServer()
    
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
