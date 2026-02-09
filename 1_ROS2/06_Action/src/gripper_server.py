#!/usr/bin/env python3
"""
Exercise 3: Monitored Action Server with Cancellation
Supports goal cancellation and state tracking
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ce_robot_interfaces.action import CountUntil
import time


class MonitoredActionServer(Node):
    def __init__(self):
        super().__init__('monitored_action_server_ex3')
        
        self._action_server = ActionServer(
            self,
            CountUntil,
            'monitored_action_ex3',
            self.execute_callback
        )
        
        self.goal_counter = 0
        self.get_logger().info('Monitored Action Server (Exercise 3) started')

    def execute_callback(self, goal_handle):
        """Execute with cancellation support"""
        self.goal_counter += 1
        goal_id = self.goal_counter
        
        target = goal_handle.request.target
        period = goal_handle.request.period
        
        self.get_logger().info(
            f'[EX3:#{goal_id}] START: count to {target} with {period}s period'
        )
        
        feedback_msg = CountUntil.Feedback()
        cancelled = False
        
        try:
            # Count from 1 to target
            for count in range(1, target + 1):
                # Check for cancellation request
                if goal_handle.is_cancel_requested:
                    self.get_logger().info(f'[EX3:#{goal_id}] CANCELLED at count {count}')
                    goal_handle.canceled()
                    cancelled = True
                    break
                
                # Publish feedback
                feedback_msg.current_count = count
                goal_handle.publish_feedback(feedback_msg)
                
                self.get_logger().info(f'[EX3:#{goal_id}] Count: {count}/{target}')
                time.sleep(period)
            
            if not cancelled:
                # Normal completion
                goal_handle.succeed()
                result = CountUntil.Result()
                result.total_count = target
                self.get_logger().info(f'[EX3:#{goal_id}] SUCCEEDED: completed {target} counts')
                return result
            else:
                # Return partial result for cancelled goal
                result = CountUntil.Result()
                result.total_count = feedback_msg.current_count
                return result
                
        except Exception as e:
            self.get_logger().error(f'[EX3:#{goal_id}] ABORTED: {str(e)}')
            goal_handle.abort()
            return CountUntil.Result(total_count=-1)


def main(args=None):
    rclpy.init(args=args)
    server = MonitoredActionServer()
    rclpy.spin(server)


if __name__ == '__main__':
    main()
