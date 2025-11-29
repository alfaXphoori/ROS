#!/usr/bin/env python3
"""
Exercise 2: Timer Action Server
Manages countdown timer with status feedback
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ce_robot_interfaces.action import TimerControl
import time
from threading import Event


class TimerServer(Node):
    def __init__(self):
        super().__init__('timer_action_server')
        self._action_server = ActionServer(
            self,
            TimerControl,
            'timer_control',
            self.execute_callback
        )
        self.get_logger().info('Timer Action Server initialized')

    def execute_callback(self, goal_handle):
        """Execute timer goal"""
        duration = goal_handle.request.duration
        auto_start = goal_handle.request.auto_start
        
        self.get_logger().info(
            f'Timer goal: duration={duration}s, auto_start={auto_start}'
        )
        
        # Validate duration
        if duration <= 0:
            self.get_logger().warn('Duration must be positive')
            goal_handle.abort()
            result_msg = TimerControl.Result()
            result_msg.total_elapsed = 0
            result_msg.completed = False
            result_msg.reason = 'Invalid duration'
            return result_msg
        
        feedback_msg = TimerControl.Feedback()
        result_msg = TimerControl.Result()
        
        start_time = time.time()
        elapsed = 0
        
        # Timer loop
        while elapsed < duration:
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Timer cancelled')
                goal_handle.canceled()
                result_msg.total_elapsed = elapsed
                result_msg.completed = False
                result_msg.reason = 'Cancelled'
                return result_msg
            
            # Send feedback
            elapsed = time.time() - start_time
            remaining = max(0, duration - elapsed)
            
            feedback_msg.elapsed_time = elapsed
            feedback_msg.remaining_time = remaining
            feedback_msg.state = 'RUNNING'
            
            self.get_logger().info(
                f'Timer: {elapsed:.1f}s / {duration}s '
                f'(remaining: {remaining:.1f}s)'
            )
            
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)
        
        # Timer completed
        goal_handle.succeed()
        result_msg.total_elapsed = duration
        result_msg.completed = True
        result_msg.reason = 'Completed'
        
        self.get_logger().info('Timer completed')
        return result_msg


def main(args=None):
    rclpy.init(args=args)
    server = TimerServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
