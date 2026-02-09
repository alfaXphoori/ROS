#!/usr/bin/env python3
"""
Exercise 2: Distance Calculator Server with Validation
Calculates distance with input validation
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ce_robot_interfaces.action import DistanceCalc
import math
import time


class DistanceCalcActionServer(Node):
    def __init__(self):
        super().__init__('distance_calc_server_ex2')
        
        self._action_server = ActionServer(
            self,
            DistanceCalc,
            'distance_calc_ex2',
            self.execute_callback
        )
        
        self.get_logger().info('Distance Calculator Server (Exercise 2) started')

    def validate_goal(self, goal):
        """Validate goal parameters"""
        errors = []
        
        # Check for valid numbers (not NaN or infinity)
        for val, name in [
            (goal.x1, 'x1'), (goal.y1, 'y1'),
            (goal.x2, 'x2'), (goal.y2, 'y2')
        ]:
            if math.isnan(val) or math.isinf(val):
                errors.append(f'{name} is invalid (NaN or infinity)')
        
        # Check if points are too far (> 1000)
        if abs(goal.x1) > 1000 or abs(goal.y1) > 1000:
            errors.append('Point 1 coordinates too large (|x| or |y| > 1000)')
        if abs(goal.x2) > 1000 or abs(goal.y2) > 1000:
            errors.append('Point 2 coordinates too large (|x| or |y| > 1000)')
        
        return errors

    def execute_callback(self, goal_handle):
        """Execute distance calculation with validation"""
        goal = goal_handle.request
        
        self.get_logger().info(
            f'[EX2] Calculating distance from ({goal.x1}, {goal.y1}) to ({goal.x2}, {goal.y2})'
        )
        
        # Validate goal
        errors = self.validate_goal(goal)
        if errors:
            self.get_logger().error('[EX2] Goal validation failed:')
            for error in errors:
                self.get_logger().error(f'  - {error}')
            goal_handle.abort()
            return DistanceCalc.Result(distance=-1.0)
        
        feedback_msg = DistanceCalc.Feedback()
        
        try:
            # Simulate calculation steps
            for step in range(1, 4):
                feedback_msg.status = f'Calculating... (step {step}/3)'
                feedback_msg.progress_percent = (step / 3) * 100
                goal_handle.publish_feedback(feedback_msg)
                
                self.get_logger().info(f'[EX2] {feedback_msg.status} ({feedback_msg.progress_percent:.0f}%)')
                time.sleep(0.5)
            
            # Calculate distance
            dx = goal.x2 - goal.x1
            dy = goal.y2 - goal.y1
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Final feedback
            feedback_msg.status = 'Complete'
            feedback_msg.progress_percent = 100.0
            goal_handle.publish_feedback(feedback_msg)
            
            # Return result
            goal_handle.succeed()
            result = DistanceCalc.Result()
            result.distance = distance
            
            self.get_logger().info(f'[EX2] Goal succeeded! Distance: {distance:.2f}')
            return result
            
        except Exception as e:
            self.get_logger().error(f'[EX2] Execution failed: {str(e)}')
            goal_handle.abort()
            return DistanceCalc.Result(distance=-1.0)


def main(args=None):
    rclpy.init(args=args)
    server = DistanceCalcActionServer()
    rclpy.spin(server)


if __name__ == '__main__':
    main()
