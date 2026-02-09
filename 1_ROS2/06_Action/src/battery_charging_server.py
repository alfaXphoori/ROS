#!/usr/bin/env python3
"""
Exercise 1: Battery Charging Server
Simulates robot autonomous charging with safety monitoring
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ce_robot_interfaces.action import BatteryCharging
import time
import random


class BatteryChargingActionServer(Node):
    def __init__(self):
        super().__init__('battery_charging_server')
        
        self._action_server = ActionServer(
            self,
            BatteryCharging,
            'battery_charging_ex1',
            self.execute_callback
        )
        
        # Simulate initial battery state
        self.current_battery = 20  # Start at 20%
        
        self.get_logger().info('🔋 Battery Charging Server started')
        self.get_logger().info(f'Initial battery level: {self.current_battery}%')

    def validate_goal(self, goal):
        """Validate charging parameters"""
        errors = []
        
        if goal.target_level < 0 or goal.target_level > 100:
            errors.append('Target level must be 0-100%')
        
        if goal.target_level <= self.current_battery:
            errors.append(f'Target ({goal.target_level}%) must be > current ({self.current_battery}%)')
        
        if goal.charging_rate <= 0 or goal.charging_rate > 10:
            errors.append('Charging rate must be 0.1-10 %/second')
        
        return errors

    def execute_callback(self, goal_handle):
        """Execute battery charging with safety monitoring"""
        goal = goal_handle.request
        start_time = time.time()
        
        self.get_logger().info(
            f'[EX1] 🔌 Starting charge: {self.current_battery}% → {goal.target_level}% at {goal.charging_rate}%/s'
        )
        
        # Validate goal
        errors = self.validate_goal(goal)
        if errors:
            self.get_logger().error('[EX1] ❌ Charging failed - validation errors:')
            for error in errors:
                self.get_logger().error(f'  - {error}')
            goal_handle.abort()
            return BatteryCharging.Result(
                final_level=self.current_battery,
                charging_time=0.0,
                success=False,
                message='; '.join(errors)
            )
        
        feedback_msg = BatteryCharging.Feedback()
        
        try:
            # Phase 1: Initialize charging
            feedback_msg.status = "Initializing"
            feedback_msg.current_level = self.current_battery
            feedback_msg.temperature = 25.0
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info('[EX1] 🔧 Initializing charging connection...')
            time.sleep(1)
            
            # Phase 2: Charging loop
            feedback_msg.status = "Charging"
            while self.current_battery < goal.target_level:
                # Simulate charging increment
                self.current_battery = min(
                    self.current_battery + goal.charging_rate,
                    goal.target_level
                )
                
                # Simulate battery temperature (increases during charging)
                feedback_msg.temperature = 25.0 + (self.current_battery / 100.0) * 15.0 + random.uniform(-2, 2)
                
                # Check for overheating
                if feedback_msg.temperature > 45.0:
                    self.get_logger().error(f'[EX1] 🔥 Overheating detected: {feedback_msg.temperature:.1f}°C')
                    goal_handle.abort()
                    return BatteryCharging.Result(
                        final_level=self.current_battery,
                        charging_time=time.time() - start_time,
                        success=False,
                        message=f'Overheating at {feedback_msg.temperature:.1f}°C'
                    )
                
                # Publish feedback
                feedback_msg.current_level = int(self.current_battery)
                goal_handle.publish_feedback(feedback_msg)
                
                self.get_logger().info(
                    f'[EX1] ⚡ Charging: {feedback_msg.current_level}% | Temp: {feedback_msg.temperature:.1f}°C'
                )
                time.sleep(1)
            
            # Phase 3: Balancing (final 5% takes longer)
            if goal.target_level >= 95:
                feedback_msg.status = "Balancing"
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info('[EX1] ⚖️  Balancing cells...')
                time.sleep(2)
            
            # Complete
            charging_time = time.time() - start_time
            feedback_msg.status = "Complete"
            goal_handle.publish_feedback(feedback_msg)
            
            goal_handle.succeed()
            result = BatteryCharging.Result()
            result.final_level = int(self.current_battery)
            result.charging_time = charging_time
            result.success = True
            result.message = f'Charged to {result.final_level}% in {charging_time:.1f}s'
            
            self.get_logger().info(f'[EX1] ✅ Charging complete: {result.message}')
            return result
            
        except Exception as e:
            self.get_logger().error(f'[EX1] ❌ Charging error: {str(e)}')
            goal_handle.abort()
            return BatteryCharging.Result(
                final_level=self.current_battery,
                charging_time=time.time() - start_time,
                success=False,
                message=f'Error: {str(e)}'
            )


def main(args=None):
    rclpy.init(args=args)
    server = BatteryChargingActionServer()
    rclpy.spin(server)


if __name__ == '__main__':
    main()
