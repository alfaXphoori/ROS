#!/usr/bin/env python3
"""
Exercise 1: Battery Charging Client
Sends charging goal and monitors progress
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ce_robot_interfaces.action import BatteryCharging


class BatteryChargingActionClient(Node):
    def __init__(self):
        super().__init__('battery_charging_client')
        self._action_client = ActionClient(
            self,
            BatteryCharging,
            'battery_charging_ex1'
        )
        self.get_logger().info('🔋 Battery Charging Client initialized')

    def send_goal(self, target_level, charging_rate):
        """Send charging goal"""
        self.get_logger().info('[EX1] 📡 Waiting for charging server...')
        self._action_client.wait_for_server()
        
        goal_msg = BatteryCharging.Goal()
        goal_msg.target_level = target_level
        goal_msg.charging_rate = charging_rate
        
        self.get_logger().info(
            f'[EX1] 🎯 Requesting charge to {target_level}% at {charging_rate}%/s'
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
            self.get_logger().error('[EX1] ❌ Charging goal rejected!')
            return
        
        self.get_logger().info('[EX1] ✅ Charging goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Receive charging feedback"""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[EX1] 📊 {fb.status}: {fb.current_level}% | {fb.temperature:.1f}°C'
        )

    def result_callback(self, future):
        """Receive final result"""
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f'[EX1] ✅ {result.message}'
            )
        else:
            self.get_logger().error(
                f'[EX1] ❌ Charging failed: {result.message}'
            )


def main(args=None):
    rclpy.init(args=args)
    client = BatteryChargingActionClient()
    
    # Request: Charge to 80% at 5%/second
    client.send_goal(target_level=80, charging_rate=5.0)
    rclpy.spin(client)


if __name__ == '__main__':
    main()
