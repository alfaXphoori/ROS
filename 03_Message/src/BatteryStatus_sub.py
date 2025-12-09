#!/usr/bin/env python3
"""
Battery Status Subscriber
Monitors battery data and generates alerts
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import BatteryStatus
from collections import deque


class BatterySubscriber(Node):
    def __init__(self):
        super().__init__('battery_subscriber')
        self.subscription = self.create_subscription(
            BatteryStatus,
            'battery_status',
            self.battery_callback,
            10
        )
        
        self.voltage_history = deque(maxlen=10)
        self.message_count = 0
        
        self.get_logger().info('Battery Subscriber started! Monitoring battery health...')

    def battery_callback(self, msg):
        """Process battery status and generate alerts"""
        self.message_count += 1
        self.voltage_history.append(msg.voltage)
        
        # Alert for critical battery
        if msg.health_status == "Critical":
            self.get_logger().error(
                f'⚠️  CRITICAL: {msg.battery_id} at {msg.charge_percentage}%! '
                f'Immediate action required!'
            )
        elif msg.health_status == "Poor":
            self.get_logger().warn(
                f'⚡ LOW BATTERY: {msg.battery_id} at {msg.charge_percentage}%'
            )
        
        # Alert for voltage anomalies
        if len(self.voltage_history) >= 5:
            avg_voltage = sum(self.voltage_history) / len(self.voltage_history)
            if abs(msg.voltage - avg_voltage) > 1.5:
                self.get_logger().warn(
                    f'⚠️  VOLTAGE SPIKE: {msg.voltage:.1f}V '
                    f'(avg: {avg_voltage:.1f}V)'
                )
        
        # Normal status logging
        charge_icon = "🔌" if msg.is_charging else "🔋"
        self.get_logger().info(
            f'[{self.message_count}] {charge_icon} {msg.battery_id} | '
            f'{msg.voltage:.1f}V | {msg.charge_percentage}% | '
            f'{msg.health_status} | Est: {msg.time_remaining}min'
        )


def main(args=None):
    rclpy.init(args=args)
    node = BatterySubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
