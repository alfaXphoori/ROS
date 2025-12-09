#!/usr/bin/env python3
"""
Battery Status Publisher
Publishes battery monitoring data with realistic simulation
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import BatteryStatus
import random
import time


class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        self.publisher_ = self.create_publisher(BatteryStatus, 'battery_status', 10)
        self.timer = self.create_timer(2.0, self.publish_battery_status)
        
        # Simulate battery discharge
        self.charge_level = 100.0
        self.is_charging = False
        self.cycle_count = 0
        
        self.get_logger().info('Battery Publisher started!')

    def get_health_status(self, charge):
        """Determine health status based on charge level"""
        if charge > 75:
            return "Good"
        elif charge > 50:
            return "Fair"
        elif charge > 20:
            return "Poor"
        else:
            return "Critical"

    def publish_battery_status(self):
        """Publish battery status with realistic simulation"""
        self.cycle_count += 1
        
        msg = BatteryStatus()
        msg.battery_id = "BAT-001"
        
        # Simulate charging/discharging cycle
        if self.charge_level <= 20:
            self.is_charging = True
        elif self.charge_level >= 95:
            self.is_charging = False
        
        # Update charge level
        if self.is_charging:
            self.charge_level = min(100.0, self.charge_level + random.uniform(2.0, 5.0))
        else:
            self.charge_level = max(0.0, self.charge_level - random.uniform(0.5, 2.0))
        
        # Calculate voltage (proportional to charge)
        msg.voltage = 10.0 + (self.charge_level / 100.0) * 4.8  # 10V-14.8V range
        
        # Calculate current
        if self.is_charging:
            msg.current = random.uniform(3.0, 5.0)  # Charging current
        else:
            msg.current = random.uniform(1.0, 3.0)  # Discharge current
        
        msg.charge_percentage = int(self.charge_level)
        msg.health_status = self.get_health_status(self.charge_level)
        msg.time_remaining = int(self.charge_level * 1.5)  # Rough estimate in minutes
        msg.is_charging = self.is_charging
        msg.timestamp = int(time.time())
        
        self.publisher_.publish(msg)
        
        status_icon = "🔌" if self.is_charging else "🔋"
        self.get_logger().info(
            f'{status_icon} {msg.battery_id} | '
            f'{msg.voltage:.1f}V | {msg.current:.1f}A | '
            f'{msg.charge_percentage}% | {msg.health_status} | '
            f'~{msg.time_remaining}min'
        )


def main(args=None):
    rclpy.init(args=args)
    node = BatteryPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
