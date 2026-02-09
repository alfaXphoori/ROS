#!/usr/bin/env python3
"""
Exercise 3: Battery Monitor Node
Real-world robot battery monitoring system with voltage, current, temperature tracking
Critical for preventing unexpected shutdowns and optimizing charging cycles
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random


class BatteryMonitorNode(Node):
    """
    Production-grade battery monitoring system for warehouse robots
    Monitors: voltage, current, temperature, state of charge (SoC), health
    """
    
    def __init__(self):
        super().__init__('battery_monitor')
        
        # Declare parameters
        self.declare_parameter('robot_id', 'AMR-BATTERY-001')
        self.declare_parameter('robot_type', 'transport')
        self.declare_parameter('zone_id', 'WAREHOUSE-MAIN-FLOOR')
        self.declare_parameter('battery_capacity_ah', 100.0)  # Amp-hours
        self.declare_parameter('battery_voltage_nominal', 48.0)  # Volts
        self.declare_parameter('low_battery_threshold', 20.0)  # Percent
        self.declare_parameter('critical_battery_threshold', 10.0)  # Percent
        self.declare_parameter('monitor_rate_hz', 2.0)
        self.declare_parameter('simulate_failure', False)  # For testing event handlers
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.zone_id = self.get_parameter('zone_id').value
        self.capacity = self.get_parameter('battery_capacity_ah').value
        self.nominal_voltage = self.get_parameter('battery_voltage_nominal').value
        self.low_threshold = self.get_parameter('low_battery_threshold').value
        self.critical_threshold = self.get_parameter('critical_battery_threshold').value
        self.rate = self.get_parameter('monitor_rate_hz').value
        self.simulate_failure = self.get_parameter('simulate_failure').value
        
        # Battery state
        self.soc = 85.0  # State of charge (%)
        self.voltage = self.nominal_voltage
        self.current = -15.0  # Negative = discharging, Positive = charging
        self.temperature = 25.0  # Celsius
        self.health = 98.0  # Battery health (%)
        self.cycles = 342  # Charge cycles completed
        self.time_to_empty = 340  # Minutes remaining
        self.status = 'normal'
        
        # Publisher for battery status
        self.publisher = self.create_publisher(
            String,
            'battery_status',
            10
        )
        
        # Timer for monitoring
        self.timer = self.create_timer(
            1.0 / self.rate,
            self.publish_battery_status
        )
        
        self.iteration = 0
        
        self.get_logger().info(f'🔋 Battery Monitor Node Started')
        self.get_logger().info(f'   Robot ID: {self.robot_id}')
        self.get_logger().info(f'   Battery: {self.capacity}Ah @ {self.nominal_voltage}V')
        self.get_logger().info(f'   Monitoring at {self.rate} Hz')
        
    def publish_battery_status(self):
        """Publish real-time battery status with realistic variations"""
        
        self.iteration += 1
        
        # Simulate battery discharge (realistic warehouse robot)
        self.soc -= 0.01  # Gradual discharge
        self.current = -15.0 + random.uniform(-2.0, 2.0)  # Discharge current variation
        self.voltage = self.nominal_voltage * (self.soc / 100.0)
        self.temperature = 25.0 + random.uniform(-1.0, 3.0)  # Temperature variation
        self.time_to_empty = int((self.soc / 100.0) * 340)
        
        # Determine status based on SoC
        if self.soc <= self.critical_threshold:
            self.status = 'critical'
            self.get_logger().warn(f'🚨 CRITICAL BATTERY: {self.soc:.1f}%')
        elif self.soc <= self.low_threshold:
            self.status = 'low'
            self.get_logger().warn(f'⚠️  LOW BATTERY: {self.soc:.1f}%')
        else:
            self.status = 'normal'
        
        # Create battery status message
        battery_data = {
            'robot_id': self.robot_id,
            'robot_type': self.robot_type,
            'zone_id': self.zone_id,
            'timestamp': self.get_clock().now().to_msg().sec,
            'battery': {
                'state_of_charge': round(self.soc, 2),
                'voltage': round(self.voltage, 2),
                'current': round(self.current, 2),
                'temperature': round(self.temperature, 2),
                'health': self.health,
                'cycles': self.cycles,
                'time_to_empty_min': self.time_to_empty,
                'capacity_ah': self.capacity,
                'status': self.status
            },
            'alerts': []
        }
        
        # Add alerts
        if self.temperature > 45.0:
            battery_data['alerts'].append('TEMPERATURE_HIGH')
        if self.soc <= self.critical_threshold:
            battery_data['alerts'].append('BATTERY_CRITICAL')
        elif self.soc <= self.low_threshold:
            battery_data['alerts'].append('BATTERY_LOW')
        if self.health < 80.0:
            battery_data['alerts'].append('BATTERY_DEGRADED')
        
        # Publish
        msg = String()
        msg.data = json.dumps(battery_data)
        self.publisher.publish(msg)
        
        # Log status periodically
        if self.iteration % int(self.rate * 10) == 0:  # Every 10 seconds
            self.get_logger().info(
                f'🔋 Battery: {self.soc:.1f}% | {self.voltage:.1f}V | '
                f'{abs(self.current):.1f}A | {self.temperature:.1f}°C | '
                f'{self.time_to_empty}min remaining'
            )
        
        # Simulate failure for testing event handlers
        if self.simulate_failure and self.iteration > 20:
            self.get_logger().error('💥 SIMULATED BATTERY MONITOR CRASH!')
            raise RuntimeError('Simulated battery monitor failure for testing')
        
        # Simulate critical shutdown
        if self.soc <= 5.0:
            self.get_logger().fatal('⚠️  CRITICAL: Battery depleted! Robot shutting down...')
            raise RuntimeError('Battery critical - emergency shutdown')


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Battery monitor exception: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
