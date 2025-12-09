#!/usr/bin/env python3
"""
Statistics and Aggregation Node
Comprehensive data analysis and reporting
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
from collections import deque
from datetime import datetime


class StatisticsNode(Node):
    def __init__(self):
        super().__init__('statistics_node')
        self.subscription = self.create_subscription(
            HardwareStatus,
            'hardware_status',
            self.status_callback,
            10
        )
        
        # Data storage
        self.temperatures = deque(maxlen=50)  # Last 50 temperatures
        self.message_times = deque(maxlen=50)  # Last 50 message timestamps
        
        # Counters
        self.total_messages = 0
        self.motor_ready_count = 0
        self.motor_not_ready_count = 0
        
        # Min/Max tracking
        self.min_temp = float('inf')
        self.max_temp = float('-inf')
        
        # Create timer for periodic reports (every 10 seconds)
        self.report_timer = self.create_timer(10.0, self.print_report)
        
        self.get_logger().info('Statistics Node started! Collecting data...')

    def status_callback(self, msg):
        """Collect data from incoming messages"""
        self.total_messages += 1
        current_time = datetime.now()
        
        # Store temperature data
        self.temperatures.append(msg.temperature)
        self.message_times.append(current_time)
        
        # Track min/max
        if msg.temperature < self.min_temp:
            self.min_temp = msg.temperature
        if msg.temperature > self.max_temp:
            self.max_temp = msg.temperature
        
        # Track motor status
        if msg.motor_ready:
            self.motor_ready_count += 1
        else:
            self.motor_not_ready_count += 1
        
        # Log basic info
        self.get_logger().info(
            f'Received #{self.total_messages}: {msg.name_robot} | '
            f'Temp: {msg.temperature}°C | Motor: {msg.motor_ready}'
        )

    def print_report(self):
        """Generate and print comprehensive statistics report"""
        if self.total_messages == 0:
            self.get_logger().info('No data received yet...')
            return
        
        # Calculate statistics
        avg_temp = sum(self.temperatures) / len(self.temperatures)
        motor_ready_pct = (self.motor_ready_count / self.total_messages) * 100
        
        # Calculate message rate
        if len(self.message_times) >= 2:
            time_span = (self.message_times[-1] - self.message_times[0]).total_seconds()
            message_rate = len(self.message_times) / time_span if time_span > 0 else 0
        else:
            message_rate = 0
        
        # Print comprehensive report
        self.get_logger().info(
            f'\n'
            f'╔══════════════════════════════════════════════════════╗\n'
            f'║           STATISTICS REPORT                          ║\n'
            f'╠══════════════════════════════════════════════════════╣\n'
            f'║ Total Messages:     {self.total_messages:>6}                        ║\n'
            f'║ Message Rate:       {message_rate:>6.2f} msg/sec                 ║\n'
            f'║                                                      ║\n'
            f'║ Temperature Statistics:                              ║\n'
            f'║   Average:          {avg_temp:>6.1f}°C                      ║\n'
            f'║   Minimum:          {self.min_temp:>6}°C                      ║\n'
            f'║   Maximum:          {self.max_temp:>6}°C                      ║\n'
            f'║   Range:            {self.max_temp - self.min_temp:>6}°C                      ║\n'
            f'║                                                      ║\n'
            f'║ Motor Status:                                        ║\n'
            f'║   Ready:            {self.motor_ready_count:>6} ({motor_ready_pct:>5.1f}%)            ║\n'
            f'║   Not Ready:        {self.motor_not_ready_count:>6} ({100-motor_ready_pct:>5.1f}%)            ║\n'
            f'╚══════════════════════════════════════════════════════╝'
        )


def main(args=None):
    rclpy.init(args=args)
    node = StatisticsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_report()  # Final report on exit
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
