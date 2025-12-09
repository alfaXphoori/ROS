#!/usr/bin/env python3
"""
Multi-Robot Hardware Monitor
Tracks multiple robots and provides per-robot statistics
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus
from collections import defaultdict, deque


class MultiRobotMonitor(Node):
    def __init__(self):
        super().__init__('multi_robot_monitor')
        self.subscription = self.create_subscription(
            HardwareStatus,
            'hardware_status',
            self.status_callback,
            10
        )
        
        # Per-robot data storage (dictionary of robot_name -> data)
        self.robot_data = defaultdict(lambda: {
            'count': 0,
            'temperatures': deque(maxlen=10),
            'motor_ready_count': 0,
            'motor_not_ready_count': 0,
            'last_seen': None
        })
        
        self.total_messages = 0
        
        # Create timer for periodic reports (every 15 seconds)
        self.report_timer = self.create_timer(15.0, self.print_multi_robot_report)
        
        self.get_logger().info('Multi-Robot Monitor started! Tracking all robots...')

    def status_callback(self, msg):
        """Collect data per robot"""
        self.total_messages += 1
        robot_name = msg.name_robot
        
        # Update robot-specific data
        data = self.robot_data[robot_name]
        data['count'] += 1
        data['temperatures'].append(msg.temperature)
        data['last_seen'] = self.get_clock().now()
        
        if msg.motor_ready:
            data['motor_ready_count'] += 1
        else:
            data['motor_not_ready_count'] += 1
        
        # Log individual message
        self.get_logger().info(
            f'[{robot_name}] Temp: {msg.temperature}°C | Motor: {msg.motor_ready} | '
            f'Robot Count: {data["count"]} | Total: {self.total_messages}'
        )

    def print_multi_robot_report(self):
        """Generate comprehensive multi-robot report"""
        if self.total_messages == 0:
            self.get_logger().info('No data received yet...')
            return
        
        num_robots = len(self.robot_data)
        
        report = f'\n'
        report += f'╔═══════════════════════════════════════════════════════════╗\n'
        report += f'║        MULTI-ROBOT MONITORING REPORT                      ║\n'
        report += f'╠═══════════════════════════════════════════════════════════╣\n'
        report += f'║ Total Messages:    {self.total_messages:>6}                              ║\n'
        report += f'║ Active Robots:     {num_robots:>6}                              ║\n'
        report += f'╠═══════════════════════════════════════════════════════════╣\n'
        
        # Per-robot statistics
        for robot_name, data in sorted(self.robot_data.items()):
            count = data['count']
            temps = list(data['temperatures'])
            
            if len(temps) > 0:
                avg_temp = sum(temps) / len(temps)
                min_temp = min(temps)
                max_temp = max(temps)
            else:
                avg_temp = min_temp = max_temp = 0
            
            motor_ready_pct = (data['motor_ready_count'] / count * 100) if count > 0 else 0
            
            report += f'║ Robot: {robot_name:<20}                        ║\n'
            report += f'║   Messages:        {count:>6}                              ║\n'
            report += f'║   Avg Temp:        {avg_temp:>6.1f}°C                          ║\n'
            report += f'║   Min/Max:         {min_temp:>3}°C / {max_temp:>3}°C                        ║\n'
            report += f'║   Motor Ready:     {motor_ready_pct:>6.1f}%                           ║\n'
            report += f'╠═══════════════════════════════════════════════════════════╣\n'
        
        report += f'╚═══════════════════════════════════════════════════════════╝'
        
        self.get_logger().info(report)
        
        # Alert for robots with high average temperature
        for robot_name, data in self.robot_data.items():
            temps = list(data['temperatures'])
            if len(temps) > 0:
                avg_temp = sum(temps) / len(temps)
                if avg_temp > 60:
                    self.get_logger().warn(
                        f'⚠️  HIGH TEMP WARNING: {robot_name} average is {avg_temp:.1f}°C!'
                    )


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_multi_robot_report()  # Final report on exit
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
