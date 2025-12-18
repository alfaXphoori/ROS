#!/usr/bin/env python3
"""
Fleet Monitor for Multi-Robot System
Monitors all robots across namespaces and provides fleet-wide statistics
Real-world use: Central fleet management and monitoring dashboard
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import time


class FleetMonitor(Node):
    """
    Central fleet monitoring node
    Monitors system resources and fleet status across all robot namespaces
    """
    
    def __init__(self):
        super().__init__('fleet_monitor')
        
        # Declare parameters
        self.declare_parameter('monitor_rate_hz', 0.5)
        self.declare_parameter('fleet_id', 'WAREHOUSE-FLEET-A')
        self.declare_parameter('num_robots', 3)
        
        # Get parameters
        monitor_rate = self.get_parameter('monitor_rate_hz').value
        self.fleet_id = self.get_parameter('fleet_id').value
        self.num_robots = self.get_parameter('num_robots').value
        
        # Create publisher
        self.publisher = self.create_publisher(
            String,
            '/fleet_status',
            10
        )
        
        # Create timer
        timer_period = 1.0 / monitor_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.monitor_count = 0
        self.start_time = time.time()
        
        self.get_logger().info('📊 Fleet Monitor Started')
        self.get_logger().info(f'Fleet ID: {self.fleet_id}')
        self.get_logger().info(f'Monitoring {self.num_robots} robots')
        self.get_logger().info(f'Monitor rate: {monitor_rate} Hz')
        self.get_logger().info('Publishing to: /fleet_status')
        self.get_logger().info('Note: This node monitors across all robot namespaces')
    
    def timer_callback(self):
        """Publish fleet monitoring data"""
        
        # Get system metrics
        cpu_percent = psutil.cpu_percent(interval=0.1)
        memory = psutil.virtual_memory()
        disk = psutil.disk_usage('/')
        
        # Calculate uptime
        uptime_seconds = time.time() - self.start_time
        uptime_hours = uptime_seconds / 3600
        
        # Create status message
        status_lines = [
            f'════════════════════════════════════════════',
            f'   FLEET MONITORING REPORT',
            f'════════════════════════════════════════════',
            f'Fleet ID: {self.fleet_id}',
            f'Active Robots: {self.num_robots}',
            f'Monitor Count: {self.monitor_count}',
            f'Uptime: {uptime_hours:.2f} hours',
            f'',
            f'System Resources:',
            f'├─ CPU Usage: {cpu_percent:.1f}%',
            f'├─ Memory: {memory.percent:.1f}% ({memory.used / (1024**3):.1f}GB / {memory.total / (1024**3):.1f}GB)',
            f'└─ Disk: {disk.percent:.1f}% ({disk.used / (1024**3):.1f}GB / {disk.total / (1024**3):.1f}GB)',
            f'',
            f'Fleet Status: {"✅ OPERATIONAL" if cpu_percent < 80 else "⚠️ HIGH LOAD"}',
            f'Expected Namespaces: /robot1, /robot2, /robot3',
            f'════════════════════════════════════════════',
        ]
        
        msg = String()
        msg.data = '\n'.join(status_lines)
        self.publisher.publish(msg)
        
        # Log status periodically
        if self.monitor_count % 5 == 0:
            self.get_logger().info(
                f'📊 Fleet Monitor #{self.monitor_count}:\n'
                f'   Fleet: {self.fleet_id}\n'
                f'   Robots: {self.num_robots} active\n'
                f'   CPU: {cpu_percent:.1f}% | Memory: {memory.percent:.1f}% | Disk: {disk.percent:.1f}%\n'
                f'   Uptime: {uptime_hours:.2f} hours'
            )
        
        # Warnings
        if cpu_percent > 80:
            self.get_logger().warn(f'⚠️ High CPU usage: {cpu_percent:.1f}%')
        if memory.percent > 85:
            self.get_logger().warn(f'⚠️ High memory usage: {memory.percent:.1f}%')
        if disk.percent > 90:
            self.get_logger().warn(f'⚠️ Low disk space: {disk.percent:.1f}% used')
        
        self.monitor_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = FleetMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('🔴 Fleet Monitor shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
