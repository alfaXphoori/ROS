#!/usr/bin/env python3
"""
Debug Monitor Node - Development & Testing Diagnostics
Real-world use: Extra logging and system health checks during development
Only runs in development or simulation mode (NOT in production)
"""

import rclpy
from rclpy.node import Node
import psutil
import time


class DebugMonitor(Node):
    def __init__(self):
        super().__init__('debug_monitor')
        
        # Declare robot parameters
        self.declare_parameter('robot_id', 'AMR-DEV-001')
        self.declare_parameter('robot_type', 'development')
        self.declare_parameter('zone_id', 'TEST-LAB')
        self.declare_parameter('diagnostic_rate_hz', 0.2)  # Every 5 seconds
        self.declare_parameter('enable_network_check', True)
        self.declare_parameter('enable_ros_diagnostics', True)
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.zone_id = self.get_parameter('zone_id').value
        diagnostic_rate = self.get_parameter('diagnostic_rate_hz').value
        self.network_check = self.get_parameter('enable_network_check').value
        self.ros_diagnostics = self.get_parameter('enable_ros_diagnostics').value
        
        # Timer for periodic diagnostics
        self.timer = self.create_timer(1.0/diagnostic_rate, self.diagnostics_callback)
        
        self.get_logger().info('🔧 DEBUG MONITOR ACTIVE')
        self.get_logger().warn('⚠️  Running in DEVELOPMENT/SIMULATION mode')
        self.get_logger().warn('⚠️  This node should NOT run in production!')
        self.get_logger().info(f'Robot ID: {self.robot_id}')
        self.get_logger().info(f'Robot Type: {self.robot_type}')
        self.get_logger().info(f'Test Zone: {self.zone_id}')
        self.get_logger().info(f'Diagnostic Rate: {diagnostic_rate:.2f} Hz')
        self.get_logger().info(f'Network Monitoring: {"Enabled" if self.network_check else "Disabled"}')
        self.get_logger().info(f'ROS Diagnostics: {"Enabled" if self.ros_diagnostics else "Disabled"}')
        
        self.start_time = time.time()
        self.check_count = 0
        self.warning_count = 0
        self.error_count = 0
        
    def diagnostics_callback(self):
        """
        Periodic system diagnostics
        Real-world: Monitor CPU, memory, network, disk during testing
        """
        self.check_count += 1
        uptime = time.time() - self.start_time
        uptime_hours = uptime / 3600
        
        # Get system metrics
        cpu_percent = psutil.cpu_percent(interval=1)
        memory = psutil.virtual_memory()
        memory_percent = memory.percent
        memory_available_gb = memory.available / (1024 ** 3)
        memory_used_gb = memory.used / (1024 ** 3)
        
        # Get disk usage
        disk = psutil.disk_usage('/')
        disk_percent = disk.percent
        disk_free_gb = disk.free / (1024 ** 3)
        
        # Get process info
        process = psutil.Process()
        process_memory_mb = process.memory_info().rss / (1024 ** 2)
        process_cpu_percent = process.cpu_percent(interval=1)
        
        # Get network stats if enabled
        network_status = "N/A"
        if self.network_check:
            net_io = psutil.net_io_counters()
            bytes_sent_mb = net_io.bytes_sent / (1024 ** 2)
            bytes_recv_mb = net_io.bytes_recv / (1024 ** 2)
            network_status = f"Sent: {bytes_sent_mb:.1f}MB | Recv: {bytes_recv_mb:.1f}MB"
        
        # Check for issues
        issues = []
        if cpu_percent > 80:
            issues.append(f'⚠️ HIGH CPU: {cpu_percent:.1f}%')
            self.warning_count += 1
        if memory_percent > 85:
            issues.append(f'⚠️ HIGH MEMORY: {memory_percent:.1f}%')
            self.warning_count += 1
        if disk_percent > 90:
            issues.append(f'⚠️ LOW DISK SPACE: {disk_free_gb:.1f}GB free')
            self.warning_count += 1
        
        status_icon = '✅' if len(issues) == 0 else '⚠️'
        status_text = 'System Healthy' if len(issues) == 0 else f'{len(issues)} Warning(s)'
        
        self.get_logger().info(
            f'\n'
            f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            f'🔍 DIAGNOSTIC CHECK #{self.check_count} [{self.robot_id}]\n'
            f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            f'🤖 Robot: {self.robot_id} ({self.robot_type})\n'
            f'📍 Zone: {self.zone_id}\n'
            f'⏱️  System Uptime: {uptime:.1f}s ({uptime_hours:.2f}h)\n'
            f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            f'💻 CPU Usage: {cpu_percent:.1f}% (Process: {process_cpu_percent:.1f}%)\n'
            f'🧠 System Memory: {memory_percent:.1f}% ({memory_used_gb:.2f}GB / {memory_used_gb + memory_available_gb:.2f}GB)\n'
            f'💾 Available RAM: {memory_available_gb:.2f} GB\n'
            f'📊 Process Memory: {process_memory_mb:.2f} MB\n'
            f'💿 Disk Usage: {disk_percent:.1f}% ({disk_free_gb:.1f}GB free)\n'
            f'🌐 Network: {network_status}\n'
            f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            f'{status_icon} Status: {status_text}\n'
            f'📈 Diagnostics: Checks: {self.check_count} | Warnings: {self.warning_count} | Errors: {self.error_count}\n'
            f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
        )
        
        # Log warnings
        for issue in issues:
            self.get_logger().warn(issue)
        
        # Critical errors
        if memory_percent > 95:
            self.error_count += 1
            self.get_logger().error('❌ CRITICAL: Memory usage critically high!')
        
        if disk_percent > 95:
            self.error_count += 1
            self.get_logger().error('❌ CRITICAL: Disk space critically low!')


def main(args=None):
    rclpy.init(args=args)
    
    debug_monitor = DebugMonitor()
    
    try:
        rclpy.spin(debug_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        debug_monitor.get_logger().info('🔴 Debug Monitor shutting down')
        debug_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
