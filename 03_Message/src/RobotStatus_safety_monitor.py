#!/usr/bin/env python3
"""
Robot Safety Monitor
Monitors robot status and generates safety alerts
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import RobotStatus
from datetime import datetime


class RobotSafetyMonitor(Node):
    def __init__(self):
        super().__init__('robot_safety_monitor')
        
        # Subscribe to robot status
        self.subscription = self.create_subscription(
            RobotStatus,
            'robot_status',
            self.status_callback,
            10
        )
        
        # Alert tracking
        self.alert_history = []
        self.last_battery_alert = 100
        self.last_health_status = "HEALTHY"
        self.consecutive_warnings = 0
        self.emergency_stop_active = False
        
        # Statistics
        self.message_count = 0
        self.critical_count = 0
        self.warning_count = 0
        
        self.get_logger().info('🛡️ Safety Monitor started - watching for alerts...')

    def get_timestamp(self):
        """Get formatted timestamp"""
        return datetime.now().strftime("%H:%M:%S")

    def log_alert(self, level, message, robot_name="Robot"):
        """Log alert with color coding"""
        timestamp = self.get_timestamp()
        
        # Color codes for terminal
        icons = {
            'CRITICAL': '🔴',
            'WARNING': '🟠',
            'INFO': '🟡',
            'OK': '🟢'
        }
        
        icon = icons.get(level, '⚪')
        alert_msg = f'{icon} [{timestamp}] {level}: {robot_name} - {message}'
        
        # Store in history
        self.alert_history.append({
            'timestamp': timestamp,
            'level': level,
            'message': message,
            'robot': robot_name
        })
        
        # Keep only last 50 alerts
        if len(self.alert_history) > 50:
            self.alert_history.pop(0)
        
        # Count alerts
        if level == 'CRITICAL':
            self.critical_count += 1
        elif level == 'WARNING':
            self.warning_count += 1
        
        # Log with appropriate level
        if level == 'CRITICAL':
            self.get_logger().error(alert_msg)
        elif level == 'WARNING':
            self.get_logger().warn(alert_msg)
        elif level == 'INFO':
            self.get_logger().info(alert_msg)
        else:
            self.get_logger().info(alert_msg)

    def check_battery(self, msg):
        """Monitor battery levels"""
        battery = msg.battery_percentage
        robot_name = msg.robot_name
        
        # Critical battery level
        if battery < 10 and self.last_battery_alert > 10:
            self.log_alert('CRITICAL', 
                f'Battery critically low: {battery}% - Immediate charging required!',
                robot_name)
            self.last_battery_alert = battery
        
        # Low battery warning
        elif battery < 20 and self.last_battery_alert >= 20:
            self.log_alert('WARNING',
                f'Battery low: {battery}% - Return to charging station',
                robot_name)
            self.last_battery_alert = battery
        
        # Battery getting low
        elif battery < 40 and self.last_battery_alert >= 40:
            self.log_alert('INFO',
                f'Battery at {battery}% - Consider charging soon',
                robot_name)
            self.last_battery_alert = battery
        
        # Battery charged
        elif battery > 90 and self.last_battery_alert < 90:
            self.log_alert('OK',
                f'Battery charged: {battery}%',
                robot_name)
            self.last_battery_alert = battery
        
        # Charging status change
        if msg.is_charging and battery < 95:
            if battery % 20 == 0:  # Log every 20%
                self.log_alert('INFO',
                    f'Charging in progress: {battery}% (ETA: {msg.battery_time_remaining} min)',
                    robot_name)

    def check_safety(self, msg):
        """Monitor safety status"""
        robot_name = msg.robot_name
        
        # Emergency stop
        if msg.emergency_stop and not self.emergency_stop_active:
            self.log_alert('CRITICAL',
                f'EMERGENCY STOP ACTIVATED! Obstacle at {msg.obstacle_distance:.2f}m',
                robot_name)
            self.emergency_stop_active = True
        elif not msg.emergency_stop and self.emergency_stop_active:
            self.log_alert('OK',
                'Emergency stop cleared - resuming operations',
                robot_name)
            self.emergency_stop_active = False
        
        # Obstacle detection (without emergency stop)
        if msg.obstacle_detected and not msg.emergency_stop:
            self.log_alert('WARNING',
                f'Obstacle detected at {msg.obstacle_distance:.2f}m - reducing speed',
                robot_name)

    def check_health(self, msg):
        """Monitor system health"""
        robot_name = msg.robot_name
        
        # Health status change
        if msg.health_status != self.last_health_status:
            if msg.health_status == 'CRITICAL':
                self.log_alert('CRITICAL',
                    f'System health CRITICAL! (CPU: {msg.cpu_usage:.1f}%, Temp: {msg.temperature:.1f}°C)',
                    robot_name)
            elif msg.health_status == 'ERROR':
                self.log_alert('WARNING',
                    f'System health ERROR (CPU: {msg.cpu_usage:.1f}%, Temp: {msg.temperature:.1f}°C)',
                    robot_name)
            elif msg.health_status == 'WARNING':
                self.log_alert('WARNING',
                    f'System health warning (CPU: {msg.cpu_usage:.1f}%, Temp: {msg.temperature:.1f}°C)',
                    robot_name)
            elif msg.health_status == 'HEALTHY' and self.last_health_status != 'HEALTHY':
                self.log_alert('OK',
                    'System health restored to normal',
                    robot_name)
            
            self.last_health_status = msg.health_status
        
        # Critical temperature
        if msg.temperature > 70:
            self.log_alert('CRITICAL',
                f'Temperature critical: {msg.temperature:.1f}°C - System overheating!',
                robot_name)
        elif msg.temperature > 60:
            self.log_alert('WARNING',
                f'Temperature high: {msg.temperature:.1f}°C',
                robot_name)
        
        # High CPU usage
        if msg.cpu_usage > 90:
            self.log_alert('WARNING',
                f'CPU usage very high: {msg.cpu_usage:.1f}%',
                robot_name)
        
        # Motors disabled
        if not msg.motors_enabled and msg.health_status != 'CRITICAL':
            self.log_alert('INFO',
                'Motors disabled - robot stationary',
                robot_name)

    def check_mission(self, msg):
        """Monitor mission status"""
        robot_name = msg.robot_name
        
        # Mission completion
        if msg.mission_progress == 100 and msg.mission_status == "COMPLETED":
            self.log_alert('OK',
                f'Mission "{msg.current_mission}" completed successfully',
                robot_name)
        
        # Mission failed
        elif msg.mission_status == "FAILED":
            self.log_alert('WARNING',
                f'Mission "{msg.current_mission}" failed at {msg.mission_progress}%',
                robot_name)
        
        # Mission paused
        elif msg.mission_status == "PAUSED":
            self.log_alert('INFO',
                f'Mission "{msg.current_mission}" paused at {msg.mission_progress}%',
                robot_name)

    def status_callback(self, msg):
        """Process incoming robot status messages"""
        self.message_count += 1
        
        # Run all safety checks
        self.check_battery(msg)
        self.check_safety(msg)
        self.check_health(msg)
        self.check_mission(msg)
        
        # Print summary every 30 messages
        if self.message_count % 30 == 0:
            self.print_summary()

    def print_summary(self):
        """Print monitoring summary"""
        self.get_logger().info(
            f'\n📊 Safety Monitor Summary:\n'
            f'   Messages: {self.message_count}\n'
            f'   🔴 Critical Alerts: {self.critical_count}\n'
            f'   🟠 Warnings: {self.warning_count}\n'
            f'   📋 Recent Alerts: {len(self.alert_history)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotSafetyMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n🛡️ Safety Monitor shutting down...')
        node.print_summary()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
