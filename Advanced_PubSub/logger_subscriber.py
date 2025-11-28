#!/usr/bin/env python3
"""
Logger Subscriber Node
Subscribes to sensor topics and logs data to a file
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from datetime import datetime
import os


class LoggerSubscriber(Node):
    def __init__(self):
        super().__init__('logger_subscriber')
        
        # Create log file
        self.log_file = '/tmp/sensor_data.log'
        self.file = open(self.log_file, 'a')
        
        # Subscribe to all sensor topics
        self.temp_subscription = self.create_subscription(
            Float64, 'sensor/temperature', self.temp_callback, 10)
        self.humidity_subscription = self.create_subscription(
            Float64, 'sensor/humidity', self.humidity_callback, 10)
        self.pressure_subscription = self.create_subscription(
            Float64, 'sensor/pressure', self.pressure_callback, 10)
        
        self.get_logger().info(f'Logger Subscriber started - Logging to {self.log_file}')

    def temp_callback(self, msg):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        log_msg = f'[{timestamp}] TEMPERATURE: {msg.data:.2f}°C\n'
        self.file.write(log_msg)
        self.file.flush()
        self.get_logger().info(f'Logged Temperature: {msg.data:.2f}°C')

    def humidity_callback(self, msg):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        log_msg = f'[{timestamp}] HUMIDITY: {msg.data:.2f}%\n'
        self.file.write(log_msg)
        self.file.flush()
        self.get_logger().info(f'Logged Humidity: {msg.data:.2f}%')

    def pressure_callback(self, msg):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        log_msg = f'[{timestamp}] PRESSURE: {msg.data:.2f} hPa\n'
        self.file.write(log_msg)
        self.file.flush()
        self.get_logger().info(f'Logged Pressure: {msg.data:.2f} hPa')

    def __del__(self):
        if hasattr(self, 'file'):
            self.file.close()


def main(args=None):
    rclpy.init(args=args)
    node = LoggerSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.__del__()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
