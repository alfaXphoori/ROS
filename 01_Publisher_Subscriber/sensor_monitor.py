#!/usr/bin/env python3
"""
Exercise 6: Sensor Monitor - Data Aggregation
Subscribes to temperature, humidity, and pressure topics
Aggregates and displays statistics every 5 seconds
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from collections import deque


class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')
        
        # Subscribe to three sensor topics
        self.temp_sub = self.create_subscription(
            Float32, 'temperature', self.temp_callback, 10
        )
        self.humidity_sub = self.create_subscription(
            Float32, 'humidity', self.humidity_callback, 10
        )
        self.pressure_sub = self.create_subscription(
            Float32, 'pressure', self.pressure_callback, 10
        )
        
        # Store latest readings
        self.latest_temp = 0.0
        self.latest_humidity = 0.0
        self.latest_pressure = 0.0
        
        # Keep history for statistics
        self.temp_history = deque(maxlen=10)
        self.humidity_history = deque(maxlen=10)
        self.pressure_history = deque(maxlen=10)
        
        # Timer to display statistics every 5 seconds
        self.timer = self.create_timer(5.0, self.display_stats)
        
        self.get_logger().info('Sensor Monitor started')

    def temp_callback(self, msg):
        self.latest_temp = msg.data
        self.temp_history.append(msg.data)
        self.get_logger().info(f'Temperature: {msg.data:.2f}°C')

    def humidity_callback(self, msg):
        self.latest_humidity = msg.data
        self.humidity_history.append(msg.data)
        self.get_logger().info(f'Humidity: {msg.data:.2f}%')

    def pressure_callback(self, msg):
        self.latest_pressure = msg.data
        self.pressure_history.append(msg.data)
        self.get_logger().info(f'Pressure: {msg.data:.2f} hPa')

    def display_stats(self):
        # Calculate statistics for each sensor
        if self.temp_history:
            temp_avg = sum(self.temp_history) / len(self.temp_history)
            temp_min = min(self.temp_history)
            temp_max = max(self.temp_history)
            self.get_logger().info(
                f'Temperature - Avg: {temp_avg:.2f}°C, '
                f'Min: {temp_min:.2f}°C, Max: {temp_max:.2f}°C'
            )

        if self.humidity_history:
            humid_avg = sum(self.humidity_history) / len(self.humidity_history)
            humid_min = min(self.humidity_history)
            humid_max = max(self.humidity_history)
            self.get_logger().info(
                f'Humidity - Avg: {humid_avg:.2f}%, '
                f'Min: {humid_min:.2f}%, Max: {humid_max:.2f}%'
            )

        if self.pressure_history:
            press_avg = sum(self.pressure_history) / len(self.pressure_history)
            press_min = min(self.pressure_history)
            press_max = max(self.pressure_history)
            self.get_logger().info(
                f'Pressure - Avg: {press_avg:.2f} hPa, '
                f'Min: {press_min:.2f} hPa, Max: {press_max:.2f} hPa'
            )

        # Display overall statistics
        if self.temp_history and self.humidity_history and self.pressure_history:
            overall_avg = (
                (sum(self.temp_history) / len(self.temp_history)) +
                (sum(self.humidity_history) / len(self.humidity_history)) +
                (sum(self.pressure_history) / len(self.pressure_history))
            ) / 3
            self.get_logger().info(f'Overall Average: {overall_avg:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = SensorMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
