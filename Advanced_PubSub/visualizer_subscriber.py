#!/usr/bin/env python3
"""
Visualizer Subscriber Node
Subscribes to sensor topics and displays data in formatted output
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from datetime import datetime
import os


class VisualizerSubscriber(Node):
    def __init__(self):
        super().__init__('visualizer_subscriber')
        
        # Subscribe to all sensor topics
        self.temp_subscription = self.create_subscription(
            Float64, 'sensor/temperature', self.temp_callback, 10)
        self.humidity_subscription = self.create_subscription(
            Float64, 'sensor/humidity', self.humidity_callback, 10)
        self.pressure_subscription = self.create_subscription(
            Float64, 'sensor/pressure', self.pressure_callback, 10)
        
        # Store latest values
        self.temperature = 0.0
        self.humidity = 0.0
        self.pressure = 0.0
        
        self.get_logger().info('Visualizer Subscriber started')

    def temp_callback(self, msg):
        self.temperature = msg.data
        self.display_dashboard()

    def humidity_callback(self, msg):
        self.humidity = msg.data
        self.display_dashboard()

    def pressure_callback(self, msg):
        self.pressure = msg.data
        self.display_dashboard()

    def display_dashboard(self):
        # Clear screen (Unix/Linux/macOS)
        os.system('clear' if os.name == 'posix' else 'cls')
        
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        print("\n" + "="*50)
        print("       🌡️  SENSOR DATA DASHBOARD 🌡️  ")
        print("="*50)
        print(f"Timestamp: {timestamp}\n")
        
        # Temperature visualization
        temp_bar = self._create_bar(self.temperature, 20, 30, "°C")
        print(f"🌡️  Temperature: {self.temperature:.2f}°C")
        print(f"    {temp_bar}\n")
        
        # Humidity visualization
        humid_bar = self._create_bar(self.humidity, 30, 80, "%")
        print(f"💧 Humidity: {self.humidity:.2f}%")
        print(f"    {humid_bar}\n")
        
        # Pressure visualization
        press_bar = self._create_bar(self.pressure, 950, 1050, "hPa")
        print(f"📊 Pressure: {self.pressure:.2f} hPa")
        print(f"    {press_bar}\n")
        
        print("="*50 + "\n")

    def _create_bar(self, value, min_val, max_val, unit):
        """Create a simple bar chart for visualization"""
        if value < min_val:
            percentage = 0
        elif value > max_val:
            percentage = 100
        else:
            percentage = int(((value - min_val) / (max_val - min_val)) * 100)
        
        bar_length = 30
        filled = int(bar_length * percentage / 100)
        bar = "█" * filled + "░" * (bar_length - filled)
        return f"[{bar}] {percentage}%"


def main(args=None):
    rclpy.init(args=args)
    node = VisualizerSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
